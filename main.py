import atexit
from dataclasses import dataclass
from enum import IntEnum

import serial


@dataclass
class VersionInfo:
    model: int
    hardware_version: int
    firmware_version_major: int
    firmware_version_minor: int
    serial_number: int


class BaudRateHex(IntEnum):
    BAUD_230400 = 0x00
    BAUD_460800 = 0x01
    # BAUD_512000 = 0x02
    # BAUD_921600 = 0x03
    # BAUD_1500000 = 0x04


class BaudRate(IntEnum):
    BAUD_230400 = 230400
    BAUD_460800 = 460800
    # BAUD_512000 = 512000
    # BAUD_921600 = 921600
    # BAUD_1500000 = 1500000


class OutputFreqHex(IntEnum):
    Freq_10Hz = 0x00
    Freq_100Hz = 0x01
    Freq_200Hz = 0x02
    Freq_500Hz = 0x03
    Freq_1000Hz = 0x04
    Freq_1800Hz = 0x05


class FilterHex(IntEnum):
    Off = 0x00
    On = 0x01


PACKET_HED1 = 0xAA
PACKET_HED2 = 0x55
START_SCAN = 0x60
STOP_SCAN = 0x61
GET_DEVICE_INFO = 0x62
SELF_TEST = 0x63
SET_OUTPUT_FREQ = 0x64
SET_FILTER = 0x65
SET_SERIAL_BAUD = 0x66
SET_FORMAT_OUTPUT_DATA = 0x67
RESTORE_FACTORY_SETTINGS = 0x68

NO_DATA = 0x00


class SDM15(object):
    """
    class for SDM15 serial communication
    """

    def __init__(self, port: str, baud_rate: BaudRate) -> None:
        self.ser = serial.Serial(port=port, baudrate=baud_rate)

        # check serial port is opened
        if not self.ser.is_open:
            raise Exception("serial port is not opened")

        self.scanning = False

        atexit.register(self.at_exit)

    def at_exit(self):
        self.stop_scan()
        self.ser.close()
        print("serial port is closed")

    def get_cmd_type(self, cmd: int) -> str:
        if cmd == START_SCAN:
            return "START_SCAN"
        elif cmd == STOP_SCAN:
            return "STOP_SCAN"
        elif cmd == GET_DEVICE_INFO:
            return "GET_DEVICE_INFO"
        elif cmd == SELF_TEST:
            return "SELF_TEST"
        elif cmd == SET_OUTPUT_FREQ:
            return "SET_OUTPUT_FREQ"
        elif cmd == SET_FILTER:
            return "SET_FILTER"
        elif cmd == SET_SERIAL_BAUD:
            return "SET_SERIAL_BAUD"
        elif cmd == SET_FORMAT_OUTPUT_DATA:
            return "SET_FORMAT_OUTPUT_DATA"
        elif cmd == RESTORE_FACTORY_SETTINGS:
            return "RESTORE_FACTORY_SETTINGS"
        else:
            return "UNKNOWN"

    @staticmethod
    def check(data: list[int]) -> int:
        # sum of data
        check_sum = sum(data)

        # get lsb
        check_sum = check_sum & 0xFF

        return check_sum

    def _write(self, cmd: bytes):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self.ser.write(cmd)
        self.ser.flush()

    def _read(self):
        while self.ser.in_waiting == 0:
            pass
        recv = self.ser.read_all()
        if recv is None or len(recv) == 0:
            raise FailedToReadError("no data received")

        recv_hex = recv.hex(":").split(":")
        recv_hex = [int(x, 16) for x in recv_hex]

        # check sum
        check_sum = recv_hex[-1]
        cal_check_sum = self.check(recv_hex[0:-1])

        if cal_check_sum != check_sum:
            print(f"check sum error: {check_sum} != {cal_check_sum}")

        return recv_hex

    def check_scanning(self):
        if self.scanning:
            raise LidarScanningError("lidar is scanning")

    def start_scan(self):
        cmd = bytes([PACKET_HED1, PACKET_HED2, START_SCAN, NO_DATA, 0x5F])
        self._write(cmd)
        self._read()
        self.scanning = True
        print("start scan")

    def stop_scan(self):
        cmd = bytes([PACKET_HED1, PACKET_HED2, STOP_SCAN, NO_DATA, 0x60])
        self._write(cmd)
        self._read()
        self.scanning = False
        print("stop scan")

    def obtain_version_info(self) -> VersionInfo:
        self.check_scanning()

        cmd = bytes([PACKET_HED1, PACKET_HED2, GET_DEVICE_INFO, NO_DATA, 0x61])

        self._write(cmd)
        recv = self._read()

        data_len = recv[3]
        data_segment = recv[4 : 4 + data_len]

        serial_number = data_segment[4 : 4 + data_len]
        serial_number = int("".join([str(x) for x in serial_number]))

        version_info = VersionInfo(
            model=data_segment[0],
            hardware_version=data_segment[1],
            firmware_version_major=data_segment[2],
            firmware_version_minor=data_segment[3],
            serial_number=serial_number,
        )

        return version_info

    def lidar_self_test(self) -> list[int]:
        self.check_scanning()

        cmd = bytes([PACKET_HED1, PACKET_HED2, SELF_TEST, NO_DATA, 0x62])

        self._write(cmd)
        recv = self._read()

        # header = recv[0:2]
        # cmd_type = recv[2]
        # print(self.get_cmd_type(cmd_type))

        data_len = recv[3]
        data_segment = recv[4 : 4 + data_len]

        self_test_result = data_segment[0]
        self_test_error_code = data_segment[1]

        if self_test_result != 0x01:
            raise SelfTestFailedError(f"self test failed error_code: {self_test_error_code}")

        self_test_data = data_segment[2:]

        return self_test_data

    def get_distance(self) -> tuple[int, int, int]:
        recv = self._read()

        # cmd_type = recv[2]

        data_len = recv[3]
        data_segment = recv[4 : 4 + data_len]

        distance_low = data_segment[0]
        distance_high = data_segment[1]

        distance = (distance_high << 8) | distance_low
        intensity = data_segment[2]
        disturb = data_segment[3]

        return distance, intensity, disturb

    def set_output_freq(self, freq: OutputFreqHex = OutputFreqHex.Freq_100Hz):
        self.check_scanning()

        cmd = [PACKET_HED1, PACKET_HED2, SET_OUTPUT_FREQ, 0x01, freq]
        check_sum = self.check(cmd)
        cmd.append(check_sum)
        cmd = bytes(cmd)
        self._write(cmd)
        recv = self._read()

        recv_freq = recv[4]

        if recv_freq != freq:
            raise Exception("set output freq failed")

        if recv_freq == OutputFreqHex.Freq_10Hz:
            print("set output freq to 10Hz")
        elif recv_freq == OutputFreqHex.Freq_100Hz:
            print("set output freq to 100Hz")
        elif recv_freq == OutputFreqHex.Freq_200Hz:
            print("set output freq to 200Hz")
        elif recv_freq == OutputFreqHex.Freq_500Hz:
            print("set output freq to 500Hz")
        elif recv_freq == OutputFreqHex.Freq_1000Hz:
            print("set output freq to 1000Hz")
        elif recv_freq == OutputFreqHex.Freq_1800Hz:
            print("set output freq to 1800Hz")

    def set_filter(self, filter: FilterHex = FilterHex.On):
        self.check_scanning()

        cmd = [PACKET_HED1, PACKET_HED2, SET_FILTER, 0x01, filter]
        check_sum = self.check(cmd)
        cmd.append(check_sum)
        cmd = bytes(cmd)
        self._write(cmd)
        recv = self._read()

        recv_filter = recv[4]

        if recv_filter != filter:
            raise Exception("set filter failed")

        if recv_filter == FilterHex.Off:
            print("set filter to off")
        elif recv_filter == FilterHex.On:
            print("set filter to on")

    def set_baud_rate(self, baud_rate: BaudRateHex = BaudRateHex.BAUD_460800):
        self.check_scanning()

        cmd = [PACKET_HED1, PACKET_HED2, SET_SERIAL_BAUD, 0x01, baud_rate]
        check_sum = self.check(cmd)
        cmd.append(check_sum)
        cmd = bytes(cmd)
        self._write(cmd)
        recv = self._read()

        recv_baud_rate = recv[4]

        if recv_baud_rate != baud_rate:
            raise Exception("set baud rate failed")

        if recv_baud_rate == BaudRateHex.BAUD_230400:
            print("set baud rate to 230400")
        elif recv_baud_rate == BaudRateHex.BAUD_460800:
            print("set baud rate to 460800")
        elif recv_baud_rate == BaudRateHex.BAUD_512000:
            print("set baud rate to 512000")
        elif recv_baud_rate == BaudRateHex.BAUD_921600:
            print("set baud rate to 921600")
        elif recv_baud_rate == BaudRateHex.BAUD_1500000:
            print("set baud rate to 1500000")


# def check(data: list[int]) -> int:
#     # sum of data
#     check_sum = sum(data) - 0x100

#     return check_sum


class CheckSumError(Exception):
    pass


class FailedToReadError(Exception):
    pass


class SelfTestFailedError(Exception):
    pass


class LidarScanningError(Exception):
    pass


if __name__ == "__main__":
    # ser = serial.Serial("/dev/ttyUSB0", 460800)
    lidar = SDM15("/dev/ttyUSB0", BaudRate.BAUD_460800)

    version_info = lidar.obtain_version_info()
    print("get version info success")
    # print(version_info)

    lidar.lidar_self_test()
    print("self test success")

    lidar.set_output_freq()
    lidar.set_filter()
    # lidar.set_baud_rate(BaudRateHex.BAUD_1500000)

    # lidar.start_scan()

    # while True:
    #     try:
    #         distance, intensity, disturb = lidar.get_distance()
    #         print(f"distance: {distance}, intensity: {intensity}, disturb: {disturb}")
    #     except KeyboardInterrupt:
    #         break
