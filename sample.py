from SDM15 import SDM15, BaudRate

if __name__ == "__main__":
    lidar = SDM15("/dev/ttyUSB0", BaudRate.BAUD_460800)

    version_info = lidar.obtain_version_info()
    print("get version info success")

    lidar.lidar_self_test()
    print("self test success")

    lidar.start_scan()

    while True:
        try:
            distance, intensity, disturb = lidar.get_distance()
            print(f"distance: {distance}, intensity: {intensity}, disturb: {disturb}")
        except KeyboardInterrupt:
            break
