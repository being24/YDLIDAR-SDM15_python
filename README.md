# YDLIDAR-SDM15_python
A python3 library for the YDLidar SDM15.

## Sample code
```python
from SDM15 import SDM15, BaudRate

if __name__ == "__main__":
    lidar = SDM15("/dev/ttyUSB0", BaudRate.BAUD_460800) # change the port name to your own port

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
```

## Tips
- While scanning, the only thins you can do is to stop the scan. If you want to change the baud rate or other settings, you should stop the scan first.
- The lidar will not start scanning until you call the `start_scan()` method.
- The development manual recommends to start SDM15 normally as follow:
  - first, get version info from the lidar
  - second, do self test and get the result
  - third, start scan
- YdLidar's USB ADAPTER BOARD(which uses cp2102) only supports baud rate of 230400, 460800, 921600. So if you want to use other baud rate, you should use other USB TO UART module like FT231X.

