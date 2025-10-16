AdvancedClass demo, Oct 9 2025:

Install [Anaconda](https://www.anaconda.com/download) at your system-level for virtual environment.

- For Linux run `bash yolo_setup.sh` 

- Mac users may run the command `sh yolo_setup.sh` (someone with Mac test before merging)

- Windows users running WSL may run the command "./yolo_setup.sh" (still should test)

After running the command, activate conda's virtual environment with `conda activate yolo` and any further `python` and `pip` commands will run isolated from your system environment.

Running the scripts with `python` after this should produce an error that it can't find the serial device `cu.usbserial-DN04T9FH`. 

If you're running macOS (with telemetry Radio or USBC cable plugged in and connected to flight controller) in then execute command `ls /dev/tty.*`, you should get an output similar or the same as `cu.usbserial-DN04T9FH`. 
Use whatever output you received to replace the first paratmer in any `open_serial("/dev/cu.usbserial-DN04T9FH", 57600)` function calls.

If youre running windows, in powershell execute `Get-WmiObject Win32_SerialPort | Select-Object DeviceID, Name, Description`. 
Expect an output similar to:
DeviceID  Name                 Description
--------  ----                 -----------
COM3      USB-SERIAL CH340     USB-SERIAL CH340
COM5      PX4 FMU V5           PX4 Autopilot

Replace first parameter in all `open_serial("/dev/cu.usbserial-DN04T9FH", 57600)` function calls with either whichever DeviceID from the output above is assocaited to the telemetry radio. 



At this point the script would work if the transmitted was plugged into the USB port.



Note: QGroundControl automatically attaches to the transmitter so you should disconnect or close it before running a script that connects to the device.