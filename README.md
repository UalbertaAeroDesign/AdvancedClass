AdvancedClass demo, Oct 9 2025:

Install [Anaconda](https://www.anaconda.com/download) at your system-level for virtual environment.

- For Linux run `bash yolo_setup.sh` 

- Mac users may run the command `sh yolo_setup.sh` (someone with Mac test before merging)

- Windows users may run the command --- (someone on Windows add command)

After running the command, activate conda's virtual environment with `conda activate yolo` and any further `python` and `pip` commands will run isolated from your system environment.

Running the scripts with `python` after this should produce an error that it can't find the serial device `cu.usbserial-DN04T9FH`. At this point the script would work if the transmitted was plugged into the USB port.