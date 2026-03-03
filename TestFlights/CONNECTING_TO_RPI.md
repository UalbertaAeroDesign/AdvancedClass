# Connecting to the AeroDesign RPi 5

## Step 1 — Join the RPi's WiFi network
On your laptop or phone, connect to:
- **Network:** `AeroDesign-RPi`
- **Password:** `aeroclub1234`

This will allow you to SSH into it so you can start the code without a monitor.
Note that for code changes, we will need wifi to pull from git so youll need a monitor for that.

The RPi broadcasts this network automatically on boot. No router or phone hotspot needed.

## Step 2 — SSH into the RPi (optional)
If you need a terminal on the RPi:
```bash
ssh aerodesign@10.42.0.1
```


## Step 3 — View the live camera stream
While a precision landing script is running, open a browser and go to:
```
http://10.42.0.1:8080/
```
The stream shows the downward-facing camera feed with the detected AprilTag outlined, current flight phase, yaw error, and altitude overlaid. Works in Chrome, Firefox, Safari, or VLC.

> **Note:** The stream only appears while `precision_land_rpi.py` is actively running on the RPi. If the page doesn't load, the script isn't running yet.

## Useful DetailsXW
| | |
|---|---|
| RPi IP | `10.42.0.1` (always) |
| SSH user | `aerodesign` |
| Stream URL | `http://10.42.0.1:8080/` |
| Stream framerate | 15 fps |
| Stream resolution | 640 × 360 |
