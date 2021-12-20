# nvidia-rtsp-startup-artifacts

Code to regenerate startup artifacts issue when setting up an RTSP server using nvarguscamerasrc 

To build and run the code to reproduce the issue:

```
mkdir build && cd build
cmake ..
make
sudo make install
```

Then enable it as a startup service through systemd: `sudo systemctl enable rtsp-server`

Reboot and connect to the RTSP stream by typing `gst-launch-1.0 playbin uri=rtsp://<ip>/video0 uridecodebin0::source::latency=0`

On one out of every few reboots, there will be extreme amounts of artifacts in the video that appears.
