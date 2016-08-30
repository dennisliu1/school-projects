Project ARMES video framework for URC 2013-2014.
The framework is split into three sections:
1. video source from the rover.
2. video broadcasting server.
3. video catching client.

gst-launch-1.0 -v -e v4l2src device=/dev/video1 num-buffers=500 ! video/x-h264,width=1280,height=720,framerate=30/1 ! tee name=t ! queue ! h264parse ! rtph264pay ! udpsink host=127.0.0.1 port=1234 t. ! queue ! h264parse ! mp4mux ! filesink location=/home/dennis/video0.mp4

gst-launch-1.0 -v -e udpsrc port=1234 ! multiudpsink clients="127.0.0.1:5000,127.0.0.1:5001"
gst-launch-1.0 -v -e udpsrc port=5001 ! "application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! xvimagesink sync=false

The GUI is currently consists of 2:
3.1. OperatorGUI = GUI for the driver.
3.2. NavigatorGUI = GUI for the navigator.

To install:
sudo apt-get install gstreamer0.10*
sudo apt-get install gstreamer1.0*
sudo apt-get install python-gst0.10*

