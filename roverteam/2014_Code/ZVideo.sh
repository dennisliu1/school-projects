gst-launch-1.0 -v -e udpsrc port=600$1 ! "application/x-rtp, payload=127" ! rtph264depay ! avdec_h264 ! xvimagesink sync=false
