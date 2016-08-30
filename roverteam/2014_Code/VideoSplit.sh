clear
gst-launch-1.0 -v -e udpsrc port=200$1 ! multiudpsink clients="127.0.0.1:500$1,127.0.0.1:600$1"
# gst-launch-1.0 -v -e udpsrc port=200$1 ! multiudpsink clients="127.0.0.1:500$1,127.0.0.1:600$1, 192.168.1.104:500$1,192.168.1.104:600$1"