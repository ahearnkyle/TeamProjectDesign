title GStream Starter
echo Starting GStreamer
:: you should not have to change this, but if you do find where the gstreamer folder is and replace with correct file path
::NOTE: QGroundControl looks for gstreamer in C:\gstreamer (need to confirm, but i read it somewhere)
C:\gstreamer\1.0\x86\bin\gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264" ! rtph264depay ! avdec_h264 ! autovideosink fps-update-interval=1000 sync=false
pause
::gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264" ! rtph264depay ! avdec_h264 ! autovideosink fps-update-interval=1000 sync=false
::gst-launch-1.0 -v udpsrc port=9000 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=f