echo 320x480 > /sys/devices/platform/ahb0/13050000.dpu/layer0/target_size
echo 320x480 > /sys/devices/platform/ahb0/13050000.dpu/layer0/src_size
echo 0x0 > /sys/devices/platform/ahb0/13050000.dpu/layer0/target_pos
echo 1 > /sys/devices/platform/ahb0/13050000.dpu/layer0/src_fmt
echo 1 > /sys/devices/platform/ahb0/13050000.dpu/layer0/enable
ffmpeg -re -c:v h264_v4l2m2m -i vedio/aisilezuotian.mp4  -vf "scale=320:480" -pix_fmt bgra -f fbdev /dev/fb0 -an &
PID1=$!
ffmpeg -re -i vedio/aisilezuotian.mp4 -vn -f alsa default &
PID2=$!
wait $PID1 $PID2
