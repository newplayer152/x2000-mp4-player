项目名：x2000 简易MP4音视频播放器

硬件： st7796s 320*480 的mipi屏幕  x2000ElmightyBoard_V1.2_20211103作为主控

**主控板mipi DSI接口 与 屏幕接口：**

<img src="images\image-20250701110023529.png" alt="image-20250701110023529" style="zoom:25%;" />

<img src="images\QQ图片20250627140616.jpg" alt="QQ图片20250627140616" style="zoom: 25%;" />

驱动文件：panel-st7796s.c  放置于 sdk/kernel/kernel-4.4.94/drivers/video/fbdev/ingnic/fb_v12/displays 

其中panel-st7796s.c主要修改 

fitipower_st7796s_320_480_cmd_list1、panel_dev_sleep_out、panel_dev_display_on、panel_dev_panel_init使之符合上电初始化时序以及panel_modes、jzdsi_pdata屏幕的固定参数

并在其中修改Makefile、Kconfig文件，使panel-st7796s.c加入编译列表

Kconfig修改:

![image-20250701114054171](images\image-20250701114054171.png)

Makefile修改：

![image-20250701114602835](images\image-20250701114602835.png)

设备树sdk/kernel/kernel-4.4.94/arch/mips/boot/dts/ingenic/halley5_v20.dts修改内容：

	display-dbi {
		compatible = "simple-bus";
		#interrupt-cells = <1>;
		#address-cells = <1>;
		#size-cells = <1>;
		ranges = <>;
		panel_st7796s {
			compatible = "ingenic,st7796s";
			status = "okay";
			pinctrl-names = "default";
			pinctrl-0 = <&smart_lcd_pb_te>;
			ingenic,vdd-en-gpio = <&gpc 3 GPIO_ACTIVE_LOW INGENIC_GPIO_NOBIAS>;
			ingenic,rst-gpio = <&gpc 4 GPIO_ACTIVE_LOW INGENIC_GPIO_NOBIAS>;
	
			port {
				panel_st7796s_ep: endpoint {
					remote-endpoint = <&dpu_out_ep>;
				};
			};
		};
	};
	#if 1 //开启背光
	backlight {
			compatible = "pwm-backlight";
			pwms = <&pwm 0 1000000>; /* arg1: pwm channel id [0~15]. arg2: period in ns. */
			brightness-levels = <0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15>;
			default-brightness-level = <4>;
		};
	#endif
make kernel-menuconfig 中需要挂载设备步骤：(图中=y 的都要逐次打开并保存)

![image-20250701124138466](images\image-20250701124138466.png)

在测试屏幕时候编写了个测试小程序 lcd_test.c,使屏幕循环变色：

<img src="images\微信图片_20250701124412.jpg" alt="微信图片_20250701124412" style="zoom:25%;" />

<img src="images\微信图片_20250701124423.jpg" alt="微信图片_20250701124423" style="zoom:25%;" />

播放MP4 的.sh文件内容如下：

echo 320x480 > /sys/devices/platform/ahb0/13050000.dpu/layer0/target_size //设置输出分辨率
echo 320x480 > /sys/devices/platform/ahb0/13050000.dpu/layer0/src_size //设置源文件输出分辨率 后面
echo 0x0 > /sys/devices/platform/ahb0/13050000.dpu/layer0/target_pos //设置起始绘画点
echo 1 > /sys/devices/platform/ahb0/13050000.dpu/layer0/src_fmt //设置色彩模式 1与后面的-pix_fmt bgra 匹配    注意屏幕会对此有限制
echo 1 > /sys/devices/platform/ahb0/13050000.dpu/layer0/enable//设置显示层级使能
ffmpeg -re -c:v h264_v4l2m2m -i vedio/aisilezuotian.mp4  -vf "scale=320:480" -pix_fmt bgra -f fbdev /dev/fb0 -an & //使用h264_v4l2m2m 硬件解码
PID1=$!
ffmpeg -re -i vedio/aisilezuotian.mp4 -vn -f alsa default &  //将音频单独线程播放这样减少卡顿
PID2=$!
wait $PID1 $PID2 //可以由ctrl+c 主动结束

加上-vf "scale=320:480"表示 这条命令会将视频内容缩放至 framebuffer 匹配的分辨率，再送入 `/dev/fb0`，从而让内容能正确覆盖整块屏。







