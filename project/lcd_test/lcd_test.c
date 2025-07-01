#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/mman.h>
#include <linux/fb.h>

#include <linux/media.h>
#include <linux/videodev2.h>
struct fb_var_screeninfo varinfo;
int fb_fd;
char *fb_mapaddr;
int main(){

    int fb_w;
    int fb_h;
    int display_w;
    int display_h;

    fb_fd = open("/dev/fb0", O_RDWR);
	if (ioctl(fb_fd, FBIOGET_VSCREENINFO, &varinfo) == -1) {
		perror("open fb error\n");
        close(fb_fd);
		return -EINVAL;
	}

    fb_w = varinfo.xres;
	fb_h = varinfo.yres;

	display_w = fb_w;
	display_h = fb_h;

    /*默认map 一个帧buffer, 按照每个像素4byte去算.*/
	fb_mapaddr=(char*)mmap(0, fb_w * fb_h *4, PROT_WRITE | PROT_READ, MAP_SHARED, fb_fd, 0);
    if((int)fb_mapaddr < 0){
        perror("mmap fb_mapaddr error\n");
        munmap(fb_mapaddr,fb_w * fb_h *4);
        close(fb_fd);
    }
    int i=0;

    // printf("bits_per_pixel = %d\n", varinfo.bits_per_pixel);
    while(1){
        for(i=0;i<(fb_w * fb_h);i++){
            *((unsigned int *)fb_mapaddr + i)=0xffffffff;
        }
        sleep(1);
        for(i=0;i<(fb_w * fb_h);i++){
            *((unsigned int *)fb_mapaddr + i)=0x006495ED;
        }
        sleep(1);
        for(i=0;i<(fb_w * fb_h);i++){
            *((unsigned int *)fb_mapaddr + i)=0x00ADD8E6;
        }
        sleep(1);
        for(i=0;i<(fb_w * fb_h);i++){
            *((unsigned int *)fb_mapaddr + i)=0x0000FF7F;
        }
        sleep(1);
        for(i=0;i<(fb_w * fb_h);i++){
            *((unsigned int *)fb_mapaddr + i)=0x00B22222;
        }
        sleep(1);
    }


}

