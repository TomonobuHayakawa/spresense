/****************************************************************************
 * examples/camera/camera_main.c
 *
 *   Copyright (C) 2017 Sony Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <sdk/config.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <debug.h>
#include <fcntl.h>
#include <time.h>
#include <semaphore.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/fs/mkfatfs.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/video/video.h>
#include <nuttx/video/isx012.h>

#include <sys/ioctl.h>
#include <sys/boardctl.h>
#include <sys/mount.h>

#include <arch/chip/pm.h>
#include <arch/board/board.h>
#include <arch/chip/cisif.h>

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
#include <arch/chip/cxd56_audio.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/nx/nx.h>
#include <nuttx/nx/nxglib.h>
#include "nximage.h"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Display of vsync timing */
/* #define CAMERA_MAIN_CISIF_INTRTRACE */

/* Insert COM Maker(ShutterSpeed/ISO) in jpeg file */
#define CAMERA_MAIN_JPEG_INFO

#define IMG_BUF_MAXSIZE         (256 * 1024)
#define IMG_BUF_INFO_SIZE       (512)

#define MAX_SLEEP_TIME          (500*1000)
#define MID_SLEEP_TIME          (30)
#define MIN_SLEEP_TIME          (1)

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
#ifndef CONFIG_EXAMPLES_CAMERA_LCD_DEVNO
#  define CONFIG_EXAMPLES_CAMERA_LCD_DEVNO 0
#endif

#define itou8(v) ((v) < 0 ? 0 : ((v) > 255 ? 255 : (v)))
#define SEND_DATA_TO_LCD        (320*240*2)
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */

#ifdef CONFIG_SYSTEM_USBMSC
#define RAMDISK_MINOR           1
#define RAMDISK_SECTORSIZE      512
#define RAMDISK_NSECTORS        512

/* writable & free memory when unlinked. */
#define RAMDISK_RDFLAGS         (RDFLAG_WRENABLED | RDFLAG_FUNLINK)

/* /dev/ram1 */
#define RAMDISK_DEVPATH         CONFIG_SYSTEM_USBMSC_DEVPATH1

#define RAMDISK_MOUNTP          "/mnt/vfat"
#endif /* CONFIG_SYSTEM_USBMSC */

#define ALING32_CHECK(x)  (x % 32)
/****************************************************************************
 * Private Types
 ****************************************************************************/
struct uyvy_s
{
  uint8_t u0;
  uint8_t y0;
  uint8_t v0;
  uint8_t y1;
};

struct capture_info_s
{
  uint8_t code;
  uint8_t last_frame;
  uint32_t size;
  uint32_t addr;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
#ifdef CONFIG_SYSTEM_USBMSC
static int  camera_main_init(void);
static int  camera_main_write_ramdisk(uint8_t *data, size_t len,
                                      video_cap_frame_info_t *info,
                                      video_crop_t *crop);
#endif /* CONFIG_SYSTEM_USBMSC */

#ifdef CAMERA_MAIN_JPEG_INFO
static int camera_main_create_info(video_cap_frame_info_t *info,
                                   video_crop_t *crop);
#endif /* CAMERA_MAIN_JPEG_INFO */

static uint64_t camera_main_get_mstime(void);

#ifdef CAMERA_MAIN_CISIF_INTRTRACE
extern void cisif_intrtrace_start(int max);
#define CISIF_INTR_TRACE_START(x)   cisif_intrtrace_start(x)
#else
#define CISIF_INTR_TRACE_START(x)
#endif /* CAMERA_MAIN_CISIF_INTRTRACE */

/****************************************************************************
 * Private Data
 ****************************************************************************/
static struct pm_cpu_freqlock_s img_lock_hv =
  PM_CPUFREQLOCK_INIT(PM_CPUFREQLOCK_TAG('C','M',0), PM_CPUFREQLOCK_FLAG_HV);

static uint8_t  camera_img_buf[IMG_BUF_MAXSIZE] __attribute__((aligned(32)));
#ifdef CONFIG_SYSTEM_USBMSC
static uint8_t  camera_ramdisk[RAMDISK_NSECTORS * RAMDISK_SECTORSIZE];
#endif /* CONFIG_SYSTEM_USBMSC */

static int v_fd;

static int camera_main_initialized = 0;
static volatile int camera_main_thread_active = 0;
static volatile int camera_main_thread_exit = 0;
static uint64_t camera_main_time_start;

#define DBG_TIME_START()    \
  camera_main_time_start = camera_main_get_mstime()
#define DBG_TIME_STOP()     \
  (camera_main_get_mstime() - camera_main_time_start)

static uint8_t camera_main_file_count = 0;
static char camera_main_filename[32];

static const char *iso_str[VIDEO_ISO_MAX] =
  {
    "NONE   ", "ISO25  ", "ISO32  ", "ISO40  ", "ISO50  ", "ISO64  ",
    "ISO100 ", "ISO125 ", "ISO160 ", "ISO200 ", "ISO250 ", "ISO320 ",
    "ISO400 ", "ISO500 ", "ISO640 ", "ISO800 ", "ISO1000", "ISO1250",
    "ISO1600"
  };

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
struct nximage_data_s g_nximage =
{
  NULL,          /* hnx */
  NULL,          /* hbkgd */
  0,             /* xres */
  0,             /* yres */
  false,         /* havpos */
  { 0 },         /* sem */
  0              /* exit code */
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
static inline int nximage_initialize(void)
{
  FAR NX_DRIVERTYPE *dev;
  nxgl_mxpixel_t color;
  int ret;

  /* Initialize the LCD device */

  printf("nximage_initialize: Initializing LCD\n");
  ret = board_lcd_initialize();
  if (ret < 0)
    {
      printf("nximage_initialize: board_lcd_initialize failed: %d\n", -ret);
      return ERROR;
    }

  /* Get the device instance */

  dev = board_lcd_getdev(CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
  if (!dev)
    {
      printf("nximage_initialize: board_lcd_getdev failed, devno=%d\n",
             CONFIG_EXAMPLES_CAMERA_LCD_DEVNO);
      return ERROR;
    }

  /* Turn the LCD on at 75% power */

  (void)dev->setpower(dev, ((3*CONFIG_LCD_MAXPOWER + 3)/4));

  /* Then open NX */

  printf("nximage_initialize: Open NX\n");
  g_nximage.hnx = nx_open(dev);
  if (!g_nximage.hnx)
    {
      printf("nximage_initialize: nx_open failed: %d\n", errno);
      return ERROR;
    }

  /* Set background color to black */

  color = 0;
  nx_setbgcolor(g_nximage.hnx, &color);
  ret = nx_requestbkgd(g_nximage.hnx, &g_nximagecb, NULL);
  if (ret < 0)
    {
      printf("nximage_initialize: nx_requestbkgd failed: %d\n", errno);
      nx_close(g_nximage.hnx);
      return ERROR;
    }

  while (!g_nximage.havepos)
    {
      (void) sem_wait(&g_nximage.sem);
    }
  printf("nximage_initialize: Screen resolution (%d,%d)\n", g_nximage.xres, g_nximage.yres);

  return 0;
}

static inline void ycbcr2rgb(uint8_t y, uint8_t cb, uint8_t cr, uint8_t *r, uint8_t *g, uint8_t *b)
{
  int _r;
  int _g;
  int _b;
  _r = (128 * (y-16) +                  202 * (cr-128) + 64) / 128;
  _g = (128 * (y-16) -  24 * (cb-128) -  60 * (cr-128) + 64) / 128;
  _b = (128 * (y-16) + 238 * (cb-128)                  + 64) / 128;
  *r = itou8(_r);
  *g = itou8(_g);
  *b = itou8(_b);
}

static inline uint16_t ycbcrtorgb565(uint8_t y, uint8_t cb, uint8_t cr)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;

  ycbcr2rgb(y, cb, cr, &r, &g, &b);
  r = (r >> 3) & 0x1f;
  g = (g >> 2) & 0x3f;
  b = (b >> 3) & 0x1f;
  return (uint16_t)(((uint16_t)r << 11) | ((uint16_t)g << 5) | (uint16_t)b);
}

/* Color conversion to show on display devices. */

static void yuv2rgb(void *buf, uint32_t size)
{
  struct uyvy_s *ptr;
  struct uyvy_s uyvy;
  uint16_t *dest;
  uint32_t i;

  ptr = buf;
  dest = buf;
  for (i = 0; i < size / 4; i++)
    {
      /* Save packed YCbCr elements due to it will be replaced with
       * converted color data.
       */

      uyvy = *ptr++;

      /* Convert color format to packed RGB565 */

      *dest++ = ycbcrtorgb565(uyvy.y0, uyvy.u0, uyvy.v0);
      *dest++ = ycbcrtorgb565(uyvy.y1, uyvy.u0, uyvy.v0);
    }
}

static int camera_main_lcd_open(void)
{
  int ret;
  ret = nximage_initialize();
  if (ret < 0)
    {
      printf("camera_main: Failed to get NX handle: %d\n", errno);
    }

  return ret;
}

static void camera_main_lcd_close(void)
{
  /* nx_releasebkgd(g_nximage.hbkgd); */
  nx_close(g_nximage.hnx);
}

static void camera_main_lcd_write(void *capturebuffer, uint32_t capturesize)
{
  /* Convert YUV color format to RGB565 */

  yuv2rgb(capturebuffer, capturesize);

  nximage_image(g_nximage.hbkgd, capturebuffer);
}
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */

static int camera_change_imgsns_state(video_img_sns_state_e state)
{
  video_api_chg_img_sns_state_t p;

  if (state >= VIDEO_STATE_MAX)
    {
      return -EINVAL;
    }

  p.state = state;

  return ioctl(v_fd, VIDEOIOC_CHG_IMGSNS_STATE, (unsigned long)&p);
}

static int camera_set_capture_param(video_mode_e mode, video_cap_param_t *cap_param)
{
  video_api_set_cap_param_t p;

  if ((mode != VIDEO_MODE_CAPTURE) && (mode != VIDEO_MODE_MONITORING))
    {
      return -EINVAL;
    }

  if (cap_param == NULL) 
    {
      return -EINVAL;
    }

  if ((cap_param->resolution < VIDEO_QVGA) ||
      (cap_param->resolution >= VIDEO_RESOLUTION_MAX) ||
      (cap_param->format < VIDEO_FORMAT_YUV) ||
      (cap_param->format >= VIDEO_FORMAT_MAX) ||
      (cap_param->framerate < VIDEO_120FPS) ||
      (cap_param->framerate >= VIDEO_FRAME_RATE_MAX))
    {
      return -EINVAL;
    }

  p.mode = mode;
  memcpy(&p.param, cap_param, sizeof(video_cap_param_t));

  return ioctl(v_fd, VIDEOIOC_SET_CAP_PARAM, (unsigned long)&p);
}

static int camera_capture_frame(
  video_mode_e mode,
  video_buffer_t *buffer,
  video_crop_t *crop,
  video_cap_frame_info_t *info)
{
  video_api_cap_frame_t p;
  int ret;

  if ((mode != VIDEO_MODE_CAPTURE) && (mode != VIDEO_MODE_MONITORING))
    {
      return -EINVAL;
    }

  if ((buffer == NULL) || (info == NULL))
    {
      return -EINVAL;
    }

  if ((ALING32_CHECK(buffer->addr) != 0) ||
      (ALING32_CHECK(buffer->size) != 0))
    {
      return -EINVAL;
    }

  p.mode = mode;
  p.buffer.addr = buffer->addr;
  p.buffer.size = buffer->size;
  memset(&p.info, 0, sizeof(video_cap_frame_info_t));

  if (crop == NULL)
    {
      p.crop_ctrl = VIDEO_DISABLE;
    }
  else
    {
      p.crop_ctrl = VIDEO_ENABLE;
      p.crop.x_offset = crop->x_offset;
      p.crop.y_offset = crop->y_offset;
    }

  ret = ioctl(v_fd, VIDEOIOC_CAP_FRAME, (unsigned long)&p);
  if (ret == 0)
    {
      memcpy(info, &p.info, sizeof(video_cap_frame_info_t));
    }
  return ret;
}

static int camera_set_imgsns_param(video_img_sns_param_t *param)
{
  video_api_set_img_sns_param_t p;

  if (param == NULL)
    {
      return -EINVAL;
    }

  if ((param->id < VIDEO_PARAM_ID_COLOR) ||
      (param->id >= VIDEO_PARAM_ID_MAX))
    {
      return -EINVAL;
    }

  memcpy(&p.param, param, sizeof(video_img_sns_param_t));

  return ioctl(v_fd, VIDEOIOC_SET_IMGSNS_PARAM, (unsigned long)&p);
}

static int camera_set_imgsns_param_all(video_img_sns_param_all_t *param)
{
  video_api_set_img_sns_param_all_t p;

  if (param == NULL)
    {
      return -EINVAL;
    }

  memcpy(&p.param, param, sizeof(video_api_set_img_sns_param_all_t));

  return ioctl(v_fd, VIDEOIOC_SET_IMGSNS_PARAM_ALL, (unsigned long)&p);
}

static int camera_write_imgsns_register(
  uint16_t addr, uint16_t regsize, uint16_t value)
{
  video_api_img_sns_reg_t p;

  if ((regsize != 1) && (regsize != 2))
    {
      return -EINVAL;
    }

  p.addr = addr;
  p.regsize = regsize;
  p.val = value;

  return ioctl(v_fd, VIDEOIOC_WR_IMGSNS_REG, (unsigned long)&p);
}

static int camera_read_imgsns_register(
  uint16_t addr, uint16_t regsize, uint16_t *value)
{
  video_api_img_sns_reg_t p;
  int ret;

  if (value == NULL)
    {
      return -EINVAL;
    }

  if ((regsize != 1) && (regsize != 2))
    {
      return -EINVAL;
    }

  p.addr = addr;
  p.regsize = regsize;

  ret = ioctl(v_fd, VIDEOIOC_RD_IMGSNS_REG, (unsigned long)&p);
  if (ret == 0)
    {
      *value = p.val;
    }

  return ret;
}

static int camera_do_halfrelease(video_auto_info_t *info, video_ctrl_e cancel)
{
  video_api_do_half_rel_t p;
  int ret;

  if (info == NULL)
    {
      return -EINVAL;
    }

  p.cancel = cancel;
  ret = ioctl(v_fd, VIDEOIOC_DO_HALF_REL, (unsigned long)&p);
  if ((ret == 0) && (cancel != VIDEO_ENABLE))
    {
      memcpy(info, &p.info, sizeof(video_auto_info_t));
    }

  return ret;
}

static int camera_id_get_auto_param(video_auto_info_t *info)
{
  video_api_get_auto_param_t p;
  int ret;

  if (info == NULL)
    {
      return -EINVAL;
    }

  memset(&p, 0, sizeof(video_api_get_auto_param_t));
  ret = ioctl(v_fd, VIDEOIOC_GET_AUTO_PARAM, (unsigned long)&p);
  if (ret == 0)
    {
      memcpy(info, &p.info, sizeof(video_auto_info_t));
    }

  return ret;
}

static int camera_id_continuous_capture(video_conti_param_t *param,
                        video_buffer_t *buffer,
                        video_crop_t *crop,
                        video_conti_cap_info_t *info)
{
  video_api_conti_cap_t p;
  int ret;

  if ((param == NULL) || (buffer == NULL) || (info == NULL))
    {
      return -EINVAL;
    }

  if ((param->mode != VIDEO_MODE_CAPTURE) ||
      (param->num > VIDEO_CONTI_CAPNUM_MAX))
    {
      return -EINVAL;
    }

  if ((ALING32_CHECK(buffer->addr) != 0) ||
      (ALING32_CHECK(buffer->size) != 0))

  {
      return -EINVAL;
    }

  p.mode = param->mode;
  p.capnum = param->num;
  p.interval = param->interval;
  p.buffer.addr = buffer->addr;
  p.buffer.size = buffer->size;
  memset(&p.info, 0, sizeof(video_conti_cap_info_t));

  if (crop == NULL)
    {
      p.crop_ctrl = VIDEO_DISABLE;
    }
  else
    {
      p.crop_ctrl = VIDEO_ENABLE;
      p.crop.x_offset = crop->x_offset;
      p.crop.y_offset = crop->y_offset;
    }

  ret = ioctl(v_fd, VIDEOIOC_CONTI_CAP, (unsigned long)&p);

  memcpy(info, &p.info, sizeof(video_conti_cap_info_t));

  return ret;
}

extern uint32_t cxd56_get_cpu_baseclk(void);
static uint64_t camera_main_get_mstime(void)
{
    struct timespec tp;
    if (clock_gettime(CLOCK_REALTIME, &tp)) {
        return 0;
    }
    return (((uint64_t)tp.tv_sec) * 1000 + tp.tv_nsec / 1000000);
}

#ifdef CONFIG_SYSTEM_USBMSC
static int camera_main_init(void)
{
  struct fat_format_s fat_fmt = FAT_FORMAT_INITIALIZER;

  /* Create a RAMDISK */
  if (0 < ramdisk_register(RAMDISK_MINOR,
                           camera_ramdisk,
                           RAMDISK_NSECTORS,
                           RAMDISK_SECTORSIZE,
                           RAMDISK_RDFLAGS))
    {
      printf("ERROR: Failed to ramdisk_register. %d\n", errno);
      return -1;
    }

  /* Create a FAT filesystem on the RAMDISK */
  if (0 < mkfatfs(RAMDISK_DEVPATH, &fat_fmt))
    {
      printf("ERROR: Failed to mkfatfs. %d\n", errno);
      return -1;
    }

  return 0;
}

static int camera_main_write_ramdisk(
  uint8_t *data,
  size_t len,
  video_cap_frame_info_t *info,
  video_crop_t *crop
)
{
  FILE *fp;
  int  fd;
  int  del = 0;

#ifdef CONFIG_SYSTEM_USBMSC
  if(0 != mount(RAMDISK_DEVPATH, RAMDISK_MOUNTP, "vfat", 0, NULL))
    {
      printf("mount error : %d\n", errno);
      return  -1;
    }
#else
    return -1;
#endif /* CONFIG_SYSTEM_USBMSC */

  camera_main_file_count++;
  if(camera_main_file_count >= 1000)
    {
      camera_main_file_count = 1;
    }

  memset(camera_main_filename, 0, sizeof(camera_main_filename));
  sprintf(camera_main_filename,
         "/mnt/vfat/VIDEO%03d.JPG",
          camera_main_file_count);
  printf("FILENAME:%s\n", camera_main_filename);

  fp = fopen(camera_main_filename, "wb");
  if (NULL == fp)
    {
      printf("fopen error : %d\n", errno);
      return -1;
    }

#ifdef CAMERA_MAIN_JPEG_INFO
  if ((info != NULL) || (crop != NULL))
    {
      camera_main_create_info(info, crop);
      len += IMG_BUF_INFO_SIZE;
      data = camera_img_buf;
    }
#endif /* CAMERA_MAIN_JPEG_INFO */

  if (len != fwrite(data, 1, len, fp))
    {
      printf("fwrite error : %d\n", errno);
      del = 1;
    }

  fflush(fp);
  fd = fileno(fp);
  fsync(fd);
  fclose(fp);

  if (del)
    {
      unlink((const char *)camera_main_filename);
    }

#ifdef CONFIG_SYSTEM_USBMSC
  umount(RAMDISK_MOUNTP);
#endif /* CONFIG_SYSTEM_USBMSC */

  return 0;
}
#endif /* CONFIG_SYSTEM_USBMSC */

static void camera_main_disable_unused_board_power(void)
{
  /* ImageSensor Power forced OFF (for reboot) */
  board_power_control(POWER_IMAGE_SENSOR, 0);

#ifdef CONFIG_BOARD_SPRESENSE
  board_flash_power_control(0);

  board_power_control(POWER_LDO_PERI, 0);
  board_power_control(POWER_LDO_EMMC, 0);
  board_power_control(POWER_AUDIO_DVDD, 0);

  /* Defaut Off */
  board_power_control(POWER_AUDIO_AVDD, 0);
  board_power_control(POWER_AUDIO_MUTE, 0);
#endif
}

#ifdef CAMERA_MAIN_JPEG_INFO
static int camera_main_create_info(
  video_cap_frame_info_t *info,
  video_crop_t *crop)
{
  uint16_t offset;
  uint16_t com_marker_len;

  printf("Insert COM Maker.\n");

  /* len = INFO_SIZE - (SOI + COM) + SOI(Orignal SOI) */
  com_marker_len = IMG_BUF_INFO_SIZE - (2 + 2) + 2;

  /* Set SOI + COM + COM Marker length */
  camera_img_buf[0] = 0xFF;
  camera_img_buf[1] = 0xD8;  /* SOI */
  camera_img_buf[2] = 0xFF;
  camera_img_buf[3] = 0xFE;  /* COM */
  camera_img_buf[4] = (uint8_t)((com_marker_len & 0xFF00) >> 8);
  camera_img_buf[5] = (uint8_t)(com_marker_len & 0x00FF);
  offset = (2 + 2 + 2);

  if (info)
    {
      /* Set information data : 1 record = 32bytes */
      snprintf((char *)&camera_img_buf[offset], 32,
               "\"ShutterSpeed\":%d", info->pict_info.shutter_speed);
      offset += 32;
      snprintf((char *)&camera_img_buf[offset], 32,
               "\"ISO\":%s\n", iso_str[info->pict_info.iso_sens]);
    }

  /* Clear Original SOI */
  camera_img_buf[IMG_BUF_INFO_SIZE]   = 0x00;
  camera_img_buf[IMG_BUF_INFO_SIZE+1] = 0x00;

  return 0;
}
#endif /* CAMERA_MAIN_JPEG_INFO */

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_BUILD_KERNEL
int camera_main(int argc, FAR char *argv[])
#else
int camera_main(int argc, char *argv[])
#endif
{
  int ret;
  int i;
  int cnt;
  uint32_t time;

  if ( strncmp(argv[1], "cap", 3)==0 || strncmp(argv[1], "moni", 4)==0)
    {
      video_mode_e mode;
      video_buffer_t buffer;
      video_cap_frame_info_t info;

      /*============================ init ====================*/
      if (!camera_main_initialized)
        {
          printf("bootcause=0x%08X\n", up_pm_get_bootcause());
          camera_main_disable_unused_board_power();
#ifdef CONFIG_SYSTEM_USBMSC
          camera_main_init();
#endif /* CONFIG_SYSTEM_USBMSC */

          DBG_TIME_START();

#ifdef CONFIG_VIDEO_ISX012
          ret = board_isx012_initialize("/dev/video", IMAGER_I2C);
          if (ret != 0)
            {
              printf("ERROR: Failed to init video. %d\n", errno);
              return -EPERM;
            }
#endif
          camera_main_initialized = 1;
        }

      v_fd = open("/dev/video0", O_CREAT);
      time = DBG_TIME_STOP();
      printf("open video : ret=%d, time=%d[ms]\n", ret, time);
      if (v_fd < 0)
        {
          printf("ERROR: Failed to open video. %d\n", errno);
          return -ENODEV;
        }

      video_cap_param_t     cap_param;
      video_img_sns_param_t imgsns_param;

      cap_param.format     = VIDEO_FORMAT_YUV;
      cap_param.resolution = VIDEO_QVGA;
      cap_param.framerate  = VIDEO_30FPS;
      ret = camera_set_capture_param(VIDEO_MODE_MONITORING, &cap_param);
      if (ret == 0)
        {
          printf("change Monitoring param : YUV QVGA@30fps\n");
        }

      cap_param.format     = VIDEO_FORMAT_JPEG;
      cap_param.resolution = VIDEO_QUADVGA;
      cap_param.framerate  = VIDEO_15FPS;
      ret = camera_set_capture_param(VIDEO_MODE_CAPTURE, &cap_param);
      if (ret == 0)
        {
          printf("change Capture param    : JPEG VGA@15fps\n");
        }

      imgsns_param.id = VIDEO_PARAM_ID_JPEG_QUALITIY;
      imgsns_param.val.jpeg_qualitiy = 75;
      ret = camera_set_imgsns_param(&imgsns_param);
      if (ret == 0)
        {
          printf("change JPEG Quality     : 75\n");
        }

      /*============================ cap / moni ====================*/
      up_pm_acquire_freqlock(&img_lock_hv);
      printf("CPU BaseClock:%dHz\n", cxd56_get_cpu_baseclk());
      if ( strncmp(argv[1], "cap", 3)==0 )
        {
          mode = VIDEO_MODE_CAPTURE;
          cnt = 1;
        }
      if ( strncmp(argv[1], "moni", 4)==0)
        {
          mode = VIDEO_MODE_MONITORING;
          cnt = 50;
        }

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
      camera_main_lcd_open();
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */

      memset(camera_img_buf, 0, sizeof(camera_img_buf));
#ifndef CAMERA_MAIN_JPEG_INFO
      buffer.addr = (uint32_t)camera_img_buf;
      buffer.size = IMG_BUF_MAXSIZE;
#else
      buffer.addr = (uint32_t)camera_img_buf + IMG_BUF_INFO_SIZE;
      buffer.size = IMG_BUF_MAXSIZE - IMG_BUF_INFO_SIZE;
#endif

      memset(&info, 0, sizeof(video_cap_frame_info_t));
      CISIF_INTR_TRACE_START(20);

      for (i=0; i < cnt; i++)
        {
          DBG_TIME_START();
          ret = camera_capture_frame(mode, &buffer, NULL, &info);
          time = DBG_TIME_STOP();
          if (ret == 0)
            {
              if (info.out_addr != buffer.addr)
                {
                  printf("The Buffer address is not match.\n");
                  printf("Dump All Buffer Data at JPEG File.\n");
#ifdef CONFIG_SYSTEM_USBMSC
                  camera_main_write_ramdisk((uint8_t *)buffer.addr,
                                            (size_t)buffer.size,
                                            &info,
                                            NULL);
#endif /* CONFIG_SYSTEM_USBMSC */
                  return -EINVAL;
                }

              if (info.cap_param.format == VIDEO_FORMAT_JPEG)
                {
#ifdef CONFIG_SYSTEM_USBMSC
                  camera_main_write_ramdisk((uint8_t *)info.out_addr,
                                            (size_t)info.out_size,
                                            &info,
                                            NULL);
#endif /* CONFIG_SYSTEM_USBMSC */
                }

              if ((info.cap_param.format == VIDEO_FORMAT_YUV) &&
                  (info.cap_param.resolution == VIDEO_QVGA))
                {
#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
                  camera_main_lcd_write((void *)buffer.addr, buffer.size);
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */
                }
            }

          printf("camera_capture_frame() : ret=%d, time=%d[ms]\n", ret, time);
        }

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
        camera_main_lcd_close();
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */
        up_pm_release_freqlock(&img_lock_hv);
        printf("Out data addr     : 0x%08X\n", info.out_addr);
        printf("Out data size     : %d\n", info.out_size);
        printf("Image Size        : %dx%d\n", info.h_size, info.v_size);
        printf("ISO Sens          : %s\n", iso_str[info.pict_info.iso_sens]);
        printf("Shutter Speed     : %d[us]\n", info.pict_info.shutter_speed);

        close(v_fd);
    }

#if 0 /******************************* org *******************************/
  if (argc < 2)
    {
      printf("Invalid command format.\n");
      return -EINVAL;
    }

  if( strncmp(argv[1], "init", 4)==0 )
    {
      if (!camera_main_initialized)
        {
          printf("bootcause=0x%08X\n", up_pm_get_bootcause());
          camera_main_disable_unused_board_power();

          camera_main_init();
          camera_main_initialized = 1;
        }

      DBG_TIME_START();

#ifdef CONFIG_VIDEO_ISX012
      ret = board_isx012_initialize("/dev/video", IMAGER_I2C);
#endif
      v_fd = open("/dev/video0", O_CREAT);
      if (v_fd < 0)
        {
          printf("ERROR: Failed to open video. %d\n", errno);
          return -ENODEV;
        }

      time = DBG_TIME_STOP();
      printf("video_init() : ret=%d, time=%d[ms]\n", ret, time);
      if (ret == 0)
        {
          video_cap_param_t cap_param;
          video_img_sns_param_t imgsns_param;

          cap_param.format     = VIDEO_FORMAT_YUV;
          cap_param.resolution = VIDEO_QVGA;
          cap_param.framerate  = VIDEO_30FPS;
          ret = camera_set_capture_param(VIDEO_MODE_MONITORING, &cap_param);
          if (ret == 0)
            {
              printf("change Monitoring param : YUV QVGA@30fps\n");
            }

          cap_param.format     = VIDEO_FORMAT_JPEG;
          cap_param.resolution = VIDEO_QUADVGA;
          cap_param.framerate  = VIDEO_15FPS;
          ret = camera_set_capture_param(VIDEO_MODE_CAPTURE, &cap_param);
          if (ret == 0)
            {
              printf("change Capture param    : JPEG VGA@15fps\n");
            }

          imgsns_param.id = VIDEO_PARAM_ID_JPEG_QUALITIY;
          imgsns_param.val.jpeg_qualitiy = 75;
          ret = camera_set_imgsns_param(&imgsns_param);
          if (ret == 0)
            {
              printf("change JPEG Quality     : 75\n");
            }
        }

    }
  else if ( strncmp(argv[1], "chgsts", 6)==0 )
    {
      video_img_sns_state_e sts;

      if ( argc != 3 )
        {
          printf("The number of arguments does not match.\n");
          return -EINVAL;
        }

      if ( strncmp(argv[2], "active", 6)==0 )
        {
          sts = VIDEO_STATE_ACTIVE;
        }
      else if ( strncmp(argv[2], "sleep", 5)==0 )
        {
          sts = VIDEO_STATE_SLEEP;
        }
      else if ( strncmp(argv[2], "powoff", 6)==0 )
        {
          sts = VIDEO_STATE_POWOFF;
        }
      else if ( strncmp(argv[2], "powon", 5)==0 )
        {
          sts = VIDEO_STATE_POWON;
        }
      else
        {
          /* parameter error */
          sts = (video_img_sns_state_e)atoi(argv[2]);
        }

      CISIF_INTR_TRACE_START(20);

      DBG_TIME_START();
      ret = camera_change_imgsns_state(sts);
      time = DBG_TIME_STOP();
      printf("camera_change_imgsns_state() : ret=%d, time=%d[ms]\n", ret, time);
    }
  else if ( strncmp(argv[1], "setcap", 6)==0 )
    {
      video_mode_e mode;
      video_cap_param_t cap_param;

      if ( argc != 6 )
        {
          printf("The number of arguments does not match.\n");
          return -EINVAL;
        }

      if ( strncmp(argv[2], "cap", 3)==0 )
        {
          mode = VIDEO_MODE_CAPTURE;
        }
      else if ( strncmp(argv[2], "moni", 4)==0 )
        {
          mode = VIDEO_MODE_MONITORING;
        }
      else
        {
          /* parameter error */
          mode = (video_mode_e)atoi(argv[2]);
        }

      cap_param.format     = (video_img_format_e)atoi(argv[3]);
      cap_param.resolution = (video_img_resolution_e)atoi(argv[4]);
      cap_param.framerate  = (video_frame_rate_e)atoi(argv[5]);

      DBG_TIME_START();
      ret = camera_set_capture_param(mode, &cap_param);
      time = DBG_TIME_STOP();
      printf("camera_set_capture_param() : ret=%d, time=%d[ms]\n", ret, time);
    }
  else if ( strncmp(argv[1], "cap", 3)==0 )
    {
      video_mode_e mode;
      video_buffer_t buffer;
      video_cap_frame_info_t info;
      video_crop_t crop;

      if (( argc != 3 ) && ( argc != 5 ))
        {
          printf("The number of arguments does not match.\n");
          return -EINVAL;
        }

      up_pm_acquire_freqlock(&img_lock_hv);
      printf("CPU BaseClock:%dHz\n", cxd56_get_cpu_baseclk());

      if ( strncmp(argv[2], "cap", 3)==0 )
        {
          mode = VIDEO_MODE_CAPTURE;
        }
      else if ( strncmp(argv[2], "moni", 4)==0 )
        {
          mode = VIDEO_MODE_MONITORING;
        }
      else
        {
          /* parameter error */
          mode = (video_mode_e)atoi(argv[2]);
        }

      camera_main_thread_stop();

      memset(camera_img_buf, 0, sizeof(camera_img_buf));
#ifndef CAMERA_MAIN_JPEG_INFO
      buffer.addr = (uint32_t)camera_img_buf;
      buffer.size = IMG_BUF_MAXSIZE;
#else
      buffer.addr = (uint32_t)camera_img_buf + IMG_BUF_INFO_SIZE;
      buffer.size = IMG_BUF_MAXSIZE - IMG_BUF_INFO_SIZE;
#endif
      CISIF_INTR_TRACE_START(20);

      if ( argc == 5 )
        {
          crop.x_offset = (uint16_t)atoi(argv[3]);
          crop.y_offset = (uint16_t)atoi(argv[4]);
          DBG_TIME_START();
          ret = camera_capture_frame(mode, &buffer, &crop, &info);
          time = DBG_TIME_STOP();
        }
      else
        {
          DBG_TIME_START();
          ret = camera_capture_frame(mode, &buffer, NULL, &info);
          time = DBG_TIME_STOP();
        }
      printf("camera_capture_frame() : ret=%d, time=%d[ms]\n", ret, time);

      printf("Out data addr     : 0x%08X\n", info.out_addr);
      printf("Out data size     : %d\n", info.out_size);
      printf("Image Size        : %dx%d\n", info.h_size, info.v_size);
      printf("ISO Sens          : %s\n", iso_str[info.pict_info.iso_sens]);
      printf("Shutter Speed     : %d[us]\n", info.pict_info.shutter_speed);

      up_pm_release_freqlock(&img_lock_hv);
      printf("CPU BaseClock:%dHz\n", cxd56_get_cpu_baseclk());

      if (ret == 0)
        {
          if (info.out_addr != buffer.addr)
            {
              printf("The Buffer address is not match.\n");
              printf("Dump All Buffer Data at JPEG File.\n");
              camera_main_write_ramdisk((uint8_t *)buffer.addr,
                                        (size_t)buffer.size,
                                        &info,
                                        NULL);
              return -EINVAL;
            }

          if (info.cap_param.format == VIDEO_FORMAT_JPEG)
            {
              camera_main_write_ramdisk((uint8_t *)info.out_addr,
                                        (size_t)info.out_size,
                                        &info,
                                        NULL);
            }

          if ((info.cap_param.format == VIDEO_FORMAT_YUV) &&
              (info.cap_param.resolution == VIDEO_QVGA))
            {
#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
              camera_main_lcd_open();
              camera_main_lcd_write((void *)buffer.addr, buffer.size);
              camera_main_lcd_close();
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */
            }
        }
      else
        {
          printf("Dump Buffer Area at JPEG File.\n");
          camera_main_write_ramdisk((uint8_t *)camera_img_buf,
                                    (size_t)IMG_BUF_MAXSIZE,
                                    NULL,
                                    NULL);
        }

    }
  else if ( strncmp(argv[1], "setimgsns", 9)==0 )
    {
      video_img_sns_param_t imgsns_param;

      if ( argc != 4 )
        {
          printf("The number of arguments does not match.\n");
          return -EINVAL;
        }

      if ( strncmp(argv[2], "color", 5)==0 )
        {
          imgsns_param.id = VIDEO_PARAM_ID_COLOR;
          imgsns_param.val.color_mode = (video_color_mode_e)atoi(argv[3]);
        }
      else if ( strncmp(argv[2], "iso", 3)==0 )
        {
          imgsns_param.id = VIDEO_PARAM_ID_ISO;
          imgsns_param.val.iso = (video_iso_e)atoi(argv[3]);
        }
      else if ( strncmp(argv[2], "shutter", 7)==0 )
        {
          imgsns_param.id = VIDEO_PARAM_ID_SHUTTER;
          imgsns_param.val.shutter = (uint16_t)atoi(argv[3]);
        }
      else if ( strncmp(argv[2], "ev", 2)==0 )
        {
          imgsns_param.id = VIDEO_PARAM_ID_EV;
          imgsns_param.val.ev = (video_ev_e)atoi(argv[3]);
        }
      else if ( strncmp(argv[2], "bright", 6)==0 )
        {
          imgsns_param.id = VIDEO_PARAM_ID_BRIGHTNESS;
          imgsns_param.val.brightness = (int8_t)atoi(argv[3]);
        }
      else if ( strncmp(argv[2], "contr", 5)==0 )
        {
          imgsns_param.id = VIDEO_PARAM_ID_CONTRAST;
          imgsns_param.val.contrast = (int8_t)atoi(argv[3]);
        }
      else if ( strncmp(argv[2], "jpegq", 5)==0 )
        {
          imgsns_param.id = VIDEO_PARAM_ID_JPEG_QUALITIY;
          imgsns_param.val.jpeg_qualitiy = (uint8_t)atoi(argv[3]);
        }
      else if ( strncmp(argv[2], "gamma", 4)==0 )
        {
          imgsns_param.id = VIDEO_PARAM_ID_YGAMMA;
          imgsns_param.val.ygamma = (video_ctrl_e)atoi(argv[3]);
        }
      else if ( strncmp(argv[2], "awb", 3)==0 )
        {
          imgsns_param.id = VIDEO_PARAM_ID_AWB;
          imgsns_param.val.awb = (video_awb_e)atoi(argv[3]);
        }
      else if ( strncmp(argv[2], "photo", 5)==0 )
        {
          imgsns_param.id = VIDEO_PARAM_ID_PHOTOMETRY;
          imgsns_param.val.photometry = (video_photometry_e)atoi(argv[3]);
        }
      else
        {
          printf("The third argument is invalid.\n");
          return -EINVAL;
        }

      DBG_TIME_START();
      ret = camera_set_imgsns_param(&imgsns_param);
      time = DBG_TIME_STOP();
      printf("camera_set_imgsns_param() : ret=%d, time=%d[ms]\n", ret, time);
    }
  else if ( strncmp(argv[1], "setimgsnsall", 12)==0 )
    {
      video_img_sns_param_all_t imgsns_param_all;

      if ( argc != 12 )
        {
          printf("The number of arguments does not match.\n");
          return -EINVAL;
        }

      imgsns_param_all.color_mode    = (video_color_mode_e)atoi(argv[2]);
      imgsns_param_all.iso           = (video_iso_e)atoi(argv[3]);
      imgsns_param_all.shutter       = (uint16_t)atoi(argv[4]);
      imgsns_param_all.ev            = (video_ev_e)atoi(argv[5]);
      imgsns_param_all.brightness    = (int8_t)atoi(argv[6]);
      imgsns_param_all.contrast      = (uint8_t)atoi(argv[7]);
      imgsns_param_all.jpeg_qualitiy = (uint8_t)atoi(argv[8]);
      imgsns_param_all.ygamma        = (video_ygamma_e)atoi(argv[9]);
      imgsns_param_all.awb           = (video_awb_e)atoi(argv[10]);
      imgsns_param_all.photometry    = (video_photometry_e)atoi(argv[11]);

      DBG_TIME_START();
      ret = camera_set_imgsns_param_all(&imgsns_param_all);
      time = DBG_TIME_STOP();
      printf("camera_set_imgsns_param_all() : ret=%d, time=%d[ms]\n",
              ret, time);
    }
  else if ( strncmp(argv[1], "wr", 2)==0 )
    {
      uint16_t addr;
      uint16_t size;
      uint16_t val;

      if (argc != 5)
        {
          printf("The number of arguments does not match.\n");
          return -EINVAL;
        }

      addr = (uint16_t)strtol(argv[2], NULL, 16);
      size = (uint16_t)atoi(argv[3]);
      val  = (uint16_t)atoi(argv[4]);

      printf("addr  :0x%04X\n", addr);
      printf("size  :%d\n", size);
      printf("val   :%d\n", val);

      DBG_TIME_START();
      ret = camera_write_imgsns_register(addr, size, val);
      time = DBG_TIME_STOP();
      printf("camera_write_imgsns_register() : ret=%d, time=%d[ms]\n",
              ret, time);
    }
  else if ( strncmp(argv[1], "rr", 2)==0 )
    {
      uint16_t addr;
      uint16_t size;
      uint16_t val;

      if (argc != 4)
        {
          printf("The number of arguments does not match.\n");
          return -EINVAL;
        }

      addr = (uint16_t)strtol(argv[2], NULL, 16);
      size = (uint16_t)atoi(argv[3]);

      DBG_TIME_START();
      ret = camera_read_imgsns_register(addr, size, &val);
      time = DBG_TIME_STOP();
      printf("camera_read_imgsns_register() : ret=%d, time=%d[ms]\n",
              ret, time);
      printf("addr  :0x%04X\n", addr);
      printf("size  :%d\n", size);
      if (size == 1)
        {
          printf("val   :0x%02X\n", (uint8_t)val);
        }
      else
        {
          printf("val   :0x%04X\n", val);
        }
    }
  else if ( strncmp(argv[1], "thread", 5)==0 )
    {
      if (argc != 3)
        {
          printf("The number of arguments does not match.\n");
          return -EINVAL;
        }

      if ( strncmp(argv[2], "monistart", 9)==0 )
        {
          ret = camera_main_thread_start(VIDEO_MODE_MONITORING);
          printf("camera_main_thread_start() : ret=%d\n", ret);
        }
      else if ( strncmp(argv[2], "capstart", 8)==0 )
        {
          ret = camera_main_thread_start(VIDEO_MODE_CAPTURE);
          printf("camera_main_thread_start() : ret=%d\n", ret);
        }
      else if ( strncmp(argv[2], "stop", 4)==0 )
        {
          ret = camera_main_thread_stop();
          printf("camera_main_thread_stop() : ret=%d\n", ret);
        }
      else
        {
          printf("The second argument is invalid.\n");
          return -EINVAL;
        }
    }
  else if ( strncmp(argv[1], "halfrel", 2)==0 )
    {
      video_auto_info_t info;
      video_ctrl_e  cancel = VIDEO_DISABLE;

      if ( argc == 3 )
        {
          if ( strncmp(argv[2], "stop", 4)==0 )
            {
              cancel = VIDEO_ENABLE;
            }

        }

      CISIF_INTR_TRACE_START(20);

      DBG_TIME_START();
      ret = camera_do_halfrelease(&info, cancel);
      time = DBG_TIME_STOP();
      printf("camera_do_halfrelease() : ret=%d, time=%d[ms]\n", ret, time);

      if (ret != 0)
        {
          return ret;
        }

      if (cancel != VIDEO_ENABLE)
        {
          int idx;

          printf("### AE  ###\n");
          printf("errscl          : %d\n", info.ae.errscl);
          printf("user_aescl      : %d\n", info.ae.user_aescl);
          printf("user_gain_level : %d\n", info.ae.user_gain_level);
          printf("err_level       : %d\n", info.ae.err_level);
          printf("sht_time        : %d\n", info.ae.sht_time);

          printf("### AWB ###\n");
          printf("ratio_r         : %d\n", info.awb.ratio_r);
          printf("ratio_b         : %d\n", info.awb.ratio_b);
          printf("awb_sts         : %d\n", info.awb.awb_sts);

          printf("### OPD ###\n");
          for(idx = 0; idx < VIDEO_AE_WINDOW_MAX; idx++)
            {
              printf("[%2d]%5d", idx, (uint16_t)info.intmean[idx]);
              if ((idx % 9) == (9-1))
                {
                  printf("\n");
                }
              else
                {
                  printf(", ");
                }
              }
          printf("[FREE]%5d\n", info.intmean_free);
        }

    }
  else if ( strncmp(argv[1], "auto", 4)==0 )
    {
      video_auto_info_t info;
      int idx;

      DBG_TIME_START();
      ret = camera_id_get_auto_param(&info);
      time = DBG_TIME_STOP();
      printf("camera_id_get_auto_param() : ret=%d, time=%d[ms]\n", ret, time);

      if (ret != 0)
        {
          return ret;
        }

      printf("### AE  ###\n");
      printf("errscl          : %d\n", info.ae.errscl);
      printf("user_aescl      : %d\n", info.ae.user_aescl);
      printf("user_gain_level : %d\n", info.ae.user_gain_level);
      printf("err_level       : %d\n", info.ae.err_level);
      printf("sht_time        : %d\n", info.ae.sht_time);

      printf("### AWB ###\n");
      printf("ratio_r         : %d\n", info.awb.ratio_r);
      printf("ratio_b         : %d\n", info.awb.ratio_b);
      printf("awb_sts         : %d\n", info.awb.awb_sts);

      printf("### OPD ###\n");
      for(idx = 0; idx < VIDEO_AE_WINDOW_MAX; idx++)
        {
          printf("[%2d]%5d", idx, (uint16_t)info.intmean[idx]);
          if ((idx % 9) == (9-1))
            {
              printf("\n");
            }
          else
            {
              printf(", ");
            }
        }
      printf("[FREE]%5d\n", info.intmean_free);

    }
  else if ( strncmp(argv[1], "seq", 3)==0 )
    {
      video_buffer_t buffer;
      video_cap_frame_info_t info;
      video_auto_info_t auto_info;
      video_mode_e mode = VIDEO_MODE_CAPTURE;
      int half_ctrl = 0;
      int wait_ctrl = 0;
      int pow_onoff = 0;

      camera_change_imgsns_state(VIDEO_STATE_SLEEP);

      if ( argc >= 3 )
        {
          if ( strncmp(argv[2], "moni", 4)==0 )
            {
              mode = VIDEO_MODE_MONITORING;
            }
        }

      if ( argc >= 4 )
        {
          if ( strncmp(argv[3], "half", 4)==0 )
            {
              half_ctrl = 1;
            }

          if ( strncmp(argv[3], "wait", 4)==0 )
            {
              wait_ctrl = 1;
            }
        }

      if ( argc >= 6 )
        {
          if ( strncmp(argv[5], "pow", 3)==0 )
            {
              pow_onoff = 1;
              camera_change_imgsns_state(VIDEO_STATE_POWOFF);
            }
        }

      CISIF_INTR_TRACE_START(60);

      up_pm_acquire_freqlock(&img_lock_hv);

      if (pow_onoff)
        {
          printf("camera_change_imgsns_state(POWON) call.\n");
          DBG_TIME_START();
          camera_change_imgsns_state(VIDEO_STATE_POWON);
          time = DBG_TIME_STOP();
          printf("camera_change_imgsns_state(POWON) time=%d[ms]\n", time);
        }

      printf("camera_change_imgsns_state(ACTIVE) call.\n");
      DBG_TIME_START();
      camera_change_imgsns_state(VIDEO_STATE_ACTIVE);
      time = DBG_TIME_STOP();
      printf("camera_change_imgsns_state(ACTIVE) time=%d[ms]\n", time);

      if (half_ctrl)
        {
          printf("camera_do_halfrelease() call.\n");
          DBG_TIME_START();
          camera_do_halfrelease(&auto_info, VIDEO_DISABLE);
          time = DBG_TIME_STOP();
          printf("camera_do_halfrelease() time=%d[ms]\n", time);

          printf("### AE  ###\n");
          printf("errscl          : %d\n", auto_info.ae.errscl);
          printf("user_aescl      : %d\n", auto_info.ae.user_aescl);
          printf("user_gain_level : %d\n", auto_info.ae.user_gain_level);
          printf("err_level       : %d\n", auto_info.ae.err_level);
          printf("sht_time        : %d\n", auto_info.ae.sht_time);

          printf("### AWB ###\n");
          printf("ratio_r         : %d\n", auto_info.awb.ratio_r);
          printf("ratio_b         : %d\n", auto_info.awb.ratio_b);
          printf("awb_sts         : %d\n", auto_info.awb.awb_sts);
        }

      if (wait_ctrl)
        {
          usleep(MAX_SLEEP_TIME);
        }

#ifndef CAMERA_MAIN_JPEG_INFO
      buffer.addr = (uint32_t)camera_img_buf;
      buffer.size = IMG_BUF_MAXSIZE;
#else
      buffer.addr = (uint32_t)camera_img_buf + IMG_BUF_INFO_SIZE;
      buffer.size = IMG_BUF_MAXSIZE - IMG_BUF_INFO_SIZE;
#endif /* CAMERA_MAIN_JPEG_INFO */

      printf("camera_capture_frame() call.\n");
      DBG_TIME_START();
      camera_capture_frame(mode, &buffer, NULL, &info);
      time = DBG_TIME_STOP();
      if (mode == VIDEO_MODE_CAPTURE)
        {
          printf("camera_capture_frame(CAP) time=%d[ms]\n", time);
        }
      else
        {
          printf("camera_capture_frame(MONI) time=%d[ms]\n", time);
          if (half_ctrl)
            {
              camera_do_halfrelease(&auto_info, VIDEO_ENABLE);
            }
        }

      printf("camera_change_imgsns_state(SLEEP) call.\n");
      DBG_TIME_START();
      camera_change_imgsns_state(VIDEO_STATE_SLEEP);
      time = DBG_TIME_STOP();
      printf("camera_change_imgsns_state(SLEEP) time=%d[ms]\n", time);

      if (pow_onoff)
        {
          printf("camera_change_imgsns_state(POWOFF) call.\n");
          DBG_TIME_START();
          camera_change_imgsns_state(VIDEO_STATE_POWOFF);
          time = DBG_TIME_STOP();
          printf("camera_change_imgsns_state(POWOFF) time=%d[ms]\n", time);
        }

      if (info.out_addr != buffer.addr)
        {
          printf("The Buffer address is not match.\n");
          printf("Dump All Buffer Data at JPEG File.\n");
          camera_main_write_ramdisk((uint8_t *)buffer.addr,
                                    (size_t)buffer.size,
                                    &info,
                                    NULL);
          return -EINVAL;
        }

      printf("ISO Sens          : %s\n", iso_str[info.pict_info.iso_sens]);
      printf("Shutter Speed     : %d[us]\n", info.pict_info.shutter_speed);

      if (info.cap_param.format == VIDEO_FORMAT_JPEG)
        {
          camera_main_write_ramdisk((uint8_t *)info.out_addr,
                                    (size_t)info.out_size,
                                    &info,
                                    NULL);
        }

      if ((info.cap_param.format == VIDEO_FORMAT_YUV) &&
          (info.cap_param.resolution == VIDEO_QVGA))
        {
#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
          camera_main_lcd_open();
          camera_main_lcd_write((void *)buffer.addr, buffer.size);
          camera_main_lcd_close();
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */
        }

    }
  else if( strncmp(argv[1], "win", 3)==0 )
    {
      if ( argc != 3 )
        {
          printf("The number of arguments does not match.\n");
          return -EINVAL;
        }

      camera_main_show_centerweitedwindow(atoi(argv[2]));
    }
  else if ( strncmp(argv[1], "conti", 5)==0 )
    {
      video_conti_param_t param;
      video_conti_cap_info_t info;
      video_buffer_t buffer;
      video_conti_frame_t *f;
      uint32_t cnt;

      if (argc != 4)
        {
          printf("The number of arguments does not match.\n");
          return -EINVAL;
        }

      param.mode = VIDEO_MODE_CAPTURE;
      param.num = (uint32_t)atoi(argv[2]);
      param.interval = (uint32_t)atoi(argv[3]);

      buffer.addr = (uint32_t)camera_img_buf;
      buffer.size = IMG_BUF_MAXSIZE;

      CISIF_INTR_TRACE_START(30);

      DBG_TIME_START();
      ret = camera_id_continuous_capture(&param, &buffer, NULL, &info);
      time = DBG_TIME_STOP();
      printf("camera_id_continuous_capture() : ret=%d, time=%d[ms]\n", ret, time);

      if (ret != 0)
        {
          return ret;
        }

      printf("Capture number    : %d\n", info.capnum);
      printf("Buffer Full       : %d\n", info.buffer_full);
      printf("Image Size        : %dx%d\n", info.h_size, info.v_size);

      for(cnt = 0; cnt < info.capnum; cnt++)
        {
          f = &info.conti_frame[cnt];
          printf("Frame Number[%d]---------------------------\n", cnt);
          printf("Out data addr     : 0x%08X\n", f->out_addr);
          printf("Out data size     : %d\n", f->out_size);
          printf("ISO Sens          : %s\n", iso_str[f->pict_info.iso_sens]);
          printf("Shutter Speed     : %d[us]\n", f->pict_info.shutter_speed);
          camera_main_write_ramdisk((uint8_t *)f->out_addr,
                                    (size_t)f->out_size,
                                    NULL,
                                    NULL);
        }
    }
  else
    {
      printf("No such a command.\n");
      return -EINVAL;
    }
#endif /******************************* org *******************************/
  return 0;
}
