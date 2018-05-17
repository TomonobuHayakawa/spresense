/****************************************************************************
 * examples/camera/camera_main.c
 *
 *   Copyright 2018 Sony Semiconductor Solutions Corporation
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
static int  camera_main_init_ramdisk(void);
static int  camera_main_write_ramdisk(uint8_t *data, size_t len,
                                      video_cap_frame_info_t *info,
                                      video_crop_t *crop);
#endif /* CONFIG_SYSTEM_USBMSC */

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

#ifdef CONFIG_SYSTEM_USBMSC
static uint8_t  camera_ramdisk[RAMDISK_NSECTORS * RAMDISK_SECTORSIZE];
#endif /* CONFIG_SYSTEM_USBMSC */

static int camera_main_initialized = 0;
static uint64_t camera_main_time_start;

#define DBG_TIME_START() (camera_main_time_start = camera_main_get_mstime())
#define DBG_TIME_STOP()  (camera_main_get_mstime() - camera_main_time_start)

static uint8_t camera_main_file_count = 0;
static char camera_main_filename[32];

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
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */

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
static int camera_main_init_ramdisk(void)
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

  if(0 != mount(RAMDISK_DEVPATH, RAMDISK_MOUNTP, "vfat", 0, NULL))
    {
      printf("mount error : %d\n", errno);
      return  -1;
    }

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

  umount(RAMDISK_MOUNTP);
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

struct v_buffer          *buffers;
static unsigned int     n_buffers;
#define IMAGE_JPG_SIZE 100000
#define IMAGE_YUV_SIZE 320*240*2

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
  int v_fd;
  uint32_t time;
  uint32_t mode;
  uint32_t loop;
  uint32_t count;
  uint32_t fsize;
  enum   v4l2_buf_type       type;
  struct v4l2_format         fmt;
  struct v4l2_requestbuffers req;
  struct v4l2_buffer         buf;

  if (argc==2 && strncmp(argv[1], "cap", 3)==0)
    {
      mode = V4L2_PIX_FMT_JPEG;
    }
  else if (argc>=2 && strncmp(argv[1], "moni", 4)==0)
    {
      mode = V4L2_PIX_FMT_UYVY;
    }
  else
    {
      mode = V4L2_PIX_FMT_UYVY;
    }

  if (argc==3 && mode==V4L2_PIX_FMT_UYVY)
    {
      loop = atoi(argv[2]);
    }
  else
    {
#ifdef CONFIG_EXAMPLES_CAMERA_INFINITE
      loop = (mode==V4L2_PIX_FMT_UYVY) ? 0xffffffff : 1;
#else
      loop = 1;
#endif
    }

  /*============================ init ===========================*/
  if (!camera_main_initialized)
    {
      printf("bootcause=0x%08X\n", up_pm_get_bootcause());
      camera_main_disable_unused_board_power();
#ifdef CONFIG_SYSTEM_USBMSC
      camera_main_init_ramdisk();
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

#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
      ret = nximage_initialize();
      if (ret < 0)
        {
          printf("camera_main: Failed to get NX handle: %d\n", errno);
          return -EPERM;
        }
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */

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

  /*============================ cap / moni ====================*/
  up_pm_acquire_freqlock(&img_lock_hv);
  printf("CPU BaseClock:%dHz\n", cxd56_get_cpu_baseclk());

  /*-init_device------------------------------------------------*/
  /* Note VIDIOC_S_FMT may change width and height. */
  memset(&fmt, 0, sizeof(v4l2_format_t));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (mode == V4L2_PIX_FMT_JPEG)
    {
      count = 1;
      fsize = IMAGE_JPG_SIZE;
      fmt.fmt.pix.width       = VIDEO_HSIZE_FULLHD;
      fmt.fmt.pix.height      = VIDEO_VSIZE_FULLHD;
    }

  if (mode == V4L2_PIX_FMT_UYVY)
    {
      count = 2;
      fsize = IMAGE_YUV_SIZE;
      fmt.fmt.pix.width       = VIDEO_HSIZE_QVGA;
      fmt.fmt.pix.height      = VIDEO_VSIZE_QVGA;
    }

  fmt.fmt.pix.pixelformat = mode;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;
  ret = ioctl(v_fd, VIDIOC_S_FMT, (unsigned long)&fmt);
  if (ret)
    {
      printf("Fail set format %d\n", errno);
      return ERROR;
    }

  /*-init_device/init_userp-------------------------------------*/
  memset(&req, 0, sizeof(v4l2_requestbuffers_t));

  req.count  = count;
  req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  ret = ioctl(v_fd, VIDIOC_REQBUFS, (unsigned long)&req);
  if (ret)
    {
      printf("Does not support user pointer i/o %d\n", errno);
      return ERROR;
    }

  buffers = memalign(32, sizeof(v_buffer_t) * count);

  if (!buffers)
    {
      printf("Out of memory\n");
      return ERROR;
    }

  for (n_buffers = 0; n_buffers < count; ++n_buffers)
    {
      buffers[n_buffers].length = fsize;
      buffers[n_buffers].start  = memalign(32, fsize);
      if (!buffers[n_buffers].start)
        {
              printf("Out of memory\n");
              return ERROR;
        }

    }

  /*-start_capturing--------------------------------------------*/
  for (i = 0; i < n_buffers; i++)
    {
      memset(&buf, 0, sizeof(v4l2_buffer_t));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;
      buf.index = i;
      buf.m.userptr = (unsigned long)buffers[i].start;
      buf.length = buffers[i].length;

      ret = ioctl(v_fd, VIDIOC_QBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          return ERROR;
        }

    }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ret = ioctl(v_fd, VIDIOC_STREAMON, (unsigned long)&type);
  if (ret)
    {
      printf("Fail STREAMON %d\n", errno);
      return ERROR;
    }

  /*-mainloop/read_frame----------------------------------------*/
  while (loop-- > 0)
    {
      memset(&buf, 0, sizeof(v4l2_buffer_t));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      ret = ioctl(v_fd, VIDIOC_DQBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail DQBUF %d\n", errno);
          return ERROR;
        }

      if (mode == V4L2_PIX_FMT_JPEG)
        {
#ifdef CONFIG_SYSTEM_USBMSC
          camera_main_write_ramdisk((uint8_t *)buf.m.userptr,
                                    (size_t)buf.bytesused,
                                    NULL,
                                    NULL);
#endif /* CONFIG_SYSTEM_USBMSC */
        }
      else if (mode == V4L2_PIX_FMT_UYVY)
        {
#ifdef CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD
          /* Convert YUV color format to RGB565 */

          yuv2rgb((void *)buf.m.userptr, buf.bytesused);

          nximage_image(g_nximage.hbkgd, (void *)buf.m.userptr);
#endif /* CONFIG_EXAMPLES_CAMERA_OUTPUT_LCD */
        }

      ret = ioctl(v_fd, VIDIOC_QBUF, (unsigned long)&buf);
      if (ret)
        {
          printf("Fail QBUF %d\n", errno);
          return ERROR;
        }
    }

  up_pm_release_freqlock(&img_lock_hv);
  close(v_fd);
  for (i = 0; i < count; i++)
    {
      free(buffers[i].start);
    }

  free(buffers);
  return 0;
}
