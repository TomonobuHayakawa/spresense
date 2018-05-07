/****************************************************************************
 * drivers/video/video.c
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
#include <nuttx/config.h>

#include <sys/ioctl.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/kmalloc.h>

#include <arch/chip/cisif.h>
#include <arch/board/board.h>
#include <nuttx/video/video.h>
#include <nuttx/video/isx012.h>

#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>

#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Display time ON/OFF */
/* #define VIDEO_TIME_MEASURE */

/* Accelerate mode switching ON/OFF */
/* #define VIDEO_ISX012_FAST_MODECHG_EN */
/* #define VIDEO_ISX012_FAST_SHT_MODE_EN */

/* Continuous exposure compensation during capture ON/OFF */
/* #define VIDEO_CONTI_BRACKET */

/* At initialization, it automatically transits to ACTIVE_MODE */
#define VIDEO_INIT_ACTIVE

/* Select gamma table */
#define VIDEO_GAMMA_DEFAULT
/* #define VIDEO_GAMMA_BASIC */
/* #define VIDEO_GAMMA_HIGH_BRIGHT */

/* Test tuning code */
/* #define VIDEO_IMG_TUNING */

#define video_printf(format, ...)   _info(format, ##__VA_ARGS__)

/*------------------
 * Message queue
 *----------------*/
#define VIDEO_API_REQ_QUEUE     "video/api_req"
#define VIDEO_API_RESP_QUEUE    "video/api_resp"

#define VIDEO_TRUE              (1)
#define VIDEO_FALSE             (0)

#define VIDEO_HSIZE_QVGA        (320)
#define VIDEO_VSIZE_QVGA        (240)
#define VIDEO_HSIZE_VGA         (640)
#define VIDEO_VSIZE_VGA         (480)
#define VIDEO_HSIZE_QUADVGA     (1280)
#define VIDEO_VSIZE_QUADVGA     (960)
#define VIDEO_HSIZE_HD          (1280)
#define VIDEO_VSIZE_HD          (720)
#define VIDEO_HSIZE_5M          (2560)
#define VIDEO_VSIZE_5M          (1920)
#define VIDEO_HSIZE_3M          (2048)
#define VIDEO_VSIZE_3M          (1536)

#define VIDEO_EOI_CORRECT_MAX_SIZE  (32)

#define VIDEO_EZOOM_OFFSET_PX   (16)

#define VIDEO_INIT_REGNUM       (5)
#define VIDEO_AE_AUTO_REGNUM    (6)
#define VIDEO_AE_NOW_REGNUM     (6)
#define VIDEO_AWB_AUTO_REGNUM   (3)
#define VIDEO_AWB_NOW_REGNUM    (3)

#define VIDEO_ISX012_HALFREL_TIMEOUT      (2*1000*1000)   /* usec */
#define VIDEO_ISX012_HALFREL_WAITTIME     (20*1000)       /* usec */
#define VIDEO_ISX012_HALF_MOVE_STS_REG    (0x01B0)
#define VIDEO_ISX012_MOVE_AWB_F           (0x01)
#define VIDEO_ISX012_MOVE_AE_F            (0x02)
#define VIDEO_CISIF_TRANSEND_TIMEOUT      (1*1000)        /* msec */

/* Debug option */
#ifdef CONFIG_DEBUG_VIDEO_ERROR
#  define videoerr(format, ...)     _err(format, ##__VA_ARGS__)
#else
#  define videoerr(x...)
#endif

#ifdef CONFIG_DEBUG_VIDEO_WARN
#  define videowarn(format, ...)   _warn(format, ##__VA_ARGS__)
#else
#  define videowarn(x...)
#endif

#ifdef CONFIG_DEBUG_VIDEO_INFO
#  define videoinfo(format, ...)   _info(format, ##__VA_ARGS__)
#else
#  define videoinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef enum
{
  VIDEO_APIID_CHG_IMGSNS_STATE = 0,
  VIDEO_APIID_CAP_FRAME,
  VIDEO_APIID_SET_CAP_PARAM,
  VIDEO_APIID_SET_IMGSNS_PARAM,
  VIDEO_APIID_SET_IMGSNS_PARAM_ALL,
  VIDEO_APIID_WRITE_IMGSNS_REG,
  VIDEO_APIID_READ_IMGSNS_REG,
  VIDEO_APIID_DO_HALFRELEASE,
  VIDEO_APIID_GET_AUTO_PARAM,
  VIDEO_APIID_CONTI_CAP,
  VIDEO_APIID_MAX,
} video_api_id_e;

typedef enum
{
  VIDEO_IMGSNS_MODE_MONI = 0,
  VIDEO_IMGSNS_MODE_CAP,
  VIDEO_IMGSNS_MODE_HALFREL,
  VIDEO_IMGSNS_MODE_MAX
} video_img_sns_mode_e;

enum video_ezoom_reg_e
{
  VIDEO_EZOOM_MAG,
  VIDEO_EZOOM_OFFSET_X,
  VIDEO_EZOOM_OFFSET_Y,
  VIDEO_EZOOM_REGNUM
};

struct video_api_msg_s
{
  video_api_id_e api_id;
  int result;
  union
  {
    video_api_chg_img_sns_state_t chg_state;
    video_api_cap_frame_t cap_frame;
    video_api_set_cap_param_t cap;
    video_api_set_img_sns_param_t imgsns;
    video_api_set_img_sns_param_all_t imgsns_all;
    video_api_img_sns_reg_t w_reg;
    video_api_img_sns_reg_t r_reg;
    video_api_do_half_rel_t halfrel;
    video_api_get_auto_param_t get_auto;
    video_api_conti_cap_t conti_cap;
  } u;
};

typedef struct video_api_msg_s video_api_msg_t;

struct video_mng_s
{
  int fd;
  pid_t pid_main;
  sem_t sem_cisifsync;
  video_cap_param_t cap_param[VIDEO_MODE_MAX];
  video_img_sns_param_all_t imgsns_param_all;
  video_img_sns_state_e imgsns_state;
  video_img_sns_mode_e imgsns_mode;
  uint8_t init;
};

typedef struct video_mng_s video_mng_t;

struct video_cisif_result_s
{
  uint32_t addr;
  uint32_t size;
  uint8_t code;
  uint8_t last_frame;
  uint8_t errint;
};

typedef struct video_cisif_result_s video_cisif_result_t;

struct video_size_s
{
  uint16_t h;
  uint16_t v;
};

typedef struct video_size_s video_size_t;

struct video_sensor_reg_s
{
  uint16_t addr;
  uint16_t regsize;
};

typedef struct video_sensor_reg_s video_sensor_reg_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static void video_callback_cisif(uint8_t code, uint8_t last_frame,
                                 uint32_t size, uint32_t addr);
static void video_init_internal_param(video_mng_t *priv);
static int  video_init_image_sensor(video_mng_t *priv);
static int  video_chg_img_sns_state(video_mng_t *priv,
                                    video_api_chg_img_sns_state_t *p);
static int  video_set_capture_param(video_mng_t *priv,
                                    video_api_set_cap_param_t *p);
static int  video_capture_frame(video_mng_t *priv, video_api_cap_frame_t *p);
static int  video_set_img_sns_param(video_mng_t *priv,
                                    video_api_set_img_sns_param_t *p);
static int  video_set_img_sns_param_all(video_mng_t *priv,
                                        video_api_set_img_sns_param_all_t *p);
static int  video_write_img_sns_reg(video_mng_t *priv,
                                    video_api_img_sns_reg_t *p);
static int  video_read_img_sns_reg(video_mng_t *priv,
                                   video_api_img_sns_reg_t *p);
static int  video_set_frame_info(video_mng_t *priv,
                                 video_api_cap_frame_t *p,
                                 video_cisif_result_t *res);
static int  video_get_picture_info(video_picture_info_t *pict_info);
static int  video_set_img_sns_crop(video_cap_param_t *param,
                                   video_crop_t *crop);
static int  video_set_img_sns_crop_off(void);
static int  video_do_half_rel(video_mng_t *priv, video_api_do_half_rel_t *p);
static int  video_get_auto_param(video_mng_t *priv,
                                 video_api_get_auto_param_t *p);
static int  video_continuous_capture(video_mng_t *priv,
                                     video_api_conti_cap_t *p);

static int  video_get_int_mean(uint16_t *buf, uint16_t *free);
static uint32_t video_correct_jpeg_size(uint32_t addr, uint32_t size);
static int  video_twaisem(sem_t *sem, uint32_t timeout_ms);

#ifdef VIDEO_TIME_MEASURE
static uint64_t video_get_msec_tim(void)
{
    struct timespec tp;
    if (clock_gettime(CLOCK_REALTIME, &tp)) {
        return 0;
    }
    return (((uint64_t)tp.tv_sec) * 1000 + tp.tv_nsec / 1000000);
}
#endif /* VIDEO_TIME_MEASURE */

/****************************************************************************
 * Private Data
 ****************************************************************************/
static isx012_t             g_v_isx;
static video_cisif_result_t g_v_cisif;
static video_mng_t         *g_v_mng;

static const isx012_rate_t video_convfps[VIDEO_FRAME_RATE_MAX] =
{
  RATE_ISX012_120FPS,   /* VIDEO_120FPS */
  RATE_ISX012_60FPS,    /* VIDEO_60FPS  */
  RATE_ISX012_30FPS,    /* VIDEO_30FPS  */
  RATE_ISX012_15FPS,    /* VIDEO_15FPS  */
  RATE_ISX012_7_5FPS,   /* VIDEO_7_5FPS */
  RATE_ISX012_5FPS      /* VIDEO_5FPS   */
};

static const video_size_t video_rs2sz[VIDEO_RESOLUTION_MAX] =
{
  { VIDEO_HSIZE_QVGA,     VIDEO_VSIZE_QVGA    },
  { VIDEO_HSIZE_VGA,      VIDEO_VSIZE_VGA     },
  { VIDEO_HSIZE_QUADVGA,  VIDEO_VSIZE_QUADVGA },
  { VIDEO_HSIZE_HD,       VIDEO_VSIZE_HD      },
  { VIDEO_HSIZE_3M,       VIDEO_VSIZE_3M      },
  { VIDEO_HSIZE_5M,       VIDEO_VSIZE_5M      }
};

static const video_sensor_reg_t video_set_imgsns_regs[VIDEO_PARAM_ID_MAX] =
{
  { 0x01C5, 1 }, /* VIDEO_PARAM_ID_COLOR        : FMODE */
  { 0x02A8, 1 }, /* VIDEO_PARAM_ID_ISO          : ISO_TYPE1 */
  { 0x02A0, 2 }, /* VIDEO_PARAM_ID_SHUTTER      : SHT_PREMODE_TYPE1 */
  { 0x0180, 1 }, /* VIDEO_PARAM_ID_EV           : EVSEL */
  { 0x01C6, 1 }, /* VIDEO_PARAM_ID_BRIGHTNESS   : UIBRIGHTNESS */
  { 0x01C7, 1 }, /* VIDEO_PARAM_ID_CONTRAST     : UICONTRAST */
  { 0x00F9, 1 }, /* VIDEO_PARAM_ID_JPEG_QUALITIY: INIT_QLTY2 */
  { 0x6C93, 1 }, /* VIDEO_PARAM_ID_YGAMMA       : YGAMMA_MODE    */
  { 0x0282, 1 }, /* VIDEO_PARAM_ID_AWB          : AWB_SN1 */
  { 0x02AC, 1 }  /* VIDEO_PARAM_ID_PHOTOMETRY   : AE_SUB_SN1 */
};

static const video_sensor_reg_t video_exif_regs[] =
{
  { 0x019A, 1 },  /* iso_sens     : ISOSENS_OUT */
  { 0x019C, 2 },  /* shutter_speed: SHT_TIME_OUT_L */
  { 0x019E, 2 }   /* shutter_speed: SHT_TIME_OUT_H */
};

static const isx012_reg_t video_ezoom_regs[VIDEO_EZOOM_REGNUM] =
{
  { 0x00A2, 0x0100, 2 },  /* EZOOM_MAG  */
  { 0x00A8, 0x0000, 2 },  /* OFFSET_X   */
  { 0x00AA, 0x0000, 2 }   /* OFFSET_Y   */
};

static const isx012_reg_t video_init_regs[] =
{
#ifdef VIDEO_ISX012_FAST_MODECHG_EN
  { 0x500A, 1,  1 },  /* FAST_MODECHG_EN   */
#endif /* VIDEO_ISX012_FAST_MODECHG_EN */
#ifdef VIDEO_ISX012_FAST_SHT_MODE_EN
  { 0x500B, 1,  1 },  /* FAST_SHT_MODE_SEL  */
#endif /* VIDEO_ISX012_FAST_SHT_MODE_EN */
#ifndef VIDEO_CONTI_BRACKET
  { 0x0181, 1,  1 },  /* CAP_HALF_AE_CTRL : HAFREL=HIGHSPEED, CAP=OFF   */
#else
  { 0x0181, 7,  1 },  /* CAP_HALF_AE_CTRL : HAFREL=HIGHSPEED, CAP=Auto  */
#endif /* VIDEO_CONTI_BRACKET */
  { 0x01AE, 1,  1 },  /* HALF_AWB_CTRL  */
  { 0x5E32, 15, 1 },  /* AESPEED_FAST   */
  { 0x5E3D, 45, 1 },  /* FASTMOVE_TIMEOUT   */
  { 0x6C93, 1,  1 }   /* YGAMMA_MODE    */
};

static const uint8_t video_convawb[VIDEO_AWB_MAX] =
{
  0x20,   /* VIDEO_AWB_ATM */
  0x04,   /* VIDEO_AWB_CLEARWEATHER  */
  0x05,   /* VIDEO_AWB_SHADE  */
  0x06,   /* VIDEO_AWB_CLOUDYWEATHER  */
  0x07,   /* VIDEO_AWB_FLUORESCENTLIGHT */
  0x08,   /* VIDEO_AWB_LIGHTBULB */
};

#ifdef VIDEO_TIME_MEASURE
static uint32_t video_time_start;
static uint32_t video_time_stop;
#define DBG_MS_TIME_START()    \
  video_time_start = (uint32_t)video_get_msec_tim()
#define DBG_MS_TIME_STOP(x)     \
  video_time_stop = (uint32_t)video_get_msec_tim(); \
  video_printf("%s: time:%d[ms]\n", \
                x,(uint32_t)(video_time_stop - video_time_start))
#define DBG_TIME_START()      DBG_MS_TIME_START()
#define DBG_TIME_STOP(x)      DBG_MS_TIME_STOP(x)
#else
#define DBG_TIME_START()      
#define DBG_TIME_STOP(x)      
#endif /* VIDEO_TIME_MEASURE */
/****************************************************************************
 * Public Data
 ****************************************************************************/

static const struct file_operations g_videofops =
{
  video_open,               /* open */
  video_close,              /* close */
  0,                        /* read */
  0,                        /* write */
  0,                        /* seek */
  video_ioctl,              /* ioctl */
#ifndef CONFIG_DISABLE_POLL
  0,                        /* poll */
#endif
  0                         /* unlink */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/
void video_set_cap_shtagc(int mode, uint16_t shutter, int8_t agc)
{
#define CAP_HALF_AE_CTRL_ADDR     (0x0181)
#define CAP_SHT_ADDR              (0x0182)
#define CAP_AGC_ADDR              (0x0184)

  isx012_reg_t reg;

  if (mode)
    {
      video_printf("ON:SHUTTER=%d,ACG=%d\n", shutter, agc);

      reg.regaddr = CAP_SHT_ADDR;
      reg.regsize = 2;
      reg.regval  = shutter;
      isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);

      reg.regaddr = CAP_AGC_ADDR;
      reg.regsize = 1;
      reg.regval  = (uint16_t)agc;
      isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);

      reg.regaddr = CAP_HALF_AE_CTRL_ADDR;
      reg.regsize = 1;
      reg.regval  = 3;
      isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);
    }
  else
    {
      video_printf("OFF:SHUTTER=%d,ACG=%d\n", shutter, agc);
      reg.regaddr = CAP_HALF_AE_CTRL_ADDR;
      reg.regsize = 1;
      reg.regval  = 0x01;
      isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);
    }
}



#ifdef VIDEO_IMG_TUNING
static int video_set_spot_window(uint8_t no)
{
#define SPOT_FRM_NUM_ADDR   0x5E33
#define SPOT_SIDEWEIGHT     0x5E34
  isx012_reg_t reg;

  reg.regaddr = SPOT_FRM_NUM_ADDR;
  reg.regsize = 1;
  reg.regval  = (uint16_t)no;
  isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);

  reg.regaddr = SPOT_SIDEWEIGHT;
  reg.regsize = 1;
  reg.regval  = 75;
  isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);

  return 0;
}

static int video_show_ae_intmean(void)
{
#define INTMEAN_REG_NUM     63
#define INTMEAN_REG_H_NUM   9
#define INTMEAN_00_ADDR     (0x8A00 + 0x0088)
  isx012_reg_t reg;
  int idx;
  uint16_t  intmean_val[INTMEAN_REG_NUM];

  /* AE Window H=9 x V=7 */
  printf("###### INTMEAN_00-62 #######\n");

  reg.regsize = 2;
  for(idx = 0; idx < INTMEAN_REG_NUM; idx++)
    {
      reg.regaddr = INTMEAN_00_ADDR + (idx << 1);
      isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
      intmean_val[idx] = (uint16_t)reg.regval;
    }

  for(idx = 0; idx < INTMEAN_REG_NUM; idx++)
    {
      printf("[%2d]%5d", idx, (uint16_t)intmean_val[idx]);
      if ((idx % INTMEAN_REG_H_NUM) == (INTMEAN_REG_H_NUM-1))
        {
          printf("\n");
        }
      else
        {
          printf(", ");
        }
    }

  return 0;
}

#define EVREF_TYPE_NUM      5
static  int video_evref_type_idx = 0;
static  uint16_t evref_type_val[EVREF_TYPE_NUM] =
{
  0x2272,
  0x3697,
  0x44E4,
  0x5157,
  0x7777,
};

static int video_set_evref_type(int idx)
{
#define EVREF_TYPE1_ADDR    (0x02FC)
  isx012_reg_t reg;

  reg.regaddr = EVREF_TYPE1_ADDR;
  reg.regsize = 2;
  reg.regval  = evref_type_val[idx];

  isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);

  video_printf("EVREF_TYPE_VAL = %d\n", reg.regval);
  return 0;
}
#endif /* VIDEO_IMG_TUNING */

#ifndef VIDEO_GAMMA_DEFAULT
static int video_gamma_tble_idx = 0;
#define GAMMA_TABLE_REG_NUM       19
#define GAMMA_TABLE_NUM           4

#define GAMMA_TABLE_DEFAULT       0
#define GAMMA_TABLE_BASIC         1
#define GAMMA_TABLE_BRIGHT        2
#define GAMMA_TABLE_HIGH_BRIGHT   3

static  uint16_t gamma_reg_val[GAMMA_TABLE_NUM][GAMMA_TABLE_REG_NUM] =
{
  /* GAMMA0 Default */
  { 0, 7, 29, 47, 61, 74, 81, 90, 97, 106,
   73, 130, 173, 204, 225, 237, 246, 262, 268},
  /* Tagami-san Basic  */
  { 0, 7, 30, 48, 62, 77, 88, 100, 108, 118,
   82, 144, 188, 214, 230, 241, 250, 259, 258},
  /* Tagami-san Bright */
  { 0, 7, 50, 84, 104, 122, 135, 146, 154, 161,
   133, 183, 206, 221, 232, 241, 250, 259, 258},
  /* High Tagami-san Bright */
  { 0, 111, 128, 138, 146, 153, 159, 164, 168, 172,
   157, 183, 206, 221, 232, 241, 250, 259, 258},
};

#define VIDEO_GAMMA_REGNUM  8
static const isx012_reg_t video_gamma_regs[VIDEO_GAMMA_REGNUM] =
{
  { 0x703A, 0x0330, 2 },  /* G0_LOWGM_ON_R  */
  { 0x703C, 0x0E00, 2 },  /* G0_0CLIP_R     */
  { 0x703E, 0x0330, 2 },  /* G0_LOWGM_ON_G  */
  { 0x7040, 0x0E00, 2 },  /* G0_0CLIP_G     */
  { 0x7042, 0x0330, 2 },  /* G0_LOWGM_ON_B  */
  { 0x7044, 0x0E00, 2 },  /* G0_0CLIP_B     */
  { 0x7046, 103,    1 },  /* G0_KNOT_GAINCTRL_TH_L */
  { 0x7047, 108,    1 },  /* G0_KNOT_GAINCTRL_TH_H */
};

static const isx012_reg_t video_gamma_regs_def[VIDEO_GAMMA_REGNUM] =
{
  { 0x703A, 0x0661, 2 },  /* G0_LOWGM_ON_R  */
  { 0x703C, 0x1E0A, 2 },  /* G0_0CLIP_R     */
  { 0x703E, 0x0661, 2 },  /* G0_LOWGM_ON_G  */
  { 0x7040, 0x1E0A, 2 },  /* G0_0CLIP_G     */
  { 0x7042, 0x0661, 2 },  /* G0_LOWGM_ON_B  */
  { 0x7044, 0x1E0A, 2 },  /* G0_0CLIP_B     */
  { 0x7046, 103,    1 },  /* G0_KNOT_GAINCTRL_TH_L */
  { 0x7047, 108,    1 },  /* G0_KNOT_GAINCTRL_TH_H */
};

static int video_set_gamma_table(int idx)
{
#define GAMMA0_ADDR               0x7000
  int no;
  isx012_reg_t reg;
  isx012_reg_t *p_gamma_reg;

  /* Set GAMMA0 registers */
  p_gamma_reg = (isx012_reg_t *)video_gamma_regs_def;
  if (idx != 0)
    {
      p_gamma_reg = (isx012_reg_t *)video_gamma_regs;
    }

  for(no = 0; no < VIDEO_GAMMA_REGNUM; no++)
    {
      isx012_ioctl(IMGIOC_READREG, (unsigned long)p_gamma_reg);
      p_gamma_reg++;
    }

  reg.regsize = 2;
  printf("gamma table = %d\n", idx);
  for(no = 0; no < GAMMA_TABLE_REG_NUM; no++)
    {
      reg.regaddr = (uint16_t)(GAMMA0_ADDR + (no << 1));
      reg.regval  = (uint16_t)gamma_reg_val[idx][no];
      isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);
      video_printf("[Knot%02d]%3d\n", no, reg.regval);
    }

  return 0;
}
#endif /* VIDEO_GAMMA_DEFAULT */

static int video_twaisem(sem_t *sem, uint32_t timeout_ms)
{
  struct timespec abstime = { 0 };
  unsigned long long tmp;
  int ret;

  clock_gettime(CLOCK_REALTIME, &abstime);
  tmp = abstime.tv_nsec / 1000000;
  tmp += timeout_ms;
  abstime.tv_sec += tmp / 1000;
  abstime.tv_nsec = tmp % 1000 * 1000000 + abstime.tv_nsec % 1000000;

  ret = sem_timedwait(sem, &abstime);
  if (ret != 0)
    {
      video_printf("ERROR: sem_timedwait() error. %d\n", errno);
      ret = ETIMEDOUT;
    }

  return ret;
}

static void video_callback_cisif(
  uint8_t code,
  uint8_t last_frame,
  uint32_t size,
  uint32_t addr)
{

  g_v_cisif.size += size;

  if (code == 0)
    {
      g_v_cisif.addr = addr;
      g_v_cisif.code = code;
      g_v_cisif.last_frame = last_frame;
      sem_post(&g_v_mng->sem_cisifsync);
    }
  else
    {
      if (g_v_cisif.errint == 0)
        {
          g_v_cisif.errint = code;
        }
    }
}

static void video_callback_cisif_conti(
  uint8_t code,
  uint8_t last_frame,
  uint32_t size,
  uint32_t addr)
{
  if (code == 0)
    {
      if (g_v_cisif.errint == 0)
        {
          g_v_cisif.size = size;
          g_v_cisif.addr = addr;
        }

      g_v_cisif.code = code;
      g_v_cisif.last_frame = last_frame;
      sem_post(&g_v_mng->sem_cisifsync);
    }
  else
    {
      if (g_v_cisif.errint == 0)
        {
          g_v_cisif.errint = code;
        }
    }
}

static int video_init_image_sensor(video_mng_t *priv)
{
  int ret;
  int idx;

  DBG_TIME_START();
  isx012_open();
  DBG_TIME_STOP("open isx012 driver");

  /* Set Monitoring / Capture parameter */
  ret = isx012_ioctl(IMGIOC_SETMODEP, (unsigned long)&g_v_isx);
  if (ret < 0)
    {
      video_printf("ERROR: Failed to ioctl IMGIOC_SETMODEP. %d\n", ret);
      return -ENODEV;
    }

  /* Set Initial parameter */
  for (idx = 0; idx < sizeof(video_init_regs)/sizeof(isx012_reg_t); idx++)
    {
      ret = isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&video_init_regs[idx]);
      if (ret < 0)
        {
          video_printf("ERROR: Failed to ioctl IMGIOC_WRITEREG. %d\n", ret);
          return -ENODEV;
        }
    }
  video_printf("video_init_regs index = %d\n", idx);

#ifdef VIDEO_GAMMA_BASIC
  video_set_gamma_table(GAMMA_TABLE_BASIC);
#endif /* VIDEO_GAMMA_BASIC */
#ifdef VIDEO_GAMMA_HIGH_BRIGHT
  video_set_gamma_table(GAMMA_TABLE_HIGH_BRIGHT);
#endif /* VIDEO_GAMMA_HIGH_BRIGHT */

#ifdef VIDEO_INIT_ACTIVE
  /* Cmera Device status Sleep -> Active */
  DBG_TIME_START();
  ret = isx012_ioctl(IMGIOC_SETSTATE, STATE_ISX012_ACTIVE);
  DBG_TIME_STOP("ioctl IMGIOC_SETSTATE ACTIVE");
  if (ret < 0)
    {
      video_printf("ERROR: Failed to ioctl IMGIOC_SETSTATE. %d\n", ret);
      return -ENODEV;
    }
  /* After Camera mode -> Monitoring */

  priv->imgsns_state = VIDEO_STATE_ACTIVE;
#else
  priv->imgsns_state = VIDEO_STATE_SLEEP;
#endif /* VIDEO_INIT_NOT_ACTIVE */
  priv->imgsns_mode  = VIDEO_IMGSNS_MODE_MONI;

  return 0;
}

static void video_init_internal_param(video_mng_t *priv)
{
  g_v_isx.cap_param.format = FORMAT_ISX012_JPEG_MODE1;
  g_v_isx.cap_param.rate = RATE_ISX012_15FPS;
  g_v_isx.cap_param.yuv_hsize = video_rs2sz[VIDEO_VGA].h;
  g_v_isx.cap_param.yuv_vsize = video_rs2sz[VIDEO_VGA].v;
  g_v_isx.cap_param.jpeg_hsize = video_rs2sz[VIDEO_VGA].h;
  g_v_isx.cap_param.jpeg_vsize = video_rs2sz[VIDEO_VGA].v;

  g_v_isx.moni_param.format = FORMAT_ISX012_YUV;
  g_v_isx.moni_param.rate = RATE_ISX012_30FPS;
  g_v_isx.moni_param.yuv_hsize = video_rs2sz[VIDEO_QVGA].h;
  g_v_isx.moni_param.yuv_vsize = video_rs2sz[VIDEO_QVGA].v;
  g_v_isx.moni_param.jpeg_hsize = video_rs2sz[VIDEO_QVGA].h;
  g_v_isx.moni_param.jpeg_vsize = video_rs2sz[VIDEO_QVGA].v;

  priv->cap_param[VIDEO_MODE_CAPTURE].format = VIDEO_FORMAT_JPEG;
  priv->cap_param[VIDEO_MODE_CAPTURE].resolution = VIDEO_VGA;
  priv->cap_param[VIDEO_MODE_CAPTURE].framerate = VIDEO_15FPS;

  priv->cap_param[VIDEO_MODE_MONITORING].format = VIDEO_FORMAT_YUV;
  priv->cap_param[VIDEO_MODE_MONITORING].resolution = VIDEO_QVGA;
  priv->cap_param[VIDEO_MODE_MONITORING].framerate = VIDEO_30FPS;
}

static int video_chg_img_sns_state(video_mng_t *priv,
                                   video_api_chg_img_sns_state_t *p)
{
  int ret;
  video_img_sns_state_e next_state;

  if (priv->imgsns_state == p->state)
    {
      /* no change state */
      return 0;
    }

  next_state = p->state;
  switch (next_state)
    {
      case VIDEO_STATE_ACTIVE:
        if (priv->imgsns_state != VIDEO_STATE_SLEEP)
          {
            return -EINVAL;
          }

        DBG_TIME_START();
        ret = isx012_ioctl(IMGIOC_SETSTATE, STATE_ISX012_ACTIVE);
        DBG_TIME_STOP("ioctl IMGIOC_SETSTATE(ACTIVE)");
        break;

      case VIDEO_STATE_SLEEP:
        if (priv->imgsns_state != VIDEO_STATE_ACTIVE)
          {
            return -EINVAL;
          }

        DBG_TIME_START();
        ret = isx012_ioctl(IMGIOC_SETSTATE, STATE_ISX012_SLEEP);
        DBG_TIME_STOP("ioctl IMGIOC_SETSTATE(SLEEP)");
        break;

      case VIDEO_STATE_POWOFF:
        DBG_TIME_START();
        ret = isx012_close();
        DBG_TIME_STOP("close -> IMGSNS_POWER_OFF");
        break;

      case VIDEO_STATE_POWON:
        if (priv->imgsns_state != VIDEO_STATE_POWOFF)
          {
            return -EINVAL;
          }

        DBG_TIME_START();
        ret = video_init_image_sensor(priv);
        DBG_TIME_STOP("open -> IMGSNS_POWER_ON");
        if (ret == 0)
          {
#ifdef VIDEO_INIT_ACTIVE
            next_state = VIDEO_STATE_ACTIVE;
#else
            next_state = VIDEO_STATE_SLEEP;
#endif /* VIDEO_INIT_ACTIVE */
          }
        break;

      default:
        ret = -EINVAL;
        break;
    }

  if (ret == 0)
    {
      priv->imgsns_state = next_state;
      priv->imgsns_mode  = VIDEO_IMGSNS_MODE_MONI;
    }

  return ret;
}

static int video_set_capture_param(video_mng_t *priv,
                                   video_api_set_cap_param_t *p)
{
  int ret;

  if (priv->imgsns_mode != VIDEO_IMGSNS_MODE_MONI)
    {
      return ERROR;
    }

  if (p->mode == VIDEO_MODE_CAPTURE)
      {
        g_v_isx.cap_param.rate = video_convfps[p->param.framerate];
      }
    else
      {
        g_v_isx.moni_param.rate = video_convfps[p->param.framerate];
      }

  if (p->param.format == VIDEO_FORMAT_YUV)
    {
      if (p->mode == VIDEO_MODE_CAPTURE)
        {
          g_v_isx.cap_param.format = FORMAT_ISX012_YUV;
          g_v_isx.cap_param.yuv_hsize = video_rs2sz[p->param.resolution].h;
          g_v_isx.cap_param.yuv_vsize = video_rs2sz[p->param.resolution].v;
        }
      else
        {
          g_v_isx.moni_param.format = FORMAT_ISX012_YUV;
          g_v_isx.moni_param.yuv_hsize = video_rs2sz[p->param.resolution].h;
          g_v_isx.moni_param.yuv_vsize = video_rs2sz[p->param.resolution].v;
        }
    }
  else
    {
      if (p->mode == VIDEO_MODE_CAPTURE)
        {
          g_v_isx.cap_param.format = FORMAT_ISX012_JPEG_MODE1;
          g_v_isx.cap_param.jpeg_hsize = video_rs2sz[p->param.resolution].h;
          g_v_isx.cap_param.jpeg_vsize = video_rs2sz[p->param.resolution].v;
        }
      else
        {
          g_v_isx.moni_param.format = FORMAT_ISX012_JPEG_MODE1;
          g_v_isx.moni_param.jpeg_hsize = video_rs2sz[p->param.resolution].h;
          g_v_isx.moni_param.jpeg_vsize = video_rs2sz[p->param.resolution].v;
        }
    }

  DBG_TIME_START();
  ret = isx012_ioctl(IMGIOC_SETMODEP, (unsigned long)&g_v_isx);
  DBG_TIME_STOP("ioctl IMGIOC_SETMODEP");
  if (ret < 0)
    {
      video_printf("ERROR: ioctl IMGIOC_SETMODEP %d.\n", ret);
      return ret;
    }

  memcpy(&priv->cap_param[p->mode], &p->param, sizeof(video_cap_param_t));

  if (priv->imgsns_state == VIDEO_STATE_SLEEP)
    {
      return ret;
    }

  DBG_TIME_START();
  ret = isx012_ioctl(IMGIOC_MONIREF, 0);
  DBG_TIME_STOP("ioctl IMGIOC_MONIREF");
  if (ret < 0)
    {
      video_printf("ERROR: ioctl IMGIOC_MONIREF %d.\n", ret);
    }

  return ret;
}

static int video_set_img_sns_crop_off(void)
{
  int idx;
  int ret;

  for (idx = 0; idx < VIDEO_EZOOM_REGNUM; idx++)
    {
      ret = isx012_ioctl(IMGIOC_WRITEREG,(unsigned long)&video_ezoom_regs[idx]);
      if (ret < 0)
        {
          break;
        }
    }

  return ret;
}

static int video_set_img_sns_crop(video_cap_param_t *param, video_crop_t *crop)
{
  int idx;
  int ret;
  uint16_t px_offset;
  uint16_t ezoom_val[VIDEO_EZOOM_REGNUM];
  uint16_t ezoom_mag_subsmpl[VIDEO_RESOLUTION_MAX] =
  {
    1024,   /* QVGA     : x4  */
     512,   /* VGA      : x2  */
     256,   /* Quad-VGA : x1  */
     256,   /* HD       : x1  */
       0,   /* 3M       : invalid  */
       0,   /* 5M       : invalid  */
  };
  uint16_t ezoom_mag_fllpx[VIDEO_RESOLUTION_MAX] =
  {
    2048,   /* QVGA     : x8  */
    1024,   /* VGA      : x4  */
     512,   /* Quad-VGA : x2  */
     512,   /* HD       : x2  */
     256,   /* 3M       : x1  */
     256    /* 5M       : x1  */
  };
  isx012_reg_t reg;

  video_printf("video_set_img_sns_crop :framerate=%d, resolution=%d\n",
                param->framerate, param->resolution);

  if ((param->framerate >= VIDEO_15FPS) && (param->framerate <= VIDEO_5FPS))
    {
      ezoom_val[VIDEO_EZOOM_MAG] = ezoom_mag_fllpx[param->resolution];
      px_offset = VIDEO_EZOOM_OFFSET_PX;
    }
  else if (param->framerate == VIDEO_30FPS)
    {
      if ((param->resolution == VIDEO_5M) || (param->resolution == VIDEO_3M))
        {
          return 0;
        }
      ezoom_val[VIDEO_EZOOM_MAG] = ezoom_mag_subsmpl[param->resolution];
      px_offset = (VIDEO_EZOOM_OFFSET_PX << 1);
    }
  else
    {
      return 0;
    }

  ezoom_val[VIDEO_EZOOM_OFFSET_X] = crop->x_offset * px_offset;
  ezoom_val[VIDEO_EZOOM_OFFSET_Y] = -(crop->y_offset) * px_offset;

  video_printf("ezoom_val: mag=%d, x_offset=%d, y_offset=%d\n",
                ezoom_val[0], (int16_t)ezoom_val[1], (int16_t)ezoom_val[2]);

  for (idx = 0; idx < VIDEO_EZOOM_REGNUM; idx++)
    {
      reg.regaddr = video_ezoom_regs[idx].regaddr;
      reg.regval  = ezoom_val[idx];
      reg.regsize = video_ezoom_regs[idx].regsize;

      ret = isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);
      if (ret < 0)
        {
          break;
        }
    }

  return ret;
}

#if 0
/* 1 Frame Capture -> Auto Monitoring mode Setting */
static void  video_SetImgSnsCaptureRegister(void)
{
  int idx;
  int ret;
  const isx012_reg_t cap_regs[] =
    {
      { 0x00B5, 0x03, 1 },  /* CAP_CARRY_OVER_F */
      { 0x00B6, 0x01, 1 },  /* CAPNUM */
    };

  for (idx = 0; idx < sizeof(cap_regs)/sizeof(isx012_reg_t); idx++)
    {
      ret = isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&cap_regs[idx]);
      if (ret < 0)
        {
          break;
        }
    }
}
#endif

static int video_capture_frame(video_mng_t *priv, video_api_cap_frame_t *p)
{
  int ret = 0;
  cisif_param_t    cis;
  video_img_format_e format;

  if ((priv->imgsns_state == VIDEO_STATE_SLEEP) ||
      (priv->imgsns_mode == VIDEO_IMGSNS_MODE_CAP))
    {
      return -EBUSY;
    }

  video_set_img_sns_crop_off();

  if (p->crop_ctrl == VIDEO_ENABLE)
    {
      DBG_TIME_START();
      video_set_img_sns_crop(&priv->cap_param[p->mode], &p->crop);
      DBG_TIME_STOP("video_set_img_sns_crop");
    }

#ifdef VIDEO_IMG_TUNING
  video_set_gamma_table(video_gamma_tble_idx);
#endif /* VIDEO_IMG_TUNING */

  if (p->mode == VIDEO_MODE_CAPTURE)
    {
      video_printf("ioctl IMGIOC_SETMODE(CAPTURE) call.\n");
      DBG_TIME_START();
      ret = isx012_ioctl(IMGIOC_SETMODE, MODE_ISX012_CAPTURE);
      DBG_TIME_STOP("ioctl IMGIOC_SETMODE(CAPTURE)");
      if (ret < 0)
        {
          video_printf("ERROR: ioctl IMGIOC_SETMODE(CAPTURE):%d\n", ret);
          return ret;
        }
      priv->imgsns_mode = VIDEO_IMGSNS_MODE_CAP;
    }

  memset(&g_v_cisif, 0, sizeof(g_v_cisif));
  format = priv->cap_param[p->mode].format;
  if (format == VIDEO_FORMAT_YUV)
    {
      cis.yuv_param.comp_func = video_callback_cisif;
    }
  else
    {
      cis.jpg_param.comp_func = video_callback_cisif;
    }

  cis.sarea.strg_addr = (uint8_t *)p->buffer.addr;
  cis.sarea.strg_size = p->buffer.size;
  cis.sarea.capnum    = 1;
  isx012_ioctl(IMGIOC_SETCISIF, (unsigned long)&cis);

  if (ret != OK)
    {
      video_printf("ERROR: cxd56_cisifcaptureframe() %d\n", ret);
      g_v_cisif.errint = ret;
      goto exit;
    }

  ret = video_twaisem(&priv->sem_cisifsync, VIDEO_CISIF_TRANSEND_TIMEOUT);
  if (ret != 0)
    {
      g_v_cisif.errint = ret;
    }

  DBG_TIME_STOP("cxd56_cisifcaptureframe() -> trans end.");

  if ((g_v_cisif.code != 0) || (g_v_cisif.errint != 0))
    {
      video_printf("ERROR :cisif err = %d\n", g_v_cisif.errint);
    }
  else
    {
      video_set_frame_info(priv, p, &g_v_cisif);
      video_get_picture_info(&p->info.pict_info);
    }

exit:
  if (p->mode == VIDEO_MODE_CAPTURE)
    {
      video_printf("ioctl IMGIOC_SETMODE(MONITORING) call.\n");
      DBG_TIME_START();
      ret = isx012_ioctl(IMGIOC_SETMODE, MODE_ISX012_MONITORING);
      DBG_TIME_STOP("ioctl IMGIOC_SETMODE(MONITORING)");
      if (ret < 0)
        {
          video_printf("ERROR: ioctl IMGIOC_SETMODE(MONITORING):%d\n", ret);
        }
      else
        {
          priv->imgsns_mode = VIDEO_IMGSNS_MODE_MONI;
        }
    }

  if (g_v_cisif.errint != 0)
    {
      ret = -g_v_cisif.errint;
    }

  return ret;
}

static uint32_t video_correct_jpeg_size(uint32_t addr, uint32_t size)
{
  uint8_t *jpg;
  uint32_t eoi_offset;

  jpg = (uint8_t *)(addr + size - 1);
  for (eoi_offset=0; eoi_offset < VIDEO_EOI_CORRECT_MAX_SIZE; eoi_offset++)
    {
      if ((*jpg == 0xD9) && (*(jpg-1) == 0xFF))
        {
          break;
        }
      jpg--;
    }

  return (size - eoi_offset);
}

static int video_set_frame_info(video_mng_t *priv, 
                                video_api_cap_frame_t *p,
                                video_cisif_result_t *res)
{
  p->info.mode = p->mode;
  memcpy(&p->info.cap_param,
         &priv->cap_param[p->mode],
         sizeof(video_cap_param_t));

  p->info.out_addr = res->addr;
  p->info.out_size = res->size;

  if (priv->cap_param[p->mode].format == VIDEO_FORMAT_YUV)
    {
      if (p->mode == VIDEO_MODE_CAPTURE)
        {
          p->info.h_size = g_v_isx.cap_param.yuv_hsize;
          p->info.v_size = g_v_isx.cap_param.yuv_vsize;
        }
      else
        {
          p->info.h_size = g_v_isx.moni_param.yuv_hsize;
          p->info.v_size = g_v_isx.moni_param.yuv_vsize;
        }
    }
  else
    {
      if (res->errint == 0)
        {
          p->info.out_size = video_correct_jpeg_size(res->addr, res->size);
        }

      if (p->mode == VIDEO_MODE_CAPTURE)
        {
          p->info.h_size = g_v_isx.cap_param.jpeg_hsize;
          p->info.v_size = g_v_isx.cap_param.jpeg_vsize;
        }
      else
        {
          p->info.h_size = g_v_isx.moni_param.jpeg_hsize;
          p->info.v_size = g_v_isx.moni_param.jpeg_vsize;
        }
    }

  return 0;
}

static int video_get_picture_info(video_picture_info_t *pict_info)
{
  uint32_t shutter_speed = 0;
  video_iso_e iso_sens = 0;
  isx012_reg_t reg;
  int ret;

  /* ISO */
  reg.regaddr = video_exif_regs[0].addr;
  reg.regsize = video_exif_regs[0].regsize;
  ret = isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
  if (ret < 0)
    {
      return ret;
    }
  iso_sens = (video_iso_e)reg.regval;

  /* Shutter low */
  reg.regaddr = video_exif_regs[1].addr;
  reg.regsize = video_exif_regs[1].regsize;
  ret = isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
  if (ret < 0)
    {
      return ret;
    }
  shutter_speed |= (uint32_t)(reg.regval & 0x0000FFFF);

  /* Shutter high */
  reg.regaddr = video_exif_regs[2].addr;
  reg.regsize = video_exif_regs[2].regsize;
  ret = isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
  if (ret < 0)
    {
      return ret;
    }
  shutter_speed |= (uint32_t)((reg.regval << 16) & 0xFFFF0000);

  pict_info->iso_sens      = iso_sens;
  pict_info->shutter_speed = shutter_speed;

  return 0;
}

static int video_set_img_sns_param(video_mng_t *priv,
                                   video_api_set_img_sns_param_t *p)
{
  isx012_reg_t reg;
  int ret;

  if (priv->imgsns_mode != VIDEO_IMGSNS_MODE_MONI)
    {
      return -EBUSY;
    }

  if (p->param.id >= VIDEO_PARAM_ID_MAX)
    {
      return -EINVAL;
    }

  switch (p->param.id)
  {
    case VIDEO_PARAM_ID_COLOR:
      reg.regval = (uint16_t)p->param.val.color_mode;
      break;

    case VIDEO_PARAM_ID_ISO:
      reg.regval = (uint16_t)p->param.val.iso;
      break;

    case VIDEO_PARAM_ID_SHUTTER:
      reg.regval = p->param.val.shutter;
      break;

    case VIDEO_PARAM_ID_EV:
      if (p->param.val.ev >= VIDEO_EV_MAX)
        {
          return -EINVAL;
        }
      reg.regval = (uint16_t)p->param.val.ev;
      break;

    case VIDEO_PARAM_ID_BRIGHTNESS:
      reg.regval = (uint16_t)p->param.val.brightness;
      break;

    case VIDEO_PARAM_ID_CONTRAST:
      reg.regval = (uint16_t)p->param.val.contrast;
      break;

    case VIDEO_PARAM_ID_JPEG_QUALITIY:
      reg.regval = (uint16_t)p->param.val.jpeg_qualitiy;
      break;

    case VIDEO_PARAM_ID_YGAMMA:
      if (p->param.val.ygamma >= VIDEO_YGAMMA_MAX)
        {
          return -EINVAL;
        }
      reg.regval = (uint16_t)p->param.val.ygamma;
      break;

    case VIDEO_PARAM_ID_AWB:
      if (p->param.val.awb >= VIDEO_AWB_MAX)
        {
          return -EINVAL;
        }
      reg.regval = (uint16_t)video_convawb[p->param.val.awb];
      break;

    case VIDEO_PARAM_ID_PHOTOMETRY:
      if (p->param.val.photometry >= VIDEO_PHOTOMETRY_MAX)
        {
          return -EINVAL;
        }
      reg.regval = (uint16_t)p->param.val.photometry;
      break;

    default:
      return -EINVAL;
  }

  reg.regaddr = video_set_imgsns_regs[p->param.id].addr;
  reg.regsize = video_set_imgsns_regs[p->param.id].regsize;
  ret = isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->imgsns_state == VIDEO_STATE_SLEEP)
    {
      return ret;
    }

  DBG_TIME_START();
  ret = isx012_ioctl(IMGIOC_MONIREF, 0);
  DBG_TIME_STOP("ioctl IMGIOC_MONIREF");
  if (ret < 0)
    {
      video_printf("ERROR: ioctl IMGIOC_MONIREF %d.\n", ret);
    }

  return ret;
}

static int video_set_img_sns_param_all(video_mng_t *priv,
                                       video_api_set_img_sns_param_all_t *p)
{
  isx012_reg_t reg;
  video_img_sns_param_id_e idx;
  int ret;

  if (priv->imgsns_mode != VIDEO_IMGSNS_MODE_MONI)
    {
      return -EBUSY;
    }

  for (idx = VIDEO_PARAM_ID_COLOR; idx < VIDEO_PARAM_ID_MAX; idx++)
    {
      switch (idx)
        {
          case VIDEO_PARAM_ID_COLOR:
            reg.regval = (uint16_t)p->param.color_mode;
            break;

          case VIDEO_PARAM_ID_ISO:
            reg.regval = (uint16_t)p->param.iso;
            break;

          case VIDEO_PARAM_ID_SHUTTER:
            reg.regval = p->param.shutter;
            break;

          case VIDEO_PARAM_ID_EV:
            if (p->param.ev >= VIDEO_EV_MAX)
              {
                return -EINVAL;
              }
            reg.regval = (uint16_t)p->param.ev;
            break;

          case VIDEO_PARAM_ID_BRIGHTNESS:
            reg.regval = (uint16_t)p->param.brightness;
            break;

          case VIDEO_PARAM_ID_CONTRAST:
            reg.regval = (uint16_t)p->param.contrast;
            break;

          case VIDEO_PARAM_ID_JPEG_QUALITIY:
            reg.regval = (uint16_t)p->param.jpeg_qualitiy;
            break;

          case VIDEO_PARAM_ID_YGAMMA:
            if (p->param.ygamma >= VIDEO_YGAMMA_MAX)
              {
                return -EINVAL;
              }
            reg.regval = (uint16_t)p->param.ygamma;
            break;

          case VIDEO_PARAM_ID_AWB:
            if (p->param.awb >= VIDEO_AWB_MAX)
              {
                return -EINVAL;
              }
            reg.regval = (uint16_t)video_convawb[p->param.awb];
            break;

          case VIDEO_PARAM_ID_PHOTOMETRY:
            if (p->param.photometry >= VIDEO_PHOTOMETRY_MAX)
              {
                return -EINVAL;
              }
            reg.regval = (uint16_t)p->param.photometry;
            break;

          default:
            break;
        }

      reg.regaddr = video_set_imgsns_regs[idx].addr;
      reg.regsize = video_set_imgsns_regs[idx].regsize;
      ret = isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);
      if (ret < 0)
        {
          break;
        }
    }

  if (priv->imgsns_state == VIDEO_STATE_SLEEP)
    {
      return ret;
    }

  DBG_TIME_START();
  ret = isx012_ioctl(IMGIOC_MONIREF, 0);
  DBG_TIME_STOP("ioctl IMGIOC_MONIREF");
  if (ret < 0)
    {
      video_printf("ERROR: ioctl IMGIOC_MONIREF %d.\n", ret);
    }

  return ret;
}

static int video_write_img_sns_reg(video_mng_t *priv,
                                   video_api_img_sns_reg_t *p)
{
  isx012_reg_t reg;
  int ret;

  reg.regaddr = p->addr;
  reg.regval  = p->val;
  reg.regsize = p->regsize;

  DBG_TIME_START();
  ret = isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);
  DBG_TIME_STOP("ioctl IMGIOC_WRITEREG");

  return ret;
}

static int video_read_img_sns_reg(video_mng_t *priv,
                                  video_api_img_sns_reg_t *p)
{
  isx012_reg_t reg;
  int ret;

  reg.regaddr = p->addr;
  reg.regsize = p->regsize;
  reg.regval  = 0;

  DBG_TIME_START();
  ret = isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
  DBG_TIME_STOP("ioctl IMGIOC_READREG");
  if (ret < 0)
    {
      return ret;
    }

  p->val = reg.regval;

  return 0;
}

static int video_WaitAutoProcessEnd(void)
{
  isx012_reg_t reg;
  uint32_t time = 0;
  int ret;
  int ae_fin = 0;
  reg.regaddr = VIDEO_ISX012_HALF_MOVE_STS_REG;
  reg.regsize = 1;
  reg.regval  = 0;

  while(time < VIDEO_ISX012_HALFREL_TIMEOUT)
    {
      ret = isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
      if (ret < 0)
        {
          return ret;
        }

      reg.regval &= (VIDEO_ISX012_MOVE_AE_F | VIDEO_ISX012_MOVE_AWB_F);
      if (!(reg.regval & VIDEO_ISX012_MOVE_AE_F) && (ae_fin == 0))
        {
          video_printf("AE  end(%02X).\n", reg.regval);
          ae_fin = 1;
        }

      if (!(reg.regval & VIDEO_ISX012_MOVE_AWB_F))
        {
          video_printf("AWB end(%02X).\n", reg.regval);
        }

      if (reg.regval == 0)
        {
          return 0;
        }

      usleep(VIDEO_ISX012_HALFREL_WAITTIME);
      time += VIDEO_ISX012_HALFREL_WAITTIME;
    }

  return -EBUSY;
}

static int  video_get_int_mean(uint16_t *buf, uint16_t *free)
{
#define INTMEAN_00_ADDR     (0x8A88)
#define INTMEAN_FREE_ADDR   (0x8B06)
  isx012_reg_t reg;
  int idx;

  reg.regsize = 2;
  for(idx = 0; idx < VIDEO_AE_WINDOW_MAX; idx++)
    {
      reg.regaddr = INTMEAN_00_ADDR + (idx << 1);
      isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
      *buf = (uint16_t)reg.regval;
      buf++;
    }

  reg.regaddr = INTMEAN_FREE_ADDR;
  isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
  *free = (uint16_t)reg.regval;

  return 0;
}

static int  video_do_half_rel(video_mng_t *priv, video_api_do_half_rel_t *p)
{
  isx012_reg_t reg;
  uint16_t getval[VIDEO_AE_NOW_REGNUM];
  int idx;
  int ret;

  const video_sensor_reg_t video_ae_now_regs[] =
  {
    { 0x01CC, 2 },  /* ERRSCL_NOW     */
    { 0x01D0, 2 },  /* USER_AESCL_NOW */
    { 0x01A5, 1 },  /* USER_GAIN_LEVEL_NOW */
    { 0x01A9, 1 },  /* ERRLEVEL_NOW   */
    { 0x019C, 2 },  /* SHT_TIME_OUT_L */
    { 0x019E, 2 },  /* SHT_TIME_OUT_H */
  };

  const video_sensor_reg_t video_awb_now_regs[] =
  {
    { 0x01B6, 2 },  /* RATIO_R_NOW */
    { 0x01B8, 2 },  /* RATIO_B_NOW */
    { 0x01BB, 1 }   /* AWB_STS_NOW */
  };

  if (priv->imgsns_state == VIDEO_STATE_SLEEP)
    {
      return -EBUSY;
    }

  if (p->cancel == VIDEO_ENABLE)
    {
      if (priv->imgsns_mode == VIDEO_IMGSNS_MODE_MONI)
        {
          return 0;
        }

      DBG_TIME_START();
      ret = isx012_ioctl(IMGIOC_SETMODE, MODE_ISX012_MONITORING);
      DBG_TIME_STOP("ioctl IMGIOC_SETMODE(MONITORING)");
      if (ret < 0)
        {
          video_printf("ERROR: ioctl IMGIOC_SETMODE(MONITORING):%d\n", ret);
          return ret;
        }
      priv->imgsns_mode  = VIDEO_IMGSNS_MODE_MONI;
    }
  else
    {
      if (priv->imgsns_mode != VIDEO_IMGSNS_MODE_MONI)
        {
          return -EBUSY;
        }

      video_printf("ioctl IMGIOC_SETMODE(HALFRELEASE) call.\n");

#ifdef VIDEO_IMG_TUNING
      video_set_evref_type(video_evref_type_idx);
      video_evref_type_idx++;
      if (video_evref_type_idx == EVREF_TYPE_NUM)
        {
          video_evref_type_idx = 0;
          video_gamma_tble_idx++;
          if (video_gamma_tble_idx == GAMMA_TABLE_NUM)
            {
              video_gamma_tble_idx = 0;
            }
        }
#endif /* VIDEO_IMG_TUNING */

      DBG_TIME_START();
      ret = isx012_ioctl(IMGIOC_SETMODE, MODE_ISX012_HALFRELEASE);
      DBG_TIME_STOP("ioctl IMGIOC_SETMODE(HALFRELEASE)");
      if (ret < 0)
        {
          video_printf("ERROR: ioctl IMGIOC_SETMODE(HALFRELEASE):%d\n", ret);
          return ret;
        }

      /* wait for AUTO process end. */
      DBG_TIME_START();
      ret = video_WaitAutoProcessEnd();
      DBG_TIME_STOP("video_WaitAutoProcessEnd()");
      if (ret != 0)
        {
          video_printf("ERROR: video_WaitAutoProcessEnd() timeout:%d\n", ret);
          isx012_ioctl(IMGIOC_SETMODE, MODE_ISX012_MONITORING);
          return ret;
      }

      for(idx = 0; idx < VIDEO_AE_NOW_REGNUM; idx++)
        {
          reg.regaddr = video_ae_now_regs[idx].addr;
          reg.regsize = video_ae_now_regs[idx].regsize;
          ret = isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
          if (ret < 0)
            {
              return ret;
            }
          getval[idx] = reg.regval;
        }

      p->info.ae.errscl = (int16_t)getval[0];
      p->info.ae.user_aescl = (int16_t)getval[1];
      p->info.ae.user_gain_level = (int8_t)getval[2];
      p->info.ae.err_level = (int8_t)getval[3];
      p->info.ae.sht_time  = (uint32_t)getval[4];
      p->info.ae.sht_time |= (uint32_t)((getval[5] << 16) & 0xFFFF0000);

      for(idx = 0; idx < VIDEO_AWB_NOW_REGNUM; idx++)
        {
          reg.regaddr = video_awb_now_regs[idx].addr;
          reg.regsize = video_awb_now_regs[idx].regsize;
          ret = isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
          if (ret < 0)
            {
              return ret;
            }
          getval[idx] = reg.regval;
        }

      p->info.awb.ratio_r = getval[0];
      p->info.awb.ratio_b = getval[1];
      p->info.awb.awb_sts = (uint8_t)getval[2];

      video_get_int_mean(&p->info.intmean[0], &p->info.intmean_free);

      priv->imgsns_mode  = VIDEO_IMGSNS_MODE_HALFREL;

    }

  return 0;
}

static int  video_get_auto_param(video_mng_t *priv,
                                 video_api_get_auto_param_t *p)
{
  isx012_reg_t reg;
  uint16_t getval[VIDEO_AE_AUTO_REGNUM];
  int idx;
  int ret;

  const video_sensor_reg_t video_ae_auto_regs[] =
  {
    { 0x01CA, 2 },  /* ERRSCL_AUTO     */
    { 0x01CE, 2 },  /* USER_AESCL_AUTO */
    { 0x01A4, 1 },  /* USER_GAIN_LEVEL_AUTO */
    { 0x01A8, 1 },  /* ERRLEVEL_AUTO   */
    { 0x01A0, 2 },  /* SHT_TIME_AUTO_L */
    { 0x01A2, 2 },  /* SHT_TIME_AUTO_H */
  };

  const video_sensor_reg_t video_awb_auto_regs[] =
  {
    { 0x01B2, 2 },  /* RATIO_R_AUTO */
    { 0x01B4, 2 },  /* RATIO_B_AUTO */
    { 0x01BA, 1 }   /* AWB_STS_AUTO */
  };

  if ((priv->imgsns_state == VIDEO_STATE_SLEEP) ||
      (priv->imgsns_mode  != VIDEO_IMGSNS_MODE_MONI))
    {
      return -EBUSY;
    }

  for(idx = 0; idx < VIDEO_AE_AUTO_REGNUM; idx++)
    {
      reg.regaddr = video_ae_auto_regs[idx].addr;
      reg.regsize = video_ae_auto_regs[idx].regsize;
      ret = isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
      if (ret < 0)
        {
          return ret;
        }
      getval[idx] = reg.regval;
    }

  p->info.ae.errscl = (int16_t)getval[0];
  p->info.ae.user_aescl = (int16_t)getval[1];
  p->info.ae.user_gain_level = (int8_t)getval[2];
  p->info.ae.err_level = (int8_t)getval[3];
  p->info.ae.sht_time  = (uint32_t)getval[4];
  p->info.ae.sht_time |= (uint32_t)((getval[5] << 16) & 0xFFFF0000);

  for(idx = 0; idx < VIDEO_AWB_AUTO_REGNUM; idx++)
    {
      reg.regaddr = video_awb_auto_regs[idx].addr;
      reg.regsize = video_awb_auto_regs[idx].regsize;
      ret = isx012_ioctl(IMGIOC_READREG, (unsigned long)&reg);
      if (ret < 0)
        {
          return ret;
        }
      getval[idx] = reg.regval;
    }

  p->info.awb.ratio_r = getval[0];
  p->info.awb.ratio_b = getval[1];
  p->info.awb.awb_sts = (uint8_t)getval[2];

  video_get_int_mean(&p->info.intmean[0], &p->info.intmean_free);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
static int video_continuous_capture(video_mng_t *priv, video_api_conti_cap_t *p)
{
  cisif_param_t    cis;
  uint32_t conti_capnum = 0;
  uint32_t correct_size = 0;
  int conti_end = VIDEO_FALSE;
  int ret;
#ifdef VIDEO_CONTI_BRACKET
  isx012_reg_t reg;
#endif /* VIDEO_CONTI_BRACKET */

  if ((priv->imgsns_state == VIDEO_STATE_SLEEP) ||
      (priv->imgsns_mode == VIDEO_IMGSNS_MODE_CAP))
    {
      return -EBUSY;
    }

  if (priv->cap_param[p->mode].format != VIDEO_FORMAT_JPEG)
    {
      return -EINVAL;
    }

  video_set_img_sns_crop_off();

  if (p->crop_ctrl == VIDEO_ENABLE)
    {
      DBG_TIME_START();
      video_set_img_sns_crop(&priv->cap_param[p->mode], &p->crop);
      DBG_TIME_STOP("video_set_img_sns_crop");
    }

#ifdef VIDEO_CONTI_BRACKET
  /* CAP_HALF_AE_CTRL : HAFREL=HIGHSPEED, CAP=AUTO  */
  reg.regval  = 0x7;
  reg.regaddr = 0x0181;
  reg.regsize = 1;
  isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);
#endif /* VIDEO_CONTI_BRACKET */

  video_printf("ioctl IMGIOC_SETMODE(CAPTURE) call.\n");
  DBG_TIME_START();
  ret = isx012_ioctl(IMGIOC_SETMODE, MODE_ISX012_CAPTURE);
  DBG_TIME_STOP("ioctl IMGIOC_SETMODE(CAPTURE)");
  if (ret < 0)
    {
      video_printf("ERROR: ioctl IMGIOC_SETMODE(CAPTURE):%d\n", ret);
      return ret;
    }

  priv->imgsns_mode = VIDEO_IMGSNS_MODE_CAP;
  memset(&g_v_cisif, 0, sizeof(g_v_cisif));
  cis.jpg_param.comp_func = video_callback_cisif_conti;
  cis.sarea.strg_addr = (uint8_t *)p->buffer.addr;
  cis.sarea.strg_size = p->buffer.size;
  cis.sarea.capnum    = p->capnum;
  cis.sarea.interval  = p->interval;
  isx012_ioctl(IMGIOC_SETCISIF, (unsigned long)&cis);

  while (!conti_end)
    {
      ret = video_twaisem(&priv->sem_cisifsync,
                          VIDEO_CISIF_TRANSEND_TIMEOUT);
      if (ret != 0)
        {
          g_v_cisif.errint = ret;
        }

      DBG_TIME_STOP("cxd56_cisifcontinuouscapture() -> trans end.");
      if ((g_v_cisif.code != 0) || (g_v_cisif.errint != 0))
        {
          video_printf("ERROR :cisif err = %d\n", g_v_cisif.errint);
          goto exit;
      }

      video_get_picture_info(&p->info.conti_frame[conti_capnum].pict_info);

      video_printf("Out Addr : %08X\n",g_v_cisif.addr);
      video_printf("Out Size : %08X\n",g_v_cisif.size);

      correct_size = video_correct_jpeg_size(g_v_cisif.addr,
                                           g_v_cisif.size);
      p->info.conti_frame[conti_capnum].out_addr = g_v_cisif.addr;
      p->info.conti_frame[conti_capnum].out_size = correct_size;

      conti_capnum++;

#ifdef VIDEO_CONTI_BRACKET
      reg.regval = (uint16_t)VIDEO_EV_P2;
      reg.regaddr = video_set_imgsns_regs[VIDEO_PARAM_ID_EV].addr;
      reg.regsize = video_set_imgsns_regs[VIDEO_PARAM_ID_EV].regsize;
      video_printf("IMGIOC_WRITEREG : EV:\n", reg.regval);
      isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);

      reg.regval = (uint16_t)127;
      reg.regaddr = video_set_imgsns_regs[VIDEO_PARAM_ID_BRIGHTNESS].addr;
      reg.regsize = video_set_imgsns_regs[VIDEO_PARAM_ID_BRIGHTNESS].regsize;
      video_printf("IMGIOC_WRITEREG : BRIGHTNESS:\n", reg.regval);
      isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);
#endif /* VIDEO_CONTI_BRACKET */

      if (conti_capnum == p->capnum)
        {
          conti_end = VIDEO_TRUE;
        }

      if (g_v_cisif.last_frame)
        {
          conti_end = VIDEO_TRUE;
        }
    }

exit:
  p->info.h_size = g_v_isx.cap_param.jpeg_hsize;
  p->info.v_size = g_v_isx.cap_param.jpeg_vsize;
  p->info.capnum = conti_capnum;

#ifdef VIDEO_CONTI_BRACKET
  /* CAP_HALF_AE_CTRL : HAFREL=HIGHSPEED, CAP=OFF  */
  reg.regval  = 0x1;
  reg.regaddr = 0x0181;
  reg.regsize = 1;
  isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);

  reg.regval = (uint16_t)VIDEO_EV_OFF;
  reg.regaddr = video_set_imgsns_regs[VIDEO_PARAM_ID_EV].addr;
  reg.regsize = video_set_imgsns_regs[VIDEO_PARAM_ID_EV].regsize;
  video_printf("IMGIOC_WRITEREG : EV:\n", reg.regval);
  isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);

  reg.regval = (uint16_t)0;
  reg.regaddr = video_set_imgsns_regs[VIDEO_PARAM_ID_BRIGHTNESS].addr;
  reg.regsize = video_set_imgsns_regs[VIDEO_PARAM_ID_BRIGHTNESS].regsize;
  video_printf("IMGIOC_WRITEREG : BRIGHTNESS:\n", reg.regval);
  isx012_ioctl(IMGIOC_WRITEREG, (unsigned long)&reg);
#endif /* VIDEO_CONTI_BRACKET */

  video_printf("ioctl IMGIOC_SETMODE(MONITORING) call.\n");
  DBG_TIME_START();
  ret = isx012_ioctl(IMGIOC_SETMODE, MODE_ISX012_MONITORING);
  DBG_TIME_STOP("ioctl IMGIOC_SETMODE(MONITORING)");
  if (ret < 0)
    {
      video_printf("ERROR: ioctl IMGIOC_SETMODE(MONITORING):%d\n", ret);
    }
  else
    {
      priv->imgsns_mode = VIDEO_IMGSNS_MODE_MONI;
    }

  if (g_v_cisif.errint != 0)
    {
      ret = -g_v_cisif.errint;
    }

  return ret;
}

int video_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  video_mng_t      *priv  = inode->i_private;
  int ret = ERROR;

  if (0 != sem_init(&priv->sem_cisifsync, 0, 0))
    {
      video_printf("ERROR: Failed to sem_init(sem_cisifsync).\n");
      return ret;
    }

  video_init_internal_param(priv);
  ret = video_init_image_sensor(priv);
  if (ret == 0)
    {
      ret = OK;
    }

  return ret;
}

int video_close(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  video_mng_t      *priv  = inode->i_private;
  video_api_chg_img_sns_state_t p;
  int ret = ERROR;

  p.state = VIDEO_STATE_POWOFF;
  ret = video_chg_img_sns_state(priv, &p);
  if (ret != 0)
    {
      return ret;
    }

  if (0 != sem_destroy(&priv->sem_cisifsync))
    {
      video_printf("ERROR: Failed to sem_destroy(sem_cisifsync).\n");
      ret = ERROR;
    }

  return ret;
}

int video_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  video_mng_t      *priv  = inode->i_private;
  int ret = OK;

  switch (cmd)
    {
      case VIDEOIOC_CHG_IMGSNS_STATE:
        ret = video_chg_img_sns_state(priv, (video_api_chg_img_sns_state_t *)arg);
        break;
      case VIDEOIOC_SET_CAP_PARAM:
        ret = video_set_capture_param(priv, (video_api_set_cap_param_t *)arg);
        break;
      case VIDEOIOC_CAP_FRAME:
        ret = video_capture_frame(priv, (video_api_cap_frame_t *)arg);
        break;
      case VIDEOIOC_CONTI_CAP:
        ret = video_continuous_capture(priv, (video_api_conti_cap_t *)arg);
        break;
      case VIDEOIOC_SET_IMGSNS_PARAM:
        ret = video_set_img_sns_param(priv, (video_api_set_img_sns_param_t *)arg);
        break;
      case VIDEOIOC_SET_IMGSNS_PARAM_ALL:
        ret = video_set_img_sns_param_all(priv, (video_api_set_img_sns_param_all_t *)arg);
        break;
      case VIDEOIOC_WR_IMGSNS_REG:
        ret = video_write_img_sns_reg(priv, (video_api_img_sns_reg_t *)arg);
        break;
      case VIDEOIOC_RD_IMGSNS_REG:
        ret = video_read_img_sns_reg(priv, (video_api_img_sns_reg_t *)arg);
        break;
      case VIDEOIOC_DO_HALF_REL:
        ret = video_do_half_rel(priv, (video_api_do_half_rel_t *)arg);
        break;
      case VIDEOIOC_GET_AUTO_PARAM:
        ret = video_get_auto_param(priv, (video_api_get_auto_param_t *)arg);
        break;
      default:
        videoerr("Unrecognized cmd: %d\n", cmd);
        ret = - ENOTTY;
        break;
    }

  return ret;
}

int video_register(FAR const char *devpath)
{
  video_mng_t *priv;
  char path[16];
  int ret;

  /* Initialize video device structure */

  priv = (FAR struct video_mng_s *)kmm_malloc(sizeof(struct video_mng_s));
  if (!priv)
    {
      videoerr("Failed to allocate instance\n");
      return -ENOMEM;
    }

  g_v_mng = priv;
  sem_init(&priv->sem_cisifsync, 0, 0);

  /* Register the character driver */

  (void) snprintf(path, sizeof(path), "%s%d", devpath, 0);
  ret = register_driver(path, &g_videofops, 0666, priv);
  if (ret < 0)
    {
      videoerr("Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }
  else
    {
      videoerr("ISX012 driver loaded successfully!\n");
    }

  return ret;
}
