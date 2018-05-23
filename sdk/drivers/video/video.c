/****************************************************************************
 * drivers/video/video.c
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

#include "isx012_reg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Display time ON/OFF */
/* #define VIDEO_TIME_MEASURE */

/* At initialization, it automatically transits to ACTIVE_MODE */
/* #define VIDEO_INIT_ACTIVE */

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

#define VIDEO_V4_BUF_MAX_CNT  (256)

#define VIDEO_DEV_PATH_LEN     (32)

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
  uint8_t init;
  uint8_t devpath[VIDEO_DEV_PATH_LEN];
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
static int  video_capture_frame(video_mng_t *priv, video_api_cap_frame_t *p);
static int  video_set_img_sns_param(video_mng_t *priv,
                                    video_api_set_img_sns_param_t *p);
static int  video_set_frame_info(video_mng_t *priv,
                                 video_api_cap_frame_t *p,
                                 video_cisif_result_t *res);
static int  video_get_picture_info(video_picture_info_t *pict_info);
static int  video_set_img_sns_crop(video_cap_param_t *param,
                                   video_crop_t *crop);
static int  video_set_img_sns_crop_off(void);

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
static video_cisif_result_t g_v_cisif;
static video_mng_t         *g_v_mng;

static v4l2_buffer_t *g_v4_buf = NULL;
static uint32_t       g_v4_buf_rcnt = 1;
static uint32_t       g_v4_buf_cnt  = 0;
static uint32_t       g_v4_buf_mode = VIDEO_MODE_MONITORING;

static const video_size_t video_rs2sz[VIDEO_RESOLUTION_MAX] =
{
  { VIDEO_HSIZE_QVGA,     VIDEO_VSIZE_QVGA    },
  { VIDEO_HSIZE_VGA,      VIDEO_VSIZE_VGA     },
  { VIDEO_HSIZE_QUADVGA,  VIDEO_VSIZE_QUADVGA },
  { VIDEO_HSIZE_HD,       VIDEO_VSIZE_HD      },
  { VIDEO_HSIZE_FULLHD,   VIDEO_VSIZE_FULLHD  },
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
  { EZOOM_MAG, 0x0100, 2 },
  { OFFSET_X,  0x0000, 2 },
  { OFFSET_Y,  0x0000, 2 },
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
  { G0_LOWGM_ON_R,         0x0330, 2 },
  { G0_0CLIP_R,            0x0E00, 2 },
  { G0_LOWGM_ON_G,         0x0330, 2 },
  { G0_0CLIP_G,            0x0E00, 2 },
  { G0_LOWGM_ON_B,         0x0330, 2 },
  { G0_0CLIP_B,            0x0E00, 2 },
  { G0_KNOT_GAINCTRL_TH_L,    103, 1 },
  { G0_KNOT_GAINCTRL_TH_H,    108, 1 },
};

static const isx012_reg_t video_gamma_regs_def[VIDEO_GAMMA_REGNUM] =
{
  { G0_LOWGM_ON_R,         0x0661, 2 },
  { G0_0CLIP_R,            0x1E0A, 2 },
  { G0_LOWGM_ON_G,         0x0661, 2 },
  { G0_0CLIP_G,            0x1E0A, 2 },
  { G0_LOWGM_ON_B,         0x0661, 2 },
  { G0_0CLIP_B,            0x1E0A, 2 },
  { G0_KNOT_GAINCTRL_TH_L,    103, 1 },
  { G0_KNOT_GAINCTRL_TH_H,    108, 1 },
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

  g_v_cisif.size = size;

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
          g_v_cisif.errint = ENOMEM;
        }
    }
}

static int video_init_image_sensor(video_mng_t *priv)
{
  DBG_TIME_START();
  isx012_open();
  DBG_TIME_STOP("open isx012 driver");

#ifdef VIDEO_GAMMA_BASIC
  video_set_gamma_table(GAMMA_TABLE_BASIC);
#endif /* VIDEO_GAMMA_BASIC */
#ifdef VIDEO_GAMMA_HIGH_BRIGHT
  video_set_gamma_table(GAMMA_TABLE_HIGH_BRIGHT);
#endif /* VIDEO_GAMMA_HIGH_BRIGHT */

#ifdef VIDEO_INIT_ACTIVE
  /* Cmera Device status Sleep -> Active */
  if (isx012_ioctl(IMGIOC_SETSTATE, STATE_ISX012_ACTIVE) < 0)
    {
      video_printf("ERROR: Failed to ioctl IMGIOC_SETSTATE. %d\n", ret);
      return -ENODEV;
    }
  /* After Camera mode -> Monitoring */

  priv->imgsns_state = VIDEO_STATE_ACTIVE;
#else
  priv->imgsns_state = VIDEO_STATE_SLEEP;
#endif /* VIDEO_INIT_NOT_ACTIVE */

  return 0;
}

static void video_init_internal_param(video_mng_t *priv)
{
  priv->cap_param[VIDEO_MODE_CAPTURE].format     = VIDEO_FORMAT_JPEG;
  priv->cap_param[VIDEO_MODE_CAPTURE].resolution = VIDEO_FULLHD;
  priv->cap_param[VIDEO_MODE_CAPTURE].framerate  = VIDEO_15FPS;

  priv->cap_param[VIDEO_MODE_MONITORING].format     = VIDEO_FORMAT_YUV;
  priv->cap_param[VIDEO_MODE_MONITORING].resolution = VIDEO_QVGA;
  priv->cap_param[VIDEO_MODE_MONITORING].framerate  = VIDEO_30FPS;
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
       0,   /* FULLHD   : invalid  */
       0,   /* 3M       : invalid  */
       0,   /* 5M       : invalid  */
  };
  uint16_t ezoom_mag_fllpx[VIDEO_RESOLUTION_MAX] =
  {
    2048,   /* QVGA     : x8  */
    1024,   /* VGA      : x4  */
     512,   /* Quad-VGA : x2  */
     512,   /* HD       : x2  */
     256,   /* FULLHD   : x1  */
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

static int video_capture_frame(video_mng_t *priv, video_api_cap_frame_t *p)
{
  int ret = 0;
  cisif_param_t    cis;
  video_img_format_e format;

  if (priv->imgsns_state == VIDEO_STATE_SLEEP)
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
  ret = isx012_ioctl(IMGIOC_SETCISIF, (unsigned long)&cis);
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
  int      cidx;
  int      midx;

  cidx = priv->cap_param[VIDEO_MODE_CAPTURE].resolution;
  midx = priv->cap_param[VIDEO_MODE_MONITORING].resolution;

  p->info.mode = p->mode;
  memcpy(&p->info.cap_param,
         &priv->cap_param[p->mode],
         sizeof(video_cap_param_t));

  p->info.out_addr = res->addr;
  p->info.out_size = res->size;

  if (p->mode == VIDEO_MODE_CAPTURE)
    {
      p->info.h_size = video_rs2sz[cidx].h;
      p->info.v_size = video_rs2sz[cidx].v;
    }
  else
    {
      p->info.h_size = video_rs2sz[midx].h;
      p->info.v_size = video_rs2sz[midx].v;
    }

  if ((priv->cap_param[p->mode].format == VIDEO_FORMAT_JPEG) &&
      (res->errint == 0))
    {
      p->info.out_size = video_correct_jpeg_size(res->addr, res->size);
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int video_open(FAR struct file *filep)
{
  FAR struct inode      *inode = filep->f_inode;
  video_mng_t           *priv  = inode->i_private;
  video_api_set_img_sns_param_t iparam;
  int ret = ERROR;

  if (0 != sem_init(&priv->sem_cisifsync, 0, 0))
    {
      video_printf("ERROR: Failed to sem_init(sem_cisifsync).\n");
      return ret;
    }

  video_init_internal_param(priv);
  ret = video_init_image_sensor(priv);
  if (ret != 0)
    {
      return ret;
    }

  iparam.param.id = VIDEO_PARAM_ID_JPEG_QUALITIY;
  iparam.param.val.jpeg_qualitiy = 75;
  ret = video_set_img_sns_param(priv, &iparam);

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

  if (g_v4_buf)
    {
      free(g_v4_buf);
    }

  return ret;
}

int video_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  video_mng_t      *priv  = inode->i_private;
  int ret = OK;

  v4l2_format_t                *fmt_lp = (v4l2_format_t *)arg;
  v4l2_requestbuffers_t        *req_lp = (v4l2_requestbuffers_t *)arg;
  v4l2_buffer_t                *buf_lp = (v4l2_buffer_t *)arg;
  video_api_cap_frame_t         cap;
  video_api_chg_img_sns_state_t stat_l;

  switch (cmd)
    {
      case VIDIOC_S_FMT:
        g_v4_buf_mode = (fmt_lp->fmt.pix.pixelformat == V4L2_PIX_FMT_UYVY) ?
          VIDEO_MODE_MONITORING : VIDEO_MODE_CAPTURE;
        break;
      case VIDIOC_REQBUFS:
        g_v4_buf_rcnt = req_lp->count;
        if (g_v4_buf_rcnt > VIDEO_V4_BUF_MAX_CNT)
          {
            ret = -ENOMEM;
          }
        else
          {
            g_v4_buf = calloc(g_v4_buf_rcnt, sizeof(v4l2_buffer_t));
            g_v4_buf_cnt = 0;
            if (g_v4_buf == NULL)
              {
                ret = -ENOMEM;
              }

          }

        break;
      case VIDIOC_QBUF:
        if (buf_lp == NULL)
          {
            ret = -EPERM;
          }
        else
          {
            if (buf_lp->index >= g_v4_buf_rcnt)
              {
                ret = -EPERM;
              }
            else if (buf_lp->index >= VIDEO_V4_BUF_MAX_CNT)
              {
                ret = -ENOMEM;
              }
            else
              {
                g_v4_buf[buf_lp->index].m.userptr = buf_lp->m.userptr;
                g_v4_buf[buf_lp->index].length    = buf_lp->length;
              }

          }

        break;
      case VIDIOC_DQBUF:
        cap.mode        = g_v4_buf_mode;
        cap.crop_ctrl   = VIDEO_DISABLE;
        cap.buffer.addr = g_v4_buf[g_v4_buf_cnt].m.userptr;
        cap.buffer.size = g_v4_buf[g_v4_buf_cnt].length;
        ret = video_capture_frame(priv, &cap);
        if (!ret)
          {
            buf_lp->m.userptr = g_v4_buf[g_v4_buf_cnt].m.userptr;
            buf_lp->length    = g_v4_buf[g_v4_buf_cnt].length;
            buf_lp->bytesused = g_v_cisif.size;
            buf_lp->index     = g_v4_buf_cnt;
            g_v4_buf_cnt = (g_v4_buf_cnt < (g_v4_buf_rcnt-1)) ? g_v4_buf_cnt+1 : 0;
          }

        break;
      case VIDIOC_STREAMON:
        stat_l.state = VIDEO_STATE_ACTIVE;
        ret = video_chg_img_sns_state(priv, &stat_l);
        usleep(100000);
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
  char path[VIDEO_DEV_PATH_LEN];
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
      strncpy((char *)g_v_mng->devpath, path, VIDEO_DEV_PATH_LEN - 1);
      videoerr("ISX012 driver loaded successfully!\n");
    }

  return ret;
}

int video_unregister(void)
{
  int ret = 0;

  if (g_v_mng)
    {
      unregister_driver((const char *)g_v_mng->devpath);
      sem_destroy(&g_v_mng->sem_cisifsync);
      kmm_free(g_v_mng);
      g_v_mng = NULL;
      return ret;
    }

  return -ENODEV;
}
