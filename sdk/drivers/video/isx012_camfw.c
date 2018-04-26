/****************************************************************************
 * drivers/video/isx012_camfw.c
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

#include <arch/chip/cisif.h>
#include <nuttx/video/isx012.h>
#include <nuttx/video/isx012_camfw.h>

#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>

#include <time.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Display time ON/OFF */
/* #define CAMFW_TIME_MEASURE */

/* Accelerate mode switching ON/OFF */
/* #define CAMFW_ISX012_FAST_MODECHG_EN */
/* #define CAMFW_ISX012_FAST_SHT_MODE_EN */

/* Continuous exposure compensation during capture ON/OFF */
/* #define CAMFW_CONTI_BRACKET */

/* At initialization, it automatically transits to ACTIVE_MODE */
#define CAMFW_INIT_ACTIVE

/* Select gamma table */
#define CAMFW_GAMMA_DEFAULT
/* #define CAMFW_GAMMA_BASIC */
/* #define CAMFW_GAMMA_HIGH_BRIGHT */

/* Test tuning code */
/* #define CAMFW_IMG_TUNING */

#define camfw_printf(format, ...)   _info(format, ##__VA_ARGS__)

/*------------------
 * Message queue
 *----------------*/
#define CAMFW_API_REQ_QUEUE     "camfw/api_req"
#define CAMFW_API_RESP_QUEUE    "camfw/api_resp"

#define CAMFW_TRUE              (1)
#define CAMFW_FALSE             (0)

#define CAMFW_HSIZE_QVGA        (320)
#define CAMFW_VSIZE_QVGA        (240)
#define CAMFW_HSIZE_VGA         (640)
#define CAMFW_VSIZE_VGA         (480)
#define CAMFW_HSIZE_QUADVGA     (1280)
#define CAMFW_VSIZE_QUADVGA     (960)
#define CAMFW_HSIZE_HD          (1280)
#define CAMFW_VSIZE_HD          (720)
#define CAMFW_HSIZE_5M          (2560)
#define CAMFW_VSIZE_5M          (1920)
#define CAMFW_HSIZE_3M          (2048)
#define CAMFW_VSIZE_3M          (1536)

#define CAMFW_ALING32_CHECK(x)  (x % 32)
#define CAMFW_EOI_CORRECT_MAX_SIZE  (32)

#define CAMFW_EZOOM_OFFSET_PX   (16)

#define CAMFW_INIT_REGNUM       (5)
#define CAMFW_AE_AUTO_REGNUM    (6)
#define CAMFW_AE_NOW_REGNUM     (6)
#define CAMFW_AWB_AUTO_REGNUM   (3)
#define CAMFW_AWB_NOW_REGNUM    (3)

#define CAMFW_ISX012_HALFREL_TIMEOUT      (2*1000*1000)   /* usec */
#define CAMFW_ISX012_HALFREL_WAITTIME     (20*1000)       /* usec */
#define CAMFW_ISX012_HALF_MOVE_STS_REG    (0x01B0)
#define CAMFW_ISX012_MOVE_AWB_F           (0x01)
#define CAMFW_ISX012_MOVE_AE_F            (0x02)
#define CAMFW_CISIF_TRANSEND_TIMEOUT      (1*1000)        /* msec */

/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef enum 
{
  CAMFW_APIID_CHG_IMGSNS_STATE = 0,
  CAMFW_APIID_CAP_FRAME,
  CAMFW_APIID_SET_CAP_PARAM,
  CAMFW_APIID_SET_IMGSNS_PARAM,
  CAMFW_APIID_SET_IMGSNS_PARAM_ALL,
  CAMFW_APIID_WRITE_IMGSNS_REG,
  CAMFW_APIID_READ_IMGSNS_REG,
  CAMFW_APIID_DO_HALFRELEASE,
  CAMFW_APIID_GET_AUTO_PARAM,
  CAMFW_APIID_CONTI_CAP,
  CAMFW_APIID_MAX,
} CamfwApiId_e;

typedef enum
{
  CAMFW_IMGSNS_MODE_MONI = 0,
  CAMFW_IMGSNS_MODE_CAP,
  CAMFW_IMGSNS_MODE_HALFREL,
  CAMFW_IMGSNS_MODE_MAX
} CamfwImgSnsMode_e;

enum camfw_ezoom_reg_e
{
  CAMFW_EZOOM_MAG,
  CAMFW_EZOOM_OFFSET_X,
  CAMFW_EZOOM_OFFSET_Y,
  CAMFW_EZOOM_REGNUM
};

typedef struct
{
  CamfwImgSnsState_e state;
} CamfwApiChgImgSnsState_t;

typedef struct
{
  CamfwMode_e mode;
  CamfwBuffer_t buffer;
  CamfwCapFrameInfo_t info;
  CamfwCrop_t crop;
  CamfwCtrl_e crop_ctrl;
} CamfwApiCapFrame_t;

typedef struct
{
  CamfwMode_e mode;
  uint32_t capnum;
  uint32_t interval;
  CamfwBuffer_t buffer;
  CamfwContiCapInfo_t info;
  CamfwCrop_t crop;
  CamfwCtrl_e crop_ctrl;
} CamfwApiContiCap_t;

typedef struct
{
  CamfwMode_e mode;
  CamfwCapParam_t param;
} CamfwApiSetCapParam_t;

typedef struct
{
  CamfwImgSnsParam_t param;
} CamfwApiSetImgSnsParam_t;

typedef struct
{
  CamfwImgSnsParamAll_t param;
} CamfwApiSetImgSnsParamAll_t;

typedef struct
{
  uint16_t addr;
  uint16_t regsize;
  uint16_t val;
} CamfwApiImgSnsReg_t;

typedef struct
{
  CamfwCtrl_e cancel;
  CamfwAutoInfo_t info;
} CamfwApiDoHalfRelease_t;

typedef struct
{
  CamfwAutoInfo_t info;
} CamfwApiGetAutoParam_t;

typedef struct
{
  CamfwApiId_e api_id;
  int result;
  union
  {
    CamfwApiChgImgSnsState_t chg_state;
    CamfwApiCapFrame_t cap_frame;
    CamfwApiSetCapParam_t cap;
    CamfwApiSetImgSnsParam_t imgsns;
    CamfwApiSetImgSnsParamAll_t imgsns_all;
    CamfwApiImgSnsReg_t w_reg;
    CamfwApiImgSnsReg_t r_reg;
    CamfwApiDoHalfRelease_t halfrel;
    CamfwApiGetAutoParam_t get_auto;
    CamfwApiContiCap_t conti_cap;
  } u;
} CamfwApiMsg_t;

typedef struct
{
  int fd;
  pid_t pid_main;
  sem_t sem_apisync;
  sem_t sem_cisifsync;
  CamfwCapParam_t cap_param[CAMFW_MODE_MAX];
  CamfwImgSnsParamAll_t imgsns_param_all;
  CamfwImgSnsState_e imgsns_state;
  CamfwImgSnsMode_e imgsns_mode;
  uint8_t init;
} CamfwMng_t;

typedef struct
{
  uint32_t addr;
  uint32_t size;
  uint8_t code;
  uint8_t last_frame;
  uint8_t errint;
} CamfwCisifResult_t;

typedef struct
{
  uint16_t h;
  uint16_t v;
} CamfwSize_t;

typedef struct
{
  uint16_t addr;
  uint16_t regsize;
} CamfwIsx012Reg_t;

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int  camfw_MainTask(int argc, char *argv[]);
static void camfw_CallbackCisif(
  uint8_t code, uint8_t last_frame, uint32_t size, uint32_t addr);
static int  camfw_SendMessage(CamfwApiId_e api_id, void *param, int size);
static void camfw_InitInternalParam(void);
static int  camfw_InitImageSensor(void);
static int  camfw_ChangeImgSnsState(CamfwApiChgImgSnsState_t *p);
static int  camfw_SetCaptureParam(CamfwApiSetCapParam_t *p);
static int  camfw_CaptureFrame(CamfwApiCapFrame_t *p);
static int  camfw_SetImgSnsParam(CamfwApiSetImgSnsParam_t *p);
static int  camfw_SetImgSnsParamAll(CamfwApiSetImgSnsParamAll_t *p);
static int  camfw_WriteImgSnsRegister(CamfwApiImgSnsReg_t *p);
static int  camfw_ReadImgSnsRegister(CamfwApiImgSnsReg_t *p);
static int  camfw_SetFrameInfo(CamfwApiCapFrame_t *p, CamfwCisifResult_t *res);
static int  camfw_GetPictureInfo(CamfwPictureInfo_t *pict_info);
static void camfw_CreateCisifParam(CamfwMode_e mode, cisif_param_t *p);
static int  camfw_SetImgSnsCrop(CamfwCapParam_t *param, CamfwCrop_t *crop);
static int  camfw_SetImgSnsCropOff(void);
static int  camfw_DoHalfRelease(CamfwApiDoHalfRelease_t *p);
static int  camfw_GetAutoParam(CamfwApiGetAutoParam_t *p);
static int  camfw_ContinuousCapture(CamfwApiContiCap_t *p);

static int  camfw_GetIntmean(uint16_t *buf, uint16_t *free);
static uint32_t camfw_CorrectJpegSize(uint32_t addr, uint32_t size);
static int  camfw_twaisem(sem_t *sem, uint32_t timeout_ms);

#ifdef CAMFW_TIME_MEASURE
static uint64_t camfw_GetMsecTime(void)
{
    struct timespec tp;
    if (clock_gettime(CLOCK_REALTIME, &tp)) {
        return 0;
    }
    return (((uint64_t)tp.tv_sec) * 1000 + tp.tv_nsec / 1000000);
}
#endif /* CAMFW_TIME_MEASURE */

/****************************************************************************
 * Private Data
 ****************************************************************************/
static isx012_t           camfw_isx;
static CamfwCisifResult_t camfw_cisif_result;
static CamfwMng_t         camfw_mng;

static const isx012_rate_t camfw_convfps[CAMFW_FRAME_RATE_MAX] =
{
  RATE_ISX012_120FPS,   /* CAMFW_120FPS */
  RATE_ISX012_60FPS,    /* CAMFW_60FPS  */
  RATE_ISX012_30FPS,    /* CAMFW_30FPS  */
  RATE_ISX012_15FPS,    /* CAMFW_15FPS  */
  RATE_ISX012_7_5FPS,   /* CAMFW_7_5FPS */
  RATE_ISX012_5FPS      /* CAMFW_5FPS   */
};

static const CamfwSize_t camfw_rs2sz[CAMFW_RESOLUTION_MAX] =
{
  { CAMFW_HSIZE_QVGA,     CAMFW_VSIZE_QVGA    },
  { CAMFW_HSIZE_VGA,      CAMFW_VSIZE_VGA     },
  { CAMFW_HSIZE_QUADVGA,  CAMFW_VSIZE_QUADVGA },
  { CAMFW_HSIZE_HD,       CAMFW_VSIZE_HD      },
  { CAMFW_HSIZE_3M,       CAMFW_VSIZE_3M      },
  { CAMFW_HSIZE_5M,       CAMFW_VSIZE_5M      }
};

static const CamfwIsx012Reg_t camfw_set_imgsns_regs[CAMFW_PARAM_ID_MAX] =
{
  { 0x01C5, 1 }, /* CAMFW_PARAM_ID_COLOR        : FMODE */
  { 0x02A8, 1 }, /* CAMFW_PARAM_ID_ISO          : ISO_TYPE1 */
  { 0x02A0, 2 }, /* CAMFW_PARAM_ID_SHUTTER      : SHT_PREMODE_TYPE1 */
  { 0x0180, 1 }, /* CAMFW_PARAM_ID_EV           : EVSEL */
  { 0x01C6, 1 }, /* CAMFW_PARAM_ID_BRIGHTNESS   : UIBRIGHTNESS */
  { 0x01C7, 1 }, /* CAMFW_PARAM_ID_CONTRAST     : UICONTRAST */
  { 0x00F9, 1 }, /* CAMFW_PARAM_ID_JPEG_QUALITIY: INIT_QLTY2 */
  { 0x6C93, 1 }, /* CAMFW_PARAM_ID_YGAMMA       : YGAMMA_MODE    */
  { 0x0282, 1 }, /* CAMFW_PARAM_ID_AWB          : AWB_SN1 */
  { 0x02AC, 1 }  /* CAMFW_PARAM_ID_PHOTOMETRY   : AE_SUB_SN1 */
};

static const CamfwIsx012Reg_t camfw_exif_regs[] =
{
  { 0x019A, 1 },  /* iso_sens     : ISOSENS_OUT */
  { 0x019C, 2 },  /* shutter_speed: SHT_TIME_OUT_L */
  { 0x019E, 2 }   /* shutter_speed: SHT_TIME_OUT_H */
};

static const isx012_reg_t camfw_ezoom_regs[CAMFW_EZOOM_REGNUM] =
{
  { 0x00A2, 0x0100, 2 },  /* EZOOM_MAG  */
  { 0x00A8, 0x0000, 2 },  /* OFFSET_X   */
  { 0x00AA, 0x0000, 2 }   /* OFFSET_Y   */
};

static const isx012_reg_t camfw_init_regs[] =
{
#ifdef CAMFW_ISX012_FAST_MODECHG_EN
  { 0x500A, 1,  1 },  /* FAST_MODECHG_EN   */
#endif /* CAMFW_ISX012_FAST_MODECHG_EN */
#ifdef CAMFW_ISX012_FAST_SHT_MODE_EN
  { 0x500B, 1,  1 },  /* FAST_SHT_MODE_SEL  */
#endif /* CAMFW_ISX012_FAST_SHT_MODE_EN */
#ifndef CAMFW_CONTI_BRACKET
  { 0x0181, 1,  1 },  /* CAP_HALF_AE_CTRL : HAFREL=HIGHSPEED, CAP=OFF   */
#else
  { 0x0181, 7,  1 },  /* CAP_HALF_AE_CTRL : HAFREL=HIGHSPEED, CAP=Auto  */
#endif /* CAMFW_CONTI_BRACKET */
  { 0x01AE, 1,  1 },  /* HALF_AWB_CTRL  */
  { 0x5E32, 15, 1 },  /* AESPEED_FAST   */
  { 0x5E3D, 45, 1 },  /* FASTMOVE_TIMEOUT   */
  { 0x6C93, 1,  1 }   /* YGAMMA_MODE    */
};

static const uint8_t camfw_convawb[CAMFW_AWB_MAX] =
{
  0x20,   /* CAMFW_AWB_ATM */
  0x04,   /* CAMFW_AWB_CLEARWEATHER  */
  0x05,   /* CAMFW_AWB_SHADE  */
  0x06,   /* CAMFW_AWB_CLOUDYWEATHER  */
  0x07,   /* CAMFW_AWB_FLUORESCENTLIGHT */
  0x08,   /* CAMFW_AWB_LIGHTBULB */
};

#ifdef CAMFW_TIME_MEASURE
static uint32_t camfw_time_start;
static uint32_t camfw_time_stop;
#define DBG_MS_TIME_START()    \
  camfw_time_start = (uint32_t)camfw_GetMsecTime()
#define DBG_MS_TIME_STOP(x)     \
  camfw_time_stop = (uint32_t)camfw_GetMsecTime(); \
  camfw_printf("%s: time:%d[ms]\n", \
                x,(uint32_t)(camfw_time_stop - camfw_time_start))
#define DBG_TIME_START()      DBG_MS_TIME_START()
#define DBG_TIME_STOP(x)      DBG_MS_TIME_STOP(x)
#else
#define DBG_TIME_START()      
#define DBG_TIME_STOP(x)      
#endif /* CAMFW_TIME_MEASURE */
/****************************************************************************
 * Public Data
 ****************************************************************************/


/****************************************************************************
 * Private Functions
 ****************************************************************************/
static const CamfwIsx012Reg_t camfw_ae_depend_regs[] =
{
  { 0x7C99, 1 }, /* AELEVEL */
  { 0x7C9A, 1 }, /* REFLEVEL */
  { 0x7C9B, 1 }, /* ILMLEVEL */
  { 0x7C9C, 1 }, /* ERRLEVEL */
  { 0x7C9D, 1 }, /* GAIN_LEVEL */
  { 0x7C9E, 1 }, /* NOCRCT_GAIN_LEVEL*/
  { 0x7C9F, 1 }, /* DARK_LEVEL */
  { 0x7CA0, 1 }, /* AFCTRL_LEVEL */
  { 0x7CAC, 2 }, /* ERRSCL */
  { 0x7CAE, 2 }  /* ILMSCL */
};

static const char *camfw_ae_depend_regs_str[] =
{
  "AELEVEL",
  "REFLEVEL",
  "ILMLEVEL",
  "ERRLEVEL",
  "GAIN_LEVEL",
  "NOCRCT_GAIN_LEVEL",
  "DARK_LEVEL",
  "AFCTRL_LEVEL",
  "ERRSCL",
  "ILMSCL"
};

static void camfw_show_ae_dependparam(void)
{
  isx012_reg_t reg;
  int idx;

  for (idx = 0;
       idx < sizeof(camfw_ae_depend_regs)/sizeof(CamfwIsx012Reg_t);
       idx++ )
    {
      reg.regval  = 0;
      reg.regsize = camfw_ae_depend_regs[idx].regsize;
      reg.regaddr = camfw_ae_depend_regs[idx].addr;

      ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
      camfw_printf("%s : %d\n", camfw_ae_depend_regs_str[idx], reg.regval);
    }
}

void camfw_set_cap_shtagc(int mode, uint16_t shutter, int8_t agc)
{
#define CAP_HALF_AE_CTRL_ADDR     (0x0181)
#define CAP_SHT_ADDR              (0x0182)
#define CAP_AGC_ADDR              (0x0184)

  isx012_reg_t reg;

  if (mode)
    {
      camfw_printf("ON:SHUTTER=%d,ACG=%d\n", shutter, agc);

      reg.regaddr = CAP_SHT_ADDR;
      reg.regsize = 2;
      reg.regval  = shutter;
      ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);

      reg.regaddr = CAP_AGC_ADDR;
      reg.regsize = 1;
      reg.regval  = (uint16_t)agc;
      ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);

      reg.regaddr = CAP_HALF_AE_CTRL_ADDR;
      reg.regsize = 1;
      reg.regval  = 3;
      ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);
    }
  else
    {
      camfw_printf("OFF:SHUTTER=%d,ACG=%d\n", shutter, agc);
      reg.regaddr = CAP_HALF_AE_CTRL_ADDR;
      reg.regsize = 1;
      reg.regval  = 0x01;
      ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);
    }
}



#ifdef CAMFW_IMG_TUNING
static int camfw_set_spot_window(uint8_t no)
{
#define SPOT_FRM_NUM_ADDR   0x5E33
#define SPOT_SIDEWEIGHT     0x5E34
  isx012_reg_t reg;

  reg.regaddr = SPOT_FRM_NUM_ADDR;
  reg.regsize = 1;
  reg.regval  = (uint16_t)no;
  ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);

  reg.regaddr = SPOT_SIDEWEIGHT;
  reg.regsize = 1;
  reg.regval  = 75;
  ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);

  return 0;
}

static int camfw_show_ae_intmean(void)
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
      ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
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
static  int camfw_evref_type_idx = 0;
static  uint16_t evref_type_val[EVREF_TYPE_NUM] =
{
  0x2272,
  0x3697,
  0x44E4,
  0x5157,
  0x7777,
};

static int camfw_set_evref_type(int idx)
{
#define EVREF_TYPE1_ADDR    (0x02FC)
  isx012_reg_t reg;

  reg.regaddr = EVREF_TYPE1_ADDR;
  reg.regsize = 2;
  reg.regval  = evref_type_val[idx];

  ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);

  camfw_printf("EVREF_TYPE_VAL = %d\n", reg.regval);
  return 0;
}
#endif /* CAMFW_IMG_TUNING */

#ifndef CAMFW_GAMMA_DEFAULT
static int camfw_gamma_tble_idx = 0;
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

#define CAMFW_GAMMA_REGNUM  8
static const isx012_reg_t camfw_gamma_regs[CAMFW_GAMMA_REGNUM] =
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

static const isx012_reg_t camfw_gamma_regs_def[CAMFW_GAMMA_REGNUM] =
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

static int camfw_set_gamma_table(int idx)
{
#define GAMMA0_ADDR               0x7000
  int no;
  isx012_reg_t reg;
  isx012_reg_t *p_gamma_reg;

  /* Set GAMMA0 registers */
  p_gamma_reg = (isx012_reg_t *)camfw_gamma_regs_def;
  if (idx != 0)
    {
      p_gamma_reg = (isx012_reg_t *)camfw_gamma_regs;
    }

  for(no = 0; no < CAMFW_GAMMA_REGNUM; no++)
    {
      ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)p_gamma_reg);
      p_gamma_reg++;
    }

  reg.regsize = 2;
  printf("gamma table = %d\n", idx);
  for(no = 0; no < GAMMA_TABLE_REG_NUM; no++)
    {
      reg.regaddr = (uint16_t)(GAMMA0_ADDR + (no << 1));
      reg.regval  = (uint16_t)gamma_reg_val[idx][no];
      ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);
      camfw_printf("[Knot%02d]%3d\n", no, reg.regval);
    }

  return 0;
}
#endif /* CAMFW_GAMMA_DEFAULT */

static int camfw_twaisem(sem_t *sem, uint32_t timeout_ms)
{
  struct timespec abstime = { 0 };
  unsigned long long tmp;
  int ret;

  clock_gettime(CLOCK_REALTIME, &abstime);
  tmp = abstime.tv_nsec / 1000000;
  tmp += timeout_ms;
  abstime.tv_sec += tmp / 1000;
  abstime.tv_nsec = tmp % 1000 * 1000000 + abstime.tv_nsec % 1000000;

  ret = sem_timedwait(&camfw_mng.sem_cisifsync, &abstime);
  if (ret != 0)
    {
      camfw_printf("ERROR: sem_timedwait() error. %d\n", errno);
      ret = ETIMEDOUT;
    }

  return ret;
}

static void camfw_CallbackCisif(
  uint8_t code,
  uint8_t last_frame,
  uint32_t size,
  uint32_t addr)
{

  camfw_cisif_result.size += size;

  if (code == 0)
    {
      camfw_cisif_result.addr = addr;
      camfw_cisif_result.code = code;
      camfw_cisif_result.last_frame = last_frame;
      sem_post(&camfw_mng.sem_cisifsync);
    }
  else
    {
      if (camfw_cisif_result.errint == 0)
        {
          camfw_cisif_result.errint = code;
        }
    }
}

static void camfw_CallbackCisifConti(
  uint8_t code,
  uint8_t last_frame,
  uint32_t size,
  uint32_t addr)
{
  if (code == 0)
    {
      if (camfw_cisif_result.errint == 0)
        {
          camfw_cisif_result.size = size;
          camfw_cisif_result.addr = addr;
        }

      camfw_cisif_result.code = code;
      camfw_cisif_result.last_frame = last_frame;
      sem_post(&camfw_mng.sem_cisifsync);
    }
  else
    {
      if (camfw_cisif_result.errint == 0)
        {
          camfw_cisif_result.errint = code;
        }
    }
}

static int camfw_SendMessage(CamfwApiId_e api_id, void *param, int size)
{
  CamfwApiMsg_t *p;
  unsigned long value;
  mqd_t mqd_req = (mqd_t)ERROR;
  mqd_t mqd_resp = (mqd_t)ERROR;
  int ret;

  p = malloc(sizeof(CamfwApiMsg_t));
  if (p == NULL)
    {
      return -ENOMEM;
    }

  p->api_id = api_id;
  memcpy(&p->u, param, size);

  mqd_req = mq_open(CAMFW_API_REQ_QUEUE, O_WRONLY);
  if (mqd_req == (mqd_t)ERROR)
    {
      camfw_printf("ERROR: mq_open(CAMFW_API_REQ_QUEUE). %d\n", errno);
      ret = -errno;
      goto exit;
    }

  mqd_resp = mq_open(CAMFW_API_RESP_QUEUE, O_RDONLY);
  if (mqd_resp == (mqd_t)ERROR)
    {
      camfw_printf("ERROR: mq_open(CAMFW_API_RESP_QUEUE). %d\n", errno);
      ret = -errno;
      goto exit;
    }

  ret = mq_send(mqd_req, (const char *)&p, sizeof(p), 0U);
  if (ret < 0)
    {
      camfw_printf("ERROR: mq_send(CAMFW_API_REQ_QUEUE). %d\n", errno);
      ret = -errno;
      goto exit;
    }

  /* Wait until API operation has finished */
  ret = mq_receive(mqd_resp, (char *)&value, sizeof(value), NULL);
  if (ret < 0)
    {
      camfw_printf("ERROR: mq_receive(CAMFW_API_RESP_QUEUE). %d\n", errno);
      ret = -errno;
      goto exit;
    }

  memcpy(param, &p->u, size);
  ret = p->result;

exit:
  if (mqd_resp != (mqd_t)ERROR)
    {
      mq_close(mqd_resp);
    }

  if (mqd_req != (mqd_t)ERROR)
    {
      mq_close(mqd_req);
    }

  if (p)
    {
      free(p);
    }

  return ret;
}

static int camfw_InitImageSensor(void)
{
  int ret;
  int idx;

  DBG_TIME_START();
  camfw_mng.fd = open("/dev/imager0", O_CREAT);
  DBG_TIME_STOP("open isx012 driver");
  if (camfw_mng.fd < 0)
    {
      camfw_printf("ERROR: Failed to open isx012. %d\n", errno);
      return -ENODEV;
    }

  /* Set Monitoring / Capture parameter */
  ret = ioctl(camfw_mng.fd, IMGIOC_SETMODEP, (unsigned long)&camfw_isx);
  if (ret < 0)
    {
      camfw_printf("ERROR: Failed to ioctl IMGIOC_SETMODEP. %d\n", ret);
      return -ENODEV;
    }

  /* Set Initial parameter */
  for (idx = 0; idx < sizeof(camfw_init_regs)/sizeof(isx012_reg_t); idx++)
    {
      ret = ioctl(camfw_mng.fd,
                  IMGIOC_WRITEREG,
                  (unsigned long)&camfw_init_regs[idx]);
      if (ret < 0)
        {
          camfw_printf("ERROR: Failed to ioctl IMGIOC_WRITEREG. %d\n", ret);
          return -ENODEV;
        }
    }
  camfw_printf("camfw_init_regs index = %d\n", idx);

#ifdef CAMFW_GAMMA_BASIC
  camfw_set_gamma_table(GAMMA_TABLE_BASIC);
#endif /* CAMFW_GAMMA_BASIC */
#ifdef CAMFW_GAMMA_HIGH_BRIGHT
  camfw_set_gamma_table(GAMMA_TABLE_HIGH_BRIGHT);
#endif /* CAMFW_GAMMA_HIGH_BRIGHT */

#ifdef CAMFW_INIT_ACTIVE
  /* Cmera Device status Sleep -> Active */
  DBG_TIME_START();
  ret = ioctl(camfw_mng.fd, IMGIOC_SETSTATE, STATE_ISX012_ACTIVE);
  DBG_TIME_STOP("ioctl IMGIOC_SETSTATE ACTIVE");
  if (ret < 0)
    {
      camfw_printf("ERROR: Failed to ioctl IMGIOC_SETSTATE. %d\n", ret);
      return -ENODEV;
    }
  /* After Camera mode -> Monitoring */

  camfw_mng.imgsns_state = CAMFW_STATE_ACTIVE;
#else
  camfw_mng.imgsns_state = CAMFW_STATE_SLEEP;
#endif /* CAMFW_INIT_NOT_ACTIVE */
  camfw_mng.imgsns_mode  = CAMFW_IMGSNS_MODE_MONI;

  return 0;
}

static void camfw_InitInternalParam(void)
{
  camfw_isx.cap_param.format = FORMAT_ISX012_JPEG_MODE1;
  camfw_isx.cap_param.rate = RATE_ISX012_15FPS;
  camfw_isx.cap_param.yuv_hsize = camfw_rs2sz[CAMFW_VGA].h;
  camfw_isx.cap_param.yuv_vsize = camfw_rs2sz[CAMFW_VGA].v;
  camfw_isx.cap_param.jpeg_hsize = camfw_rs2sz[CAMFW_VGA].h;
  camfw_isx.cap_param.jpeg_vsize = camfw_rs2sz[CAMFW_VGA].v;

  camfw_isx.moni_param.format = FORMAT_ISX012_YUV;
  camfw_isx.moni_param.rate = RATE_ISX012_30FPS;
  camfw_isx.moni_param.yuv_hsize = camfw_rs2sz[CAMFW_QVGA].h;
  camfw_isx.moni_param.yuv_vsize = camfw_rs2sz[CAMFW_QVGA].v;
  camfw_isx.moni_param.jpeg_hsize = camfw_rs2sz[CAMFW_QVGA].h;
  camfw_isx.moni_param.jpeg_vsize = camfw_rs2sz[CAMFW_QVGA].v;

  camfw_mng.cap_param[CAMFW_MODE_CAPTURE].format = CAMFW_FORMAT_JPEG;
  camfw_mng.cap_param[CAMFW_MODE_CAPTURE].resolution = CAMFW_VGA;
  camfw_mng.cap_param[CAMFW_MODE_CAPTURE].framerate = CAMFW_15FPS;

  camfw_mng.cap_param[CAMFW_MODE_MONITORING].format = CAMFW_FORMAT_YUV;
  camfw_mng.cap_param[CAMFW_MODE_MONITORING].resolution = CAMFW_QVGA;
  camfw_mng.cap_param[CAMFW_MODE_MONITORING].framerate = CAMFW_30FPS;
}

static int camfw_ChangeImgSnsState(CamfwApiChgImgSnsState_t *p)
{
  int ret;
  CamfwImgSnsState_e next_state;

  if ((camfw_mng.imgsns_state != CAMFW_STATE_POWOFF) && (camfw_mng.fd < 0))
    {
      return -ENODEV;
    }

  if (camfw_mng.imgsns_state == p->state)
    {
      /* no change state */
      return 0;
    }

  next_state = p->state;
  switch (next_state)
    {
      case CAMFW_STATE_ACTIVE:
        if (camfw_mng.imgsns_state != CAMFW_STATE_SLEEP)
          {
            return -EINVAL;
          }

        DBG_TIME_START();
        ret = ioctl(camfw_mng.fd, IMGIOC_SETSTATE, STATE_ISX012_ACTIVE);
        DBG_TIME_STOP("ioctl IMGIOC_SETSTATE(ACTIVE)");
        break;

      case CAMFW_STATE_SLEEP:
        if (camfw_mng.imgsns_state != CAMFW_STATE_ACTIVE)
          {
            return -EINVAL;
          }

        DBG_TIME_START();
        ret = ioctl(camfw_mng.fd, IMGIOC_SETSTATE, STATE_ISX012_SLEEP);
        DBG_TIME_STOP("ioctl IMGIOC_SETSTATE(SLEEP)");
        break;

      case CAMFW_STATE_POWOFF:
        DBG_TIME_START();
        ret = close(camfw_mng.fd);
        DBG_TIME_STOP("close -> IMGSNS_POWER_OFF");
        if (ret == 0)
          {
            camfw_mng.fd = -1;
          }

        ret = cxd56_cisiffinalize();
        if (ret != 0)
          {
            camfw_printf("ERROR: cxd56_cisiffinalize() %d.\n", ret);
          }

        break;

      case CAMFW_STATE_POWON:
        if (camfw_mng.imgsns_state != CAMFW_STATE_POWOFF)
          {
            return -EINVAL;
          }

        DBG_TIME_START();
        ret = camfw_InitImageSensor();
        DBG_TIME_STOP("open -> IMGSNS_POWER_ON");
        if (ret == 0)
          {
#ifdef CAMFW_INIT_ACTIVE
            next_state = CAMFW_STATE_ACTIVE;
#else
            next_state = CAMFW_STATE_SLEEP;
#endif /* CAMFW_INIT_ACTIVE */
          }

        ret = cxd56_cisifinit();
        if (ret != 0)
          {
            camfw_printf("ERROR: cxd56_cisifinit() %d.\n", ret);
          }

        break;

      default:
        ret = -EINVAL;
        break;
    }

  if (ret == 0)
    {
      camfw_mng.imgsns_state = next_state;
      camfw_mng.imgsns_mode  = CAMFW_IMGSNS_MODE_MONI;
    }

  return ret;
}

static int camfw_SetCaptureParam(CamfwApiSetCapParam_t *p)
{
  int ret;

  if (camfw_mng.fd < 0)
    {
      return -ENODEV;
    }

  if (camfw_mng.imgsns_mode != CAMFW_IMGSNS_MODE_MONI)
    {
      return ERROR;
    }

  if (p->mode == CAMFW_MODE_CAPTURE)
      {
        camfw_isx.cap_param.rate = camfw_convfps[p->param.framerate];
      }
    else
      {
        camfw_isx.moni_param.rate = camfw_convfps[p->param.framerate];
      }

  if (p->param.format == CAMFW_FORMAT_YUV)
    {
      if (p->mode == CAMFW_MODE_CAPTURE)
        {
          camfw_isx.cap_param.format = FORMAT_ISX012_YUV;
          camfw_isx.cap_param.yuv_hsize = camfw_rs2sz[p->param.resolution].h;
          camfw_isx.cap_param.yuv_vsize = camfw_rs2sz[p->param.resolution].v;
        }
      else
        {
          camfw_isx.moni_param.format = FORMAT_ISX012_YUV;
          camfw_isx.moni_param.yuv_hsize = camfw_rs2sz[p->param.resolution].h;
          camfw_isx.moni_param.yuv_vsize = camfw_rs2sz[p->param.resolution].v;
        }
    }
  else
    {
      if (p->mode == CAMFW_MODE_CAPTURE)
        {
          camfw_isx.cap_param.format = FORMAT_ISX012_JPEG_MODE1;
          camfw_isx.cap_param.jpeg_hsize = camfw_rs2sz[p->param.resolution].h;
          camfw_isx.cap_param.jpeg_vsize = camfw_rs2sz[p->param.resolution].v;
        }
      else
        {
          camfw_isx.moni_param.format = FORMAT_ISX012_JPEG_MODE1;
          camfw_isx.moni_param.jpeg_hsize = camfw_rs2sz[p->param.resolution].h;
          camfw_isx.moni_param.jpeg_vsize = camfw_rs2sz[p->param.resolution].v;
        }
    }

  DBG_TIME_START();
  ret = ioctl(camfw_mng.fd, IMGIOC_SETMODEP, (unsigned long)&camfw_isx);
  DBG_TIME_STOP("ioctl IMGIOC_SETMODEP");
  if (ret < 0)
    {
      camfw_printf("ERROR: ioctl IMGIOC_SETMODEP %d.\n", ret);
      return ret;
    }

  memcpy(&camfw_mng.cap_param[p->mode], &p->param, sizeof(CamfwCapParam_t));

  if (camfw_mng.imgsns_state == CAMFW_STATE_SLEEP)
    {
      return ret;
    }

  DBG_TIME_START();
  ret = ioctl(camfw_mng.fd, IMGIOC_MONIREF, 0);
  DBG_TIME_STOP("ioctl IMGIOC_MONIREF");
  if (ret < 0)
    {
      camfw_printf("ERROR: ioctl IMGIOC_MONIREF %d.\n", ret);
    }

  return ret;
}

static int camfw_SetImgSnsCropOff(void)
{
  int idx;
  int ret;

  for (idx = 0; idx < CAMFW_EZOOM_REGNUM; idx++)
    {
      ret = ioctl(camfw_mng.fd,
                  IMGIOC_WRITEREG,
                  (unsigned long)&camfw_ezoom_regs[idx]);
      if (ret < 0)
        {
          break;
        }
    }

  return ret;
}

static int camfw_SetImgSnsCrop(CamfwCapParam_t *param, CamfwCrop_t *crop)
{
  int idx;
  int ret;
  uint16_t px_offset;
  uint16_t ezoom_val[CAMFW_EZOOM_REGNUM];
  uint16_t ezoom_mag_subsmpl[CAMFW_RESOLUTION_MAX] =
  {
    1024,   /* QVGA     : x4  */
     512,   /* VGA      : x2  */
     256,   /* Quad-VGA : x1  */
     256,   /* HD       : x1  */
       0,   /* 3M       : invalid  */
       0,   /* 5M       : invalid  */
  };
  uint16_t ezoom_mag_fllpx[CAMFW_RESOLUTION_MAX] =
  {
    2048,   /* QVGA     : x8  */
    1024,   /* VGA      : x4  */
     512,   /* Quad-VGA : x2  */
     512,   /* HD       : x2  */
     256,   /* 3M       : x1  */
     256    /* 5M       : x1  */
  };
  isx012_reg_t reg;

  camfw_printf("camfw_SetImgSnsCrop :framerate=%d, resolution=%d\n",
                param->framerate, param->resolution);

  if ((param->framerate >= CAMFW_15FPS) && (param->framerate <= CAMFW_5FPS))
    {
      ezoom_val[CAMFW_EZOOM_MAG] = ezoom_mag_fllpx[param->resolution];
      px_offset = CAMFW_EZOOM_OFFSET_PX;
    }
  else if (param->framerate == CAMFW_30FPS)
    {
      if ((param->resolution == CAMFW_5M) || (param->resolution == CAMFW_3M))
        {
          return 0;
        }
      ezoom_val[CAMFW_EZOOM_MAG] = ezoom_mag_subsmpl[param->resolution];
      px_offset = (CAMFW_EZOOM_OFFSET_PX << 1);
    }
  else
    {
      return 0;
    }

  ezoom_val[CAMFW_EZOOM_OFFSET_X] = crop->x_offset * px_offset;
  ezoom_val[CAMFW_EZOOM_OFFSET_Y] = -(crop->y_offset) * px_offset;

  camfw_printf("ezoom_val: mag=%d, x_offset=%d, y_offset=%d\n",
                ezoom_val[0], (int16_t)ezoom_val[1], (int16_t)ezoom_val[2]);

  for (idx = 0; idx < CAMFW_EZOOM_REGNUM; idx++)
    {
      reg.regaddr = camfw_ezoom_regs[idx].regaddr;
      reg.regval  = ezoom_val[idx];
      reg.regsize = camfw_ezoom_regs[idx].regsize;

      ret = ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);
      if (ret < 0)
        {
          break;
        }
    }

  return ret;
}

#if 0
/* 1 Frame Capture -> Auto Monitoring mode Setting */
static void  camfw_SetImgSnsCaptureRegister(void)
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
      ret = ioctl(camfw_mng.fd,
                  IMGIOC_WRITEREG,
                  (unsigned long)&cap_regs[idx]);
      if (ret < 0)
        {
          break;
        }
    }
}
#endif

static void camfw_CreateCisifParam(CamfwMode_e mode, cisif_param_t *p)
{
  CamfwImgResolution_e resolution;
  CamfwImgFormat_e format;

  memset(p, 0, sizeof(cisif_param_t));

  resolution = camfw_mng.cap_param[mode].resolution;
  format = camfw_mng.cap_param[mode].format;

  if (format == CAMFW_FORMAT_YUV)
    {
      p->format = FORMAT_CISIF_YUV;
      p->yuv_param.hsize = camfw_rs2sz[resolution].h;
      p->yuv_param.vsize = camfw_rs2sz[resolution].v;
      p->yuv_param.comp_func = camfw_CallbackCisif;
    }
  else
    {
      p->format = FORMAT_CISIF_JPEG;
      p->jpg_param.comp_func = camfw_CallbackCisif;
    }
}

static int camfw_CaptureFrame(CamfwApiCapFrame_t *p)
{
  int ret;
  cisif_sarea_t cis_area;
  cisif_param_t cis_param;

  if (camfw_mng.fd < 0)
    {
      return -ENODEV;
    }

  if ((camfw_mng.imgsns_state == CAMFW_STATE_SLEEP) ||
      (camfw_mng.imgsns_mode == CAMFW_IMGSNS_MODE_CAP))
    {
      return -EBUSY;
    }

  camfw_SetImgSnsCropOff();

  if (p->crop_ctrl == CAMFW_ENABLE)
    {
      DBG_TIME_START();
      camfw_SetImgSnsCrop(&camfw_mng.cap_param[p->mode], &p->crop);
      DBG_TIME_STOP("camfw_SetImgSnsCrop");
    }

#ifdef CAMFW_IMG_TUNING
  camfw_set_gamma_table(camfw_gamma_tble_idx);
#endif /* CAMFW_IMG_TUNING */

  if (p->mode == CAMFW_MODE_CAPTURE)
    {
      camfw_printf("ioctl IMGIOC_SETMODE(CAPTURE) call.\n");
      DBG_TIME_START();
      ret = ioctl(camfw_mng.fd, IMGIOC_SETMODE, MODE_ISX012_CAPTURE);
      DBG_TIME_STOP("ioctl IMGIOC_SETMODE(CAPTURE)");
      if (ret < 0)
        {
          camfw_printf("ERROR: ioctl IMGIOC_SETMODE(CAPTURE):%d\n", ret);
          return ret;
        }
      camfw_mng.imgsns_mode = CAMFW_IMGSNS_MODE_CAP;
    }

  memset(&camfw_cisif_result, 0, sizeof(camfw_cisif_result));

  camfw_CreateCisifParam(p->mode, &cis_param);
  cis_area.strg_addr = (uint8_t *)p->buffer.addr;
  cis_area.strg_size = p->buffer.size;

  DBG_TIME_START();
  if (cis_param.format == FORMAT_CISIF_YUV)
    {
      ret = cxd56_cisifcaptureframe(&cis_param, &cis_area, NULL);
    }
  else
    {
      ret = cxd56_cisifcaptureframe(&cis_param, NULL, &cis_area);
    }

  if (ret != OK)
    {
      camfw_printf("ERROR: cxd56_cisifcaptureframe() %d\n", ret);
      camfw_cisif_result.errint = ret;
      goto exit;
    }

  ret = camfw_twaisem(&camfw_mng.sem_cisifsync, CAMFW_CISIF_TRANSEND_TIMEOUT);
  if (ret != 0)
    {
      camfw_cisif_result.errint = ret;
    }

  DBG_TIME_STOP("cxd56_cisifcaptureframe() -> trans end.");

  if ((camfw_cisif_result.code != 0) || (camfw_cisif_result.errint != 0))
    {
      camfw_printf("ERROR :cisif err = %d\n", camfw_cisif_result.errint);
    }
  else
    {
      camfw_SetFrameInfo(p, &camfw_cisif_result);
      camfw_GetPictureInfo(&p->info.pict_info);
    }

exit:
  if (p->mode == CAMFW_MODE_CAPTURE)
    {
      camfw_printf("ioctl IMGIOC_SETMODE(MONITORING) call.\n");
      DBG_TIME_START();
      ret = ioctl(camfw_mng.fd, IMGIOC_SETMODE, MODE_ISX012_MONITORING);
      DBG_TIME_STOP("ioctl IMGIOC_SETMODE(MONITORING)");
      if (ret < 0)
        {
          camfw_printf("ERROR: ioctl IMGIOC_SETMODE(MONITORING):%d\n", ret);
        }
      else
        {
          camfw_mng.imgsns_mode = CAMFW_IMGSNS_MODE_MONI;
        }
    }

  if (camfw_cisif_result.errint != 0)
    {
      ret = -camfw_cisif_result.errint;
    }

  return ret;
}

static uint32_t camfw_CorrectJpegSize(uint32_t addr, uint32_t size)
{
  uint8_t *jpg;
  uint32_t eoi_offset;

  jpg = (uint8_t *)(addr + size - 1);
  for (eoi_offset=0; eoi_offset < CAMFW_EOI_CORRECT_MAX_SIZE; eoi_offset++)
    {
      if ((*jpg == 0xD9) && (*(jpg-1) == 0xFF))
        {
          break;
        }
      jpg--;
    }

  return (size - eoi_offset);
}

static int camfw_SetFrameInfo(CamfwApiCapFrame_t *p, CamfwCisifResult_t *res)
{
  p->info.mode = p->mode;
  memcpy(&p->info.cap_param,
         &camfw_mng.cap_param[p->mode],
         sizeof(CamfwCapParam_t));

  p->info.out_addr = res->addr;
  p->info.out_size = res->size;

  if (camfw_mng.cap_param[p->mode].format == CAMFW_FORMAT_YUV)
    {
      if (p->mode == CAMFW_MODE_CAPTURE)
        {
          p->info.h_size = camfw_isx.cap_param.yuv_hsize;
          p->info.v_size = camfw_isx.cap_param.yuv_vsize;
        }
      else
        {
          p->info.h_size = camfw_isx.moni_param.yuv_hsize;
          p->info.v_size = camfw_isx.moni_param.yuv_vsize;
        }
    }
  else
    {
      if (res->errint == 0)
        {
          p->info.out_size = camfw_CorrectJpegSize(res->addr, res->size);
        }

      if (p->mode == CAMFW_MODE_CAPTURE)
        {
          p->info.h_size = camfw_isx.cap_param.jpeg_hsize;
          p->info.v_size = camfw_isx.cap_param.jpeg_vsize;
        }
      else
        {
          p->info.h_size = camfw_isx.moni_param.jpeg_hsize;
          p->info.v_size = camfw_isx.moni_param.jpeg_vsize;
        }
    }

  return 0;
}

static int camfw_GetPictureInfo(CamfwPictureInfo_t *pict_info)
{
  uint32_t shutter_speed = 0;
  CamfwIso_e iso_sens = 0;
  isx012_reg_t reg;
  int ret;

  /* ISO */
  reg.regaddr = camfw_exif_regs[0].addr;
  reg.regsize = camfw_exif_regs[0].regsize;
  ret = ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
  if (ret < 0)
    {
      return ret;
    }
  iso_sens = (CamfwIso_e)reg.regval;

  /* Shutter low */
  reg.regaddr = camfw_exif_regs[1].addr;
  reg.regsize = camfw_exif_regs[1].regsize;
  ret = ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
  if (ret < 0)
    {
      return ret;
    }
  shutter_speed |= (uint32_t)(reg.regval & 0x0000FFFF);

  /* Shutter high */
  reg.regaddr = camfw_exif_regs[2].addr;
  reg.regsize = camfw_exif_regs[2].regsize;
  ret = ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
  if (ret < 0)
    {
      return ret;
    }
  shutter_speed |= (uint32_t)((reg.regval << 16) & 0xFFFF0000);

  pict_info->iso_sens      = iso_sens;
  pict_info->shutter_speed = shutter_speed;

  return 0;
}

static int camfw_SetImgSnsParam(CamfwApiSetImgSnsParam_t *p)
{
  isx012_reg_t reg;
  int ret;

  if (camfw_mng.fd < 0)
    {
      return -ENODEV;
    }

  if (camfw_mng.imgsns_mode != CAMFW_IMGSNS_MODE_MONI)
    {
      return -EBUSY;
    }

  if (p->param.id >= CAMFW_PARAM_ID_MAX)
    {
      return -EINVAL;
    }

  switch (p->param.id)
  {
    case CAMFW_PARAM_ID_COLOR:
      reg.regval = (uint16_t)p->param.val.color_mode;
      break;

    case CAMFW_PARAM_ID_ISO:
      reg.regval = (uint16_t)p->param.val.iso;
      break;

    case CAMFW_PARAM_ID_SHUTTER:
      reg.regval = p->param.val.shutter;
      break;

    case CAMFW_PARAM_ID_EV:
      if (p->param.val.ev >= CAMFW_EV_MAX)
        {
          return -EINVAL;
        }
      reg.regval = (uint16_t)p->param.val.ev;
      break;

    case CAMFW_PARAM_ID_BRIGHTNESS:
      reg.regval = (uint16_t)p->param.val.brightness;
      break;

    case CAMFW_PARAM_ID_CONTRAST:
      reg.regval = (uint16_t)p->param.val.contrast;
      break;

    case CAMFW_PARAM_ID_JPEG_QUALITIY:
      reg.regval = (uint16_t)p->param.val.jpeg_qualitiy;
      break;

    case CAMFW_PARAM_ID_YGAMMA:
      if (p->param.val.ygamma >= CAMFW_YGAMMA_MAX)
        {
          return -EINVAL;
        }
      reg.regval = (uint16_t)p->param.val.ygamma;
      break;

    case CAMFW_PARAM_ID_AWB:
      if (p->param.val.awb >= CAMFW_AWB_MAX)
        {
          return -EINVAL;
        }
      reg.regval = (uint16_t)camfw_convawb[p->param.val.awb];
      break;

    case CAMFW_PARAM_ID_PHOTOMETRY:
      if (p->param.val.photometry >= CAMFW_PHOTOMETRY_MAX)
        {
          return -EINVAL;
        }
      reg.regval = (uint16_t)p->param.val.photometry;
      break;

    default:
      return -EINVAL;
  }

  reg.regaddr = camfw_set_imgsns_regs[p->param.id].addr;
  reg.regsize = camfw_set_imgsns_regs[p->param.id].regsize;
  ret = ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);
  if (ret < 0)
    {
      return ret;
    }

  if (camfw_mng.imgsns_state == CAMFW_STATE_SLEEP)
    {
      return ret;
    }

  DBG_TIME_START();
  ret = ioctl(camfw_mng.fd, IMGIOC_MONIREF, 0);
  DBG_TIME_STOP("ioctl IMGIOC_MONIREF");
  if (ret < 0)
    {
      camfw_printf("ERROR: ioctl IMGIOC_MONIREF %d.\n", ret);
    }

  return ret;
}

static int camfw_SetImgSnsParamAll(CamfwApiSetImgSnsParamAll_t *p)
{
  isx012_reg_t reg;
  CamfwImgSnsParamId_e idx;
  int ret;

  if (camfw_mng.fd < 0)
    {
      return -ENODEV;
    }

  if (camfw_mng.imgsns_mode != CAMFW_IMGSNS_MODE_MONI)
    {
      return -EBUSY;
    }

  for (idx = CAMFW_PARAM_ID_COLOR; idx < CAMFW_PARAM_ID_MAX; idx++)
    {
      switch (idx)
        {
          case CAMFW_PARAM_ID_COLOR:
            reg.regval = (uint16_t)p->param.color_mode;
            break;

          case CAMFW_PARAM_ID_ISO:
            reg.regval = (uint16_t)p->param.iso;
            break;

          case CAMFW_PARAM_ID_SHUTTER:
            reg.regval = p->param.shutter;
            break;

          case CAMFW_PARAM_ID_EV:
            if (p->param.ev >= CAMFW_EV_MAX)
              {
                return -EINVAL;
              }
            reg.regval = (uint16_t)p->param.ev;
            break;

          case CAMFW_PARAM_ID_BRIGHTNESS:
            reg.regval = (uint16_t)p->param.brightness;
            break;

          case CAMFW_PARAM_ID_CONTRAST:
            reg.regval = (uint16_t)p->param.contrast;
            break;

          case CAMFW_PARAM_ID_JPEG_QUALITIY:
            reg.regval = (uint16_t)p->param.jpeg_qualitiy;
            break;

          case CAMFW_PARAM_ID_YGAMMA:
            if (p->param.ygamma >= CAMFW_YGAMMA_MAX)
              {
                return -EINVAL;
              }
            reg.regval = (uint16_t)p->param.ygamma;
            break;

          case CAMFW_PARAM_ID_AWB:
            if (p->param.awb >= CAMFW_AWB_MAX)
              {
                return -EINVAL;
              }
            reg.regval = (uint16_t)camfw_convawb[p->param.awb];
            break;

          case CAMFW_PARAM_ID_PHOTOMETRY:
            if (p->param.photometry >= CAMFW_PHOTOMETRY_MAX)
              {
                return -EINVAL;
              }
            reg.regval = (uint16_t)p->param.photometry;
            break;

          default:
            break;
        }

      reg.regaddr = camfw_set_imgsns_regs[idx].addr;
      reg.regsize = camfw_set_imgsns_regs[idx].regsize;
      ret = ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);
      if (ret < 0)
        {
          break;
        }
    }

  if (camfw_mng.imgsns_state == CAMFW_STATE_SLEEP)
    {
      return ret;
    }

  DBG_TIME_START();
  ret = ioctl(camfw_mng.fd, IMGIOC_MONIREF, 0);
  DBG_TIME_STOP("ioctl IMGIOC_MONIREF");
  if (ret < 0)
    {
      camfw_printf("ERROR: ioctl IMGIOC_MONIREF %d.\n", ret);
    }

  return ret;
}

static int camfw_WriteImgSnsRegister(CamfwApiImgSnsReg_t *p)
{
  isx012_reg_t reg;
  int ret;

  if (camfw_mng.fd < 0)
    {
      return -ENODEV;
    }

  reg.regaddr = p->addr;
  reg.regval  = p->val;
  reg.regsize = p->regsize;

  DBG_TIME_START();
  ret = ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);
  DBG_TIME_STOP("ioctl IMGIOC_WRITEREG");

  return ret;
}

static int camfw_ReadImgSnsRegister(CamfwApiImgSnsReg_t *p)
{
  isx012_reg_t reg;
  int ret;

  if (camfw_mng.fd < 0)
    {
      return -ENODEV;
    }

  reg.regaddr = p->addr;
  reg.regsize = p->regsize;
  reg.regval  = 0;

  DBG_TIME_START();
  ret = ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
  DBG_TIME_STOP("ioctl IMGIOC_READREG");
  if (ret < 0)
    {
      return ret;
    }

  p->val = reg.regval;

  return 0;
}

static int camfw_WaitAutoProcessEnd(void)
{
  isx012_reg_t reg;
  uint32_t time = 0;
  int ret;
  int ae_fin = 0;
  reg.regaddr = CAMFW_ISX012_HALF_MOVE_STS_REG;
  reg.regsize = 1;
  reg.regval  = 0;

  while(time < CAMFW_ISX012_HALFREL_TIMEOUT)
    {
      ret = ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
      if (ret < 0)
        {
          return ret;
        }

      reg.regval &= (CAMFW_ISX012_MOVE_AE_F | CAMFW_ISX012_MOVE_AWB_F);
      if (!(reg.regval & CAMFW_ISX012_MOVE_AE_F) && (ae_fin == 0))
        {
          camfw_printf("AE  end(%02X).\n", reg.regval);
          ae_fin = 1;
        }

      if (!(reg.regval & CAMFW_ISX012_MOVE_AWB_F))
        {
          camfw_printf("AWB end(%02X).\n", reg.regval);
        }

      if (reg.regval == 0)
        {
          return 0;
        }

      usleep(CAMFW_ISX012_HALFREL_WAITTIME);
      time += CAMFW_ISX012_HALFREL_WAITTIME;
    }

  return -EBUSY;
}

static int  camfw_GetIntmean(uint16_t *buf, uint16_t *free)
{
#define INTMEAN_00_ADDR     (0x8A88)
#define INTMEAN_FREE_ADDR   (0x8B06)
  isx012_reg_t reg;
  int idx;

  reg.regsize = 2;
  for(idx = 0; idx < CAMFW_AE_WINDOW_MAX; idx++)
    {
      reg.regaddr = INTMEAN_00_ADDR + (idx << 1);
      ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
      *buf = (uint16_t)reg.regval;
      buf++;
    }

  reg.regaddr = INTMEAN_FREE_ADDR;
  ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
  *free = (uint16_t)reg.regval;

  return 0;
}

static int  camfw_DoHalfRelease(CamfwApiDoHalfRelease_t *p)
{
  isx012_reg_t reg;
  uint16_t getval[CAMFW_AE_NOW_REGNUM];
  int idx;
  int ret;

  const CamfwIsx012Reg_t camfw_ae_now_regs[] =
  {
    { 0x01CC, 2 },  /* ERRSCL_NOW     */
    { 0x01D0, 2 },  /* USER_AESCL_NOW */
    { 0x01A5, 1 },  /* USER_GAIN_LEVEL_NOW */
    { 0x01A9, 1 },  /* ERRLEVEL_NOW   */
    { 0x019C, 2 },  /* SHT_TIME_OUT_L */
    { 0x019E, 2 },  /* SHT_TIME_OUT_H */
  };

  const CamfwIsx012Reg_t camfw_awb_now_regs[] =
  {
    { 0x01B6, 2 },  /* RATIO_R_NOW */
    { 0x01B8, 2 },  /* RATIO_B_NOW */
    { 0x01BB, 1 }   /* AWB_STS_NOW */
  };

  if (camfw_mng.fd < 0)
    {
      return -ENODEV;
    }

  if (camfw_mng.imgsns_state == CAMFW_STATE_SLEEP)
    {
      return -EBUSY;
    }

  if (p->cancel == CAMFW_ENABLE)
    {
      if (camfw_mng.imgsns_mode == CAMFW_IMGSNS_MODE_MONI)
        {
          return 0;
        }

      DBG_TIME_START();
      ret = ioctl(camfw_mng.fd, IMGIOC_SETMODE, MODE_ISX012_MONITORING);
      DBG_TIME_STOP("ioctl IMGIOC_SETMODE(MONITORING)");
      if (ret < 0)
        {
          camfw_printf("ERROR: ioctl IMGIOC_SETMODE(MONITORING):%d\n", ret);
          return ret;
        }
      camfw_mng.imgsns_mode  = CAMFW_IMGSNS_MODE_MONI;
    }
  else
    {
      if (camfw_mng.imgsns_mode != CAMFW_IMGSNS_MODE_MONI)
        {
          return -EBUSY;
        }

      camfw_printf("ioctl IMGIOC_SETMODE(HALFRELEASE) call.\n");

#ifdef CAMFW_IMG_TUNING
      camfw_set_evref_type(camfw_evref_type_idx);
      camfw_evref_type_idx++;
      if (camfw_evref_type_idx == EVREF_TYPE_NUM)
        {
          camfw_evref_type_idx = 0;
          camfw_gamma_tble_idx++;
          if (camfw_gamma_tble_idx == GAMMA_TABLE_NUM)
            {
              camfw_gamma_tble_idx = 0;
            }
        }
#endif /* CAMFW_IMG_TUNING */

      DBG_TIME_START();
      ret = ioctl(camfw_mng.fd, IMGIOC_SETMODE, MODE_ISX012_HALFRELEASE);
      DBG_TIME_STOP("ioctl IMGIOC_SETMODE(HALFRELEASE)");
      if (ret < 0)
        {
          camfw_printf("ERROR: ioctl IMGIOC_SETMODE(HALFRELEASE):%d\n", ret);
          return ret;
        }

      /* wait for AUTO process end. */
      DBG_TIME_START();
      ret = camfw_WaitAutoProcessEnd();
      DBG_TIME_STOP("camfw_WaitAutoProcessEnd()");
      if (ret != 0)
        {
          camfw_printf("ERROR: camfw_WaitAutoProcessEnd() timeout:%d\n", ret);
          ioctl(camfw_mng.fd, IMGIOC_SETMODE, MODE_ISX012_MONITORING);
          return ret;
      }

      for(idx = 0; idx < CAMFW_AE_NOW_REGNUM; idx++)
        {
          reg.regaddr = camfw_ae_now_regs[idx].addr;
          reg.regsize = camfw_ae_now_regs[idx].regsize;
          ret = ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
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

      for(idx = 0; idx < CAMFW_AWB_NOW_REGNUM; idx++)
        {
          reg.regaddr = camfw_awb_now_regs[idx].addr;
          reg.regsize = camfw_awb_now_regs[idx].regsize;
          ret = ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
          if (ret < 0)
            {
              return ret;
            }
          getval[idx] = reg.regval;
        }

      p->info.awb.ratio_r = getval[0];
      p->info.awb.ratio_b = getval[1];
      p->info.awb.awb_sts = (uint8_t)getval[2];

      camfw_GetIntmean(&p->info.intmean[0], &p->info.intmean_free);
      camfw_show_ae_dependparam();

      camfw_mng.imgsns_mode  = CAMFW_IMGSNS_MODE_HALFREL;

    }

  return 0;
}

static int  camfw_GetAutoParam(CamfwApiGetAutoParam_t *p)
{
  isx012_reg_t reg;
  uint16_t getval[CAMFW_AE_AUTO_REGNUM];
  int idx;
  int ret;

  const CamfwIsx012Reg_t camfw_ae_auto_regs[] =
  {
    { 0x01CA, 2 },  /* ERRSCL_AUTO     */
    { 0x01CE, 2 },  /* USER_AESCL_AUTO */
    { 0x01A4, 1 },  /* USER_GAIN_LEVEL_AUTO */
    { 0x01A8, 1 },  /* ERRLEVEL_AUTO   */
    { 0x01A0, 2 },  /* SHT_TIME_AUTO_L */
    { 0x01A2, 2 },  /* SHT_TIME_AUTO_H */
  };

  const CamfwIsx012Reg_t camfw_awb_auto_regs[] =
  {
    { 0x01B2, 2 },  /* RATIO_R_AUTO */
    { 0x01B4, 2 },  /* RATIO_B_AUTO */
    { 0x01BA, 1 }   /* AWB_STS_AUTO */
  };

  if (camfw_mng.fd < 0)
    {
      return -ENODEV;
    }

  if ((camfw_mng.imgsns_state == CAMFW_STATE_SLEEP) ||
      (camfw_mng.imgsns_mode  != CAMFW_IMGSNS_MODE_MONI))
    {
      return -EBUSY;
    }

  for(idx = 0; idx < CAMFW_AE_AUTO_REGNUM; idx++)
    {
      reg.regaddr = camfw_ae_auto_regs[idx].addr;
      reg.regsize = camfw_ae_auto_regs[idx].regsize;
      ret = ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
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

  for(idx = 0; idx < CAMFW_AWB_AUTO_REGNUM; idx++)
    {
      reg.regaddr = camfw_awb_auto_regs[idx].addr;
      reg.regsize = camfw_awb_auto_regs[idx].regsize;
      ret = ioctl(camfw_mng.fd, IMGIOC_READREG, (unsigned long)&reg);
      if (ret < 0)
        {
          return ret;
        }
      getval[idx] = reg.regval;
    }

  p->info.awb.ratio_r = getval[0];
  p->info.awb.ratio_b = getval[1];
  p->info.awb.awb_sts = (uint8_t)getval[2];

  camfw_GetIntmean(&p->info.intmean[0], &p->info.intmean_free);

  return 0;
}

static int camfw_MainTask(int argc, char *argv[])
{
  CamfwApiMsg_t *p;
  unsigned long value = 0;
  struct mq_attr mq_attr;
  mode_t mode = 0666;
  mqd_t mqd_req;
  mqd_t mqd_resp;
  int ret;

  camfw_mng.init = CAMFW_FALSE;

  mq_attr.mq_maxmsg  = 5;
  mq_attr.mq_msgsize = sizeof(CamfwApiMsg_t *);
  mq_attr.mq_flags   = 0;

  mqd_req=mq_open(CAMFW_API_REQ_QUEUE, O_RDONLY | O_CREAT, mode, &mq_attr);
  if (mqd_req < 0)
    {
      camfw_printf("ERROR: Failed to mq_open(CAMFW_API_REQ).\n");
      goto err_exit;
    }

  mq_attr.mq_maxmsg  = 1;
  mq_attr.mq_msgsize = sizeof(value);
  mq_attr.mq_flags   = 0;

  mqd_resp=mq_open(CAMFW_API_RESP_QUEUE, O_WRONLY | O_CREAT, mode, &mq_attr);
  if (mqd_resp < 0)
    {
      camfw_printf("ERROR: Failed to mq_open(CAMFW_API_RESP).\n");
      goto err_exit;
    }

  if (0 != sem_init(&camfw_mng.sem_cisifsync, 0, 0))
    {
      camfw_printf("ERROR: Failed to sem_init(sem_cisifsync).\n");
      goto err_exit;
    }

  camfw_InitInternalParam();
  ret = camfw_InitImageSensor();
  if (ret != 0)
    {
      goto err_exit;
    }

  ret = cxd56_cisifinit();
  if (ret != 0)
    {
      camfw_printf("ERROR: cxd56_cisifinit() %d.\n", ret);
      goto err_exit;
    }

  camfw_mng.init = CAMFW_TRUE;
  if (0 != sem_post(&camfw_mng.sem_apisync))
    {
      camfw_printf("ERROR: Failed to sem_post(sem_apisync).\n");
      goto exit;
    }

  for (;;)
    {
      p = NULL;
      mq_receive(mqd_req, (char *)&p, sizeof(p), NULL);
      if (p == NULL)
        {
          continue;
        }

      switch (p->api_id)
        {
          case CAMFW_APIID_CHG_IMGSNS_STATE:
            ret = camfw_ChangeImgSnsState(&p->u.chg_state);
            break;

          case CAMFW_APIID_CAP_FRAME:
            ret = camfw_CaptureFrame(&p->u.cap_frame);
            break;

          case CAMFW_APIID_SET_CAP_PARAM:
            ret = camfw_SetCaptureParam(&p->u.cap);
            break;

          case CAMFW_APIID_SET_IMGSNS_PARAM:
            ret = camfw_SetImgSnsParam(&p->u.imgsns);
            break;

          case CAMFW_APIID_SET_IMGSNS_PARAM_ALL:
            ret = camfw_SetImgSnsParamAll(&p->u.imgsns_all);
            break;

          case CAMFW_APIID_WRITE_IMGSNS_REG:
            ret = camfw_WriteImgSnsRegister(&p->u.w_reg);
            break;

          case CAMFW_APIID_READ_IMGSNS_REG:
            ret = camfw_ReadImgSnsRegister(&p->u.r_reg);
            break;

          case CAMFW_APIID_DO_HALFRELEASE:
            ret = camfw_DoHalfRelease(&p->u.halfrel);
            break;

          case CAMFW_APIID_GET_AUTO_PARAM:
            ret = camfw_GetAutoParam(&p->u.get_auto);
            break;

          case CAMFW_APIID_CONTI_CAP:
            ret = camfw_ContinuousCapture(&p->u.conti_cap);
            break;

          default:
            camfw_printf("ERROR: Unknown API ID=%d\n", p->api_id);
            continue;
        }

      p->result = ret;
      mq_send(mqd_resp, (const char *)&value, sizeof(value), 0U);

    }

err_exit:
  if (0 != sem_post(&camfw_mng.sem_apisync))
    {
      camfw_printf("ERROR: Failed to sem_post(sem_apisync).\n");
    }

exit:
  mq_close(mqd_req);
  mq_close(mqd_resp);
  sem_destroy(&camfw_mng.sem_cisifsync);
  close(camfw_mng.fd);
  camfw_mng.init = CAMFW_FALSE;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int camfw_init(void)
{
  int ret = ERROR;

  if (camfw_mng.init)
    {
      return 0;
    }

  if (0 != sem_init(&camfw_mng.sem_apisync, 0, 0))
    {
      return ERROR;
    }

  camfw_mng.pid_main = task_create("camfw_MainTask",
                                    CONFIG_CAMFW_TASK_PRIORITY,
                                    CONFIG_CAMFW_TASK_STKSIZE,
                                    camfw_MainTask, NULL);
  if (camfw_mng.pid_main == ERROR)
    {
      goto err_exit;
    }

  if (0 !=sem_wait(&camfw_mng.sem_apisync))
    {
      goto err_exit;
    }

  if (!camfw_mng.init)
    {
      goto err_exit;
    }

  ret = 0;

err_exit:
  sem_destroy(&camfw_mng.sem_apisync);

  return ret;
}

int camfw_change_imgsns_state(CamfwImgSnsState_e state)
{
  CamfwApiChgImgSnsState_t p;

  if (state >= CAMFW_STATE_MAX)
    {
      return -EINVAL;
    }

  p.state = state;

  return camfw_SendMessage(CAMFW_APIID_CHG_IMGSNS_STATE,
                           &p,
                           sizeof(CamfwApiChgImgSnsState_t));
}

int camfw_set_capture_param(CamfwMode_e mode, CamfwCapParam_t *cap_param)
{
  CamfwApiSetCapParam_t p;

  if ((mode != CAMFW_MODE_CAPTURE) && (mode != CAMFW_MODE_MONITORING))
    {
      return -EINVAL;
    }

  if (cap_param == NULL) 
    {
      return -EINVAL;
    }

  if ((cap_param->resolution < CAMFW_QVGA) ||
      (cap_param->resolution >= CAMFW_RESOLUTION_MAX) ||
      (cap_param->format < CAMFW_FORMAT_YUV) ||
      (cap_param->format >= CAMFW_FORMAT_MAX) ||
      (cap_param->framerate < CAMFW_120FPS) ||
      (cap_param->framerate >= CAMFW_FRAME_RATE_MAX))
    {
      return -EINVAL;
    }

  p.mode = mode;
  memcpy(&p.param, cap_param, sizeof(CamfwCapParam_t));

  return camfw_SendMessage(CAMFW_APIID_SET_CAP_PARAM,
                           &p,
                           sizeof(CamfwApiSetCapParam_t));
}

int camfw_capture_frame(
  CamfwMode_e mode,
  CamfwBuffer_t *buffer,
  CamfwCrop_t *crop,
  CamfwCapFrameInfo_t *info)
{
  CamfwApiCapFrame_t p;
  int ret;

  if ((mode != CAMFW_MODE_CAPTURE) && (mode != CAMFW_MODE_MONITORING))
    {
      return -EINVAL;
    }

  if ((buffer == NULL) || (info == NULL))
    {
      return -EINVAL;
    }

  if ((CAMFW_ALING32_CHECK(buffer->addr) != 0) ||
      (CAMFW_ALING32_CHECK(buffer->size) != 0))
    {
      return -EINVAL;
    }

  p.mode = mode;
  p.buffer.addr = buffer->addr;
  p.buffer.size = buffer->size;
  memset(&p.info, 0, sizeof(CamfwCapFrameInfo_t));

  if (crop == NULL)
    {
      p.crop_ctrl = CAMFW_DISABLE;
    }
  else
    {
      p.crop_ctrl = CAMFW_ENABLE;
      p.crop.x_offset = crop->x_offset;
      p.crop.y_offset = crop->y_offset;
    }

  ret = camfw_SendMessage(CAMFW_APIID_CAP_FRAME,
                          &p,
                          sizeof(CamfwApiCapFrame_t));

  memcpy(info, &p.info, sizeof(CamfwCapFrameInfo_t));

  return ret;
}

int camfw_set_imgsns_param(CamfwImgSnsParam_t *param)
{
  CamfwApiSetImgSnsParam_t p;

  if (param == NULL)
    {
      return -EINVAL;
    }

  if ((param->id < CAMFW_PARAM_ID_COLOR) ||
      (param->id >= CAMFW_PARAM_ID_MAX))
    {
      return -EINVAL;
    }

  memcpy(&p.param, param, sizeof(CamfwImgSnsParam_t));

  return camfw_SendMessage(CAMFW_APIID_SET_IMGSNS_PARAM,
                           &p,
                           sizeof(CamfwApiSetImgSnsParam_t));
}

int camfw_set_imgsns_param_all(CamfwImgSnsParamAll_t *param)
{
  CamfwApiSetImgSnsParamAll_t p;

  if (param == NULL)
    {
      return -EINVAL;
    }

  memcpy(&p.param, param, sizeof(CamfwApiSetImgSnsParamAll_t));

  return camfw_SendMessage(CAMFW_APIID_SET_IMGSNS_PARAM_ALL,
                           &p,
                           sizeof(CamfwApiSetImgSnsParamAll_t));
}

int camfw_write_imgsns_register(
  uint16_t addr, uint16_t regsize, uint16_t value)
{
  CamfwApiImgSnsReg_t p;

  if ((regsize != 1) && (regsize != 2))
    {
      return -EINVAL;
    }

  p.addr = addr;
  p.regsize = regsize;
  p.val = value;

  return camfw_SendMessage(CAMFW_APIID_WRITE_IMGSNS_REG,
                           &p,
                           sizeof(CamfwApiImgSnsReg_t));
}

int camfw_read_imgsns_register(
  uint16_t addr, uint16_t regsize, uint16_t *value)
{
  CamfwApiImgSnsReg_t p;
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

  ret = camfw_SendMessage(CAMFW_APIID_READ_IMGSNS_REG,
                          &p,
                          sizeof(CamfwApiImgSnsReg_t));
  if (ret == 0)
    {
      *value = p.val;
    }

  return ret;
}

int camfw_do_halfrelease(CamfwAutoInfo_t *info, CamfwCtrl_e cancel)
{
  CamfwApiDoHalfRelease_t p;
  int ret;

  if (info == NULL)
    {
      return -EINVAL;
    }

  p.cancel = cancel;
  ret = camfw_SendMessage(CAMFW_APIID_DO_HALFRELEASE,
                          &p,
                          sizeof(CamfwApiDoHalfRelease_t));
  if ((ret == 0) && (cancel != CAMFW_ENABLE))
    {
      memcpy(info, &p.info, sizeof(CamfwAutoInfo_t));
    }

  return ret;
}

int camfw_get_auto_param(CamfwAutoInfo_t *info)
{
  CamfwApiGetAutoParam_t p;
  int ret;

  if (info == NULL)
    {
      return -EINVAL;
    }

  memset(&p, 0, sizeof(CamfwApiGetAutoParam_t));
  ret = camfw_SendMessage(CAMFW_APIID_GET_AUTO_PARAM,
                          &p,
                          sizeof(CamfwApiGetAutoParam_t));
  if (ret == 0)
    {
      memcpy(info, &p.info, sizeof(CamfwAutoInfo_t));
    }

  return ret;
}

int camfw_continuous_capture(CamfwContiParam_t *param,
                        CamfwBuffer_t *buffer,
                        CamfwCrop_t *crop,
                        CamfwContiCapInfo_t *info)
{
  CamfwApiContiCap_t p;
  int ret;

  if ((param == NULL) || (buffer == NULL) || (info == NULL))
    {
      return -EINVAL;
    }

  if ((param->mode != CAMFW_MODE_CAPTURE) ||
      (param->num > CAMFW_CONTI_CAPNUM_MAX))
    {
      return -EINVAL;
    }

  if ((CAMFW_ALING32_CHECK(buffer->addr) != 0) ||
      (CAMFW_ALING32_CHECK(buffer->size) != 0))

  {
      return -EINVAL;
    }

  p.mode = param->mode;
  p.capnum = param->num;
  p.interval = param->interval;
  p.buffer.addr = buffer->addr;
  p.buffer.size = buffer->size;
  memset(&p.info, 0, sizeof(CamfwContiCapInfo_t));

  if (crop == NULL)
    {
      p.crop_ctrl = CAMFW_DISABLE;
    }
  else
    {
      p.crop_ctrl = CAMFW_ENABLE;
      p.crop.x_offset = crop->x_offset;
      p.crop.y_offset = crop->y_offset;
    }

  ret = camfw_SendMessage(CAMFW_APIID_CONTI_CAP,
                          &p,
                          sizeof(CamfwApiContiCap_t));

  memcpy(info, &p.info, sizeof(CamfwContiCapInfo_t));

  return ret;
}

static int camfw_ContinuousCapture(CamfwApiContiCap_t *p)
{
  cisif_sarea_t cis_area;
  cisif_param_t cis_param;
  uint32_t conti_capnum = 0;
  uint32_t correct_size = 0;
  int conti_end = CAMFW_FALSE;
  int ret;
#ifdef CAMFW_CONTI_BRACKET
  isx012_reg_t reg;
#endif /* CAMFW_CONTI_BRACKET */

  if (camfw_mng.fd < 0)
    {
      return -ENODEV;
    }

  if ((camfw_mng.imgsns_state == CAMFW_STATE_SLEEP) ||
      (camfw_mng.imgsns_mode == CAMFW_IMGSNS_MODE_CAP))
    {
      return -EBUSY;
    }

  if (camfw_mng.cap_param[p->mode].format != CAMFW_FORMAT_JPEG)
    {
      return -EINVAL;
    }

  camfw_SetImgSnsCropOff();

  if (p->crop_ctrl == CAMFW_ENABLE)
    {
      DBG_TIME_START();
      camfw_SetImgSnsCrop(&camfw_mng.cap_param[p->mode], &p->crop);
      DBG_TIME_STOP("camfw_SetImgSnsCrop");
    }

#ifdef CAMFW_CONTI_BRACKET
  /* CAP_HALF_AE_CTRL : HAFREL=HIGHSPEED, CAP=AUTO  */
  reg.regval  = 0x7;
  reg.regaddr = 0x0181;
  reg.regsize = 1;
  ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);
#endif /* CAMFW_CONTI_BRACKET */

  camfw_printf("ioctl IMGIOC_SETMODE(CAPTURE) call.\n");
  DBG_TIME_START();
  ret = ioctl(camfw_mng.fd, IMGIOC_SETMODE, MODE_ISX012_CAPTURE);
  DBG_TIME_STOP("ioctl IMGIOC_SETMODE(CAPTURE)");
  if (ret < 0)
    {
      camfw_printf("ERROR: ioctl IMGIOC_SETMODE(CAPTURE):%d\n", ret);
      return ret;
    }

  camfw_mng.imgsns_mode = CAMFW_IMGSNS_MODE_CAP;
  memset(&camfw_cisif_result, 0, sizeof(camfw_cisif_result));

  camfw_CreateCisifParam(p->mode, &cis_param);
  cis_param.jpg_param.comp_func = camfw_CallbackCisifConti;
  cis_area.strg_addr  = (uint8_t *)p->buffer.addr;
  cis_area.strg_size  = p->buffer.size;

  /* Start CISIF Capture */
  DBG_TIME_START();
  ret = cxd56_cisifcontinuouscapture(&cis_param,
                                     &cis_area,
                                     p->capnum,
                                     p->interval);
  if (ret != OK)
    {
      camfw_printf("ERROR: cxd56_cisifcontinuouscapture() %d\n", ret);
      goto exit;
    }

  while (!conti_end)
    {
      ret = camfw_twaisem(&camfw_mng.sem_cisifsync,
                          CAMFW_CISIF_TRANSEND_TIMEOUT);
      if (ret != 0)
        {
          camfw_cisif_result.errint = ret;
        }

      DBG_TIME_STOP("cxd56_cisifcontinuouscapture() -> trans end.");
      if ((camfw_cisif_result.code != 0) || (camfw_cisif_result.errint != 0))
        {
          camfw_printf("ERROR :cisif err = %d\n", camfw_cisif_result.errint);
          goto exit;
      }

      camfw_GetPictureInfo(&p->info.conti_frame[conti_capnum].pict_info);

      camfw_printf("Out Addr : %08X\n",camfw_cisif_result.addr);
      camfw_printf("Out Size : %08X\n",camfw_cisif_result.size);

      correct_size = camfw_CorrectJpegSize(camfw_cisif_result.addr,
                                           camfw_cisif_result.size);
      p->info.conti_frame[conti_capnum].out_addr = camfw_cisif_result.addr;
      p->info.conti_frame[conti_capnum].out_size = correct_size;

      conti_capnum++;

#ifdef CAMFW_CONTI_BRACKET
      reg.regval = (uint16_t)CAMFW_EV_P2;
      reg.regaddr = camfw_set_imgsns_regs[CAMFW_PARAM_ID_EV].addr;
      reg.regsize = camfw_set_imgsns_regs[CAMFW_PARAM_ID_EV].regsize;
      camfw_printf("IMGIOC_WRITEREG : EV:\n", reg.regval);
      ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);

      reg.regval = (uint16_t)127;
      reg.regaddr = camfw_set_imgsns_regs[CAMFW_PARAM_ID_BRIGHTNESS].addr;
      reg.regsize = camfw_set_imgsns_regs[CAMFW_PARAM_ID_BRIGHTNESS].regsize;
      camfw_printf("IMGIOC_WRITEREG : BRIGHTNESS:\n", reg.regval);
      ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);
#endif /* CAMFW_CONTI_BRACKET */

      if (conti_capnum == p->capnum)
        {
          conti_end = CAMFW_TRUE;
        }

      if (camfw_cisif_result.last_frame)
        {
          conti_end = CAMFW_TRUE;
        }
    }

exit:
  p->info.h_size = camfw_isx.cap_param.jpeg_hsize;
  p->info.v_size = camfw_isx.cap_param.jpeg_vsize;
  p->info.capnum = conti_capnum;

#ifdef CAMFW_CONTI_BRACKET
  /* CAP_HALF_AE_CTRL : HAFREL=HIGHSPEED, CAP=OFF  */
  reg.regval  = 0x1;
  reg.regaddr = 0x0181;
  reg.regsize = 1;
  ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);

  reg.regval = (uint16_t)CAMFW_EV_OFF;
  reg.regaddr = camfw_set_imgsns_regs[CAMFW_PARAM_ID_EV].addr;
  reg.regsize = camfw_set_imgsns_regs[CAMFW_PARAM_ID_EV].regsize;
  camfw_printf("IMGIOC_WRITEREG : EV:\n", reg.regval);
  ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);

  reg.regval = (uint16_t)0;
  reg.regaddr = camfw_set_imgsns_regs[CAMFW_PARAM_ID_BRIGHTNESS].addr;
  reg.regsize = camfw_set_imgsns_regs[CAMFW_PARAM_ID_BRIGHTNESS].regsize;
  camfw_printf("IMGIOC_WRITEREG : BRIGHTNESS:\n", reg.regval);
  ioctl(camfw_mng.fd, IMGIOC_WRITEREG, (unsigned long)&reg);
#endif /* CAMFW_CONTI_BRACKET */

  camfw_printf("ioctl IMGIOC_SETMODE(MONITORING) call.\n");
  DBG_TIME_START();
  ret = ioctl(camfw_mng.fd, IMGIOC_SETMODE, MODE_ISX012_MONITORING);
  DBG_TIME_STOP("ioctl IMGIOC_SETMODE(MONITORING)");
  if (ret < 0)
    {
      camfw_printf("ERROR: ioctl IMGIOC_SETMODE(MONITORING):%d\n", ret);
    }
  else
    {
      camfw_mng.imgsns_mode = CAMFW_IMGSNS_MODE_MONI;
    }

  if (camfw_cisif_result.errint != 0)
    {
      ret = -camfw_cisif_result.errint;
    }

  return ret;
}

