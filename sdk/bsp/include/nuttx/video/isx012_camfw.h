/****************************************************************************
 * bsp/include/nuttx/video/isx012_camfw.h
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

#ifndef __BSP_INCLUDE_NUTTX_VIDEO__ISX012_CAMFW_H
#define __BSP_INCLUDE_NUTTX_VIDEO__ISX012_CAMFW_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
#define CONFIG_CAMFW_TASK_PRIORITY    100
#define CONFIG_CAMFW_TASK_STKSIZE     2048

#define CAMFW_JPEG_QUALITIY_MAX       100
#define CAMFW_JPEG_QUALITIY_MIN       1

#define CAMFW_CROP_30FPS_VGA_X_MAX    (320)
#define CAMFW_CROP_30FPS_VGA_X_MIN    (-320)
#define CAMFW_CROP_30FPS_VGA_Y_MAX    (240)
#define CAMFW_CROP_30FPS_VGA_Y_MIN    (-240)
#define CAMFW_CROP_30FPS_QVGA_X_MAX   (480)
#define CAMFW_CROP_30FPS_QVGA_X_MIN   (-480)
#define CAMFW_CROP_30FPS_QVGA_Y_MAX   (360)
#define CAMFW_CROP_30FPS_QVGA_Y_MIN   (-360)

#define CAMFW_CROP_FLLPX_HD_X_MAX     (640)
#define CAMFW_CROP_FLLPX_HD_X_MIN     (-640)
#define CAMFW_CROP_FLLPX_HD_Y_MAX     (600)
#define CAMFW_CROP_FLLPX_HD_Y_MIN     -600
#define CAMFW_CROP_FLLPX_VGA_X_MAX    (960)
#define CAMFW_CROP_FLLPX_VGA_X_MIN    (-960)
#define CAMFW_CROP_FLLPX_VGA_Y_MAX    (720)
#define CAMFW_CROP_FLLPX_VGA_Y_MIN    (-720)
#define CAMFW_CROP_FLLPX_QVGA_X_MAX   (1120)
#define CAMFW_CROP_FLLPX_QVGA_X_MIN   (-1120)
#define CAMFW_CROP_FLLPX_QVGA_Y_MAX   (800)
#define CAMFW_CROP_FLLPX_QVGA_Y_MIN   (-800)

#define CAMFW_CONTI_CAPNUM_MAX        (5)
#define CAMFW_AE_WINDOW_MAX           (63)
/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef enum
{
  CAMFW_STATE_ACTIVE = 0,
  CAMFW_STATE_SLEEP,
  CAMFW_STATE_POWOFF,
  CAMFW_STATE_POWON,
  CAMFW_STATE_MAX
} CamfwImgSnsState_e;

typedef enum
{
  CAMFW_MODE_CAPTURE = 0,
  CAMFW_MODE_MONITORING,
  CAMFW_MODE_MAX
} CamfwMode_e;

typedef enum
{
  CAMFW_FORMAT_YUV = 0,
  CAMFW_FORMAT_JPEG,
  CAMFW_FORMAT_MAX
} CamfwImgFormat_e;

typedef enum
{
  CAMFW_QVGA = 0,
  CAMFW_VGA,
  CAMFW_QUADVGA,
  CAMFW_HD,
  CAMFW_3M,
  CAMFW_5M,
  CAMFW_RESOLUTION_MAX
} CamfwImgResolution_e;

typedef enum
{
  CAMFW_120FPS = 0,
  CAMFW_60FPS,
  CAMFW_30FPS,
  CAMFW_15FPS,
  CAMFW_7_5FPS,
  CAMFW_5FPS,
  CAMFW_FRAME_RATE_MAX
} CamfwFrameRate_e;

typedef enum
{
  CAMFW_PARAM_ID_COLOR = 0,
  CAMFW_PARAM_ID_ISO,
  CAMFW_PARAM_ID_SHUTTER,
  CAMFW_PARAM_ID_EV,
  CAMFW_PARAM_ID_BRIGHTNESS,
  CAMFW_PARAM_ID_CONTRAST,
  CAMFW_PARAM_ID_JPEG_QUALITIY,
  CAMFW_PARAM_ID_YGAMMA,
  CAMFW_PARAM_ID_AWB,
  CAMFW_PARAM_ID_PHOTOMETRY,
  CAMFW_PARAM_ID_MAX
} CamfwImgSnsParamId_e;

typedef enum
{
  CAMFW_COLOR_NORMAL = 0,
  CAMFW_COLOR_MONO = 4
} CamfwColorMode_e;

typedef enum
{
  CAMFW_ISO_NONE = 0,
  CAMFW_ISO25,
  CAMFW_ISO32,
  CAMFW_ISO40,
  CAMFW_ISO50,
  CAMFW_ISO64,
  CAMFW_ISO80,
  CAMFW_ISO100,
  CAMFW_ISO125,
  CAMFW_ISO160,
  CAMFW_ISO200,
  CAMFW_ISO250,
  CAMFW_ISO320,
  CAMFW_ISO400,
  CAMFW_ISO500,
  CAMFW_ISO640,
  CAMFW_ISO800,
  CAMFW_ISO1000,
  CAMFW_ISO1250,
  CAMFW_ISO1600,
  CAMFW_ISO_MAX
} CamfwIso_e;

typedef enum
{
  CAMFW_EV_M2 = -6,
  CAMFW_EV_M5_3,
  CAMFW_EV_M4_3,
  CAMFW_EV_M1,
  CAMFW_EV_M2_3,
  CAMFW_EV_M1_3,
  CAMFW_EV_OFF,
  CAMFW_EV_P1_3,
  CAMFW_EV_P2_3,
  CAMFW_EV_P1,
  CAMFW_EV_P4_3,
  CAMFW_EV_P5_3,
  CAMFW_EV_P2,
  CAMFW_EV_MAX
} CamfwEv_e;

typedef enum
{
  CAMFW_YGAMMA_AUTO = 0,
  CAMFW_YGAMMA_OFF,
  CAMFW_YGAMMA_MAX
} CamfwYGamma_e;

typedef enum
{
  CAMFW_AWB_ATM = 0,
  CAMFW_AWB_CLEARWEATHER,
  CAMFW_AWB_SHADE,
  CAMFW_AWB_CLOUDYWEATHER,
  CAMFW_AWB_FLUORESCENTLIGHT,
  CAMFW_AWB_LIGHTBULB,
  CAMFW_AWB_MAX
} CamfwAwb_e;

typedef enum
{
  CAMFW_PHOTOMETRY_AVERAGE = 0,
  CAMFW_PHOTOMETRY_CENTERWEIGHT,
  CAMFW_PHOTOMETRY_SPOT,
  CAMFW_PHOTOMETRY_MULTIPATTERN,
  CAMFW_PHOTOMETRY_MAX
} CamfwPhotometry_e;

typedef enum
{
  CAMFW_DISABLE = 0,
  CAMFW_ENABLE,
  CAMFW_CTRL_MAX
} CamfwCtrl_e;

typedef struct
{
  CamfwImgFormat_e format;
  CamfwImgResolution_e resolution;
  CamfwFrameRate_e framerate;
} CamfwCapParam_t;

typedef struct
{
  uint32_t addr;
  uint32_t size;
} CamfwBuffer_t;

typedef struct
{
  int16_t x_offset;
  int16_t y_offset;
} CamfwCrop_t;

typedef struct
{
  uint32_t shutter_speed; /* units : micro second  */
  CamfwIso_e iso_sens;
} CamfwPictureInfo_t;

typedef struct
{
  CamfwMode_e mode;
  CamfwCapParam_t cap_param;
  CamfwPictureInfo_t pict_info;
  uint32_t out_addr;
  uint32_t out_size;
  uint16_t h_size;
  uint16_t v_size;
} CamfwCapFrameInfo_t;

typedef struct
{
  CamfwMode_e mode;
  uint32_t num;
  uint32_t interval;
} CamfwContiParam_t;

typedef struct
{
  CamfwPictureInfo_t pict_info;
  uint32_t out_addr;
  uint32_t out_size;
} CamfwContiFrame_t;

typedef struct
{
  CamfwMode_e mode;
  CamfwCapParam_t cap_param;
  uint16_t h_size;
  uint16_t v_size;
  uint32_t buffer_full;
  uint32_t capnum;
  CamfwContiFrame_t conti_frame[CAMFW_CONTI_CAPNUM_MAX];
} CamfwContiCapInfo_t;

typedef struct
{
  CamfwImgSnsParamId_e id;
  union
  {
    CamfwColorMode_e color_mode;
    CamfwIso_e iso;
    uint16_t shutter;
    CamfwEv_e ev;
    int8_t brightness;
    uint8_t contrast;
    uint8_t jpeg_qualitiy;
    CamfwYGamma_e ygamma;
    CamfwAwb_e awb;
    CamfwPhotometry_e photometry;
  } val;
} CamfwImgSnsParam_t;

typedef struct
{
  CamfwColorMode_e color_mode;
  CamfwIso_e iso;
  uint16_t shutter;
  CamfwEv_e ev;
  int8_t brightness;
  int8_t contrast;
  uint8_t jpeg_qualitiy;
  CamfwYGamma_e ygamma;
  CamfwAwb_e awb;
  CamfwPhotometry_e photometry;
} CamfwImgSnsParamAll_t;

typedef struct
{
  int16_t errscl;
  int16_t user_aescl;
  uint32_t sht_time;
  int8_t user_gain_level;
  int8_t err_level;
} CamfwAutoAeInfo_t;

typedef struct
{
  uint16_t ratio_r;
  uint16_t ratio_b;
  uint8_t awb_sts;
} CamfwAutoAwbInfo_t;

typedef struct
{
  CamfwAutoAeInfo_t ae;
  CamfwAutoAwbInfo_t awb;
  uint16_t  intmean[CAMFW_AE_WINDOW_MAX];
  uint16_t  intmean_free;
} CamfwAutoInfo_t;

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" 
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int camfw_init(void);
int camfw_change_imgsns_state(CamfwImgSnsState_e state);
int camfw_set_capture_param(CamfwMode_e mode, CamfwCapParam_t *param);
int camfw_capture_frame(CamfwMode_e mode,
                        CamfwBuffer_t *buffer,
                        CamfwCrop_t *crop,
                        CamfwCapFrameInfo_t *info);
int camfw_continuous_capture(CamfwContiParam_t *param,
                        CamfwBuffer_t *buffer,
                        CamfwCrop_t *crop,
                        CamfwContiCapInfo_t *info);
int camfw_set_imgsns_param(CamfwImgSnsParam_t *param);
int camfw_set_imgsns_param_all(CamfwImgSnsParamAll_t *param);
int camfw_write_imgsns_register(uint16_t addr, uint16_t size, uint16_t val);
int camfw_read_imgsns_register(uint16_t addr, uint16_t size, uint16_t *val);
int camfw_do_halfrelease(CamfwAutoInfo_t *info, CamfwCtrl_e cancel);
int camfw_get_auto_param(CamfwAutoInfo_t *info);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BSP_INCLUDE_NUTTX_VIDEO__ISX012_CAMFW_H */
