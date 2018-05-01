/****************************************************************************
 * bsp/include/nuttx/video/video.h
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

#ifndef __BSP_INCLUDE_NUTTX_VIDEO_VIDEO_H
#define __BSP_INCLUDE_NUTTX_VIDEO_VIDEO_H

/****************************************************************************
 * Included Files
 ****************************************************************************/
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/
#define CONFIG_VIDEO_TASK_PRIORITY    100
#define CONFIG_VIDEO_TASK_STKSIZE     2048

#define VIDEO_JPEG_QUALITIY_MAX       100
#define VIDEO_JPEG_QUALITIY_MIN       1

#define VIDEO_CROP_30FPS_VGA_X_MAX    (320)
#define VIDEO_CROP_30FPS_VGA_X_MIN    (-320)
#define VIDEO_CROP_30FPS_VGA_Y_MAX    (240)
#define VIDEO_CROP_30FPS_VGA_Y_MIN    (-240)
#define VIDEO_CROP_30FPS_QVGA_X_MAX   (480)
#define VIDEO_CROP_30FPS_QVGA_X_MIN   (-480)
#define VIDEO_CROP_30FPS_QVGA_Y_MAX   (360)
#define VIDEO_CROP_30FPS_QVGA_Y_MIN   (-360)

#define VIDEO_CROP_FLLPX_HD_X_MAX     (640)
#define VIDEO_CROP_FLLPX_HD_X_MIN     (-640)
#define VIDEO_CROP_FLLPX_HD_Y_MAX     (600)
#define VIDEO_CROP_FLLPX_HD_Y_MIN     -600
#define VIDEO_CROP_FLLPX_VGA_X_MAX    (960)
#define VIDEO_CROP_FLLPX_VGA_X_MIN    (-960)
#define VIDEO_CROP_FLLPX_VGA_Y_MAX    (720)
#define VIDEO_CROP_FLLPX_VGA_Y_MIN    (-720)
#define VIDEO_CROP_FLLPX_QVGA_X_MAX   (1120)
#define VIDEO_CROP_FLLPX_QVGA_X_MIN   (-1120)
#define VIDEO_CROP_FLLPX_QVGA_Y_MAX   (800)
#define VIDEO_CROP_FLLPX_QVGA_Y_MIN   (-800)

#define VIDEO_CONTI_CAPNUM_MAX        (5)
#define VIDEO_AE_WINDOW_MAX           (63)
/****************************************************************************
 * Public Types
 ****************************************************************************/
typedef enum
{
  VIDEO_STATE_ACTIVE = 0,
  VIDEO_STATE_SLEEP,
  VIDEO_STATE_POWOFF,
  VIDEO_STATE_POWON,
  VIDEO_STATE_MAX
} video_img_sns_state_e;

typedef enum
{
  VIDEO_MODE_CAPTURE = 0,
  VIDEO_MODE_MONITORING,
  VIDEO_MODE_MAX
} video_mode_e;

typedef enum
{
  VIDEO_FORMAT_YUV = 0,
  VIDEO_FORMAT_JPEG,
  VIDEO_FORMAT_MAX
} video_img_format_e;

typedef enum
{
  VIDEO_QVGA = 0,
  VIDEO_VGA,
  VIDEO_QUADVGA,
  VIDEO_HD,
  VIDEO_3M,
  VIDEO_5M,
  VIDEO_RESOLUTION_MAX
} video_img_resolution_e;

typedef enum
{
  VIDEO_120FPS = 0,
  VIDEO_60FPS,
  VIDEO_30FPS,
  VIDEO_15FPS,
  VIDEO_7_5FPS,
  VIDEO_5FPS,
  VIDEO_FRAME_RATE_MAX
} video_frame_rate_e;

typedef enum
{
  VIDEO_PARAM_ID_COLOR = 0,
  VIDEO_PARAM_ID_ISO,
  VIDEO_PARAM_ID_SHUTTER,
  VIDEO_PARAM_ID_EV,
  VIDEO_PARAM_ID_BRIGHTNESS,
  VIDEO_PARAM_ID_CONTRAST,
  VIDEO_PARAM_ID_JPEG_QUALITIY,
  VIDEO_PARAM_ID_YGAMMA,
  VIDEO_PARAM_ID_AWB,
  VIDEO_PARAM_ID_PHOTOMETRY,
  VIDEO_PARAM_ID_MAX
} video_img_sns_param_id_e;

typedef enum
{
  VIDEO_COLOR_NORMAL = 0,
  VIDEO_COLOR_MONO = 4
} video_color_mode_e;

typedef enum
{
  VIDEO_ISO_NONE = 0,
  VIDEO_ISO25,
  VIDEO_ISO32,
  VIDEO_ISO40,
  VIDEO_ISO50,
  VIDEO_ISO64,
  VIDEO_ISO80,
  VIDEO_ISO100,
  VIDEO_ISO125,
  VIDEO_ISO160,
  VIDEO_ISO200,
  VIDEO_ISO250,
  VIDEO_ISO320,
  VIDEO_ISO400,
  VIDEO_ISO500,
  VIDEO_ISO640,
  VIDEO_ISO800,
  VIDEO_ISO1000,
  VIDEO_ISO1250,
  VIDEO_ISO1600,
  VIDEO_ISO_MAX
} video_iso_e;

typedef enum
{
  VIDEO_EV_M2 = -6,
  VIDEO_EV_M5_3,
  VIDEO_EV_M4_3,
  VIDEO_EV_M1,
  VIDEO_EV_M2_3,
  VIDEO_EV_M1_3,
  VIDEO_EV_OFF,
  VIDEO_EV_P1_3,
  VIDEO_EV_P2_3,
  VIDEO_EV_P1,
  VIDEO_EV_P4_3,
  VIDEO_EV_P5_3,
  VIDEO_EV_P2,
  VIDEO_EV_MAX
} video_ev_e;

typedef enum
{
  VIDEO_YGAMMA_AUTO = 0,
  VIDEO_YGAMMA_OFF,
  VIDEO_YGAMMA_MAX
} video_ygamma_e;

typedef enum
{
  VIDEO_AWB_ATM = 0,
  VIDEO_AWB_CLEARWEATHER,
  VIDEO_AWB_SHADE,
  VIDEO_AWB_CLOUDYWEATHER,
  VIDEO_AWB_FLUORESCENTLIGHT,
  VIDEO_AWB_LIGHTBULB,
  VIDEO_AWB_MAX
} video_awb_e;

typedef enum
{
  VIDEO_PHOTOMETRY_AVERAGE = 0,
  VIDEO_PHOTOMETRY_CENTERWEIGHT,
  VIDEO_PHOTOMETRY_SPOT,
  VIDEO_PHOTOMETRY_MULTIPATTERN,
  VIDEO_PHOTOMETRY_MAX
} video_photometry_e;

typedef enum
{
  VIDEO_DISABLE = 0,
  VIDEO_ENABLE,
  VIDEO_CTRL_MAX
} video_ctrl_e;

typedef struct
{
  video_img_format_e     format;
  video_img_resolution_e resolution;
  video_frame_rate_e     framerate;
} video_cap_param_t;

typedef struct
{
  uint32_t addr;
  uint32_t size;
} video_buffer_t;

typedef struct
{
  int16_t x_offset;
  int16_t y_offset;
} video_crop_t;

typedef struct
{
  uint32_t    shutter_speed; /* units : micro second  */
  video_iso_e iso_sens;
} video_picture_info_t;

typedef struct
{
  video_mode_e         mode;
  video_cap_param_t    cap_param;
  video_picture_info_t pict_info;
  uint32_t             out_addr;
  uint32_t             out_size;
  uint16_t             h_size;
  uint16_t             v_size;
} video_cap_frame_info_t;

typedef struct
{
  video_mode_e mode;
  uint32_t     num;
  uint32_t     interval;
} video_conti_param_t;

typedef struct
{
  video_picture_info_t pict_info;
  uint32_t             out_addr;
  uint32_t             out_size;
} video_conti_frame_t;

typedef struct
{
  video_mode_e        mode;
  video_cap_param_t   cap_param;
  uint16_t            h_size;
  uint16_t            v_size;
  uint32_t            buffer_full;
  uint32_t            capnum;
  video_conti_frame_t conti_frame[VIDEO_CONTI_CAPNUM_MAX];
} video_conti_cap_info_t;

typedef struct
{
  video_img_sns_param_id_e id;
  union
  {
    video_color_mode_e     color_mode;
    video_iso_e            iso;
    uint16_t               shutter;
    video_ev_e             ev;
    int8_t                 brightness;
    uint8_t                contrast;
    uint8_t                jpeg_qualitiy;
    video_ygamma_e         ygamma;
    video_awb_e            awb;
    video_photometry_e     photometry;
  } val;
} video_img_sns_param_t;

typedef struct
{
  video_color_mode_e color_mode;
  video_iso_e        iso;
  uint16_t           shutter;
  video_ev_e         ev;
  int8_t             brightness;
  int8_t             contrast;
  uint8_t            jpeg_qualitiy;
  video_ygamma_e     ygamma;
  video_awb_e        awb;
  video_photometry_e photometry;
} video_img_sns_param_all_t;

typedef struct
{
  int16_t  errscl;
  int16_t  user_aescl;
  uint32_t sht_time;
  int8_t   user_gain_level;
  int8_t   err_level;
} video_auto_ae_info_t;

typedef struct
{
  uint16_t ratio_r;
  uint16_t ratio_b;
  uint8_t  awb_sts;
} video_auto_awb_info_t;

typedef struct
{
  video_auto_ae_info_t  ae;
  video_auto_awb_info_t awb;
  uint16_t              intmean[VIDEO_AE_WINDOW_MAX];
  uint16_t              intmean_free;
} video_auto_info_t;

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
int video_init(void);
int video_change_imgsns_state(video_img_sns_state_e state);
int video_id_set_capture_param(video_mode_e mode, video_cap_param_t *param);
int video_id_capture_frame(video_mode_e mode,
                        video_buffer_t *buffer,
                        video_crop_t *crop,
                        video_cap_frame_info_t *info);
int video_id_continuous_capture(video_conti_param_t *param,
                        video_buffer_t *buffer,
                        video_crop_t *crop,
                        video_conti_cap_info_t *info);
int video_set_imgsns_param(video_img_sns_param_t *param);
int video_set_imgsns_param_all(video_img_sns_param_all_t *param);
int video_write_imgsns_register(uint16_t addr, uint16_t size, uint16_t val);
int video_read_imgsns_register(uint16_t addr, uint16_t size, uint16_t *val);
int video_do_halfrelease(video_auto_info_t *info, video_ctrl_e cancel);
int video_id_get_auto_param(video_auto_info_t *info);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __BSP_INCLUDE_NUTTX_VIDEO_VIDEO_H */
