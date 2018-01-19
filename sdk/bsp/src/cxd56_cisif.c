/****************************************************************************
 * arch/arm/src/cxd56xx/cxd56_cisif.c
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

#include <string.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "up_arch.h"

#include "chip/cxd56_cisif.h"
#include "cxd56_cisif.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define YUV_VSIZE_MIN (64)
#define YUV_HSIZE_MIN (96)
#define YUV_VSIZE_MAX (360)
#define YUV_HSIZE_MAX (480)

#ifdef CONFIG_CXD56_CISIF_DEBUG
#  define ciferr    _err
#  define cifwarn   _warn
#  define cifinfo   _info
#else
#  define ciferr(x...)
#  define cifwarn(x...)
#  define cifinfo(x...)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

typedef enum
{
  Standby,
  Ready,
  Capture,
  StartMonitoring,
  Monitoring,
  Stop,
} State;

typedef enum
{
  CisifTypeYuvData,
  CisifTypeJpgData,
  CisifTypeMax,
} CisifDataType;

typedef void (*IntcFancTable)(uint8_t code);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static CisifFormat s_input_format;
static State s_state = Standby;
static uint8_t *s_ycc_strage_addr[2] = { NULL, NULL };
static uint8_t *s_jpg_strage_addr[2] = { NULL, NULL };
static uint8_t s_bank = 0;

Notify_callback_t s_notify_callback_func[CisifTypeMax];
Comp_callback_t s_comp_callback_func[CisifTypeMax];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cisifVsInt(uint8_t code);
static void cisifYccAxiTrdnInt(uint8_t code);
static void cisifYccNstorageInt(uint8_t code);
static void cisifJpgAxiTrdnInt(uint8_t code);
static void cisifJpgNstorageInt(uint8_t code);
static void cisifYccErrInt(uint8_t code);
static void cisifJpgErrInt(uint8_t code);

static int cisifIntcHandler(int irq, FAR void *context, FAR void *arg);
static void cisifRegWrite(uint16_t reg, uint32_t val);
static uint32_t cisifRegRead(uint16_t reg);

const IntcFancTable intcomp_func[] =
  {
    cisifVsInt,          /* VS_INT */
    NULL,                /* EOY_INT */
    NULL,                /* SOY_INT */
    NULL,                /* EOI_INT */
    NULL,                /* SOI_INT */
    NULL,                /* YCC_VACT_END_INT */
    NULL,                /* JPG_VACT_END_INT */
    cisifYccAxiTrdnInt,  /* YCC_AXI_TRDN_INT */
    cisifYccNstorageInt, /* YCC_NSTORAGE_INT */
    NULL,                /* YCC_DAREA_END_INT */
    cisifJpgAxiTrdnInt,  /* JPG_AXI_TRDN_INT */
    cisifJpgNstorageInt, /* JPG_NSTORAGE_INT */
    NULL,                /* JPG_DAREA_END_INT */
    NULL,                /* reserve */
    NULL,                /* reserve */
    NULL,                /* VLATCH_INT */
    cisifYccErrInt,      /* SIZE_OVER_INT */
    cisifYccErrInt,      /* SIZE_UNDER_INT */
    cisifYccErrInt,      /* YCC_MARKER_ERR_INT */
    cisifYccErrInt,      /* YCC_AXI_TRERR_INT */
    cisifYccErrInt,      /* YCC_FIFO_OVF_INT */
    cisifYccErrInt,      /* YCC_MEM_OVF_INT */
    NULL,                /* reserve */
    NULL,                /* reserve */
    cisifJpgErrInt,      /* JPG_MARKER_ERR_INT */
    cisifJpgErrInt,      /* JPG_AXI_TRERR_INT */
    cisifJpgErrInt,      /* JPG_FIFO_OVF_INT */
    cisifJpgErrInt,      /* JPG_MEM_OVF_INT */
    cisifJpgErrInt,      /* JPG_ERR_STATUS_INT */
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/*******************************************************************************
 * cisifVsInt
 *******************************************************************************/

static void cisifVsInt(uint8_t code)
{
  switch (s_state)
    {
    case Standby:
      ciferr("invalid state\n");
      break;

    case Ready:
      break;

    case Capture:
      s_state = Stop;

      cisifRegWrite(CISIF_DIN_ENABLE, 1);
      cisifRegWrite(CISIF_EXE_CMD, 1);

      break;

    case StartMonitoring:
      s_state = Monitoring;

      cisifRegWrite(CISIF_DIN_ENABLE, 1);
      cisifRegWrite(CISIF_EXE_CMD, 1);

      break;

    case Monitoring:
      s_bank ^= 1;

      if (s_ycc_strage_addr[s_bank] != NULL)
        {
          cisifRegWrite(CISIF_YCC_START_ADDR, (uint32_t)s_ycc_strage_addr[s_bank]);
        }

      if (s_jpg_strage_addr[s_bank] != NULL)
        {
          cisifRegWrite(CISIF_JPG_START_ADDR, (uint32_t)s_jpg_strage_addr[s_bank]);
        }

      cisifRegWrite(CISIF_EXE_CMD, 1);

      break;

    case Stop:
      s_state = Ready;
      s_bank ^= 1;

      cisifRegWrite(CISIF_DIN_ENABLE, 0);
      cisifRegWrite(CISIF_EXE_CMD, 1);

      break;

    default:
      ciferr("invalid state\n");
      break;

    }
}

/*******************************************************************************
 * cisifYccAxiTrdnInt
 *******************************************************************************/

static void cisifYccAxiTrdnInt(uint8_t code)
{
  uint32_t size;

  size = cisifRegRead(CISIF_YCC_DSTRG_CONT);
  if (s_state == Ready)
    {
      s_comp_callback_func[CisifTypeYuvData](0, 1, size, (uint32_t)s_ycc_strage_addr[s_bank ^ 1]);
    }
  else
    {
      s_comp_callback_func[CisifTypeYuvData](0, 0, size, (uint32_t)s_ycc_strage_addr[s_bank ^ 1]);
    }

  cisifRegWrite(CISIF_YCC_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisifYccNstorageInt
 *******************************************************************************/

static void cisifYccNstorageInt(uint8_t code)
{
  uint32_t size;

  size = cisifRegRead(CISIF_YCC_DSTRG_CONT);
  s_notify_callback_func[CisifTypeYuvData](0, size, (uint32_t)s_ycc_strage_addr[s_bank ^ 1]);
  cisifRegWrite(CISIF_YCC_DREAD_CONT, size);
}

/*******************************************************************************
 * cisifJpgAxiTrdnInt
 *******************************************************************************/

static void cisifJpgAxiTrdnInt(uint8_t code)
{
  uint32_t size;

  size = cisifRegRead(CISIF_JPG_DSTRG_CONT);
  if (s_state == Ready)
    {
      s_comp_callback_func[CisifTypeJpgData](0, 1, size, (uint32_t)s_jpg_strage_addr[s_bank ^ 1]);
    }
  else
    {
      s_comp_callback_func[CisifTypeJpgData](0, 0, size, (uint32_t)s_jpg_strage_addr[s_bank ^ 1]);
    }

  cisifRegWrite(CISIF_JPG_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisifJpgNstorageInt
 *******************************************************************************/

static void cisifJpgNstorageInt(uint8_t code)
{
  uint32_t size;

  size = cisifRegRead(CISIF_JPG_DSTRG_CONT);
  s_notify_callback_func[CisifTypeJpgData](0, size, (uint32_t)s_jpg_strage_addr[s_bank ^ 1]);
  cisifRegWrite(CISIF_JPG_DREAD_CONT, size);
}

/*******************************************************************************
 * cisifYccErrInt
 *******************************************************************************/

static void cisifYccErrInt(uint8_t code)
{
  uint32_t size;

  size = cisifRegRead(CISIF_YCC_DSTRG_CONT);
  s_comp_callback_func[CisifTypeYuvData](code, 1, size, (uint32_t)s_ycc_strage_addr[s_bank ^ 1]);
  cisifRegWrite(CISIF_YCC_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisifJpgErrInt
 *******************************************************************************/

static void cisifJpgErrInt(uint8_t code)
{
  uint32_t size;

  size = cisifRegRead(CISIF_JPG_DSTRG_CONT);
  s_comp_callback_func[CisifTypeJpgData](code, 1, size, (uint32_t)s_jpg_strage_addr[s_bank ^ 1]);
  cisifRegWrite(CISIF_JPG_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisifIntcHandler
 *******************************************************************************/

static int cisifIntcHandler(int irq, FAR void *context, FAR void *arg)
{
  uint32_t value;
  uint32_t enable;
  uint8_t index;

  value = cisifRegRead(CISIF_INTR_STAT);

  cisifRegWrite(CISIF_INTR_CLEAR, value & ALL_CLEAR_INT);

  enable = cisifRegRead(CISIF_INTR_ENABLE);
  value = (value & enable);

  for (index = 0; index < sizeof(intcomp_func) / sizeof(intcomp_func[0]); index++)
    {
      if ((value & (1 << index)) != 0)
        {
          intcomp_func[index](index);
        }
    }

  return OK;
}

/*******************************************************************************
 * cisifRegWrite
 *******************************************************************************/

static void cisifRegWrite(uint16_t reg, uint32_t val)
{
  putreg32(val, CXD56_CISIF_BASE + reg);
  return;
}

/*******************************************************************************
 * cisifRegRead
 *******************************************************************************/

static uint32_t cisifRegRead(uint16_t reg)
{
  return getreg32(CXD56_CISIF_BASE + reg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * CISIF Driver Initialize
 ****************************************************************************/

ResCode cxd56_cisifinit(CisifParam_t *pCisifPar)
{
  uint32_t act_size = 0;
  uint32_t cisif_mode;
  uint32_t interrupts;

  if (s_state != Standby && s_state != Ready)
    {
      return E_INVALID_STATE;
    }

  if (s_state == Standby)
    {
      /* CISIF Power ON */
      /* supposed that APP_SUB is already ON */
      /* TODO confirm */
    }

  /* disable interrupt */

  cisifRegWrite(CISIF_INTR_DISABLE, ALL_DISABLE_INT);

  if (pCisifPar == NULL)
    {
      return E_INVALID_PARAMETER;
    }

  switch (pCisifPar->format)
    {
    case CisifFormatYuv:
      if (pCisifPar->jpg_param.notify_size != 0 || pCisifPar->jpg_param.notify_func != NULL ||
          pCisifPar->jpg_param.comp_func != NULL)
        {
          return E_INVALID_PARAMETER;
        }

      if (pCisifPar->yuv_param.comp_func == NULL)
        {
          return E_INVALID_PARAMETER;
        }

      cisif_mode = MODE_YUV_TRS_EN;
      break;

    case CisifFormatJpeg:
      if (pCisifPar->yuv_param.hsize != 0 || pCisifPar->yuv_param.vsize != 0 || pCisifPar->yuv_param.notify_size != 0 ||
          pCisifPar->yuv_param.notify_func != NULL || pCisifPar->yuv_param.comp_func != NULL)
        {
          return E_INVALID_PARAMETER;
        }

      if (pCisifPar->jpg_param.comp_func == NULL)
        {
          return E_INVALID_PARAMETER;
        }

      cisif_mode = MODE_JPG_TRS_EN;
      break;

    case CisifFormatInterleave:
      if (pCisifPar->yuv_param.comp_func == NULL || pCisifPar->jpg_param.comp_func == NULL)
        {
          return E_INVALID_PARAMETER;
        }

      cisif_mode = MODE_INTLEV_TRS_EN;
      break;

    default:
      return E_INVALID_PARAMETER;
    }

  if (pCisifPar->yuv_param.comp_func != NULL)
    {
      if (pCisifPar->yuv_param.hsize < YUV_HSIZE_MIN || pCisifPar->yuv_param.hsize > YUV_HSIZE_MAX ||
          pCisifPar->yuv_param.vsize < YUV_VSIZE_MIN || pCisifPar->yuv_param.vsize > YUV_VSIZE_MAX)
        {
          return E_INVALID_PARAMETER;
        }

      if (pCisifPar->yuv_param.notify_func == NULL)
        {
          if (pCisifPar->yuv_param.notify_size != 0)
            {
              return E_INVALID_PARAMETER;
            }
        }
      else
        {
          if (pCisifPar->yuv_param.notify_size == 0)
            {
              return E_INVALID_PARAMETER;
            }
          else if (pCisifPar->yuv_param.notify_size % 32 != 0)
            {
              return E_INVALID_PARAMETER;
            }
        }

      act_size = (pCisifPar->yuv_param.vsize & 0x1FF) << 16;
      act_size |= pCisifPar->yuv_param.hsize & 0x1FF;
    }

  if (pCisifPar->jpg_param.comp_func != NULL)
    {
      if (pCisifPar->jpg_param.notify_func == NULL)
        {
          if (pCisifPar->jpg_param.notify_size != 0)
            {
              return E_INVALID_PARAMETER;
            }
        }
      else
        {
          if (pCisifPar->jpg_param.notify_size == 0)
            {
              return E_INVALID_PARAMETER;
            }
          else if (pCisifPar->jpg_param.notify_size % 32 != 0)
            {
              return E_INVALID_PARAMETER;
            }
        }
    }

  cisifRegWrite(CISIF_MODE, cisif_mode);
  cisifRegWrite(CISIF_ACT_SIZE, act_size); /* transfer size */
  cisifRegWrite(CISIF_CIS_SIZE, act_size); /* input size */
  cisifRegWrite(CISIF_YCC_NSTRG_SIZE, pCisifPar->yuv_param.notify_size);
  cisifRegWrite(CISIF_JPG_NSTRG_SIZE, pCisifPar->jpg_param.notify_size);

  s_input_format = pCisifPar->format;
  s_notify_callback_func[CisifTypeYuvData] = pCisifPar->yuv_param.notify_func;
  s_comp_callback_func[CisifTypeYuvData] = pCisifPar->yuv_param.comp_func;
  s_notify_callback_func[CisifTypeJpgData] = pCisifPar->jpg_param.notify_func;
  s_comp_callback_func[CisifTypeJpgData] = pCisifPar->jpg_param.comp_func;

  s_state = Ready;

  /* V-Sync enable interrupt */
  cisifRegWrite(CISIF_INTR_CLEAR, ALL_CLEAR_INT);
  interrupts = VS_INT
      | YCC_AXI_TRDN_INT
      | JPG_AXI_TRDN_INT
      | SIZE_OVER_INT
      | SIZE_UNDER_INT
      | YCC_MARKER_ERR_INT
      | YCC_AXI_TRERR_INT
      | YCC_FIFO_OVF_INT
      | YCC_MEM_OVF_INT
      | JPG_MARKER_ERR_INT
      | JPG_AXI_TRERR_INT
      | JPG_FIFO_OVF_INT
      | JPG_MEM_OVF_INT
      | JPG_ERR_STATUS_INT;

  if (s_notify_callback_func[CisifTypeYuvData] != NULL)
    {
      interrupts |= YCC_NSTORAGE_INT;
    }

  if (s_notify_callback_func[CisifTypeJpgData] != NULL)
    {
      interrupts |= JPG_NSTORAGE_INT;
    }

  cisifRegWrite(CISIF_INTR_ENABLE, interrupts);

  /* CISIF interrupt handler */

  irq_attach(CXD56_IRQ_CISIF, cisifIntcHandler, NULL);
  up_enable_irq(CXD56_IRQ_CISIF);

  cisifRegWrite(CISIF_INTR_CLEAR, ALL_CLEAR_INT);
  cisifRegWrite(CISIF_EXE_CMD, 1);

  return E_OK;
}

/*******************************************************************************
 * cisifCaptureFrame
 *******************************************************************************/

ResCode cxd56_cisifcaptureframe(StrageArea *yuv_area, StrageArea *jpg_area)
{
  if (s_state != Ready)
    {
      return E_INVALID_STATE;
    }

  switch (s_input_format)
    {
    case CisifFormatYuv:
      if (yuv_area == NULL || jpg_area != NULL)
        {
          return E_INVALID_PARAMETER;
        }
      break;

    case CisifFormatJpeg:
      if (yuv_area != NULL || jpg_area == NULL)
        {
          return E_INVALID_PARAMETER;
        }
      break;

    case CisifFormatInterleave:
      if (yuv_area == NULL || jpg_area == NULL)
        {
          return E_INVALID_PARAMETER;
        }
      break;

    default:
      ciferr("invalid format\n");
      return E_SETTING_FAILED;
    }

  if (yuv_area != NULL)
    {
      if (yuv_area->strg_addr == NULL || yuv_area->strg_size == 0 ||
          yuv_area->strg_size % 32 != 0 || (uint32_t)yuv_area->strg_addr % 32 != 0)
        {
          return E_INVALID_PARAMETER;
        }

      s_ycc_strage_addr[0] = (uint8_t *)yuv_area->strg_addr;
      cisifRegWrite(CISIF_YCC_DAREA_SIZE, yuv_area->strg_size);
      cisifRegWrite(CISIF_YCC_START_ADDR, (uint32_t)yuv_area->strg_addr);
    }

  if (jpg_area != NULL)
    {
      if (jpg_area->strg_addr == NULL || jpg_area->strg_size == 0 ||
          jpg_area->strg_size % 32 != 0 || (uint32_t)jpg_area->strg_addr % 32 != 0)
        {
          return E_INVALID_PARAMETER;
        }

      s_jpg_strage_addr[0] = (uint8_t *)jpg_area->strg_addr;
      cisifRegWrite(CISIF_JPG_DAREA_SIZE, jpg_area->strg_size);
      cisifRegWrite(CISIF_JPG_START_ADDR, (uint32_t)jpg_area->strg_addr);
    }

  s_bank = 0;
  s_state = Capture;

  return E_OK;
}

/*******************************************************************************
 * cisifStartMonitoring
 *******************************************************************************/

ResCode cxd56_cisifstartmonitoring(BankStrageArea *yuv_area, BankStrageArea *jpg_area)
{
  if (s_state != Ready)
    {
      return E_INVALID_STATE;
    }

  switch (s_input_format)
    {
    case CisifFormatYuv:
      if (yuv_area == NULL || jpg_area != NULL)
        {
          return E_INVALID_PARAMETER;
        }
      break;

    case CisifFormatJpeg:
      if (yuv_area != NULL || jpg_area == NULL)
        {
          return E_INVALID_PARAMETER;
        }
      break;

    case CisifFormatInterleave:
      if (yuv_area == NULL || jpg_area == NULL)
        {
          return E_INVALID_PARAMETER;
        }
      break;

    default:
      ciferr("invalid format\n");
      return E_SETTING_FAILED;
    }

  if (yuv_area != NULL)
    {
      if (yuv_area->strg_addr_0 == NULL || yuv_area->strg_addr_1 == NULL || yuv_area->strg_size == 0
          || yuv_area->strg_size % 32 != 0 || (uint32_t)yuv_area->strg_addr_0 % 32 != 0
          || (uint32_t)yuv_area->strg_addr_1 % 32 != 0)
        {
          return E_INVALID_PARAMETER;
        }

      s_ycc_strage_addr[0] = (uint8_t *)yuv_area->strg_addr_0;
      s_ycc_strage_addr[1] = (uint8_t *)yuv_area->strg_addr_1;
      cisifRegWrite(CISIF_YCC_DAREA_SIZE, yuv_area->strg_size);
      cisifRegWrite(CISIF_YCC_START_ADDR, (uint32_t)s_ycc_strage_addr[0]);
    }

  if (jpg_area != NULL)
    {
      if (jpg_area->strg_addr_0 == NULL || jpg_area->strg_addr_1 == NULL || jpg_area->strg_size == 0
          || jpg_area->strg_size % 32 != 0 || (uint32_t)jpg_area->strg_addr_0 % 32 != 0
          || (uint32_t)jpg_area->strg_addr_1 % 32 != 0)
        {
          return E_INVALID_PARAMETER;
        }

      s_jpg_strage_addr[0] = (uint8_t *)jpg_area->strg_addr_0;
      s_jpg_strage_addr[1] = (uint8_t *)jpg_area->strg_addr_1;
      cisifRegWrite(CISIF_JPG_DAREA_SIZE, jpg_area->strg_size);
      cisifRegWrite(CISIF_JPG_START_ADDR, (uint32_t)s_jpg_strage_addr[0]);
    }

  s_bank = 0;
  s_state = StartMonitoring;

  return E_OK;
}

/*******************************************************************************
 * cisifStopMonitoring
 *******************************************************************************/

ResCode cxd56_cisifstopmonitoring()
{
  if (s_state != Monitoring)
    {
      return E_INVALID_STATE;
    }

  s_state = Stop;

  return E_OK;
}
