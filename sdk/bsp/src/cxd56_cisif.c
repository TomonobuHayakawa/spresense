/****************************************************************************
 * bsp/src/cxd56_cisif.c
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
#include <errno.h>
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

enum state_e
{
  STATE_STANDBY,
  STATE_READY,
  STATE_CAPTURE,
  STATE_START_MONITORING,
  STATE_MONITORING,
  STATE_STOP,
};

typedef enum state_e state_t;

enum type_cisif_e
{
  TYPE_CISIF_YUV,
  TYPE_CISIF_JPEG,
  TYPE_CISIF_MAX,
};

typedef void (*intc_func_table)(uint8_t code);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static format_cisif_t s_input_format;
static state_t g_state = STATE_STANDBY;
static uint8_t *g_ycc_strage_addr[2] = { NULL, NULL };
static uint8_t *g_jpg_strage_addr[2] = { NULL, NULL };
static uint8_t g_bank = 0;

notify_callback_t g_notify_callback_func[TYPE_CISIF_MAX];
comp_callback_t   g_comp_callback_func[TYPE_CISIF_MAX];

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void cisif_vs_int(uint8_t code);
static void cisif_ycc_axi_trdn_int(uint8_t code);
static void cisif_ycc_nstorage_int(uint8_t code);
static void cisif_jpg_axi_trdn_int(uint8_t code);
static void cisif_jpg_nstorage_int(uint8_t code);
static void cisif_ycc_err_int(uint8_t code);
static void cisif_jpg_err_int(uint8_t code);

static int      cisif_intc_handler(int irq, FAR void *context, FAR void *arg);
static void     cisif_reg_write(uint16_t reg, uint32_t val);
static uint32_t cisif_reg_read(uint16_t reg);


const intc_func_table g_intcomp_func[] =
  {
    cisif_vs_int,            /* VS_INT */
    NULL,                    /* EOY_INT */
    NULL,                    /* SOY_INT */
    NULL,                    /* EOI_INT */
    NULL,                    /* SOI_INT */
    NULL,                    /* YCC_VACT_END_INT */
    NULL,                    /* JPG_VACT_END_INT */
    cisif_ycc_axi_trdn_int,  /* YCC_AXI_TRDN_INT */
    cisif_ycc_nstorage_int,  /* YCC_NSTORAGE_INT */
    NULL,                    /* YCC_DAREA_END_INT */
    cisif_jpg_axi_trdn_int,  /* JPG_AXI_TRDN_INT */
    cisif_jpg_nstorage_int,  /* JPG_NSTORAGE_INT */
    NULL,                    /* JPG_DAREA_END_INT */
    NULL,                    /* reserve */
    NULL,                    /* reserve */
    NULL,                    /* VLATCH_INT */
    cisif_ycc_err_int,       /* SIZE_OVER_INT */
    cisif_ycc_err_int,       /* SIZE_UNDER_INT */
    cisif_ycc_err_int,       /* YCC_MARKER_ERR_INT */
    cisif_ycc_err_int,       /* YCC_AXI_TRERR_INT */
    cisif_ycc_err_int,       /* YCC_FIFO_OVF_INT */
    cisif_ycc_err_int,       /* YCC_MEM_OVF_INT */
    NULL,                    /* reserve */
    NULL,                    /* reserve */
    cisif_jpg_err_int,       /* JPG_MARKER_ERR_INT */
    cisif_jpg_err_int,       /* JPG_AXI_TRERR_INT */
    cisif_jpg_err_int,       /* JPG_FIFO_OVF_INT */
    cisif_jpg_err_int,       /* JPG_MEM_OVF_INT */
    cisif_jpg_err_int,       /* JPG_ERR_STATUS_INT */
  };

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/*******************************************************************************
 * cisif_vs_int
 *******************************************************************************/
static void cisif_vs_int(uint8_t code)
{
  switch (g_state)
    {
      case STATE_STANDBY:
        ciferr("invalid state\n");
        break;

      case STATE_READY:
        break;

      case STATE_CAPTURE:
        g_state = STATE_STOP;
        cisif_reg_write(CISIF_DIN_ENABLE, 1);
        cisif_reg_write(CISIF_EXE_CMD, 1);
        break;

      case STATE_START_MONITORING:
        g_state = STATE_MONITORING;
        cisif_reg_write(CISIF_DIN_ENABLE, 1);
        cisif_reg_write(CISIF_EXE_CMD, 1);
        break;

      case STATE_MONITORING:
        g_bank ^= 1;
        if (g_ycc_strage_addr[g_bank] != NULL)
          {
            cisif_reg_write(CISIF_YCC_START_ADDR, (uint32_t)g_ycc_strage_addr[g_bank]);
          }
        if (g_jpg_strage_addr[g_bank] != NULL)
          {
            cisif_reg_write(CISIF_JPG_START_ADDR, (uint32_t)g_jpg_strage_addr[g_bank]);
          }
        cisif_reg_write(CISIF_EXE_CMD, 1);
        break;

      case STATE_STOP:
        g_state = STATE_READY;
        g_bank ^= 1;
        cisif_reg_write(CISIF_DIN_ENABLE, 0);
        cisif_reg_write(CISIF_EXE_CMD, 1);
        break;

      default:
        ciferr("invalid state\n");
        break;
    }

}

/*******************************************************************************
 * cisif_ycc_axi_trdn_int
 *******************************************************************************/
static void cisif_ycc_axi_trdn_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  if (g_state == STATE_READY)
    {
      g_comp_callback_func[TYPE_CISIF_YUV](0, 1, size, (uint32_t)g_ycc_strage_addr[g_bank ^ 1]);
    }
  else
    {
      g_comp_callback_func[TYPE_CISIF_YUV](0, 0, size, (uint32_t)g_ycc_strage_addr[g_bank ^ 1]);
    }

  cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisif_ycc_nstorage_int
 *******************************************************************************/
static void cisif_ycc_nstorage_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  g_notify_callback_func[TYPE_CISIF_YUV](0, size, (uint32_t)g_ycc_strage_addr[g_bank ^ 1]);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, size);
}

/*******************************************************************************
 * cisif_jpg_axi_trdn_int
 *******************************************************************************/
static void cisif_jpg_axi_trdn_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
  if (g_state == STATE_READY)
    {
      g_comp_callback_func[TYPE_CISIF_JPEG](0, 1, size, (uint32_t)g_jpg_strage_addr[g_bank ^ 1]);
    }
  else
    {
      g_comp_callback_func[TYPE_CISIF_JPEG](0, 0, size, (uint32_t)g_jpg_strage_addr[g_bank ^ 1]);
    }

  cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisif_jpg_nstorage_int
 *******************************************************************************/
static void cisif_jpg_nstorage_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
  g_notify_callback_func[TYPE_CISIF_JPEG](0, size, (uint32_t)g_jpg_strage_addr[g_bank ^ 1]);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, size);
}

/*******************************************************************************
 * cisif_ycc_err_int
 *******************************************************************************/
static void cisif_ycc_err_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_YCC_DSTRG_CONT);
  g_comp_callback_func[TYPE_CISIF_YUV](code, 1, size, (uint32_t)g_ycc_strage_addr[g_bank ^ 1]);
  cisif_reg_write(CISIF_YCC_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisif_jpg_err_int
 *******************************************************************************/
static void cisif_jpg_err_int(uint8_t code)
{
  uint32_t size;

  size = cisif_reg_read(CISIF_JPG_DSTRG_CONT);
  g_comp_callback_func[TYPE_CISIF_JPEG](code, 1, size, (uint32_t)g_jpg_strage_addr[g_bank ^ 1]);
  cisif_reg_write(CISIF_JPG_DREAD_CONT, 0);
}

/*******************************************************************************
 * cisif_intc_handler
 *******************************************************************************/
static int cisif_intc_handler(int irq, FAR void *context, FAR void *arg)
{
  uint32_t value;
  uint32_t enable;
  uint8_t  index;

  value = cisif_reg_read(CISIF_INTR_STAT);
  cisif_reg_write(CISIF_INTR_CLEAR, value & ALL_CLEAR_INT);
  cifinfo("int stat %08x\n", value);

  enable = cisif_reg_read(CISIF_INTR_ENABLE);
  value = (value & enable);

  for (index = 0; index < sizeof(g_intcomp_func) / sizeof(g_intcomp_func[0]); index++)
    {
      if ((value & (1 << index)) != 0)
        {
          g_intcomp_func[index](index);
        }

    }

  return OK;
}

/*******************************************************************************
 * cisif_reg_write
 *******************************************************************************/
static void cisif_reg_write(uint16_t reg, uint32_t val)
{
  putreg32(val, CXD56_CISIF_BASE + reg);
  return;
}

/*******************************************************************************
 * cisif_reg_read
 *******************************************************************************/
static uint32_t cisif_reg_read(uint16_t reg)
{
  return getreg32(CXD56_CISIF_BASE + reg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
/****************************************************************************
 * CISIF Driver Initialize
 ****************************************************************************/
int cxd56_cisifinit(cisif_param_t *pCisifPar)
{
  uint32_t act_size = 0;
  uint32_t cisif_mode;
  uint32_t interrupts;

  if (g_state != STATE_STANDBY && g_state != STATE_READY)
    {
      return -EPERM;
    }

  if (g_state == STATE_STANDBY)
    {
      /* CISIF Power ON */
      /* supposed that APP_SUB is already ON */
      /* TODO confirm */
    }

  /* disable interrupt */

  cisif_reg_write(CISIF_INTR_DISABLE, ALL_DISABLE_INT);

  if (pCisifPar == NULL)
    {
      return -EINVAL;
    }

  switch (pCisifPar->format)
    {
      case FORMAT_CISIF_YUV:
        if (pCisifPar->jpg_param.notify_size != 0    ||
            pCisifPar->jpg_param.notify_func != NULL ||
            pCisifPar->jpg_param.comp_func   != NULL)
          {
            return -EINVAL;
          }

        if (pCisifPar->yuv_param.comp_func == NULL)
          {
            return -EINVAL;
          }

        cisif_mode = MODE_YUV_TRS_EN;
        break;

      case FORMAT_CISIF_JPEG:
        if (pCisifPar->yuv_param.hsize       != 0    ||
            pCisifPar->yuv_param.vsize       != 0    ||
            pCisifPar->yuv_param.notify_size != 0    ||
            pCisifPar->yuv_param.notify_func != NULL ||
            pCisifPar->yuv_param.comp_func   != NULL)
          {
            return -EINVAL;
          }

        if (pCisifPar->jpg_param.comp_func == NULL)
          {
            return -EINVAL;
          }

        cisif_mode = MODE_JPG_TRS_EN;
        break;

      case FORMAT_CISIF_INTERLEAVE:
        if (pCisifPar->yuv_param.comp_func == NULL ||
            pCisifPar->jpg_param.comp_func == NULL)
          {
            return -EINVAL;
          }

        cisif_mode = MODE_INTLEV_TRS_EN;
        break;

      default:
        return -EINVAL;
    }

  if (pCisifPar->yuv_param.comp_func != NULL)
    {
      if (pCisifPar->yuv_param.hsize < YUV_HSIZE_MIN ||
          pCisifPar->yuv_param.hsize > YUV_HSIZE_MAX ||
          pCisifPar->yuv_param.vsize < YUV_VSIZE_MIN ||
          pCisifPar->yuv_param.vsize > YUV_VSIZE_MAX)
        {
          return -EINVAL;
        }

      if (pCisifPar->yuv_param.notify_func == NULL)
        {
          if (pCisifPar->yuv_param.notify_size != 0)
            {
              return -EINVAL;
            }

        }
      else
        {
          if (pCisifPar->yuv_param.notify_size == 0)
            {
              return -EINVAL;
            }
          else if (pCisifPar->yuv_param.notify_size % 32 != 0)
            {
              return -EINVAL;
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
              return -EINVAL;
            }

        }
      else
        {
          if (pCisifPar->jpg_param.notify_size == 0)
            {
              return -EINVAL;
            }
          else if (pCisifPar->jpg_param.notify_size % 32 != 0)
            {
              return -EINVAL;
            }

        }
    }

  cisif_reg_write(CISIF_MODE, cisif_mode);
  cisif_reg_write(CISIF_ACT_SIZE, act_size); /* transfer size */
  cisif_reg_write(CISIF_CIS_SIZE, act_size); /* input size */
  cisif_reg_write(CISIF_YCC_NSTRG_SIZE, pCisifPar->yuv_param.notify_size);
  cisif_reg_write(CISIF_JPG_NSTRG_SIZE, pCisifPar->jpg_param.notify_size);

  s_input_format = pCisifPar->format;
  g_notify_callback_func[TYPE_CISIF_YUV]  = pCisifPar->yuv_param.notify_func;
  g_comp_callback_func[TYPE_CISIF_YUV]    = pCisifPar->yuv_param.comp_func;
  g_notify_callback_func[TYPE_CISIF_JPEG] = pCisifPar->jpg_param.notify_func;
  g_comp_callback_func[TYPE_CISIF_JPEG]   = pCisifPar->jpg_param.comp_func;

  g_state = STATE_READY;

  /* V-Sync enable interrupt */
  cisif_reg_write(CISIF_INTR_CLEAR, ALL_CLEAR_INT);
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

  if (g_notify_callback_func[TYPE_CISIF_YUV] != NULL)
    {
      interrupts |= YCC_NSTORAGE_INT;
    }

  if (g_notify_callback_func[TYPE_CISIF_JPEG] != NULL)
    {
      interrupts |= JPG_NSTORAGE_INT;
    }

  cisif_reg_write(CISIF_INTR_ENABLE, interrupts);

  /* CISIF interrupt handler */
  irq_attach(CXD56_IRQ_CISIF, cisif_intc_handler, NULL);
  up_enable_irq(CXD56_IRQ_CISIF);
  cisif_reg_write(CISIF_INTR_CLEAR, ALL_CLEAR_INT);
  cisif_reg_write(CISIF_EXE_CMD, 1);

  return OK;
}

/*******************************************************************************
 * cisifCaptureframe
 *******************************************************************************/
int cxd56_cisifcaptureframe(cisif_sarea_t *yuv_area, cisif_sarea_t *jpg_area)
{
  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  switch (s_input_format)
    {
      case FORMAT_CISIF_YUV:
        if (yuv_area == NULL || jpg_area != NULL)
          {
            return -EINVAL;
          }

        break;

      case FORMAT_CISIF_JPEG:
        if (yuv_area != NULL || jpg_area == NULL)
          {
            return -EINVAL;
          }

        break;

      case FORMAT_CISIF_INTERLEAVE:
        if (yuv_area == NULL || jpg_area == NULL)
          {
            return -EINVAL;
          }

        break;

      default:
        ciferr("invalid format\n");
        return -EPERM;
    }

  if (yuv_area != NULL)
    {
      if (yuv_area->strg_addr == NULL   ||
          yuv_area->strg_size == 0      ||
          yuv_area->strg_size % 32 != 0 ||
          (uint32_t)yuv_area->strg_addr % 32 != 0)
        {
          return -EINVAL;
        }

      g_ycc_strage_addr[0] = (uint8_t *)yuv_area->strg_addr;
      cisif_reg_write(CISIF_YCC_DAREA_SIZE, yuv_area->strg_size);
      cisif_reg_write(CISIF_YCC_START_ADDR, (uint32_t)yuv_area->strg_addr);
    }

  if (jpg_area != NULL)
    {
      if (jpg_area->strg_addr == NULL   ||
          jpg_area->strg_size == 0      ||
          jpg_area->strg_size % 32 != 0 ||
          (uint32_t)jpg_area->strg_addr % 32 != 0)
        {
          return -EINVAL;
        }

      g_jpg_strage_addr[0] = (uint8_t *)jpg_area->strg_addr;
      cisif_reg_write(CISIF_JPG_DAREA_SIZE, jpg_area->strg_size);
      cisif_reg_write(CISIF_JPG_START_ADDR, (uint32_t)jpg_area->strg_addr);
    }

  g_bank  = 0;
  g_state = STATE_CAPTURE;

  return OK;
}

/*******************************************************************************
 * cisifStartMonitoring
 *******************************************************************************/
int cxd56_cisifstartmonitoring(cisif_bank_sarea_t *yuv_area, cisif_bank_sarea_t *jpg_area)
{
  if (g_state != STATE_READY)
    {
      return -EPERM;
    }

  switch (s_input_format)
    {
      case FORMAT_CISIF_YUV:
        if (yuv_area == NULL || jpg_area != NULL)
          {
            return -EINVAL;
          }

        break;

      case FORMAT_CISIF_JPEG:
        if (yuv_area != NULL || jpg_area == NULL)
          {
            return -EINVAL;
          }

        break;

      case FORMAT_CISIF_INTERLEAVE:
        if (yuv_area == NULL || jpg_area == NULL)
          {
            return -EINVAL;
          }

        break;

      default:
        ciferr("invalid format\n");
        return -EPERM;
    }

  if (yuv_area != NULL)
    {
      if (yuv_area->strg_addr_0 == NULL ||
          yuv_area->strg_addr_1 == NULL ||
          yuv_area->strg_size   == 0    ||
          yuv_area->strg_size % 32 != 0 ||
          (uint32_t)yuv_area->strg_addr_0 % 32 != 0 ||
          (uint32_t)yuv_area->strg_addr_1 % 32 != 0)
        {
          return -EINVAL;
        }

      g_ycc_strage_addr[0] = (uint8_t *)yuv_area->strg_addr_0;
      g_ycc_strage_addr[1] = (uint8_t *)yuv_area->strg_addr_1;
      cisif_reg_write(CISIF_YCC_DAREA_SIZE, yuv_area->strg_size);
      cisif_reg_write(CISIF_YCC_START_ADDR, (uint32_t)g_ycc_strage_addr[0]);
    }

  if (jpg_area != NULL)
    {
      if (jpg_area->strg_addr_0 == NULL ||
          jpg_area->strg_addr_1 == NULL ||
          jpg_area->strg_size == 0      ||
          jpg_area->strg_size % 32 != 0 ||
          (uint32_t)jpg_area->strg_addr_0 % 32 != 0 ||
          (uint32_t)jpg_area->strg_addr_1 % 32 != 0)
        {
          return -EINVAL;
        }

      g_jpg_strage_addr[0] = (uint8_t *)jpg_area->strg_addr_0;
      g_jpg_strage_addr[1] = (uint8_t *)jpg_area->strg_addr_1;
      cisif_reg_write(CISIF_JPG_DAREA_SIZE, jpg_area->strg_size);
      cisif_reg_write(CISIF_JPG_START_ADDR, (uint32_t)g_jpg_strage_addr[0]);
    }

  g_bank  = 0;
  g_state = STATE_START_MONITORING;

  return OK;
}

/*******************************************************************************
 * cisifStopMonitoring
 *******************************************************************************/
int cxd56_cisifstopmonitoring()
{
  if (g_state != STATE_MONITORING)
    {
      return -EPERM;
    }

  g_state = STATE_STOP;

  return OK;
}

