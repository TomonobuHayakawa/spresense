/****************************************************************************
 * modules/audio/dma_controller/audio_dma_drv_api.h
 *
 *   Copyright (C) 2016, 2017 Sony Corporation
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

#ifndef __MODULES_AUDIO_DMA_CONTROLLER_BCA_DRV_H
#define __MODULES_AUDIO_DMA_CONTROLLER_BCA_DRV_H

#include <arch/chip/cxd56_audio.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define DMAC_MAX_SIZE     4096
#define DMAC_MIN_SIZE_POL 32
#define DMAC_MIN_SIZE_INT 240


#define AS_DMAC_BYTE_WT_24BIT 4
#define AS_DMAC_BYTE_WT_16BIT 2

#define AS_DMAC_CMD_BUF_STATUS_EMPTY 3

#define BUSY_LOOP(cnt)                    \
{                                         \
  volatile unsigned int tmp_value = 1;    \
  int loop = 0;                           \
  for (loop=0 ; loop<(int)(cnt) ; loop++) \
    {                                     \
      tmp_value++;                        \
    }                                     \
}

typedef enum
{
  AS_DMATRNSCMPLT_NONE,
  AS_DMATRNSCMPLT_OK,
  AS_DMATRNSCMPLT_MAX_ENTRY
} asDmacTrnsCmpltResult;

typedef enum
{
  AS_DMATRNSSTOP_INCMPLT,
  AS_DMATRNSSTOP_CMPLT,
  AS_DMATRNSSTOP_MAX_ENTRY
} asDmacTrnsStopStatus;

/** DMAC transfer result code */
typedef enum
{
  E_AS_BB_DMA_OK,          /* nomal end */
  E_AS_BB_DMA_ILLEGAL,     /* illegal end */
  E_AS_BB_DMA_ERR_INT,     /* interrupt error */
  E_AS_BB_DMA_UNDERFLOW,   /* underflow */
  E_AS_BB_DMA_OVERFLOW,    /* overflow */
  E_AS_BB_DMA_ERR_REQUEST, /* request error */
  E_AS_BB_DMA_PARAM,       /* parameter error */
  E_AS_BB_DMA_ERR_START,   /* transfer start error */
  E_AS_BB_DMA_ERR_BUS      /* bus error */
} E_AS_BB;

/** DMAC State */
typedef enum
{
  AS_DMA_STATE_BOOTED,   /* BOOTED */
  AS_DMA_STATE_STOP,     /* STOP */
  AS_DMA_STATE_READY,    /* READY */
  AS_DMA_STATE_PREPARE,  /* PREPARE */
  AS_DMA_STATE_RUN,      /* RUN */
  AS_DMA_STATE_FLUSH,    /* FLUSH */
  AS_DMA_STATE_MAX_ENTRY /* MAX ENTRY */
} asDmaState;

/** AS_ErrorCb callback function parameter */
typedef struct AudioDrvDmaError_
{
  asDmacSelId dmac_id; /* [in] Error DMAC ID */
  E_AS_BB     status;  /* [in] Error fact */
  asDmaState  state;   /* [in] DMAC state */
} AudioDrvDmaError;

/** AS_DmaDoneCb callback function parameter */
typedef struct AudioDrvDmaResult_
{
  E_AS_BB     result;  /* [in] DMAC transfer result */
  bool        endflg;  /* [in] DMAC transfer end data flg */
  asDmacSelId dmac_id; /* [in] DMAC ID */
  uint32_t    addr1;   /* [in] DMAC transfer data address1 */
  uint16_t    size1;   /* [in] DMAC transfer data size1 */
  uint32_t    addr2;   /* [in] DMAC transfer data address2 */
  uint16_t    size2;   /* [in] DMAC transfer data size2 */
} AudioDrvDmaResult;

/** DMAC transfer error callback function */
typedef void (* AS_ErrorCb)(AudioDrvDmaError *pParam);
/** DMAC transfer done callback function */
typedef void (* AS_DmaDoneCb)(AudioDrvDmaResult *pParam);

/** #AS_InitDmac function parameter */
typedef struct
{
  asDmacSelId  dmacId;         /* [in] DMAC ID */
  asSampFmt    format;         /* [in] data format */
  AS_ErrorCb   p_error_func;   /* [in] DMAC transfer error callback */
  AS_DmaDoneCb p_dmadone_func; /* [in] DMAC transfer done callback */
  bool         fade_en;        /* [in] auto fade mode, TRUE:ENABLE */
} asInitDmacParam;

/**
 * @brief Init DMAC
 *
 * @param[in] asInitDmacParam* Init DMAC parameter
 *
 * @retval E_AS return code
 */
E_AS AS_InitDmac(asInitDmacParam *pInitDmacParam);

/**
 * @brief Start DMAC
 *
 * @param[in] asDmacSelId DMAC ID
 *
 * @retval E_AS return code
 */
E_AS AS_StartDmac(asDmacSelId dmacId);


/** #AS_ReadDmac and #AS_WriteDmac function parameter */
typedef struct
{
  asDmacSelId dmacId;   /* [in] DMAC ID */
  uint32_t    addr;     /* [in] DMAC transfer data address */
  uint16_t    size;     /* [in] DMAC transfer data size */
  uint32_t    addr2;    /* [in] DMAC transfer data address2 */
  uint16_t    size2;    /* [in] DMAC transfer data size2 */
  bool        validity; /* [in] Frame validity (true:valid) */
} asReadDmacParam, asWriteDmacParam;

/**
 * @brief Read DMAC
 *
 * @param[in] asReadDmacParam* Read DMAC parameter
 *
 * @retval E_AS return code
 */
E_AS AS_ReadDmac(asReadDmacParam *pReadDmacParam);

/**
 * @brief Write DMAC
 *
 * @param[in] asWriteDmacParam* Write DMAC parameter
 *
 * @retval E_AS return code
 */
E_AS AS_WriteDmac(asWriteDmacParam *pWriteDmacParam);


/** Select DMAC stop mode */
typedef enum
{
  AS_DMASTOPMODE_NORMAL,    /* NORMAL */
  AS_DMASTOPMODE_IMMEDIATE, /* IMMEDIATE */
  AS_DMASTOPMODE_MAX_ENTRY  /* MAX ENTRY */
} asDmacStopMode;

/**
 * @brief Stop DMAC
 *
 * @param[in] asDmacSelId DMAC ID
 * @param[in] asDmacStopMode Stop mode
 *
 * @retval E_AS return code
 */
E_AS AS_StopDmac(asDmacSelId dmacId, asDmacStopMode stopMode);

/**
 * @brief Get numbers of the DMAC ready command
 *
 * @param[in] asDmacSelId DMAC ID
 * @param[out] uint32_t* Numbers of the DMAC ready command
 *
 * @retval E_AS return code
 */
E_AS AS_GetReadyCmdNumDmac(asDmacSelId dmacId, uint32_t *pResult);

/**
 * @brief Regist DMA callback from interrupt handler
 *
 * @param[in] asDmacSelId DMAC ID
 * @param[in] AS_DmaIntCb DMA interrupt handler callback function pointer
 *
 * @retval E_AS return code
 */
E_AS AS_RegistDmaIntCb(asDmacSelId dmacId, AS_DmaIntCb p_dmaIntCb);

/**
 * @brief Notify DMA completed
 *
 * @param[in] asDmacSelId DMAC ID
 * @param[in] code type of interrupt
 *
 * @retval E_AS return code
 */
E_AS AS_NotifyDmaCmplt(asDmacSelId dmacId, E_AS_DMA_INT code);

#ifdef __cplusplus
} /* end of extern "C" */
#endif /* __cplusplus */

#endif /* __MODULES_AUDIO_DMA_CONTROLLER_BCA_DRV_H */
