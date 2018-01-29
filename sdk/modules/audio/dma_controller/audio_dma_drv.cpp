/****************************************************************************
 * modules/audio/dma_controller/audio_dma_drv.cpp
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

#include <nuttx/kmalloc.h>
#include <debug.h>
#include <arch/chip/cxd56_audio.h>

#include "memutils/os_utils/chateau_osal.h"
#include "memutils/common_utils/common_attention.h"
#include "audio/audio_high_level_api.h"
#include "debug/dbg_log.h"
#include "audio_dma_drv.h"
#include "audio_dma_buffer.h"

/* Currently, a timing to start fade is when set setting of previous frame.
 * Because, the volume will going down about 6db/ms, therefore after about
 * 5ms, the volume is very small which is hard to hear.
 * However, if you'd like to take fade time strictly,
 * enable "FADECTRL_BY_FADETERM".
 */
/* #define FADECTRL_BY_FADETERM */

#define CHECK_DMA_CH_SHIFT

#define FADE_QUEUE_COUNT 2
#define STOP_QUEUE_COUNT 1
#define LAST_QUEUE_COUNT 1

#define DMA_BUFFER_MAX_SIZE 1024
#define DMA_BUFFER_POOL_SEG_SIZE 0x00002000

#define TIMEOUT_CNT 10000    /* >160MHz/48kHz */
#define RETRY_CNT 10

extern "C" uint32_t cxd56_get_cpu_baseclk(void);

/*--------------------------------------------------------------------*/
static E_AS syncDmacTrg(asDmacSelId dmacId, bool nointr)
{
  uint32_t cpu_baseclk = cxd56_get_cpu_baseclk();
  uint32_t wait_err_cnt_normal = (2*cpu_baseclk/48000);
  uint32_t wait_err_cnt_hires = (wait_err_cnt_normal/4);

  E_AS rtCode = E_AS_DMAC_ERR_START;
  uint32_t chsel = 0;
  uint32_t context = 0;
  uint32_t retry_count = 0;
  uint32_t smp_int_flg = 0;
  uint32_t wait_smp_count = 0;
  uint32_t err_int_flg = 0;
  uint32_t wait_err_count = 0;
  uint32_t cmd_buf_stat = 0;
  uint32_t wait_buf_count = 0;
  uint32_t done_int_flg = 0;
  uint32_t wait_done_count = 0;

  getDmacChannel(dmacId, &chsel);   /* Read ch setting */
  clearDmacErrIntStatus(dmacId);    /* Err clear */
  clearDmacSmpIntStatus(dmacId);    /* Smp clear */
  setDmacDoneIntMask(dmacId, true); /* Int mask */

  while (retry_count++ < RETRY_CNT)
    {
      Chateau_LockInterrupt(&context);

      wait_smp_count = 0;

      while (wait_smp_count++ < TIMEOUT_CNT)
        {
          getDmacSmpIntStatus(dmacId, &smp_int_flg);

          /* Wait smp. */

          if(smp_int_flg == 1)
            {
              break;
            }
        }

      /* reset ch + set ch + dma start */

      setDmacTrgWithChSel(dmacId, nointr, chsel);

      Chateau_UnlockInterrupt(&context);

      if (GetClkMode() == AS_CLK_MODE_HIRES)
        {
          wait_err_count = wait_err_cnt_hires;
        }
      else
        {
          wait_err_count = wait_err_cnt_normal;
        }

      BUSY_LOOP(wait_err_count);

      getDmacErrIntStatus(dmacId, &err_int_flg);

      /* Err check. */

      if (err_int_flg == 1)
        {
          setDmacStop(dmacId);           /* Dma stop */
          clearDmacErrIntStatus(dmacId); /* Err clear */

          wait_buf_count = 0;

          while (wait_buf_count++ < TIMEOUT_CNT)
            {
              getDmacMonbufStatus(dmacId, &cmd_buf_stat);

              /* Wait dma buffer empty. */

              if (cmd_buf_stat == AS_DMAC_CMD_BUF_STATUS_EMPTY)
                {
                  wait_done_count = 0;

                  while (wait_done_count++ < TIMEOUT_CNT)
                    {
                      getDmacDoneIntStatus(dmacId, &done_int_flg);

                      /* Wait done. */

                      if (done_int_flg == 1)
                        {
                          break;
                        }
                    }

                  clearDmacDoneIntStatus(dmacId); /* Done clear */

                  break;
                }
            }

          _info("retry(%d:%d,%d,%d)\n",
              retry_count, wait_smp_count, wait_buf_count, wait_done_count);
        }
      else
        {
          rtCode = E_AS_OK;
          break;
        }
  }

  setDmacDoneIntMask(dmacId, false);

  return rtCode;
}

/*--------------------------------------------------------------------*/
static E_AS startDmacWithCheck(asDmacSelId dmacId,
                               uint32_t addr,
                               uint32_t sample,
                               bool nointr,
                               bool errCheck)
{
  E_AS rtCode = E_AS_OK;
  uint32_t stat = 0;

  rtCode = getDmacCmdStatus(dmacId, &stat);

  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  if (stat != 1)
    {
      return E_AS_DMAC_BUSY;
    }

  rtCode = setDmacAddr(dmacId, addr);

  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  rtCode = setDmacSample(dmacId, sample);

  if (rtCode != E_AS_OK)
    {
      return rtCode;
    }

  if (errCheck == true )
    {
      rtCode = syncDmacTrg(dmacId, nointr);
    }
  else
    {
      rtCode = setDmacTrg(dmacId, nointr);
    }

  return rtCode;
}

AsDmaDrv::dmaDrvFuncTbl AsDmaDrv::m_func_tbl[] =
{
  {
    EvtInit,

    {                           /* DmaController status:  */
      &AsDmaDrv::init,          /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::init,          /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::illegal,       /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::illegal,       /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::illegal,       /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::illegal        /*   AS_DMA_STATE_FLUSH   */
    }
  },

  {
    EvtRun,

    {                           /* DmaController status:  */
      &AsDmaDrv::illegal,       /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::runDmaOnStop,  /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::runDmaOnReady, /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::runDma,        /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::runDmaOnRun,   /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::illegal        /*   AS_DMA_STATE_FLUSH   */
    }
  },

  {
    EvtStop,

    {                           /* DmaController status:  */
        &AsDmaDrv::illegal,     /*   AS_DMA_STATE_BOOTED  */
        &AsDmaDrv::illegal,     /*   AS_DMA_STATE_STOP    */
        &AsDmaDrv::stop,        /*   AS_DMA_STATE_READY   */
        &AsDmaDrv::stop,        /*   AS_DMA_STATE_PREPARE */
        &AsDmaDrv::stopOnRun,   /*   AS_DMA_STATE_RUN     */
        &AsDmaDrv::illegal      /*   AS_DMA_STATE_FLUSH   */
    }
  },

  {
    EvtCmplt,

    {                            /* DmaController status:  */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::dmaCmpltOnRun,  /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::dmaCmpltOnFlush /*   AS_DMA_STATE_FLUSH   */
    }
  },

  {
    EvtGetInfo,

    {                            /* DmaController status:  */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::getInfo,        /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::getInfo,        /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::getInfo,        /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::getInfo,        /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::getInfo         /*   AS_DMA_STATE_FLUSH   */
    }
  },

  {
    EvtDmaErr,

    {                            /* DmaController status:  */
      &AsDmaDrv::dmaErrInt,      /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::dmaErrInt,      /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::dmaErrInt,      /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::dmaErrInt,      /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::dmaErrIntOnRun, /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::dmaErrInt       /*   AS_DMA_STATE_FLUSH   */
    }
  },

  {
    EvtBusErr,

    {                            /* DmaController status:  */
      &AsDmaDrv::dmaErrBus,      /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::dmaErrBus,      /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::dmaErrBus,      /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::dmaErrBus,      /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::dmaErrBusOnRun, /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::dmaErrBus       /*   AS_DMA_STATE_FLUSH   */
    }
  },

  {
    EvtStart,

    {                            /* DmaController status:  */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_BOOTED  */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_STOP    */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_READY   */
      &AsDmaDrv::startDma,       /*   AS_DMA_STATE_PREPARE */
      &AsDmaDrv::illegal,        /*   AS_DMA_STATE_RUN     */
      &AsDmaDrv::illegal         /*   AS_DMA_STATE_FLUSH   */
    }
  }
};

uint32_t AsDmaDrv::m_funcTblNum = sizeof(m_func_tbl) / sizeof(m_func_tbl[0]);

/*--------------------------------------------------------------------*/
void AsDmaDrv::readyQuePush(const AudioDrvDmaRunParam &dmaParam)
{
  if (!m_ready_que.push(dmaParam))
    {
      ERROR_ATTENTION(AS_MODULE_ID_AUDIO_DRIVER,
                      AS_ATTENTION_SUB_CODE_DMA_ERROR);
    }
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::readyQuePop()
{
  if (!m_ready_que.pop())
    {
      ERROR_ATTENTION(AS_MODULE_ID_AUDIO_DRIVER,
                      AS_ATTENTION_SUB_CODE_DMA_ERROR);
    }
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::runningQuePush(const AudioDrvDmaRunParam &dmaParam)
{
  if (!m_running_que.push(dmaParam))
    {
      ERROR_ATTENTION(AS_MODULE_ID_AUDIO_DRIVER,
                      AS_ATTENTION_SUB_CODE_DMA_ERROR);
    }
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::runningQuePop()
{
  if (!m_running_que.pop())
    {
      ERROR_ATTENTION(AS_MODULE_ID_AUDIO_DRIVER,
                      AS_ATTENTION_SUB_CODE_DMA_ERROR);
    }
}

/*--------------------------------------------------------------------*/
AsDmaDrv::dmaDrvFuncTbl* AsDmaDrv::searchFuncTbl(ExternalEvent event)
{
  dmaDrvFuncTbl *p_tbl = NULL;

  for (uint32_t i=0 ; i<m_funcTblNum ; i++)
    {
      if ((m_func_tbl + i)->event == event)
        {
          p_tbl = m_func_tbl + i;
          break;
        }
    }

  F_ASSERT(p_tbl);

  return p_tbl;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::parse(ExternalEvent event, void *p_param)
{
  dmaDrvFuncTbl *p_tbl = searchFuncTbl(event);

  return (this->*(p_tbl->p_func[m_state]))(p_param);
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::illegal(void *p_param)
{
  DMAC_ERR(AS_ATTENTION_SUB_CODE_ILLEGAL_REQUEST);
  dmaErrCb(E_AS_BB_DMA_ILLEGAL);

  return false;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::init(void *p_param)
{
  AudioDrvDmaInitParam *initParam =
    reinterpret_cast<AudioDrvDmaInitParam*>(p_param);

  m_ready_que.clear();
  m_running_que.clear();
  m_error_func = initParam->p_error_func;
  m_dma_byte_len = initParam->dma_byte_len;
  m_ch_num = initParam->ch_num;
  m_dmadone_func = initParam->p_dmadone_func;

  if (m_dmadone_func == NULL)
    {
      m_min_size = DMAC_MIN_SIZE_POL;
    }
  else
    {
      m_min_size = DMAC_MIN_SIZE_INT;
    }

  if (!m_level_ctrl.init(m_dmac_id, true, true))
    {
      return false;
    }

  asDmac_EnableInt();

  Chateau_EnableInterrupt(AUDMIC_IRQn);
  Chateau_EnableInterrupt(AUDI2S1_IRQn);
  Chateau_EnableInterrupt(AUDI2S2_IRQn);

  m_state = AS_DMA_STATE_READY;

  _info("READY(%d)\n", initParam->dmac_id);

  return true;
}

/*--------------------------------------------------------------------*/
E_AS AsDmaDrv::setDmaCmd(asDmacSelId dmac_id,
                         uint32_t addr,
                         uint32_t size,
                         bool nointr,
                         bool dma_run_1st)
{
  E_AS rtCode = E_AS_OK;

#ifndef CHECK_DMA_CH_SHIFT
  dma_run_1st = false;
#endif

  rtCode = startDmacWithCheck(dmac_id, addr, size, nointr, dma_run_1st);

  if (rtCode == E_AS_DMAC_ERR_START)
    {
      dmaErrCb(E_AS_BB_DMA_ERR_START);
    }
  else if (rtCode == E_AS_DMAC_BUSY)
    {
      dmaErrCb(E_AS_BB_DMA_ERR_REQUEST);
    }
  else if (rtCode != E_AS_OK)
    {
      dmaErrCb(E_AS_BB_DMA_PARAM);
    }

  return rtCode;
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::runDmaSplitRequest(AudioDrvDmaRunParam *pDmaParam,
                                  uint32_t addr,
                                  uint16_t size)
{
  uint32_t sizeCalc = 0;

  sizeCalc = size;

  while (sizeCalc != 0)
    {
      if ((m_dmac_id == AS_DMAC_SEL_AC_IN)
       && ((m_ch_num % 2) == 1)
       && (m_dma_byte_len == AS_DMAC_BYTE_WT_16BIT))
        {
          pDmaParam->split_addr = (uint32_t)m_dma_buffer[m_dma_buf_cnt];

          if (m_dma_buf_cnt == 0)
            {
              m_dma_buf_cnt = 1;
            }
          else
            {
              m_dma_buf_cnt = 0;
            }

          pDmaParam->addr_dest =
            addr + ((size - sizeCalc) * m_ch_num * m_dma_byte_len);
        }
      else
        {
          pDmaParam->split_addr =
            addr + ((size - sizeCalc) * m_ch_num * m_dma_byte_len);
        }

      /* If sample num is over 1024(DMA_BUFFER_MAX_SIZE), transfer unit is
       * divided into every 1024 samples. However, the sample num is.
       * 1024 < x <= 2048, transfer unit will be half of it. This is to
       * prevent too fewer transfer unit will be.
       */

      if (sizeCalc > DMA_BUFFER_MAX_SIZE * 2)
        {
          pDmaParam->split_size = DMA_BUFFER_MAX_SIZE;
        }
      else if ((DMA_BUFFER_MAX_SIZE < sizeCalc)
            && (sizeCalc <= DMA_BUFFER_MAX_SIZE * 2))
        {
          pDmaParam->split_size = sizeCalc / 2;
        }
      else
        {
          pDmaParam->split_size = sizeCalc;
        }

      sizeCalc -= pDmaParam->split_size;
      pDmaParam->overlap_cnt -= 1;

      readyQuePush(*pDmaParam);
    }
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::runDma(void *p_param)
{
  AudioDrvDmaRunParam dmaParam;

  dmaParam.run_dmac_param = *(reinterpret_cast<asReadDmacParam*>(p_param));

  uint32_t size1_cnt = 0;
  uint32_t size2_cnt = 0;

  if ((GetDmaDataFormat() == AS_DMA_DATA_FORMAT_RL)
   && (m_dma_byte_len == AS_DMAC_BYTE_WT_16BIT))
    {
      switch (m_dmac_id)
        {
          case AS_DMAC_SEL_I2S_OUT:
          case AS_DMAC_SEL_I2S2_OUT:
              AS_AudioDrvDmaGetSwapData(dmaParam.run_dmac_param.addr,
                                        dmaParam.run_dmac_param.size);

              if (dmaParam.run_dmac_param.size2 != 0)
                {
                  AS_AudioDrvDmaGetSwapData(dmaParam.run_dmac_param.addr2,
                                            dmaParam.run_dmac_param.size2);
                }
              break;

          default:
              break;
        }
    }

  /* Process of DMA request */

  size1_cnt = dmaParam.run_dmac_param.size / DMA_BUFFER_MAX_SIZE;
  if (dmaParam.run_dmac_param.size % DMA_BUFFER_MAX_SIZE)
    {
      size1_cnt += 1;
    }

  size2_cnt = dmaParam.run_dmac_param.size2 / DMA_BUFFER_MAX_SIZE;
  if (dmaParam.run_dmac_param.size2 % DMA_BUFFER_MAX_SIZE)
    {
      size2_cnt += 1;
    }

  dmaParam.overlap_cnt = size1_cnt + size2_cnt;

  if ((m_ready_que.size() + dmaParam.overlap_cnt) > READY_QUEUE_NUM)
    {
      _info("OVERFLOW(%d) rdy(%d)\n", m_dmac_id, m_ready_que.size());
      dmaErrCb(E_AS_BB_DMA_OVERFLOW);
    }
  else
    {
      runDmaSplitRequest(&dmaParam,
                         dmaParam.run_dmac_param.addr,
                         dmaParam.run_dmac_param.size);

      runDmaSplitRequest(&dmaParam,
                         dmaParam.run_dmac_param.addr2,
                         dmaParam.run_dmac_param.size2);
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::runDmaOnStop(void *p_param)
{
  m_ready_que.clear();
  m_running_que.clear();

  runDma(p_param);

  m_state = AS_DMA_STATE_PREPARE;

  _info("PREPARE(%d)\n", m_dmac_id);

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::runDmaOnReady(void *p_param)
{
  if (m_level_ctrl.setFadeRamp(&m_fade_required_sample) != true)
    {
      return false;
    }

  runDma(p_param);

  m_state = AS_DMA_STATE_PREPARE;

  _info("PREPARE(%d)\n", m_dmac_id);

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::runDmaOnRun(void *p_param)
{
  runDma(p_param);

  while ((m_ready_que.size() > 0)
      && (m_running_que.size() < RUNNING_QUEUE_NUM))
    {
      const AudioDrvDmaRunParam& reqParam = m_ready_que.top();

      runningQuePush(reqParam);

      setDmaCmd(reqParam.run_dmac_param.dmacId,
                reqParam.split_addr,
                reqParam.split_size,
                false,
                false);

      readyQuePop();
    }

  /* Fade control. */

  fadeControl();

  return true;
}


/*--------------------------------------------------------------------*/
bool AsDmaDrv::startDma(void *p_param)
{
  bool run_1st = true;
  E_AS rtCode = E_AS_OK;
  m_dmac_id = *(reinterpret_cast<asDmacSelId*>(p_param));

  /* Move request from ready queue to running queue (= DMA transfer queue). */

  while ((m_ready_que.size() > 0)
      && (m_running_que.size() < RUNNING_QUEUE_NUM))
    {
      const AudioDrvDmaRunParam& reqParam = m_ready_que.top();

      runningQuePush(reqParam);

      rtCode = setDmaCmd(reqParam.run_dmac_param.dmacId,
                         reqParam.split_addr,
                         reqParam.split_size,
                         false,
                         run_1st);

      run_1st = false;

      readyQuePop();

      if (rtCode == E_AS_DMAC_ERR_START)
        {
          break;
        }
    }

  if (rtCode != E_AS_DMAC_ERR_START)
    {
      clearDmacErrIntStatus(m_dmac_id);

      setDmacErrIntMask(m_dmac_id, false);

      m_state = AS_DMA_STATE_RUN;

      _info("RUN(%d)\n", m_dmac_id);
    }

  /* Volume control */

  fadeControl();

  return true;
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::dmaCmplt(void)
{
  const AudioDrvDmaRunParam& dmaParam = m_running_que.top();

  if (((m_ch_num % 2) == 1) && (m_dma_byte_len == AS_DMAC_BYTE_WT_16BIT))
    {
      AS_AudioDrvDmaGetMicInput(dmaParam.split_size,
                                m_ch_num,
                                dmaParam.split_addr,
                                (void *)dmaParam.addr_dest);
    }

  if ((GetDmaDataFormat() == AS_DMA_DATA_FORMAT_RL)
   && (m_dma_byte_len == AS_DMAC_BYTE_WT_16BIT) )
    {
      switch (m_dmac_id)
        {
          case AS_DMAC_SEL_I2S_IN:
          case AS_DMAC_SEL_I2S2_IN:
              AS_AudioDrvDmaGetSwapData(dmaParam.split_addr,
                                        dmaParam.split_size);
              break;

          default:
              break;
        }
    }

  if (dmaParam.overlap_cnt == 0)
    {
      if (m_dmadone_func != NULL)
        {
          AudioDrvDmaResult resultParam;

          resultParam.result  = E_AS_BB_DMA_OK;
          resultParam.dmac_id = dmaParam.run_dmac_param.dmacId;
          resultParam.addr1   = dmaParam.run_dmac_param.addr;
          resultParam.size1   = dmaParam.run_dmac_param.size;
          resultParam.addr2   = dmaParam.run_dmac_param.addr2;
          resultParam.size2   = dmaParam.run_dmac_param.size2;
          resultParam.endflg  = false;

          if (m_state == AS_DMA_STATE_FLUSH)
            {
              if ((m_running_que.size() <= LAST_QUEUE_COUNT)
               && (m_ready_que.size() == 0))
                {
                  resultParam.endflg  = true;
                }
            }

          (*m_dmadone_func)(&resultParam);
        }
    }

  runningQuePop();

  while ((m_ready_que.size() > 0)
      && (m_running_que.size() < RUNNING_QUEUE_NUM))
    {
      const AudioDrvDmaRunParam& reqParam = m_ready_que.top();

      runningQuePush(reqParam);

      setDmaCmd(reqParam.run_dmac_param.dmacId,
                reqParam.split_addr,
                reqParam.split_size,
                false,
                false);

      readyQuePop();
    }
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaCmpltOnRun(void *p_param)
{
  dmaCmplt();
  fadeControl();

  if (m_running_que.size() == 0)
    {
      asBca_StopDmac(m_dmac_id);

      dmaErrCb(E_AS_BB_DMA_UNDERFLOW);

      m_state = AS_DMA_STATE_PREPARE;

      _info("PREPARE(%d)\n", m_dmac_id);
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaCmpltOnFlush(void *p_param)
{
  dmaCmplt();

  if (((m_ready_que.size() + m_running_que.size()) <  FADE_QUEUE_COUNT))
    {
      volumeCtrl(true, true);
    }
  else
    {
      fadeControl();
    }

  if (m_running_que.size() == STOP_QUEUE_COUNT)
    {
      asBca_StopDmac(m_dmac_id);
      setDmacErrIntMask(m_dmac_id, true); /* TODO: should be deleted for ES */
    }
  else if (m_running_que.size() == 0 )
    {
      m_state = AS_DMA_STATE_STOP;
      _info("STOP(%d)\n", m_dmac_id);
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::stop(void *p_param)
{
  AudioDrvDmaStopParam *stopParam =
    reinterpret_cast<AudioDrvDmaStopParam*>(p_param);

  /* Populate requst of DMA stop */

  m_ready_que.clear();

  m_state = AS_DMA_STATE_STOP;

  _info("STOP(%d)\n", stopParam->dmac_id);

  m_dmac_id = stopParam->dmac_id; /* c/m for build warning */

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::stopOnRun(void *p_param)
{
  AudioDrvDmaStopParam *stopParam =
    reinterpret_cast<AudioDrvDmaStopParam*>(p_param);

  /* Populate requst of DMA stop */

  if (stopParam->stop_mode == AudioDrvDmaStopImmediate)
    {
      m_ready_que.clear();
    }

  if (((m_ready_que.size() + m_running_que.size()) <  FADE_QUEUE_COUNT))
    {
      volumeCtrl(true, true);
    }

  if ((m_ready_que.size() + m_running_que.size()) <=  STOP_QUEUE_COUNT)
    {
      asBca_StopDmac(m_dmac_id);
    }

  m_state = AS_DMA_STATE_FLUSH;

  _info("FLUSH(%d)\n", stopParam->dmac_id);

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaErrInt(void *p_param)
{
  dmaErrCb(E_AS_BB_DMA_ERR_INT);

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaErrIntOnRun(void *p_param)
{
  dmaErrInt(p_param);

  m_state = AS_DMA_STATE_READY;

  return true;
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::dmaErrCb(E_AS_BB err_code)
{
  /* If dma error, force mute. */

  volumeCtrl(false, false);

  if (m_error_func != NULL)
    {
      AudioDrvDmaError errorParam;

      errorParam.dmac_id = m_dmac_id;
      errorParam.status  = err_code;
      errorParam.state   = m_state;

      (*m_error_func)(&errorParam);
    }

  switch (err_code)
    {
      case E_AS_BB_DMA_UNDERFLOW:
          ERROR_ATTENTION(AS_MODULE_ID_AUDIO_DRIVER,
                          AS_ATTENTION_SUB_CODE_DMA_UNDERFLOW);
          break;

      case E_AS_BB_DMA_OVERFLOW:
          ERROR_ATTENTION(AS_MODULE_ID_AUDIO_DRIVER,
                          AS_ATTENTION_SUB_CODE_DMA_OVERFLOW);
          break;

      case E_AS_BB_DMA_ILLEGAL:
      case E_AS_BB_DMA_ERR_INT:
      case E_AS_BB_DMA_PARAM:
      case E_AS_BB_DMA_ERR_START:
      case E_AS_BB_DMA_ERR_REQUEST:
          ERROR_ATTENTION(AS_MODULE_ID_AUDIO_DRIVER,
                          AS_ATTENTION_SUB_CODE_DMA_ERROR);
          break;

      case E_AS_BB_DMA_ERR_BUS:
          FATAL_ATTENTION(AS_MODULE_ID_AUDIO_DRIVER,
                          AS_ATTENTION_SUB_CODE_DMA_ERROR);
          break;

      default:
          break;
    }
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaErrBus(void *p_param)
{
  dmaErrCb(E_AS_BB_DMA_ERR_BUS);

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::dmaErrBusOnRun(void *p_param)
{
  dmaErrBus(p_param);

  m_state = AS_DMA_STATE_PREPARE;

  return true;
}

/*--------------------------------------------------------------------*/
bool AsDmaDrv::getInfo(void *p_param)
{
  AudioDrvDmaInfo *dmaInfo = reinterpret_cast<AudioDrvDmaInfo*>(p_param);

  dmaInfo->result        = E_AS_BB_DMA_OK;
  dmaInfo->dmac_id       = m_dmac_id;
  dmaInfo->running_wait  = m_running_que.size();
  dmaInfo->running_empty = RUNNING_QUEUE_NUM - dmaInfo->running_wait;
  dmaInfo->ready_wait    = m_ready_que.size();
  dmaInfo->ready_empty   = READY_QUEUE_NUM - dmaInfo->ready_wait;
  dmaInfo->state         = m_state;

  return true;
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::allocDmaBuffer(asDmacSelId dmac_id)
{
  if(dmac_id == AS_DMAC_SEL_AC_IN)
    {
      m_dma_buffer[0] = (FAR uint32_t *)kmm_malloc(DMA_BUFFER_POOL_SEG_SIZE);

      if (!m_dma_buffer[0])
        {
          F_ASSERT(0);
        }

      m_dma_buffer[1] = (FAR uint32_t *)kmm_malloc(DMA_BUFFER_POOL_SEG_SIZE);

      if (!m_dma_buffer[1])
        {
          F_ASSERT(0);
        }
    }
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::freeDmaBuffer(asDmacSelId dmac_id)
{
  if(dmac_id == AS_DMAC_SEL_AC_IN)
    {
      kmm_free(m_dma_buffer[0]);
      kmm_free(m_dma_buffer[1]);
    }
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::fadeControl(void)
{
  /* Check frame validity within one transfer frame from current frame. */
  /* Is there a invalid frame within one frame (include current frame), */
  /* mute request will be publised.                                     */

  AudioDrvDmaRunParam queParam;
  uint8_t que_stored_num = m_running_que.size() + m_ready_que.size();
  bool validity = false;
#ifdef FADECTRL_BY_FADETERM
  uint32_t samples_to_transfer = 0;
#endif /* FADECTRL_BY_FADETERM */
  uint8_t cnt = 0;

  for (cnt = 0; cnt < que_stored_num; cnt++)
    {
      if (cnt < m_running_que.size())
        {
          queParam = m_running_que.at(cnt);
        }
      else
        {
          queParam = m_ready_que.at(cnt - m_running_que.size());
        }

      if (!queParam.run_dmac_param.validity)
        {
          validity = false;
          break;
        }
      else
        {
#ifdef FADECTRL_BY_FADETERM
          if (samples_to_transfer >= m_fade_required_sample)
#endif /* FADECTRL_BY_FADETERM */
          if (cnt >= 1)
            {
              validity = true;
              break;
            }
        }

#ifdef FADECTRL_BY_FADETERM
      samples_to_transfer += queParam.split_size;
#endif /* FADECTRL_BY_FADETERM */
    }

  /* If valid frame is not exist within term,
   * next frame is assume to invalide frame.
   */

  volumeCtrl(validity, false);
}

/*--------------------------------------------------------------------*/
void AsDmaDrv::volumeCtrl(bool validity, bool is_last_frame)
{
  /* Mute/unmute a.s.a.p, therefore do not send inter task message. */

  LevelCtrl::LevelCtrlCmd cmd = LevelCtrl::CmdMuteOff;

  /* Check auto fade */

  if (m_level_ctrl.getAutoFade())
    {
      if(is_last_frame)
        {
          cmd = LevelCtrl::CmdMuteOn;
        }
    }

  /* Check validity of data */

  if (!validity)
    {
      cmd = LevelCtrl::CmdMuteOn;
    }

  m_level_ctrl.exec(cmd, false);
}
