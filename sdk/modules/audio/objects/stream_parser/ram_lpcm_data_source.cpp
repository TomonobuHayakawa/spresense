/****************************************************************************
 * modules/audio/objects/stream_parser/ram_lpcm_data_source.cpp
 *
 *   Copyright (C)  2017 Sony Corporation
 *   Author: Tomonobu Hayakawa <Tomonobu.Hayakawa@sony.com>
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

#include "ram_lpcm_data_source.h"
#include "debug/dbg_log.h"

__WIEN2_BEGIN_NAMESPACE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

bool RawLpcmDataSource::init(const InitInputDataManagerParam &param)
{
  if (!checkSimpleFifoHandler(param))
    {
      return false;
    }
  setInitParam(param);
  return true;
}

InputDataManagerObject::GetEsResult
  RawLpcmDataSource::getEs(FAR void *es_buf, FAR uint32_t *es_size)
{
  size_t size = 0;
  if (!getOccupiedSize(&size))
    {
      return EsEnd;
    }
  uint32_t read_size = 0;
  if (size >= *es_size)
    {
      read_size = *es_size;
    }
  else
    {
      read_size = size;
    }
  size_t poll_size = 0;
  if (!simpleFifoPoll(es_buf, read_size, &poll_size))
    {
      return EsEnd;
    }
  *es_size = poll_size;
  return EsExist;
}

bool RawLpcmDataSource::finish()
{
  return true;
}

bool RawLpcmDataSource::getSamplingRate(FAR uint32_t* p_sampling_rate)
{
  *p_sampling_rate = m_in_sampling_rate;
  return true;
}

bool RawLpcmDataSource::getChNum(FAR uint32_t* p_ch_num)
{
  *p_ch_num = m_ch_num;
  return true;
}

bool RawLpcmDataSource::getBitPerSample(FAR uint32_t *p_bit_per_sample)
{
  /* Currently not implemented. */

  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__WIEN2_END_NAMESPACE
