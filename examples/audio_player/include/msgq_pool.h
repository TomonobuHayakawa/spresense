/****************************************************************************
 * audio_player/include/msgq_pool.h
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
 * 3. Neither the name of Sony Semiconductor Solutions Corporation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
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

#ifndef MSGQ_POOL_H_INCLUDED
#define MSGQ_POOL_H_INCLUDED

#include "msgq_id.h"

extern const MsgQueDef MsgqPoolDefs[NUM_MSGQ_POOLS] = {
   /* n_drm, n_size, n_num, h_drm, h_size, h_num */
  { 0x00000000, 0, 0, 0x00000000, 0, 0, 0 }, /* MSGQ_NULL */
  { 0xfe374, 88, 4, 0xffffffff, 0, 0 }, /* MSGQ_AUD_MNG */
  { 0xfe4d4, 64, 2, 0xffffffff, 0, 0 }, /* MSGQ_AUD_APP */
  { 0xfe554, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_DSP */
  { 0xfe5b8, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PFDSP0 */
  { 0xfe61c, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PFDSP1 */
  { 0xfe680, 48, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PLY */
  { 0xfe770, 48, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_SFX */
  { 0xfe860, 48, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_OUTPUT_MIX */
  { 0xfe9e0, 32, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_PLY */
  { 0xfebe0, 16, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_PLY_SYNC */
  { 0xfec60, 32, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_SFX */
  { 0xfee60, 16, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_SFX_SYNC */
};

#endif /* MSGQ_POOL_H_INCLUDED */
