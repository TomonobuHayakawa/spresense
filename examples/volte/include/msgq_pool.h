/*
 * msgq_pool.h -- Message queue pool layout definition.
 *
 * This file was created by msgq_layout.conf
 * !!! CAUTION! don't edit this file manually !!!
 *
 *   Notes: (C) Copyright 2012 Sony Corporation
 */

#ifndef MSGQ_POOL_H_INCLUDED
#define MSGQ_POOL_H_INCLUDED

#include "msgq_id.h"

extern const MsgQueDef MsgqPoolDefs[NUM_MSGQ_POOLS] = {
   /* n_drm, n_size, n_num, h_drm, h_size, h_num */
  { 0x00000000, 0, 0, 0x00000000, 0, 0, 0 }, /* MSGQ_NULL */
  { 0xfc440, 88, 3, 0xffffffff, 0, 0 }, /* MSGQ_AUD_MGR */
  { 0xfc548, 40, 2, 0xffffffff, 0, 0 }, /* MSGQ_AUD_APP */
  { 0xfc598, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_DSP */
  { 0xfc5fc, 48, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PLY */
  { 0xfc6ec, 48, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_OUTPUT_MIX */
  { 0xfc86c, 52, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_SOUND_EFFECT */
  { 0xfc970, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RCG_CMD */
  { 0xfc9d4, 24, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_CAP_MIC */
  { 0xfcb54, 16, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_CAP_MIC_SYNC */
  { 0xfcbd4, 24, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_CAP_I2S */
  { 0xfcd54, 16, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_CAP_I2S_SYNC */
  { 0xfcdd4, 32, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_SPHP */
  { 0xfcfd4, 16, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_SPHP_SYNC */
  { 0xfd054, 32, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_I2S */
  { 0xfd254, 16, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_I2S_SYNC */
};

#endif /* MSGQ_POOL_H_INCLUDED */
