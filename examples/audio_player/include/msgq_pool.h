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
  { 0xfe374, 88, 4, 0xffffffff, 0, 0 }, /* MSGQ_AUD_MNG */
  { 0xfe4d4, 40, 2, 0xffffffff, 0, 0 }, /* MSGQ_AUD_APP */
  { 0xfe524, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_DSP */
  { 0xfe588, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PFDSP0 */
  { 0xfe5ec, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PFDSP1 */
  { 0xfe650, 48, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_PLY */
  { 0xfe740, 48, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_SFX */
  { 0xfe830, 48, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_OUTPUT_MIX */
  { 0xfe9b0, 32, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_PLY */
  { 0xfebb0, 16, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_PLY_SYNC */
  { 0xfec30, 32, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_SFX */
  { 0xfee30, 16, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RND_SFX_SYNC */
};

#endif /* MSGQ_POOL_H_INCLUDED */
