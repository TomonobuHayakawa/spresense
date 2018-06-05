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
  { 0xfe1dc, 88, 3, 0xffffffff, 0, 0 }, /* MSGQ_AUD_MGR */
  { 0xfe2e4, 40, 2, 0xffffffff, 0, 0 }, /* MSGQ_AUD_APP */
  { 0xfe334, 20, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_DSP */
  { 0xfe398, 48, 5, 0xffffffff, 0, 0 }, /* MSGQ_AUD_RECORDER */
  { 0xfe488, 24, 16, 0xffffffff, 0, 0 }, /* MSGQ_AUD_CAP */
  { 0xfe608, 16, 8, 0xffffffff, 0, 0 }, /* MSGQ_AUD_CAP_SYNC */
};

#endif /* MSGQ_POOL_H_INCLUDED */
