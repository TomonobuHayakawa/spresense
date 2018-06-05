/*
 * msgq_id.h -- Message queue pool ID and macro definition.
 *
 * This file was created by msgq_layout.conf
 * !!! CAUTION! don't edit this file manually !!!
 *
 *   Notes: (C) Copyright 2012 Sony Corporation
 */

#ifndef MSGQ_ID_H_INCLUDED
#define MSGQ_ID_H_INCLUDED

/* Message area size: 1672 bytes */
#define MSGQ_TOP_DRM	0xfe000
#define MSGQ_END_DRM	0xfe688

/* Message area fill value after message poped */
#define MSG_FILL_VALUE_AFTER_POP	0x0

/* Message parameter type match check */
#define MSG_PARAM_TYPE_MATCH_CHECK	false

/* Message queue pool IDs */
#define MSGQ_NULL	0
#define MSGQ_AUD_MGR	1
#define MSGQ_AUD_APP	2
#define MSGQ_AUD_DSP	3
#define MSGQ_AUD_RECORDER	4
#define MSGQ_AUD_CAP	5
#define MSGQ_AUD_CAP_SYNC	6
#define NUM_MSGQ_POOLS	7

/* User defined constants */

/************************************************************************/
#define MSGQ_AUD_MGR_QUE_BLOCK_DRM	0xfe044
#define MSGQ_AUD_MGR_N_QUE_DRM	0xfe1dc
#define MSGQ_AUD_MGR_N_SIZE	88
#define MSGQ_AUD_MGR_N_NUM	3
#define MSGQ_AUD_MGR_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_MGR_H_SIZE	0
#define MSGQ_AUD_MGR_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_APP_QUE_BLOCK_DRM	0xfe088
#define MSGQ_AUD_APP_N_QUE_DRM	0xfe2e4
#define MSGQ_AUD_APP_N_SIZE	40
#define MSGQ_AUD_APP_N_NUM	2
#define MSGQ_AUD_APP_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_APP_H_SIZE	0
#define MSGQ_AUD_APP_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_DSP_QUE_BLOCK_DRM	0xfe0cc
#define MSGQ_AUD_DSP_N_QUE_DRM	0xfe334
#define MSGQ_AUD_DSP_N_SIZE	20
#define MSGQ_AUD_DSP_N_NUM	5
#define MSGQ_AUD_DSP_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_DSP_H_SIZE	0
#define MSGQ_AUD_DSP_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_RECORDER_QUE_BLOCK_DRM	0xfe110
#define MSGQ_AUD_RECORDER_N_QUE_DRM	0xfe398
#define MSGQ_AUD_RECORDER_N_SIZE	48
#define MSGQ_AUD_RECORDER_N_NUM	5
#define MSGQ_AUD_RECORDER_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_RECORDER_H_SIZE	0
#define MSGQ_AUD_RECORDER_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_CAP_QUE_BLOCK_DRM	0xfe154
#define MSGQ_AUD_CAP_N_QUE_DRM	0xfe488
#define MSGQ_AUD_CAP_N_SIZE	24
#define MSGQ_AUD_CAP_N_NUM	16
#define MSGQ_AUD_CAP_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_CAP_H_SIZE	0
#define MSGQ_AUD_CAP_H_NUM	0
/************************************************************************/
#define MSGQ_AUD_CAP_SYNC_QUE_BLOCK_DRM	0xfe198
#define MSGQ_AUD_CAP_SYNC_N_QUE_DRM	0xfe608
#define MSGQ_AUD_CAP_SYNC_N_SIZE	16
#define MSGQ_AUD_CAP_SYNC_N_NUM	8
#define MSGQ_AUD_CAP_SYNC_H_QUE_DRM	0xffffffff
#define MSGQ_AUD_CAP_SYNC_H_SIZE	0
#define MSGQ_AUD_CAP_SYNC_H_NUM	0
#endif /* MSGQ_ID_H_INCLUDED */
