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

/* Message area size: 636 bytes */
#define MSGQ_TOP_DRM  0xc0000
#define MSGQ_END_DRM  0xc027c

/* Message area fill value after message poped */
#define MSG_FILL_VALUE_AFTER_POP  0x0

/* Message parameter type match check */
#define MSG_PARAM_TYPE_MATCH_CHECK  false

/* Message queue pool IDs */
#define MSGQ_NULL  0
#define MSGQ_AUD_MGR  1
#define MSGQ_AUD_APP  2
#define NUM_MSGQ_POOLS  3

/* User defined constants */

/************************************************************************/
#define MSGQ_AUD_MGR_QUE_BLOCK_DRM  0xc0044
#define MSGQ_AUD_MGR_N_QUE_DRM  0xc00cc
#define MSGQ_AUD_MGR_N_SIZE  88
#define MSGQ_AUD_MGR_N_NUM  4
#define MSGQ_AUD_MGR_H_QUE_DRM  0xffffffff
#define MSGQ_AUD_MGR_H_SIZE  0
#define MSGQ_AUD_MGR_H_NUM  0
/************************************************************************/
#define MSGQ_AUD_APP_QUE_BLOCK_DRM  0xc0088
#define MSGQ_AUD_APP_N_QUE_DRM  0xc022c
#define MSGQ_AUD_APP_N_SIZE  40
#define MSGQ_AUD_APP_N_NUM  2
#define MSGQ_AUD_APP_H_QUE_DRM  0xffffffff
#define MSGQ_AUD_APP_H_SIZE  0
#define MSGQ_AUD_APP_H_NUM  0
#endif /* MSGQ_ID_H_INCLUDED */
