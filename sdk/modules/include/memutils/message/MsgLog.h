/****************************************************************************
 * modules/include/memutils/message/MsgLog.h
 *
 *   Copyright (C) 2012,2013,2014 Sony Corporation
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
 * 3. Neither the name Sony nor the names of its contributors
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

#ifndef MSG_LOG_H_INCLUDED
#define MSG_LOG_H_INCLUDED

#include "memutils/common_utils/common_types.h"
//#include "dmp_id.h"

struct MsgLog {
	uint8_t		m_kind;		/* log kind */
	uint8_t		m_task;		/* call task. 0 is ISR */
	uint16_t	m_type;		/* message type */
	uint16_t	m_pri_dest;	/* priority (MSB) and destination queue ID */
	uint16_t	m_reply;	/* reply queue ID */
	uint8_t		m_src_cpu;	/* source CPU-ID */
	uint8_t		m_stored;	/* stored count of target queue */
	uint16_t	m_param_size;	/* parameter size */
	uint32_t	m_param;	/* parameter (head 4byte) */
public:
	MsgLog(uint8_t kind, uint16_t type, uint16_t dest, uint8_t pri, uint16_t reply,
			uint8_t src_cpu, uint16_t stored, uint16_t param_size, uint32_t param) :
		m_kind(kind),
//		m_task(DMP_GET_TASK_ID),
		m_task(0),
		m_type(type),
		m_pri_dest((pri << 15) | dest),
		m_reply(reply),
		m_src_cpu(src_cpu),
		m_stored(static_cast<uint8_t>(stored)),
		m_param_size(param_size),
		m_param(param)
		{}

	MsgLog(uint8_t kind, uint16_t dest, uint8_t pri, uint16_t stored, MsgPacket* msg) :
		m_kind(kind),
//		m_task(DMP_GET_TASK_ID),
		m_task(0),
		m_type(msg->getType()),
		m_pri_dest((pri << 15) | dest),
		m_reply(msg->getReply()),
		m_src_cpu(msg->getSrcCpu()),
		m_stored(static_cast<uint8_t>(stored)),
		m_param_size(msg->getParamSize()),
		m_param(msg->peekParamHead())
		{}
}; /* struct MsgLog */

/* LogAnalyzerの共用化のため、同じ構造体を使用して、m_kindフィールドで切り分ける */
typedef MsgLog MsgSeqLog;	/* m_kind: 's': send, 'i': sendIsr, 'r': recv */
typedef MsgLog MsgRetryLog;	/* m_kind: 'b': begin retry, 'n': normal end, 'e': error */

/* dmp_layout.confのDMP_MSG_SEQ2エントリがtrueの時に有効 */
#if defined(DMP_MSG_SEQ2_NUM) && DMP_MSG_SEQ2_NUM != 0
	/* 複数タスクとISRのログをシーケンシャルに格納するため、割込み禁止を使用 */
#define DUMP_MSG_SEQ_LOCK(p)	do { InterruptLock _lock_; DMP_MSG_SEQ2_SEQ_LOG(p); } while (0)
#define DUMP_MSG_SEQ(p)		DMP_MSG_SEQ2_SEQ_LOG(p)
#else
#define DUMP_MSG_SEQ_LOCK(p)
#define DUMP_MSG_SEQ(p)
#endif /* defined(DMP_MSG_SEQ2_NUM) && DMP_MSG_SEQ2_NUM != 0 */

struct MsgPeakLog {
	uint16_t	m_stored;		/* stored count of target queue */
	uint8_t		m_task;			/* call task. 0 is ISR */
	uint8_t		resv;			/* reserved */
	uint16_t	m_msg_type;		/* message type */
	uint16_t	m_msg_reply;		/* reply queue ID */
	uint16_t	m_top_msg_type;		/* message type */
	uint16_t	m_top_msg_reply;	/* reply queue ID */
public:
	MsgPeakLog(uint16_t stored, MsgPacket* msg, MsgPacket* top_msg) :
		m_stored(stored),
//		m_task(DMP_GET_TASK_ID),
		m_task(0),
		m_msg_type(msg->getType()),
		m_msg_reply(msg->getReply()),
		m_top_msg_type(top_msg->getType()),
		m_top_msg_reply(top_msg->getReply())
		{}
}; /* struct MsgPeakLog */

/* dmp_layout.confのDMP_MSG_PEAKエントリがtrueの時に有効 */
#if defined(DMP_MSG_PEAK_NUM) && DMP_MSG_PEAK_NUM != 0
#define DUMP_MSG_PEAK(id, pri, p)	DMP_MSG_PEAK_IDX_LOG((((id) - 1) * NumMsgPri + (pri)), (p))
#else
#define DUMP_MSG_PEAK(id, pri, p)
#endif /* defined(DMP_MSG_PEAK_NUM) && DMP_MSG_PEAK_NUM != 0 */

/* dmp_layout.confにDMP_MSGLIB_NUMエントリがない場合は、ここで空定義しておく */
#ifndef DMP_MSGLIB_NUM
#define DMP_MSGLIB_DEBUG_PRINT(...)
#define DMP_MSGLIB_DEBUG(p)
#define DMP_MSGLIB_INFO(p)
#define DMP_MSGLIB_NOTICE(p)
#define DMP_MSGLIB_WARN(p)
#define DMP_MSGLIB_ERROR(p)
#endif /* DMP_MEMMGR_NUM */

#endif /* MSG_LOG_H_INCLUDED */
