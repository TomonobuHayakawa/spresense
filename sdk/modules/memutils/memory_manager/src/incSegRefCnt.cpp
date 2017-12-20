/********************************************************************************
 * apps/memutils/memory_manager/src/incSegRefcnt.cpp
 *
 * Description: Memory Manager Lite's incSegRefCnt implement.
 *
 *   Copyright (C) 2014-16 Sony Corporation. All rights reserved.
 *   Author: Satoru AIZAWA
 *           Tomonobu Hayakawa
 *           Masahiro Takeyama
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
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
 ********************************************************************************/

#include "ScopedLock.h"
#include "memutils/memory_manager/MemHandleBase.h"

namespace MemMgrLite {

/*****************************************************************
 * メモリハンドルのコピー代入演算子
 *****************************************************************/
MemHandleBase&	MemHandleBase::operator=(const MemHandleBase& mh)
{
	if (!isSame(mh)) {
		freeSeg();
		m_proxy = mh.m_proxy;
		if (mh.isAvail()) {
			Manager::incSegRefCnt(mh.getPoolId(), mh.getSegNo());
		}
	}
	return *this;
}

/*****************************************************************
 * メモリプールの該当セグメントの参照カウントを増やす
 *****************************************************************/
void MemPool::incSegRefCnt(NumSeg seg_no)
{
	D_ASSERT(seg_no != NullSegNo && seg_no <= getPoolNumSegs());

	NumSeg ref_idx = seg_no - 1;
	D_ASSERT(m_ref_cnt_array[ref_idx] != 0);	/* 使用中のはず */

	ScopedLock lock;
	++m_ref_cnt_array[ref_idx];
	D_ASSERT(m_ref_cnt_array[ref_idx] != 0);	/* ラップチェック */
}

} /* end of namespace MemMgrLite */

/* incSegRefCnt.cxx */
