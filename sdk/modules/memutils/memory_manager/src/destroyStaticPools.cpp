/****************************************************************************
 * modules/memutils/memory_manager/src/destroyStaticPool.cpp
 *
 *   Copyright (C) 2014,2015,2016 Sony Corporation
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

#include "ScopedLock.h"
#include "memutils/memory_manager/Manager.h"

namespace MemMgrLite {

/*****************************************************************
 * 静的メモリプール群の破棄(削除)
 * 静的メモリプール未生成時は、何もしない
 *****************************************************************/
void Manager::destroyStaticPools()
{
#ifdef USE_MEMMGR_DEBUG_OUTPUT
	printf("Manager::destroyStaticPools: layout_no=%d\n", getCurrentLayoutNo());
#endif

	if (isStaticPoolAvailable()) {
		ScopedLock lock;

		/* プールID=0は予約 */
		for (uint32_t i = 1; i < theManager->m_pool_num; ++i) {
			if (theManager->m_static_pools[i] != NULL) {
				destroyPool(theManager->m_static_pools[i]);
				theManager->m_static_pools[i] = NULL;
			}
		}
		theManager->m_layout_no = BadLayoutNo;	/* レイアウト番号を無効化 */
	}
}

} /* end of namespace MemMgrLite */

/* destroyStaticPools.cxx */
