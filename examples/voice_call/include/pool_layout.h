/*
 * pool_layout.h -- MemMgrLite layout definition.
 *
 * This file was created by mem_layout.conf
 * !!! CAUTION! don't edit this file manually !!!
 *
 *   Notes: (C) Copyright 2014 Sony Corporation
 */
#ifndef POOL_LAYOUT_H_INCLUDED
#define POOL_LAYOUT_H_INCLUDED

#include "memutils/memory_manager/MemMgrTypes.h"

namespace MemMgrLite {

MemPool* static_pools[NUM_MEM_POOLS];

extern const PoolAttr MemoryPoolLayouts[NUM_MEM_LAYOUTS][NUM_MEM_POOLS] = {
 {/* Layout:0 */
  /* pool_ID          type       seg fence  addr        size         */
  { MIC_IN_BUF_POOL , BasicType,   5, true, 0x000c0008, 0x00000960 },  /* AUDIO_WORK_AREA */
  { I2S_IN_BUF_POOL , BasicType,   5, true, 0x000c0970, 0x000012c0 },  /* AUDIO_WORK_AREA */
  { HP_OUT_BUF_POOL , BasicType,   5, true, 0x000c1c38, 0x000012c0 },  /* AUDIO_WORK_AREA */
  { I2S_OUT_BUF_POOL, BasicType,   5, true, 0x000c2f00, 0x000012c0 },  /* AUDIO_WORK_AREA */
  { MFE_OUT_BUF_POOL, BasicType,   8, true, 0x000c41c8, 0x00000500 },  /* AUDIO_WORK_AREA */
 },
}; /* end of MemoryPoolLayouts */

}  /* end of namespace MemMgrLite */

#endif /* POOL_LAYOUT_H_INCLUDED */
