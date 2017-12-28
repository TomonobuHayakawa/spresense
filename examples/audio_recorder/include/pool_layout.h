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
  { OUTPUT_BUF_POOL , BasicType,   2, true, 0x000c0008, 0x00003000 },  /* AUDIO_WORK_AREA */
  { MIC_IN_BUF_POOL , BasicType,   5, true, 0x000c3010, 0x00008700 },  /* AUDIO_WORK_AREA */
  { ENC_APU_CMD_POOL, BasicType,   3, true, 0x000cb718, 0x00000114 },  /* AUDIO_WORK_AREA */
  { SRC_APU_CMD_POOL, BasicType,   3, true, 0x000cb838, 0x00000114 },  /* AUDIO_WORK_AREA */
 },
}; /* end of MemoryPoolLayouts */

}  /* end of namespace MemMgrLite */

#endif /* POOL_LAYOUT_H_INCLUDED */
