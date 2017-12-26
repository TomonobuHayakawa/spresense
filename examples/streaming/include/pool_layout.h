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
  { DEC_ES_MAIN_BUF_POOL, BasicType,   4, true, 0x000c0008, 0x00006000 },  /* AUDIO_WORK_AREA */
  { REND_PCM_BUF_POOL, BasicType,   9, true, 0x000c6010, 0x00012000 },  /* AUDIO_WORK_AREA */
  { REND_PCM_SUB_BUF_POOL, BasicType,   9, true, 0x000d8018, 0x00012000 },  /* AUDIO_WORK_AREA */
  { DEC_APU_CMD_POOL, BasicType,  10, true, 0x000ea020, 0x00000398 },  /* AUDIO_WORK_AREA */
  { DEC_ES_SUB_BUF_POOL, BasicType,   4, true, 0x000ea3c0, 0x00003000 },  /* AUDIO_WORK_AREA */
 },
}; /* end of MemoryPoolLayouts */

}  /* end of namespace MemMgrLite */

#endif /* POOL_LAYOUT_H_INCLUDED */
