/****************************************************************************
 * apps/examples/volte/include/memory_layout.h
 *
 *   Copyright (C) 2016-2017 Sony Corporation. All rights reserved.
 *   Author: Suzunosuke Hida<Suzunosuke.Hida@sony.com>
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
 
#ifndef __APPS_EXAMPLES_VOLTE_INCLUDE_MEMORY_LAYOUT_H
#define __APPS_EXAMPLES_VOLTE_INCLUDE_MEMORY_LAYOUT_H

#define MEM_LAYOUT_PLAYER       (0x00)
#define MEM_LAYOUT_SOUNDEFFECT  (0x01)
#define MEM_LAYOUT_RECORDER     (0xff)
#define MEM_LAYOUT_VOICETRIGGER (0xff)
#define MEM_LAYOUT_OUTPUTMIX    (0xff)

#endif /* __APPS_EXAMPLES_VOLTE_INCLUDE_MEMORY_LAYOUT_H */
