############################################################################
# apps/examples/lte/Makefile
#
#   Copyright (C) 2016 Sony Corporation. All rights reserved.
#   Author: Mitsuo Hiragane <Mitsuo.Hiragane@sony.com>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor Sony nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

AUDINC =
AUDINC += -I./include

CFLAGS += $(AUDINC)

CFLAGS += -D_POSIX

CXXFLAGS += $(AUDINC)

CXXFLAGS += -D_POSIX

CXXFLAGS += -DUSE_MEMMGR_FENCE

# volte built-in application info

CONFIG_EXAMPLES_STREAMING_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_EXAMPLES_STREAMING_STACKSIZE ?= 2048

APPNAME = streaming
PRIORITY = $(CONFIG_EXAMPLES_STREAMING_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_STREAMING_STACKSIZE)

# streaming Example

ASRCS =
CXXSRCS = memutils_wrapper.cxx
MAINSRC = streaming_main.c streaming_audio.c streaming_playlist.c streaming_lte.c

CONFIG_EXAMPLES_STREAMING_PROGNAME ?= streaming$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_STREAMING_PROGNAME)

include $(APPDIR)/Application.mk
