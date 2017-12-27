############################################################################
# examples/volte/Makefile
#
#   Copyright (C) 2017 Sony Corporation.
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

CFLAGS   += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "./include"}
CFLAGS   += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(SDKDIR)/modules/include/net"}
CXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "./include"}
CXXFLAGS += ${shell $(INCDIR) $(INCDIROPT) "$(CC)" "$(SDKDIR)/modules/include/net"}

CFLAGS += -D_POSIX

CFLAGS += -DENABLE_LTE_UTILITY_MODULE
CFLAGS += -DENABLE_LTE_SMS_MODULE
CFLAGS += -DENABLE_LTE_VOLTE_MODULE

CXXFLAGS += -D_POSIX

CXXFLAGS += -DUNUSE_SPINLOCK_MANAGER
CXXFLAGS += -DUSE_MEMMGR_FENCE

# volte built-in application info

CONFIG_EXAMPLES_VOLTE_PRIORITY ?= SCHED_PRIORITY_DEFAULT
CONFIG_EXAMPLES_VOLTE_STACKSIZE ?= 2048

APPNAME = volte
PRIORITY = $(CONFIG_EXAMPLES_VOLTE_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_VOLTE_STACKSIZE)

# volte Example

ASRCS =
CXXSRCS = volte_audio.cxx
MAINSRC = volte_main.c

CONFIG_EXAMPLES_VOLTE_PROGNAME ?= volte$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_VOLTE_PROGNAME)

include $(APPDIR)/Application.mk
