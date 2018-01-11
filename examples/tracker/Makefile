############################################################################
# apps/examples/tracker/Makefile
#
#   Copyright (C) 2017 Sony Corporation. All rights reserved.
#   Author: Yutaka Miyajima <Yutaka.Miyajima@sony.com>
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

ifeq ($(WINTOOL),y)
CFLAGS += -isystem "$(shell cygpath -w $(SDKDIR)/modules/include/net)"
CXXFLAGS += -isystem "$(shell cygpath -w $(SDKDIR)/modules/include/net)"
else
CFLAGS += -isystem $(SDKDIR)/modules/include/net
CXXFLAGS += -isystem $(SDKDIR)/modules/include/net
endif

CFLAGS += -DLWIP_IPV6=0

ifeq ($(WINTOOL),y)
TRACKERINC += "${shell cygpath -w ./include}"
else
TRACKERINC += "./include"
endif

CXXFLAGS += -I$(TRACKERINC)
CFLAGS += -I$(TRACKERINC)

# Tracker Example

ASRCS =
CSRCS =
CXXSRCS = 
MAINSRC = tracker_main.c tracker_location.c tracker_net_client.c

ifeq ($(CONFIG_EXAMPLES_TRACKER_TAP_TRIGGER),y)
CXXSRCS += tracker_sensing_tap.cxx
endif

ifeq ($(CONFIG_EXAMPLES_TRACKER_BUTTON_TRIGGER),y)
MAINSRC += tracker_button.c
endif

ifeq ($(CONFIG_EXAMPLES_TRACKER_USING_NXFB),y)
MAINSRC += tracker_nxfb.c tracker_nxfb_bkgd.c
endif

CONFIG_EXAMPLES_TRACKER_PROGNAME ?= tracker$(EXEEXT)
PROGNAME = $(CONFIG_EXAMPLES_TRACKER_PROGNAME)

# tracker built-in application info

APPNAME = tracker
PRIORITY = $(CONFIG_EXAMPLES_TRACKER_PRIORITY)
STACKSIZE = $(CONFIG_EXAMPLES_TRACKER_STACKSIZE)

include $(APPDIR)/Application.mk
