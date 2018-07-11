############################################################################
# examples/fft/Makefile
#
#   Copyright 2018 Sony Semiconductor Solutions Corporation
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
# 3. Neither the name of Sony Semiconductor Solutions Corporation nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
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

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
include $(APPDIR)/Make.defs

# ELF Example

APPNAME = fft
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = 4096

ASRCS =
CSRCS =
MAINSRC = fft_main.c
CSRCS += arm_convolution_example_f32.c math_helper.c
CSRCS += dsp_rpc.c

ifeq ($(CONFIG_EXAMPLES_FFT_DISABLE_MATH_OFFLOAD),y)

# DSP/Source/TransformFunctions

ASRCS += arm_bitreversal2.S
CSRCS += arm_bitreversal.c
CSRCS += arm_cfft_f32.c
CSRCS += arm_cfft_q15.c
CSRCS += arm_cfft_q31.c
CSRCS += arm_cfft_radix2_f32.c
CSRCS += arm_cfft_radix2_init_f32.c
CSRCS += arm_cfft_radix2_init_q15.c
CSRCS += arm_cfft_radix2_init_q31.c
CSRCS += arm_cfft_radix2_q15.c
CSRCS += arm_cfft_radix2_q31.c
CSRCS += arm_cfft_radix4_f32.c
CSRCS += arm_cfft_radix4_init_f32.c
CSRCS += arm_cfft_radix4_init_q15.c
CSRCS += arm_cfft_radix4_init_q31.c
CSRCS += arm_cfft_radix4_q15.c
CSRCS += arm_cfft_radix4_q31.c
CSRCS += arm_cfft_radix8_f32.c
CSRCS += arm_dct4_f32.c
CSRCS += arm_dct4_init_f32.c
CSRCS += arm_dct4_init_q15.c
CSRCS += arm_dct4_init_q31.c
CSRCS += arm_dct4_q15.c
CSRCS += arm_dct4_q31.c
CSRCS += arm_rfft_f32.c
CSRCS += arm_rfft_fast_f32.c
CSRCS += arm_rfft_fast_init_f32.c
CSRCS += arm_rfft_init_f32.c
CSRCS += arm_rfft_init_q15.c
CSRCS += arm_rfft_init_q31.c
CSRCS += arm_rfft_q15.c
CSRCS += arm_rfft_q31.c

# DSP/Source/SupportFunctions

CSRCS += arm_fill_f32.c
CSRCS += arm_copy_f32.c

# DSP/Source/ComplexMathFunctions

CSRCS += arm_cmplx_mult_cmplx_f32.c

# DSP/Source/CommonTables

CSRCS += arm_common_tables.c

endif

# Predefined must be specify for building CMSIS-DSP libraries

CFLAGS += -DARM_MATH_CM4 -D__FPU_PRESENT=1

CFLAGS += -Idsp/CMSIS/Core/Include
CFLAGS += -Idsp/CMSIS/DSP/Include

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))
MAINOBJ = $(MAINSRC:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS) $(MAINSRC)
OBJS = $(AOBJS) $(COBJS)

ifneq ($(CONFIG_BUILD_KERNEL),y)
  OBJS += $(MAINOBJ)
endif

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BIN = ..\libexamples$(LIBEXT)
else
ifeq ($(WINTOOL),y)
  BIN = ..\\libexamples$(LIBEXT)
else
  BIN = ../libexamples$(LIBEXT)
endif
endif

ifeq ($(WINTOOL),y)
  INSTALL_DIR = "${shell cygpath -w $(BIN_DIR)}"
else
  INSTALL_DIR = $(BIN_DIR)
endif

CONFIG_FFT_TEST_PROGNAME ?= fft$(EXEEXT)
PROGNAME = $(CONFIG_FFT_TEST_PROGNAME)

ROOTDEPPATH  = --dep-path .

ifeq ($(CONFIG_EXAMPLES_FFT_DISABLE_MATH_OFFLOAD),y)

ROOTDEPPATH += --dep-path dsp/CMSIS/DSP/Source/TransformFunctions
ROOTDEPPATH += --dep-path dsp/CMSIS/DSP/Source/SupportFunctions
ROOTDEPPATH += --dep-path dsp/CMSIS/DSP/Source/ComplexMathFunctions
ROOTDEPPATH += --dep-path dsp/CMSIS/DSP/Source/CommonTables

# Build targets

VPATH  = dsp/CMSIS/DSP/Source/TransformFunctions
VPATH += dsp/CMSIS/DSP/Source/SupportFunctions
VPATH += dsp/CMSIS/DSP/Source/ComplexMathFunctions
VPATH += dsp/CMSIS/DSP/Source/CommonTables

endif

all: .built
.PHONY: .built really_build clean_dsp clean depend distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(MAINOBJ): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

# This is a little messy.  The build is broken into two pieces:  (1) the
# dsp/ subdir build that auto-generates several files, and (2) the real
# build.  This is done because we need a fresh build context after auto-
# generating the source files.

really_build: $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	@touch .built

.built:
ifneq ($(CONFIG_EXAMPLES_FFT_DISABLE_MATH_OFFLOAD),y)
	@$(MAKE) -C dsp TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV)
endif
	@$(MAKE) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" really_build

ifeq ($(CONFIG_BUILD_KERNEL),y)
$(BIN_DIR)$(DELIM)$(PROGNAME): $(OBJS) $(MAINOBJ)
	@echo "LD: $(PROGNAME)"
	$(Q) $(LD) $(LDELFFLAGS) $(LDLIBPATH) -o $(INSTALL_DIR)$(DELIM)$(PROGNAME) $(ARCHCRT0OBJ) $(MAINOBJ) $(LDLIBS)
	$(Q) $(NM) -u  $(INSTALL_DIR)$(DELIM)$(PROGNAME)

install: $(BIN_DIR)$(DELIM)$(PROGNAME)

else
install:

endif

ifeq ($(CONFIG_NSH_BUILTIN_APPS),y)
$(BUILTIN_REGISTRY)$(DELIM)$(APPNAME)_main.bdat: $(DEPCONFIG) Makefile
	$(call REGISTER,$(APPNAME),$(PRIORITY),$(STACKSIZE),$(APPNAME)_main)

context: $(BUILTIN_REGISTRY)$(DELIM)$(APPNAME)_main.bdat
else
context:
endif

.depend:
	@$(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	@touch $@

depend: .depend

clean_dsp:
	@$(MAKE) -C dsp TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" CROSSDEV=$(CROSSDEV) clean

clean: clean_dsp
	$(call DELFILE, .built)
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
.PHONY: preconfig
preconfig:
