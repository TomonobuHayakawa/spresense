############################################################################
# examples/fft/dsp/math/Makefile
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

-include $(TOPDIR)/Make.defs

ifeq ($(WINTOOL),y)
NUTTXLIB = "${shell cygpath -w $(TOPDIR)$(DELIM)lib}"
else
NUTTXLIB = "$(TOPDIR)$(DELIM)lib"
endif

LDLIBPATH +=  -L $(NUTTXLIB)

LIBGCC = "$(shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name)"
LDLIBS += -lasmpw

BIN = dspmath

CSRCS = dspmath_main.c

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
CSRCS += arm_cmplx_mult_cmplx_q15.c
CSRCS += arm_cmplx_mult_cmplx_q31.c

# DSP/Source/CommonTables

CSRCS += arm_common_tables.c
CSRCS += arm_const_structs.c

# DSP/Source/BasicMathFunctions

CSRCS += arm_scale_f32.c
CSRCS += arm_mult_f32.c
CSRCS += arm_mult_q15.c
CSRCS += arm_mult_q31.c
CSRCS += arm_shift_q15.c
CSRCS += arm_shift_q31.c

CFLAGS += -O3
ifeq ($(WINTOOL),y)
CFLAGS += -I"$(shell cygpath -w $(APPDIR))"
CFLAGS += -I"$(shell cygpath -w $(TOPDIR)/asmp/worker)"
CFLAGS += -I"$(shell cygpath -w $(APPDIR)/examples/fft)"
CFLAGS += -I"$(shell cygpath -w $(APPDIR)/examples/fft/dsp/CMSIS/Core/Include)"
CFLAGS += -I"$(shell cygpath -w $(APPDIR)/examples/fft/dsp/CMSIS/DSP/Include)"
else
CFLAGS += -I$(APPDIR)
CFLAGS += -I$(TOPDIR)/asmp/worker
CFLAGS += -I$(APPDIR)/examples/fft
CFLAGS += -I$(APPDIR)/examples/fft/dsp/CMSIS/Core/Include
CFLAGS += -I$(APPDIR)/examples/fft/dsp/CMSIS/DSP/Include
endif

CFLAGS += -DARM_MATH_CM4 -D__FPU_PRESENT=1

VPATH  = ../CMSIS/DSP/Source/TransformFunctions
VPATH += ../CMSIS/DSP/Source/SupportFunctions
VPATH += ../CMSIS/DSP/Source/BasicMathFunctions
VPATH += ../CMSIS/DSP/Source/ComplexMathFunctions
VPATH += ../CMSIS/DSP/Source/CommonTables

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

all: $(BIN)

$(COBJS): %$(OBJEXT): %.c
	@echo "CC: $<"
	$(Q) $(CC) -c $(CFLAGS) $< -o $@

$(AOBJS): %$(OBJEXT): %.S
	@echo "AS: $<"
	$(Q) $(CC) -c $(AFLAGS) $< -o $@

$(BIN): $(COBJS) $(AOBJS)
	@echo "LD: $@"
	$(Q) $(LD) $(LDRAWELFFLAGS) -e main --gc-sections $(LDLIBPATH) -o $@ \
		--start-group $(ARCHCRT0OBJ) $^ $(LDLIBS) $(LIBGCC) --end-group
	$(Q) cp $(BIN) $(BIN).debug.elf
	$(Q) $(STRIP) -d $(BIN)

clean:
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(BIN).debug.elf)
	$(call CLEAN)

install:
	$(Q) mkdir -p $(ROMFS_DIR)
ifeq ($(WINTOOL),y)
	$(Q) $(STRIP) -d -o "$(shell cygpath -w $(ROMFS_DIR)/$(BIN))" $(BIN)
else
	$(Q) $(STRIP) -d -o $(ROMFS_DIR)/$(BIN) $(BIN)
endif
