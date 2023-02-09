############################################################################
# modules/asmp/worker/worker_libc.mk
#
#   Copyright 2023 Sony Semiconductor Solutions Corporation
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

ifneq ($(CONFIG_ASMP_WORKER_LIBC),)

NXINC_PATH = $(SDKDIR)/../nuttx/include
LIBC_PATH = $(SDKDIR)/../nuttx/libs/libc

LIBC_LIBS  = ctype
LIBC_LIBS += fixedmath
LIBC_LIBS += string
LIBC_LIBS += queue

LIBC_EXTRA_SRC = lib_modff.c lib_floorf.c lib_fabsf.c

LIBC_LIBSPATH = $(patsubst %,$(LIBC_PATH)/%,$(LIBC_LIBS))
LIBC_TGTPATH = $(LIBC_LIBSPATH) $(LIBC_PATH)/math
LIBC_CSRCS = $(notdir $(foreach p,$(LIBC_LIBSPATH),$(wildcard $(p)/*.c))) $(LIBC_EXTRA_SRC)
LIBC_DEPPATH = --dep-path $(NXINC_PATH) --dep-path $(NXINC_PATH)/nuttx/lib $(patsubst %,--dep-path %,$(LIBC_TGTPATH))
LIBC_INCPATH = -I$(NXINC_PATH) -I$(NXINC_PATH)/nuttx/lib -I$(LIBC_PATH) -DCONFIG_ARCH_STDARG_H

endif
