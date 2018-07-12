############################################################################
# examples/fft/dsp/Makefile
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

ALL_SUBDIRS = math
BUILD_SUBDIRS = math

define DIR_template
$(1)_$(2):
	$(Q) $(MAKE) -C $(1) $(3) TOPDIR="$(TOPDIR)" APPDIR="$(APPDIR)" ROMFS_DIR="$(ROMFS_DIR)" CROSSDEV=$(CROSSDEV) DSPLIBDIR="$(DSPLIBDIR)"
endef

all: build
.PHONY: all build clean install

$(foreach DIR, $(BUILD_SUBDIRS), $(eval $(call DIR_template,$(DIR),build, all)))
$(foreach DIR, $(ALL_SUBDIRS), $(eval $(call DIR_template,$(DIR),clean,clean)))
$(foreach DIR, $(BUILD_SUBDIRS), $(eval $(call DIR_template,$(DIR),install,install)))

# Build program(s) in each sud-directory

build: $(foreach DIR, $(BUILD_SUBDIRS), $(DIR)_build)

# Clean each subdirectory

clean: $(foreach DIR, $(ALL_SUBDIRS), $(DIR)_clean)
