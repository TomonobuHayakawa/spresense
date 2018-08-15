
KERNEL_DIR ?= ../nuttx

ifeq ($(V),1)
export Q :=
else
ifeq ($(V),2)
export Q :=
else
export Q := @
endif
endif

NUTTX_DIR := $(shell pwd | sed -e 's/ /\\ /g')/$(KERNEL_DIR)

# build_kernellibs
#
#

build_kernellibs:
	$(MAKE) -C $(NUTTX_DIR) pass2deps
	$(MAKE) -C $(NUTTX_DIR)/arch/arm/src export_startup TOPDIR=$(NUTTX_DIR)

# clean_kernellibs
#
#

clean_kernellibs:
	$(MAKE) -C $(NUTTX_DIR) clean

# config targets
#
#

config:
	$(Q) kconfig-conf Kconfig

oldconfig:
	$(Q) kconfig-conf --oldconfig Kconfig

olddefconfig:
	$(Q) kconfig-conf --olddefconfig Kconfig

menuconfig:
	$(Q) kconfig-mconf Kconfig

qconfig:
	$(Q) kconfig-qconf Kconfig

gconfig:
	$(Q) kconfig-gconf Kconfig

savedefconfig:

clean: clean_kernellibs

.PHONY: build clean clean_kernellibs
