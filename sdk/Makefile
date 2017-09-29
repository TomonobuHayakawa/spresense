############################################################################
# TODO: write here
#
#
############################################################################
# XXX: Sorry, this make file only for Linux/Cygwin

# Path to NuttX kernel source tree, it must be relative path.

NUTTXDIR ?= ../nuttx

SDKDIR := $(shell pwd | sed -e 's/ /\\ /g')
KERNELDIR := $(shell pwd | sed -e 's/ /\\ /g')/$(NUTTXDIR)

-include $(KERNELDIR)/.config
-include $(SDKDIR)/.config
include $(KERNELDIR)/tools/Config.mk
include Make.defs

ifeq ($(V),1)
export Q :=
else
ifeq ($(V),2)
export Q :=
else
export Q := @
endif
endif

ifeq ($(CONFIG_BUILD_PROTECTED),y)
include $(KERNELDIR)/ProtectedLibs.mk
else
ifeq ($(CONFIG_BUILD_KERNEL),y)
include $(KERNELDIR)/KernelLibs.mk
else
include $(KERNELDIR)/FlatLibs.mk
endif
endif

PHONY :=

all: buildsdk

buildall: buildkernel buildsdk

# buildkernel
#
#

buildkernel:
	$(MAKE) $(MAKEOPTS) -C $(KERNELDIR) pass2deps
	$(MAKE) $(MAKEOPTS) -C $(KERNELDIR)/arch/arm/src export_startup TOPDIR=$(KERNELDIR)
PHONY += buildkernel

buildsdk:
	$(Q)
PHONY += buildsdk

# clean_kernel
#
#

cleankernel:
	$(MAKE) -C $(KERNELDIR) clean
PHONY += cleankernel

cleansdk:
	$(Q)
PHONY += cleansdk

# config targets
#
#

config:
	$(Q) kconfig-conf Kconfig
PHONY += config

oldconfig:
	$(Q) kconfig-conf --oldconfig Kconfig
PHONY += oldconfig

olddefconfig:
	$(Q) kconfig-conf --olddefconfig Kconfig
PHONY += olddefconfig

menuconfig:
	$(Q) kconfig-mconf Kconfig
PHONY += menuconfig

qconfig:
	$(Q) kconfig-qconf Kconfig
PHONY += qconfig

gconfig:
	$(Q) kconfig-gconf Kconfig
PHONY += gconfig

savedefconfig:
PHONY += savedefconfig

clean: cleansdk
PHONY += clean

.PHONY: $(PHONY)
