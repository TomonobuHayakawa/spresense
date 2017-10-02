############################################################################
# TODO: write here
#
#
############################################################################
# XXX: Sorry, this make file only for Linux/Cygwin

# Path to NuttX kernel source tree, it must be relative path.

NUTTXDIR ?= ../nuttx

SDKDIR := $(shell pwd | sed -e 's/ /\\ /g')
TOPDIR := $(shell pwd | sed -e 's/ /\\ /g')/$(NUTTXDIR)

-include $(TOPDIR)/.config
-include $(SDKDIR)/.config
include $(TOPDIR)/tools/Config.mk
include $(TOPDIR)/Make.defs

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
include $(TOPDIR)/ProtectedLibs.mk
else
ifeq ($(CONFIG_BUILD_KERNEL),y)
include $(TOPDIR)/KernelLibs.mk
else
include $(TOPDIR)/FlatLibs.mk
endif
endif

PHONY :=

all: buildsdk

buildall: buildkernel buildsdk

# buildkernel
#
#

buildkernel:
	$(MAKE) $(MAKEOPTS) -C $(TOPDIR) pass2deps
#	$(MAKE) $(MAKEOPTS) -C $(TOPDIR)/arch/arm/src export_startup TOPDIR=$(TOPDIR)
PHONY += buildkernel

buildsdk: bsp/include/sdk/config.h test
	$(Q)
PHONY += buildsdk

# clean_kernel
#
#

cleankernel:
	$(MAKE) -C $(TOPDIR) clean
PHONY += cleankernel

cleansdk:
	$(call DELFILE, bsp/include/sdk/config.h)
PHONY += cleansdk

bsp/include/sdk/config.h: $(TOPDIR)/.config
	$(Q) tools/mkconfig.py > $@

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

clean: cleansdk cleantest
PHONY += clean

test: bsp/include/sdk/config.h
	$(Q) $(MAKE) -C modules/asmp TOPDIR=$(TOPDIR) SDKDIR=$(SDKDIR)
PHONY += test

cleantest:
	$(Q) $(MAKE) -C modules/asmp TOPDIR=$(TOPDIR) SDKDIR=$(SDKDIR) clean
PHONY += cleantest

.PHONY: $(PHONY)
