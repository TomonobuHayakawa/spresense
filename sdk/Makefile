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
-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

ifeq ($(V),1)
export Q :=
else
ifeq ($(V),2)
export Q :=
else
export Q := @
endif
endif

# Dummy. It needs libapps.a to be a member of EXPORTLIBS.

APPDIR := ../apps

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

LDSTARTGROUP ?= --start-group
LDENDGROUP ?= --end-group

NUTTXNAME = nuttx
BIN       = $(NUTTXNAME)$(EXEEXT)
SPK       = $(NUTTXNAME).spk

KDEFINE = ${shell $(TOPDIR)/tools/define.sh "$(CC)" __KERNEL__}

all: $(BIN)

buildall: buildkernel $(BIN)

# buildkernel
#
#

buildkernel:
ifneq ($(KERNCONF),)
	$(Q) $(SDKDIR)/tools/config.py --kernel $(KERNCONF)
endif
	$(Q) $(MAKE) $(MAKEOPTS) -C $(TOPDIR) pass2deps
	$(Q) install $(sort $(patsubst %,$(TOPDIR)/%,$(EXPORTLIBS))) lib
PHONY += buildkernel

# Include library build recipes

include LibTargets.mk

SDKDEPDIRS += $(SDKMODDIRS)
SDKCLEANDIRS += $(SDKMODDIRS)

# T.B.D
ifeq ($(WINTOOL),y)
  LIBPATHS += -L"${shell cygpath -w "$(SDKDIR)/lib"}"
else
  LIBPATHS += -L"$(SDKDIR)/lib"
endif

LINKLIBS = $(patsubst lib/%,%,$(NUTTXLIBS) $(SDKLIBS))
LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))

$(BIN): depend $(SDKLIBS)
	$(Q) $(MAKE) -C bsp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" LINKLIBS="$(LINKLIBS)" EXTRADEFINES=$(KDEFINE) $(BIN)
	@echo "SPK: $(BIN).spk"
	$(Q) if [ -x $(MKSPK) ]; then \
		$(MKSPK) -c 2 $(BIN) nuttx $(BIN).spk >/dev/null ; \
	else \
		echo mkspk tool not found. ; \
	fi

libs: $(SDKLIBS)

context: bsp/include/sdk/config.h
PHONY += context

clean_context:
	$(call DELFILE, bsp/include/sdk/config.h)

PHONY += clean_context

depend: context
	$(Q) for dir in $(SDKDEPDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" EXTRADEFINES=$(KDEFINE) depend; \
	done
PHONY += depend

# clean_kernel
#
#

cleankernel:
	$(MAKE) -C $(TOPDIR) clean
PHONY += cleankernel

cleansdk:
	$(call DELFILE, .kconfig.tmp)
	$(call DELFILE, bsp/include/sdk/config.h)
	$(call DELFILE, bsp/include/sdk/version.h)
	$(call DELFILE, $(SDKLIBS))
	$(Q) for dir in $(SDKCLEANDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" clean; \
	done
PHONY += cleansdk

bsp/include/sdk/config.h: $(SDKDIR)/.config tools/mkconfig.py
	$(Q) tools/mkconfig.py > $@

bsp/include/sdk/version.h: $(SDKDIR)/.version
# TBD

# For NuttX configuration

configkernel oldconfigkernel olddefconfigkernel menuconfigkernel qconfigkernel gconfigkernel:
	$(Q) $(MAKE) -C $(TOPDIR) $(patsubst %kernel,%,$@)
PHONY += configkernel oldconfigkernel olddefconfigkernel menuconfigkernel qconfigkernel gconfigkernel

# config targets
#
#

config: .kconfig.tmp
	$(Q) SDKDIR=$(SDKDIR) kconfig-conf $<
PHONY += config

oldconfig: .kconfig.tmp
	$(Q) SDKDIR=$(SDKDIR) kconfig-conf --oldconfig $<
PHONY += oldconfig

olddefconfig: .kconfig.tmp
	$(Q) SDKDIR=$(SDKDIR) kconfig-conf --olddefconfig $<
PHONY += olddefconfig

menuconfig: .kconfig.tmp
	$(Q) SDKDIR=$(SDKDIR) kconfig-mconf $<
PHONY += menuconfig

qconfig: .kconfig.tmp
	$(Q) SDKDIR=$(SDKDIR) kconfig-qconf $<
PHONY += qconfig

gconfig: .kconfig.tmp
	$(Q) SDKDIR=$(SDKDIR) kconfig-gconf $<
PHONY += gconfig

savedefconfig: .kconfig.tmp
	$(Q) SDKDIR=$(SDKDIR) kconfig-conf --savedefconfig defconfig $<
PHONY += savedefconfig

.kconfig.tmp:
	$(Q) tools/gentmpkconfig.py $@
PHONY += .kconfig.tmp

clean: cleansdk
PHONY += clean

.PHONY: $(PHONY)
