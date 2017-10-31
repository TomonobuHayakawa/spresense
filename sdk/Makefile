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

# buildkernel
#
# Separated build target for NuttX kernel libraries. If KERNCONF specified by
# option, then reconfigure and build them all.
# After install them, we need to replace nsh builtin list to ours, so
# remove builtin_list.o from libapps.a if exists.
# If nsh disabled at NuttX configuration, then our builtin_list will be removed
# automatically.

buildkernel:
ifneq ($(KERNCONF),)
	$(Q) $(SDKDIR)/tools/config.py --kernel $(KERNCONF)
endif
	$(Q) $(MAKE) $(MAKEOPTS) -C $(TOPDIR) pass2deps
PHONY += buildkernel

installkernel:
	$(Q) install $(sort $(patsubst %,$(TOPDIR)/%,$(EXPORTLIBS))) lib
	$(Q) if [ -f lib/libapps.a ]; then $(ARCROSSDEV)ar d lib/libapps.a builtin_list.o; fi
PHONY += installkernel

SDKMODDIRS =
CONTEXTDIRS =
EXTLIBS =
EXTSUBDIRS =

# Include library build recipes

include LibTargets.mk

SDKDEPDIRS = $(SDKMODDIRS) $(EXTSUBDIRS)
SDKCLEANDIRS = $(SDKMODDIRS) $(EXTSUBDIRS) lib
CONTEXTDIRS += $(EXTSUBDIRS)

# T.B.D
ifeq ($(WINTOOL),y)
  LIBPATHS += -L"${shell cygpath -w "$(SDKDIR)/lib"}"
else
  LIBPATHS += -L"$(SDKDIR)/lib"
endif

LINKLIBS = $(patsubst lib/%,%,$(NUTTXLIBS) $(SDKLIBS) $(EXTLIBS))
LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))

$(BIN): depend libs
	$(Q) $(MAKE) -C bsp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" LINKLIBS="$(LINKLIBS)" EXTRADEFINES=$(KDEFINE) $(BIN)
	@echo "SPK: $(BIN).spk"
	$(Q) if [ -x $(MKSPK) ]; then \
		$(MKSPK) -c 2 $(BIN) nuttx $(BIN).spk >/dev/null ; \
	else \
		echo mkspk tool not found. ; \
	fi

libs: $(EXPORTLIBS) $(SDKLIBS) $(EXTLIBS)

$(EXPORTLIBS): installkernel

context: check_context bsp/include/sdk/config.h
	$(Q) for dir in $(CONTEXTDIRS) ; do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" context; \
	done
PHONY += context

# check_context
#
# This target checks if NuttX has been configured.  NuttX is configured using
# the script tools/configure.sh.  That script will install certain files in
# the top-level NuttX build directory.  This target verifies that those
# configuration files have been installed and that NuttX is ready to be built.

check_context:
	$(Q) if [ ! -e $(TOPDIR)/.config -o ! -e $(TOPDIR)/Make.defs ]; then \
		echo "" ; echo "Nuttx has not been configured:" ; \
		echo "  tools/config.py -k <target>" ; echo "" ; \
		exit 1 ; \
	fi
	$(Q) if [ ! -e $(SDKDIR)/.config ]; then \
		echo "" ; echo "SDK has not been configured:" ; \
		echo "  tools/config.py <target>" ; echo "" ; \
		exit 1 ; \
	fi

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
	$(Q) $(MAKE) -C $(TOPDIR) clean
PHONY += cleankernel

distcleankernel:
	$(Q) $(MAKE) -C $(TOPDIR) distclean
PHONY += distcleankernel

subdir_clean:
	$(Q) for dir in $(SDKCLEANDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" clean; \
	done
PHONY += subdir_clean

clean: subdir_clean
	$(call DELFILE, .kconfig.tmp)
	$(call DELFILE, bsp/include/sdk/config.h)
	$(call DELFILE, bsp/include/sdk/version.h)
	$(call DELFILE, $(SDKLIBS))
PHONY += clean

subdir_distclean:
	$(Q) for dir in $(SDKCLEANDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" distclean; \
	done
PHONY += subdir_distclean

distclean: clean subdir_distclean
	$(call DELFILE, .config)
	$(call DELFILE, .config.old)

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

.kconfig.tmp: ext_preconfig
	$(Q) tools/gentmpkconfig.py $@

ext_preconfig:
ifneq ($(EXTSUBDIRS),)
	$(Q) for dir in $(EXTSUBDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" preconfig; \
	done
endif

ext_clean:
ifneq ($(EXTSUBDIRS),)
	$(Q) for dir in $(EXTSUBDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" clean; \
	done
endif

ext_distclean:
ifneq ($(EXTSUBDIRS),)
	$(Q) for dir in $(EXTSUBDIRS); do \
		$(MAKE) -C $$dir TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" distclean; \
	done
endif
PHONY += ext_preconfig ext_clean ext_distclean

.PHONY: $(PHONY)
