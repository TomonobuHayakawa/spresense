
# ASMP framework library

ifeq ($(CONFIG_ASMP),y)
SDKLIBS += lib$(DELIM)libasmp$(LIBEXT)
SDKMODDIRS += modules$(DELIM)asmp
endif

modules$(DELIM)asmp$(DELIM)libasmp$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)asmp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libasmp$(LIBEXT)

lib$(DELIM)libasmp$(LIBEXT): modules$(DELIM)asmp$(DELIM)libasmp$(LIBEXT)
	$(Q) install $< $@

# BSP archive rule

SDKLIBS += lib$(DELIM)libbsp$(LIBEXT)
SDKMODDIRS += bsp
bsp$(DELIM)libbsp$(LIBEXT): context
	$(Q) $(MAKE) -C bsp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libbsp$(LIBEXT)

lib$(DELIM)libbsp$(LIBEXT): bsp$(DELIM)libbsp$(LIBEXT)
	$(Q) install $< $@

# External drivers

SDKLIBS += lib$(DELIM)libextdrivers$(LIBEXT)
SDKMODDIRS += drivers
drivers$(DELIM)libextdrivers$(LIBEXT): context
	$(Q) $(MAKE) -C drivers TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libextdrivers$(LIBEXT)

lib$(DELIM)libextdrivers$(LIBEXT): drivers$(DELIM)libextdrivers$(LIBEXT)
	$(Q) install $< $@

# Glob external sub directories which contains 'LibTarget.mk' file.

define ExtSubDirectory_template
	include $(1)/LibTarget.mk
endef

EXTSUBDIRS = $(dir $(wildcard ../*/LibTarget.mk))

$(foreach SDIR, $(EXTSUBDIRS), $(eval $(call ExtSubDirectory_template,$(SDIR))))
