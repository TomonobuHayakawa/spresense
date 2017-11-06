
# ASMP framework library

ifeq ($(CONFIG_ASMP),y)
SDKLIBS += lib$(DELIM)libasmp$(LIBEXT)
SDKMODDIRS += modules$(DELIM)asmp
endif
SDKCLEANDIRS += modules$(DELIM)asmp

modules$(DELIM)asmp$(DELIM)libasmp$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)asmp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libasmp$(LIBEXT)

lib$(DELIM)libasmp$(LIBEXT): modules$(DELIM)asmp$(DELIM)libasmp$(LIBEXT)
	$(Q) install $< $@

# System utility library

SDKLIBS += lib$(DELIM)libsystem$(LIBEXT)
SDKMODDIRS += system
SDKCLEANDIRS += system
CONTEXTDIRS += system

system$(DELIM)libsystem$(LIBEXT): context
	$(Q) $(MAKE) -C system TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libsystem$(LIBEXT)

lib$(DELIM)libsystem$(LIBEXT): system$(DELIM)libsystem$(LIBEXT)
	$(Q) install $< $@

# BSP archive rule

SDKLIBS += lib$(DELIM)libbsp$(LIBEXT)
SDKMODDIRS += bsp
SDKCLEANDIRS += bsp

bsp$(DELIM)libbsp$(LIBEXT): context
	$(Q) $(MAKE) -C bsp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libbsp$(LIBEXT)

lib$(DELIM)libbsp$(LIBEXT): bsp$(DELIM)libbsp$(LIBEXT)
	$(Q) install $< $@

# External drivers

SDKLIBS += lib$(DELIM)libextdrivers$(LIBEXT)
SDKMODDIRS += drivers
SDKCLEANDIRS += drivers

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
