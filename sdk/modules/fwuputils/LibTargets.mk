
ifeq ($(CONFIG_FWUPUTILS),y)
SDKLIBS += lib$(DELIM)libfwuputils$(LIBEXT)
SDKMODDIRS += modules$(DELIM)fwuputils
#CONTEXTDIRS += modules$(DELIM)fwuputils
endif
SDKCLEANDIRS += modules$(DELIM)fwuputils

modules$(DELIM)fwuputils$(DELIM)libfwuputils$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)fwuputils TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libfwuputils$(LIBEXT)

lib$(DELIM)libfwuputils$(LIBEXT): modules$(DELIM)fwuputils$(DELIM)libfwuputils$(LIBEXT)
	$(Q) install $< $@
