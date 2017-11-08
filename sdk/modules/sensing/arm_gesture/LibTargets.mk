
ifeq ($(CONFIG_ARMGESTURE),y)
SDKLIBS += lib$(DELIM)libarmgesture$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)armgesture
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)armgesture
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)armgesture

modules$(DELIM)sensing$(DELIM)armgesture$(DELIM)libarmgesture$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)armgesture TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libarmgesture$(LIBEXT)

lib$(DELIM)libarmgesture$(LIBEXT): modules$(DELIM)sensing$(DELIM)armgesture$(DELIM)libarmgesture$(LIBEXT)
	$(Q) install $< $@
