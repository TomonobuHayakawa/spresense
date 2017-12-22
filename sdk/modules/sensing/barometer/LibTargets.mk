
ifeq ($(CONFIG_SENSING_BAROMETER),y)
SDKLIBS += lib$(DELIM)libbarometer$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)barometer
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)barometer
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)barometer

modules$(DELIM)sensing$(DELIM)barometer$(DELIM)libbarometer$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)barometer TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libbarometer$(LIBEXT)

lib$(DELIM)libbarometer$(LIBEXT): modules$(DELIM)sensing$(DELIM)barometer$(DELIM)libbarometer$(LIBEXT)
	$(Q) install $< $@
