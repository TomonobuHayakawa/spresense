
ifeq ($(CONFIG_CXD56_GNSS),y)
SDKLIBS += lib$(DELIM)libgnss$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)gnss
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)gnss
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)gnss

modules$(DELIM)sensing$(DELIM)gnss$(DELIM)libgnss$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)gnss TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libgnss$(LIBEXT)

lib$(DELIM)libgnss$(LIBEXT): modules$(DELIM)sensing$(DELIM)gnss$(DELIM)libgnss$(LIBEXT)
	$(Q) install $< $@

ifeq ($(CONFIG_GPSUTILS_CXD56NMEA_LIB),y)
EXTLIBS += lib$(DELIM)libcxd56nmea$(LIBEXT)
endif

lib$(DELIM)libcxd56nmea$(LIBEXT): modules$(DELIM)sensing$(DELIM)gnss$(DELIM)cxd56nmea$(DELIM)libcxd56nmea$(LIBEXT)
	$(Q) install $< $@
