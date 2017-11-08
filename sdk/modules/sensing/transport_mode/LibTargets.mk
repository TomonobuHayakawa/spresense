
ifeq ($(CONFIG_TRAM),y)
SDKLIBS += lib$(DELIM)libtram$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)tram
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)tram
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)tram

modules$(DELIM)sensing$(DELIM)tram$(DELIM)libtram$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)tram TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libtram$(LIBEXT)

lib$(DELIM)libtram$(LIBEXT): modules$(DELIM)sensing$(DELIM)tram$(DELIM)libtram$(LIBEXT)
	$(Q) install $< $@
