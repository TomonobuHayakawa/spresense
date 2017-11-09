
ifeq ($(CONFIG_TRAM),y)
SDKLIBS += lib$(DELIM)libtram$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)transport_mode
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)transport_mode
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)transport_mode

modules$(DELIM)sensing$(DELIM)transport_mode$(DELIM)libtram$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)transport_mode TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libtram$(LIBEXT)

lib$(DELIM)libtram$(LIBEXT): modules$(DELIM)sensing$(DELIM)transport_mode$(DELIM)libtram$(LIBEXT)
	$(Q) install $< $@
