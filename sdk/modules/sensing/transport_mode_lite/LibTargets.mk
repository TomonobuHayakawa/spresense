
ifeq ($(CONFIG_SENSING_TRAMLITE),y)
SDKLIBS += lib$(DELIM)libtramlite$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)transport_mode_lite
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)transport_mode_lite
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)transport_mode_lite

modules$(DELIM)sensing$(DELIM)transport_mode_lite$(DELIM)libtramlite$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)transport_mode_lite TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libtramlite$(LIBEXT)

lib$(DELIM)libtramlite$(LIBEXT): modules$(DELIM)sensing$(DELIM)transport_mode_lite$(DELIM)libtramlite$(LIBEXT)
	$(Q) install $< $@
