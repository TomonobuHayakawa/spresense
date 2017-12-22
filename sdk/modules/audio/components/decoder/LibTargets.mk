# Decoder component library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libdecoder$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)decoder
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)decoder

modules$(DELIM)audio$(DELIM)components$(DELIM)decoder$(DELIM)libdecoder$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)components$(DELIM)decoder TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libdecoder$(LIBEXT)

lib$(DELIM)libdecoder$(LIBEXT): modules$(DELIM)audio$(DELIM)components$(DELIM)decoder$(DELIM)libdecoder$(LIBEXT)
	$(Q) install $< $@
