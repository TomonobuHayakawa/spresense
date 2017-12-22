# Encoder component library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libencoder$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)encoder
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)encoder

modules$(DELIM)audio$(DELIM)components$(DELIM)encoder$(DELIM)libencoder$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)components$(DELIM)encoder TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libencoder$(LIBEXT)

lib$(DELIM)libencoder$(LIBEXT): modules$(DELIM)audio$(DELIM)components$(DELIM)encoder$(DELIM)libencoder$(LIBEXT)
	$(Q) install $< $@
