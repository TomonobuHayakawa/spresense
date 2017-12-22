# Recognition component library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)librecognition$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)recognition
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)recognition

modules$(DELIM)audio$(DELIM)components$(DELIM)recognition$(DELIM)librecognition$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)components$(DELIM)recognition TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" librecognition$(LIBEXT)

lib$(DELIM)librecognition$(LIBEXT): modules$(DELIM)audio$(DELIM)components$(DELIM)recognition$(DELIM)librecognition$(LIBEXT)
	$(Q) install $< $@
