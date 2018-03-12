# Audio Library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libsdkaudio$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio
endif
SDKCLEANDIRS += modules$(DELIM)audio

modules$(DELIM)audio$(DELIM)libsdkaudio$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libsdkaudio$(LIBEXT)

lib$(DELIM)libsdkaudio$(LIBEXT): modules$(DELIM)audio$(DELIM)libsdkaudio$(LIBEXT)
	$(Q) install $< $@

