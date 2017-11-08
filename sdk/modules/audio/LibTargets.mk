# Audio library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libaudio$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio
endif
SDKCLEANDIRS += modules$(DELIM)audio

modules$(DELIM)audio$(DELIM)libaudio$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libaudio$(LIBEXT)

lib$(DELIM)libaudio$(LIBEXT): modules$(DELIM)audio$(DELIM)libaudio$(LIBEXT)
	$(Q) install $< $@
