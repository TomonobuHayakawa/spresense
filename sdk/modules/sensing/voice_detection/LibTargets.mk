
ifeq ($(CONFIG_SENSING_VAD),y)
SDKLIBS += lib$(DELIM)libvad$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)voice_detection
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)voice_detection
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)voice_detection

modules$(DELIM)sensing$(DELIM)voice_detection$(DELIM)libvad$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)voice_detection TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libvad$(LIBEXT)

lib$(DELIM)libvad$(LIBEXT): modules$(DELIM)sensing$(DELIM)voice_detection$(DELIM)libvad$(LIBEXT)
	$(Q) install $< $@
