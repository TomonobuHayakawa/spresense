# Output mixer library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)liboutputmixer$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)output_mixer
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)output_mixer

modules$(DELIM)audio$(DELIM)objects$(DELIM)output_mixer$(DELIM)liboutputmixer$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)objects$(DELIM)output_mixer TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" liboutputmixer$(LIBEXT)

lib$(DELIM)liboutputmixer$(LIBEXT): modules$(DELIM)audio$(DELIM)objects$(DELIM)output_mixer$(DELIM)liboutputmixer$(LIBEXT)
	$(Q) install $< $@
