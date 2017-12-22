# Media player library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libsoundrecognizer$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)sound_recognizer
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)sound_recognizer

modules$(DELIM)audio$(DELIM)objects$(DELIM)sound_recognizer$(DELIM)libsoundrecognizer$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)objects$(DELIM)sound_recognizer TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libsoundrecognizer$(LIBEXT)

lib$(DELIM)libsoundrecognizer$(LIBEXT): modules$(DELIM)audio$(DELIM)objects$(DELIM)sound_recognizer$(DELIM)libsoundrecognizer$(LIBEXT)
	$(Q) install $< $@
