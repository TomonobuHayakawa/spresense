# Media recorder library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libmediarecorder$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)media_recorder
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)media_recorder

modules$(DELIM)audio$(DELIM)objects$(DELIM)media_recorder$(DELIM)libmediarecorder$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)objects$(DELIM)media_recorder TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libmediarecorder$(LIBEXT)

lib$(DELIM)libmediarecorder$(LIBEXT): modules$(DELIM)audio$(DELIM)objects$(DELIM)media_recorder$(DELIM)libmediarecorder$(LIBEXT)
	$(Q) install $< $@
