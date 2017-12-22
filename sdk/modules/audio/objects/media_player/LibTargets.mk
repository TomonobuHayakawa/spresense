# Media player library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libmediaplayer$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)media_player
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)media_player

modules$(DELIM)audio$(DELIM)objects$(DELIM)media_player$(DELIM)libmediaplayer$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)objects$(DELIM)media_player TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libmediaplayer$(LIBEXT)

lib$(DELIM)libmediaplayer$(LIBEXT): modules$(DELIM)audio$(DELIM)objects$(DELIM)media_player$(DELIM)libmediaplayer$(LIBEXT)
	$(Q) install $< $@
