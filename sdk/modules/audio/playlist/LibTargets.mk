# Playlist library

ifeq ($(CONFIG_SDK_AUDIO),y)
ifeq ($(CONFIG_AUDIOUTILS_PLAYLIST),y)
SDKLIBS += lib$(DELIM)libplaylist$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)playlist
endif
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)playlist

modules$(DELIM)audio$(DELIM)playlist$(DELIM)libplaylist$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)playlist TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libplaylist$(LIBEXT)

lib$(DELIM)libplaylist$(LIBEXT): modules$(DELIM)audio$(DELIM)playlist$(DELIM)libplaylist$(LIBEXT)
	$(Q) install $< $@
