# Stream parser wrapper library

ifeq ($(CONFIG_SDK_AUDIO),y)
ifeq ($(CONFIG_AUDIOUTILS_PLAYER),y)
SDKLIBS += lib$(DELIM)libstreamparserwrapper$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)stream_parser
endif
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)stream_parser

modules$(DELIM)audio$(DELIM)objects$(DELIM)stream_parser$(DELIM)libstreamparserwrapper$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)objects$(DELIM)stream_parser TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libstreamparserwrapper$(LIBEXT)

lib$(DELIM)libstreamparserwrapper$(LIBEXT): modules$(DELIM)audio$(DELIM)objects$(DELIM)stream_parser$(DELIM)libstreamparserwrapper$(LIBEXT)
	$(Q) install $< $@
