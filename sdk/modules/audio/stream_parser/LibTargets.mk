# Stream parser library

ifeq ($(CONFIG_SDK_AUDIO),y)
ifeq ($(CONFIG_AUDIOUTILS_PLAYER),y)
SDKLIBS += lib$(DELIM)libstreamparser$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)stream_parser
endif
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)stream_parser

modules$(DELIM)audio$(DELIM)stream_parser$(DELIM)libstreamparser$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)stream_parser TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libstreamparser$(LIBEXT)

lib$(DELIM)libstreamparser$(LIBEXT): modules$(DELIM)audio$(DELIM)stream_parser$(DELIM)libstreamparser$(LIBEXT)
	$(Q) install $< $@
