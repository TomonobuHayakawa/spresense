# Filter component library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libfilter$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)filter
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)filter

modules$(DELIM)audio$(DELIM)components$(DELIM)filter$(DELIM)libfilter$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)components$(DELIM)filter TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libfilter$(LIBEXT)

lib$(DELIM)libfilter$(LIBEXT): modules$(DELIM)audio$(DELIM)components$(DELIM)filter$(DELIM)libfilter$(LIBEXT)
	$(Q) install $< $@
