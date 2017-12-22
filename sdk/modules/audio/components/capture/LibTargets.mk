# Capture component library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libcapture$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)capture
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)capture

modules$(DELIM)audio$(DELIM)components$(DELIM)capture$(DELIM)libcapture$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)components$(DELIM)capture TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libcapture$(LIBEXT)

lib$(DELIM)libcapture$(LIBEXT): modules$(DELIM)audio$(DELIM)components$(DELIM)capture$(DELIM)libcapture$(LIBEXT)
	$(Q) install $< $@
