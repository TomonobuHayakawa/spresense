# Audio manager library

ifeq ($(CONFIG_SDK_AUDIO),y)
ifeq ($(CONFIG_AUDIOUTILS_MANAGER),y)
SDKLIBS += lib$(DELIM)libaudiomanager$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)manager
endif
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)manager

modules$(DELIM)audio$(DELIM)manager$(DELIM)libaudiomanager$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)manager TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libaudiomanager$(LIBEXT)

lib$(DELIM)libaudiomanager$(LIBEXT): modules$(DELIM)audio$(DELIM)manager$(DELIM)libaudiomanager$(LIBEXT)
	$(Q) install $< $@
