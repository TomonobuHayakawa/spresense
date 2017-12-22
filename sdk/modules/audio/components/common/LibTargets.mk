# Component commom library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libcmpcommon$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)common
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)common

modules$(DELIM)audio$(DELIM)components$(DELIM)common$(DELIM)libcmpcommon$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)components$(DELIM)common TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libcmpcommon$(LIBEXT)

lib$(DELIM)libcmpcommon$(LIBEXT): modules$(DELIM)audio$(DELIM)components$(DELIM)common$(DELIM)libcmpcommon$(LIBEXT)
	$(Q) install $< $@
