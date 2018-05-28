# Audio Library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libsdkaudio$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio
endif
SDKCLEANDIRS += modules$(DELIM)audio
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)dsp$(DELIM)worker
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)dsp$(DELIM)worker$(DELIM)lib

.PHONY: workerbin asmpwlib

modules$(DELIM)audio$(DELIM)libsdkaudio$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libsdkaudio$(LIBEXT)

lib$(DELIM)libsdkaudio$(LIBEXT): modules$(DELIM)audio$(DELIM)libsdkaudio$(LIBEXT)
	$(Q) install $< $@
