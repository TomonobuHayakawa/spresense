
ifeq ($(CONFIG_LTE),y)
SDKLIBS += lib$(DELIM)liblte$(LIBEXT)
SDKLIBS += lib$(DELIM)libltecli$(LIBEXT)
SDKMODDIRS += modules$(DELIM)lte
#CONTEXTDIRS += modules$(DELIM)lte
endif
SDKCLEANDIRS += modules$(DELIM)lte

modules$(DELIM)lte$(DELIM)libltecli$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)lte TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libltecli$(LIBEXT)

lib$(DELIM)liblte$(LIBEXT): modules$(DELIM)lte$(DELIM)lib$(DELIM)liblte$(LIBEXT)
	$(Q) install $< $@

lib$(DELIM)libltecli$(LIBEXT): modules$(DELIM)lte$(DELIM)libltecli$(LIBEXT)
	$(Q) install $< $@
