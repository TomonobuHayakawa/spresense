
ifeq ($(CONFIG_LTE),y)
SDKLIBS += lib$(DELIM)liblte$(LIBEXT)
SDKMODDIRS += modules$(DELIM)lte
#CONTEXTDIRS += modules$(DELIM)lte
endif
SDKCLEANDIRS += modules$(DELIM)lte

modules$(DELIM)lte$(DELIM)liblte$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)lte TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" liblte$(LIBEXT)

lib$(DELIM)liblte$(LIBEXT): modules$(DELIM)lte$(DELIM)liblte$(LIBEXT)
	$(Q) install $< $@
