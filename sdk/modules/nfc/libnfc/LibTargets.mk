
ifeq ($(CONFIG_LIBNFC),y)
SDKLIBS += lib$(DELIM)libnfc$(LIBEXT)
SDKMODDIRS += modules$(DELIM)nfc$(DELIM)libnfc
#CONTEXTDIRS += modules$(DELIM)nfc$(DELIM)libnfc
endif
SDKCLEANDIRS += modules$(DELIM)nfc$(DELIM)libnfc

modules$(DELIM)nfc$(DELIM)libnfc$(DELIM)libnfc$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)nfc$(DELIM)libnfc TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libnfc$(LIBEXT)

lib$(DELIM)libnfc$(LIBEXT): modules$(DELIM)nfc$(DELIM)libnfc$(DELIM)libnfc$(LIBEXT)
	$(Q) install $< $@
