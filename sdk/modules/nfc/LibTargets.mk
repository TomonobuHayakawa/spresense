
ifeq ($(CONFIG_NFC),y)
SDKLIBS += lib$(DELIM)libnfc$(LIBEXT)
SDKMODDIRS += modules$(DELIM)nfc
#CONTEXTDIRS += modules$(DELIM)nfc
endif
SDKCLEANDIRS += modules$(DELIM)nfc

modules$(DELIM)nfc$(DELIM)libnfc$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)nfc TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libnfc$(LIBEXT)

lib$(DELIM)libnfc$(LIBEXT): modules$(DELIM)nfc$(DELIM)libnfc$(LIBEXT)
	$(Q) install $< $@
