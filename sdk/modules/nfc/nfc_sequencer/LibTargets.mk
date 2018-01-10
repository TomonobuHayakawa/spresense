
ifeq ($(CONFIG_NFC_SEQUENCER),y)
SDKLIBS += lib$(DELIM)libnfc_sequencer$(LIBEXT)
SDKMODDIRS += modules$(DELIM)nfc$(DELIM)nfc_sequencer
#CONTEXTDIRS += modules$(DELIM)nfc$(DELIM)nfc_sequencer
endif
SDKCLEANDIRS += modules$(DELIM)nfc$(DELIM)nfc_sequencer

modules$(DELIM)nfc$(DELIM)nfc_sequencer$(DELIM)libnfc_sequencer$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)nfc$(DELIM)nfc_sequencer TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libnfc_sequencer$(LIBEXT)

lib$(DELIM)libnfc_sequencer$(LIBEXT): modules$(DELIM)nfc$(DELIM)nfc_sequencer$(DELIM)libnfc_sequencer$(LIBEXT)
	$(Q) install $< $@
