
ifeq ($(CONFIG_BLUETOOTH),y)
SDKLIBS += lib$(DELIM)libbluetooth$(LIBEXT)
SDKMODDIRS += modules$(DELIM)bluetooth
endif
SDKCLEANDIRS += modules$(DELIM)bluetooth

modules$(DELIM)bluetooth$(DELIM)libbluetooth$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)bluetooth TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libbluetooth$(LIBEXT)

lib$(DELIM)libbluetooth$(LIBEXT): modules$(DELIM)bluetooth$(DELIM)libbluetooth$(LIBEXT)
	$(Q) install $< $@
