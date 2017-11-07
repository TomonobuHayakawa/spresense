# External drivers

SDKLIBS += lib$(DELIM)libextdrivers$(LIBEXT)
SDKMODDIRS += drivers
SDKCLEANDIRS += drivers

drivers$(DELIM)libextdrivers$(LIBEXT): context
	$(Q) $(MAKE) -C drivers TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libextdrivers$(LIBEXT)

lib$(DELIM)libextdrivers$(LIBEXT): drivers$(DELIM)libextdrivers$(LIBEXT)
	$(Q) install $< $@
