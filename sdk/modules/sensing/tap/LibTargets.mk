
ifeq ($(CONFIG_TAP),y)
SDKLIBS += lib$(DELIM)libtap$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)tap
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)tap
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)tap

modules$(DELIM)sensing$(DELIM)tap$(DELIM)libtap$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)tap TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libtap$(LIBEXT)

lib$(DELIM)libtap$(LIBEXT): modules$(DELIM)sensing$(DELIM)tap$(DELIM)libtap$(LIBEXT)
	$(Q) install $< $@
