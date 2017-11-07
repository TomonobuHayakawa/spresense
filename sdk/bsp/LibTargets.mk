# BSP archive rule

SDKLIBS += lib$(DELIM)libbsp$(LIBEXT)
SDKMODDIRS += bsp
SDKCLEANDIRS += bsp

bsp$(DELIM)libbsp$(LIBEXT): context
	$(Q) $(MAKE) -C bsp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libbsp$(LIBEXT)

lib$(DELIM)libbsp$(LIBEXT): bsp$(DELIM)libbsp$(LIBEXT)
	$(Q) install $< $@
