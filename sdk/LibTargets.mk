
SDKLIBS += lib$(DELIM)libasmp$(LIBEXT)
SDKMODDIRS += modules$(DELIM)asmp
modules$(DELIM)asmp$(DELIM)libasmp$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)asmp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libasmp$(LIBEXT)

lib$(DELIM)libasmp$(LIBEXT): modules$(DELIM)asmp$(DELIM)libasmp$(LIBEXT)
	$(Q) install modules$(DELIM)asmp$(DELIM)libasmp$(LIBEXT) lib$(DELIM)libasmp$(LIBEXT)

# BSP archive rule

SDKLIBS += lib$(DELIM)libbsp$(LIBEXT)
SDKMODDIRS += bsp
bsp$(DELIM)libbsp$(LIBEXT): context
	$(Q) $(MAKE) -C bsp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libbsp$(LIBEXT)

lib$(DELIM)libbsp$(LIBEXT): bsp$(DELIM)libbsp$(LIBEXT)
	$(Q) install bsp$(DELIM)libbsp$(LIBEXT) lib$(DELIM)libbsp$(LIBEXT)
