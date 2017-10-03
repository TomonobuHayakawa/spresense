
SDKLIBS += lib$(DELIM)libasmp$(LIBEXT)
SDKMODDIRS += modules$(DELIM)asmp
modules$(DELIM)asmp$(DELIM)libasmp$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)asmp TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libasmp$(LIBEXT)

lib$(DELIM)libasmp$(LIBEXT): modules$(DELIM)asmp$(DELIM)libasmp$(LIBEXT)
	$(Q) install modules$(DELIM)asmp$(DELIM)libasmp$(LIBEXT) lib$(DELIM)libasmp$(LIBEXT)
