
#ifeq ($(CONFIG_EXTNETUTILS),y)
SDKLIBS += lib$(DELIM)libextnetutils$(LIBEXT)
SDKMODDIRS += modules$(DELIM)netutils
#CONTEXTDIRS += modules$(DELIM)netutils
#endif
SDKCLEANDIRS += modules$(DELIM)netutils

modules$(DELIM)netutils$(DELIM)libextnetutils$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)netutils TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libextnetutils$(LIBEXT)

lib$(DELIM)libextnetutils$(LIBEXT): modules$(DELIM)netutils$(DELIM)libextnetutils$(LIBEXT)
	$(Q) install $< $@
