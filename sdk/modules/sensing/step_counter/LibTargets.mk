
ifeq ($(CONFIG_STEPCOUNTER),y)
SDKLIBS += lib$(DELIM)libstepcounter$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)stepcounter
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)stepcounter
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)stepcounter

modules$(DELIM)sensing$(DELIM)stepcounter$(DELIM)libstepcounter$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)stepcounter TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libstepcounter$(LIBEXT)

lib$(DELIM)libstepcounter$(LIBEXT): modules$(DELIM)sensing$(DELIM)stepcounter$(DELIM)libstepcounter$(LIBEXT)
	$(Q) install $< $@
