
ifeq ($(CONFIG_STEPCOUNTER),y)
SDKLIBS += lib$(DELIM)libstepcounter$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)step_counter
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)step_counter
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)step_counter

modules$(DELIM)sensing$(DELIM)step_counter$(DELIM)libstepcounter$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)step_counter TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libstepcounter$(LIBEXT)

lib$(DELIM)libstepcounter$(LIBEXT): modules$(DELIM)sensing$(DELIM)step_counter$(DELIM)libstepcounter$(LIBEXT)
	$(Q) install $< $@
