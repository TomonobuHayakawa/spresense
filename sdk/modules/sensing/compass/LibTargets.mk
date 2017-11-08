
ifeq ($(CONFIG_COMPASS),y)
SDKLIBS += lib$(DELIM)libcompass$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)compass
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)compass
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)compass

modules$(DELIM)sensing$(DELIM)compass$(DELIM)libcompass$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)compass TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libcompass$(LIBEXT)

lib$(DELIM)libcompass$(LIBEXT): modules$(DELIM)sensing$(DELIM)compass$(DELIM)libcompass$(LIBEXT)
	$(Q) install $< $@
