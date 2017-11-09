
ifeq ($(CONFIG_ARMGESTURE),y)
SDKLIBS += lib$(DELIM)libarmgesture$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)arm_gesture
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)arm_gesture
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)arm_gesture

modules$(DELIM)sensing$(DELIM)arm_gesture$(DELIM)libarmgesture$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)arm_gesture TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libarmgesture$(LIBEXT)

lib$(DELIM)libarmgesture$(LIBEXT): modules$(DELIM)sensing$(DELIM)arm_gesture$(DELIM)libarmgesture$(LIBEXT)
	$(Q) install $< $@
