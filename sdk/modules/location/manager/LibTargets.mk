
ifeq ($(CONFIG_LOCATION_MANAGER),y)
SDKLIBS += lib$(DELIM)liblocationmgr$(LIBEXT)
SDKMODDIRS += modules$(DELIM)location$(DELIM)manager
#CONTEXTDIRS += modules$(DELIM)location$(DELIM)manager
endif
SDKCLEANDIRS += modules$(DELIM)location$(DELIM)manager

modules$(DELIM)location$(DELIM)manager$(DELIM)liblocationmgr$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)location$(DELIM)manager TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" liblocationmgr$(LIBEXT)

lib$(DELIM)liblocationmgr$(LIBEXT): modules$(DELIM)location$(DELIM)manager$(DELIM)liblocationmgr$(LIBEXT)
	$(Q) install $< $@
