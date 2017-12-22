
ifeq ($(CONFIG_SENSING_MANAGER),y)
SDKLIBS += lib$(DELIM)libsensingmgr$(LIBEXT)
SDKMODDIRS += modules$(DELIM)sensing$(DELIM)manager
#CONTEXTDIRS += modules$(DELIM)sensing$(DELIM)manager
endif
SDKCLEANDIRS += modules$(DELIM)sensing$(DELIM)manager

modules$(DELIM)sensing$(DELIM)manager$(DELIM)libsensingmgr$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)sensing$(DELIM)manager TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libsensingmgr$(LIBEXT)

lib$(DELIM)libsensingmgr$(LIBEXT): modules$(DELIM)sensing$(DELIM)manager$(DELIM)libsensingmgr$(LIBEXT)
	$(Q) install $< $@
