
ifeq ($(CONFIG_CMDFW),y)
SDKLIBS += lib$(DELIM)libcmdfw$(LIBEXT)
SDKMODDIRS += modules$(DELIM)cmdfw
#CONTEXTDIRS += modules$(DELIM)cmdfw
endif
SDKCLEANDIRS += modules$(DELIM)cmdfw

modules$(DELIM)cmdfw$(DELIM)libcmdfw$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)cmdfw TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libcmdfw$(LIBEXT)

lib$(DELIM)libcmdfw$(LIBEXT): modules$(DELIM)cmdfw$(DELIM)libcmdfw$(LIBEXT)
	$(Q) install $< $@
