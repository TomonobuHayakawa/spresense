# Message library

ifeq ($(CONFIG_MEMUTILS),y)
ifeq ($(CONFIG_MEMUTILS_MESSAGE),y)
SDKLIBS += lib$(DELIM)libmessage$(LIBEXT)
SDKMODDIRS += modules$(DELIM)memutils$(DELIM)message
endif
endif
SDKCLEANDIRS += modules$(DELIM)memutils$(DELIM)message

modules$(DELIM)memutils$(DELIM)message$(DELIM)libmessage$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)memutils$(DELIM)message TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libmessage$(LIBEXT)

lib$(DELIM)libmessage$(LIBEXT): modules$(DELIM)memutils$(DELIM)message$(DELIM)libmessage$(LIBEXT)
	$(Q) install $< $@
