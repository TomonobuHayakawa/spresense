
ifeq ($(CONFIG_NET_LWIP),y)
SDKLIBS += lib$(DELIM)libextnet$(LIBEXT)
SDKMODDIRS += modules$(DELIM)net
#CONTEXTDIRS += modules$(DELIM)net
endif
SDKCLEANDIRS += modules$(DELIM)net

modules$(DELIM)net$(DELIM)libextnet$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)net TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libextnet$(LIBEXT)

lib$(DELIM)libextnet$(LIBEXT): modules$(DELIM)net$(DELIM)libextnet$(LIBEXT)
	$(Q) install $< $@
