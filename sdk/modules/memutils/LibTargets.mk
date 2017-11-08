
ifeq ($(CONFIG_MEMUTILS),y)
SDKLIBS += lib$(DELIM)libmemutils$(LIBEXT)
SDKMODDIRS += modules$(DELIM)memutils
#CONTEXTDIRS += modules$(DELIM)memutils
endif
SDKCLEANDIRS += modules$(DELIM)memutils

modules$(DELIM)memutils$(DELIM)libmemutils$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)memutils TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libmemutils$(LIBEXT)

lib$(DELIM)libmemutils$(LIBEXT): modules$(DELIM)memutils$(DELIM)libmemutils$(LIBEXT)
	$(Q) install $< $@
