# Memory manager library

ifeq ($(CONFIG_MEMUTILS),y)
ifeq ($(CONFIG_MEMUTILS_MEMORY_MANAGER),y)
SDKLIBS += lib$(DELIM)libmemorymanager$(LIBEXT)
SDKMODDIRS += modules$(DELIM)memutils$(DELIM)memory_manager
endif
endif
SDKCLEANDIRS += modules$(DELIM)memutils$(DELIM)memory_manager

modules$(DELIM)memutils$(DELIM)memory_manager$(DELIM)libmemorymanager$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)memutils$(DELIM)memory_manager TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libmemorymanager$(LIBEXT)

lib$(DELIM)libmemorymanager$(LIBEXT): modules$(DELIM)memutils$(DELIM)memory_manager$(DELIM)libmemorymanager$(LIBEXT)
	$(Q) install $< $@
