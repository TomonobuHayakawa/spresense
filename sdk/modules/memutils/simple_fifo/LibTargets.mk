# Simple fifo library

ifeq ($(CONFIG_MEMUTILS),y)
ifeq ($(CONFIG_MEMUTILS_SIMPLE_FIFO),y)
SDKLIBS += lib$(DELIM)libsimplefifo$(LIBEXT)
SDKMODDIRS += modules$(DELIM)memutils$(DELIM)simple_fifo
endif
endif
SDKCLEANDIRS += modules$(DELIM)memutils$(DELIM)simple_fifo

modules$(DELIM)memutils$(DELIM)simple_fifo$(DELIM)libsimplefifo$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)memutils$(DELIM)simple_fifo TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libsimplefifo$(LIBEXT)

lib$(DELIM)libsimplefifo$(LIBEXT): modules$(DELIM)memutils$(DELIM)simple_fifo$(DELIM)libsimplefifo$(LIBEXT)
	$(Q) install $< $@
