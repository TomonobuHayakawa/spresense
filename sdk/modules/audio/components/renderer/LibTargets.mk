# Renderer component library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)librenderer$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)renderer
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)components$(DELIM)renderer

modules$(DELIM)audio$(DELIM)components$(DELIM)renderer$(DELIM)librenderer$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)components$(DELIM)renderer TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" librenderer$(LIBEXT)

lib$(DELIM)librenderer$(LIBEXT): modules$(DELIM)audio$(DELIM)components$(DELIM)renderer$(DELIM)librenderer$(LIBEXT)
	$(Q) install $< $@
