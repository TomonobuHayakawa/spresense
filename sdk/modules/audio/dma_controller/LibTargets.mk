# DMA controller library

ifeq ($(CONFIG_SDK_AUDIO),y)

# Conditions to build DMA controller 

CONDITION_DMAC = $(CONFIG_AUDIOUTILS_PLAYER)
CONDITION_DMAC += $(CONFIG_AUDIOUTILS_RECORDER)
CONDITION_DMAC += $(CONFIG_AUDIOUTILS_VOICE_COMMAND)
CONDITION_DMAC += $(CONFIG_AUDIOUTILS_VOICE_CALL)

ifeq ($(findstring y, $(CONDITION_DMAC)),y)
SDKLIBS += lib$(DELIM)libdmacontroller$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)dma_controller
endif
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)dma_controller

modules$(DELIM)audio$(DELIM)dma_controller$(DELIM)libdmacontroller$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)dma_controller TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libdmacontroller$(LIBEXT)

lib$(DELIM)libdmacontroller$(LIBEXT): modules$(DELIM)audio$(DELIM)dma_controller$(DELIM)libdmacontroller$(LIBEXT)
	$(Q) install $< $@
