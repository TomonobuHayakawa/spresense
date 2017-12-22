# DSP driver library

ifeq ($(CONFIG_SDK_AUDIO),y)

# Conditions to build dsp driver

CONDITION_DSPDRV = $(CONFIG_AUDIOUTILS_PLAYER)
CONDITION_DSPDRV += $(CONFIG_AUDIOUTILS_RECORDER)
CONDITION_DSPDRV += $(CONFIG_AUDIOUTILS_VOICE_COMMAND)
CONDITION_DSPDRV += $(CONFIG_AUDIOUTILS_VOICE_CALL)

ifeq ($(findstring y, $(CONDITION_DSPDRV)),y)
SDKLIBS += lib$(DELIM)libdspdriver$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)dsp_driver
endif
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)dsp_driver

modules$(DELIM)audio$(DELIM)dsp_driver$(DELIM)libdspdriver$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)dsp_driver TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libdspdriver$(LIBEXT)

lib$(DELIM)libdspdriver$(LIBEXT): modules$(DELIM)audio$(DELIM)dsp_driver$(DELIM)libdspdriver$(LIBEXT)
	$(Q) install $< $@
