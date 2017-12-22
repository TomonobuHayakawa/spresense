# Sound effector library

ifeq ($(CONFIG_SDK_AUDIO),y)
SDKLIBS += lib$(DELIM)libsoundeffector$(LIBEXT)
SDKMODDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)sound_effector
endif
SDKCLEANDIRS += modules$(DELIM)audio$(DELIM)objects$(DELIM)sound_effector

modules$(DELIM)audio$(DELIM)objects$(DELIM)sound_effector$(DELIM)libsoundeffector$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)audio$(DELIM)objects$(DELIM)sound_effector TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libsoundeffector$(LIBEXT)

lib$(DELIM)libsoundeffector$(LIBEXT): modules$(DELIM)audio$(DELIM)objects$(DELIM)sound_effector$(DELIM)libsoundeffector$(LIBEXT)
	$(Q) install $< $@
