# Select objects by function which is indicated by config.

ifeq ($(CONFIG_AUDIOUTILS_PLAYER),y)
include modules/audio/objects/media_player/LibTargets.mk
include modules/audio/objects/output_mixer/LibTargets.mk
include modules/audio/objects/stream_parser/LibTargets.mk
endif

ifeq ($(CONFIG_AUDIOUTILS_RECORDER),y)
include modules/audio/objects/media_recorder/LibTargets.mk
endif

ifeq ($(findstring y, $(CONFIG_AUDIOUTILS_VOICE_CALL) $(CONFIG_AUDIOUTILS_VOICE_COMMAND)),y)
include modules/audio/objects/sound_effector/LibTargets.mk
endif

ifeq ($(CONFIG_AUDIOUTILS_VOICE_COMMAND),y)
include modules/audio/objects/sound_recognizer/LibTargets.mk
endif

