# Select components build by config

ifeq ($(CONFIG_AUDIOUTILS_CAPTURE),y)
include modules/audio/components/capture/LibTargets.mk
endif

ifeq ($(CONFIG_AUDIOUTILS_DECODER),y)
include modules/audio/components/decoder/LibTargets.mk
endif

ifeq ($(CONFIG_AUDIOUTILS_ENCODER),y)
include modules/audio/components/encoder/LibTargets.mk
endif

ifeq ($(CONFIG_AUDIOUTILS_FILTER),y)
include modules/audio/components/filter/LibTargets.mk
endif

ifeq ($(CONFIG_AUDIOUTILS_RECOGNITION),y)
include modules/audio/components/recognition/LibTargets.mk
endif

ifeq ($(CONFIG_AUDIOUTILS_RENDERER),y)
include modules/audio/components/renderer/LibTargets.mk
endif

ifeq ($(CONFIG_AUDIOUTILS_COMPONENT_COMMON),y)
include modules/audio/components/common/LibTargets.mk
endif

