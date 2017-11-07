
# Include sub level library build rules

include $(wildcard */LibTargets.mk)

# Glob external sub directories which contains 'LibTarget.mk' file.

define ExtSubDirectory_template
	include $(1)/LibTarget.mk
endef

EXTSUBDIRS = $(dir $(wildcard ../*/LibTarget.mk))

$(foreach SDIR, $(EXTSUBDIRS), $(eval $(call ExtSubDirectory_template,$(SDIR))))
