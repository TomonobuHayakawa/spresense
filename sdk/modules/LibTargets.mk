# Find and include module library rules except skeleton

include $(filter-out modules/skeleton/%,$(wildcard modules/*/LibTargets.mk))
