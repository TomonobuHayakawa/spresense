#
# This Makefile included from Makefile in the top of SDK directory.
# SDK build system uses below variables to control build.
#
# SDKLIBS      - Library files to be linked.
# SDKMODDIRS   - Module directories.
# SDKCLEANDIRS - Clean directories. It must be added whenever configured or not.
# CONTEXTDIRS  - Directories for target in context build stage. This variable can be omitted.
#                If you want to do prepare for build, add it and write context target in the Makefile.
# 

ifeq ($(CONFIG_SKELETON),y)
SDKLIBS += lib$(DELIM)libskeleton$(LIBEXT)
SDKMODDIRS += modules$(DELIM)skeleton
#CONTEXTDIRS += modules$(DELIM)skeleton
endif
SDKCLEANDIRS += modules$(DELIM)skeleton

# Below 2 rules are mandatory, replace all of 'skeleton' to your library name or directory.

modules$(DELIM)skeleton$(DELIM)libskeleton$(LIBEXT): context
	$(Q) $(MAKE) -C modules$(DELIM)skeleton TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libskeleton$(LIBEXT)

lib$(DELIM)libskeleton$(LIBEXT): modules$(DELIM)skeleton$(DELIM)libskeleton$(LIBEXT)
	$(Q) install $< $@
