# System utility library

SDKLIBS += lib$(DELIM)libsystem$(LIBEXT)
SDKMODDIRS += system
SDKCLEANDIRS += system
CONTEXTDIRS += system

system$(DELIM)libsystem$(LIBEXT): context
	$(Q) $(MAKE) -C system TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libsystem$(LIBEXT)

lib$(DELIM)libsystem$(LIBEXT): system$(DELIM)libsystem$(LIBEXT)
	$(Q) install $< $@
