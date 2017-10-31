# XXX

$(SDKDIR)$(DELIM)..$(DELIM)examples$(DELIM)libexamples$(LIBEXT): context
	$(Q) $(MAKE) -C $(dir $@) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" $(notdir $@)

lib$(DELIM)libexamples$(LIBEXT): $(SDKDIR)$(DELIM)..$(DELIM)examples$(DELIM)libexamples$(LIBEXT)
	$(Q) install $< $@

EXTLIBS += lib$(DELIM)libexamples$(LIBEXT)
