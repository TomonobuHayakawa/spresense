# XXX

EXAMPLES_DIR := $(SDKDIR)/../examples

$(EXAMPLES_DIR)$(DELIM)libexamples$(LIBEXT): context
	$(Q) $(MAKE) -C $(dir $@) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" $(notdir $@)

lib$(DELIM)libexamples$(LIBEXT): $(EXAMPLES_DIR)$(DELIM)libexamples$(LIBEXT)
	$(Q) install $< $@

EXTLIBS += lib$(DELIM)libexamples$(LIBEXT)
