
ifeq ($(CONFIG_LCD_ET014TT1),y)
EXTLIBS += lib$(DELIM)libSWTCON2_Cortex-M4_wchar32.a
endif

lib$(DELIM)libSWTCON2_Cortex-M4_wchar32.a: contrib$(DELIM)libSWTCON2_Cortex-M4_wchar32.a
	$(Q) install $< $@
