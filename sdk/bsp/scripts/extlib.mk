ifeq ($(CONFIG_RIL),y)
  ifeq ($(WINTOOL),y)
    RILLIB = "${shell cygpath -w $(TOPDIR)$(DELIM)$(CONFIG_APPS_DIR)$(DELIM)ril}"
  else
    RILLIB = "$(TOPDIR)$(DELIM)$(CONFIG_APPS_DIR)$(DELIM)ril"
  endif
  LIBPATHS += -L $(RILLIB)
  EXTRA_LIBS += -lril
endif

ifeq ($(CONFIG_EINK_ET014TT1),y)
  ifeq ($(WINTOOL),y)
    EINKLIB = "${shell cygpath -w $(TOPDIR)$(DELIM)$(CONFIG_APPS_DIR)$(DELIM)swtcon}"
  else
    EINKLIB = "$(TOPDIR)$(DELIM)$(CONFIG_APPS_DIR)$(DELIM)swtcon"
  endif
  LIBPATHS += -L $(EINKLIB)
  EXTRA_LIBS += -lSWTCON2_Cortex-M4_wchar32
endif

ifeq ($(CONFIG_GPSUTILS_CXD56NMEA_LIB),y)
  ifeq ($(APPDIR),)
    APPDIR := $(TOPDIR)$(DELIM)$(CONFIG_APPS_DIR)
  endif
  ifeq ($(WINTOOL),y)
    CXD56NMEALIBPATH = "${shell cygpath -w $(APPDIR)$(DELIM)gpsutils$(DELIM)cxd56nmea}"
  else
    CXD56NMEALIBPATH = "$(APPDIR)$(DELIM)gpsutils$(DELIM)cxd56nmea"
  endif
  LIBPATHS += -L $(CXD56NMEALIBPATH)
  EXTRA_LIBS += -lcxd56nmea
endif
