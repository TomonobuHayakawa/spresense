############################################################################
# modules/nfc/libnfc/Makefile
#
#   Copyright (C) 2017 Sony Corporation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor Sony nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
###########################################################################

# Do not change

-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs
DELIM ?= $(strip /)
CXXEXT ?= .cpp

OUTPUT_PATH:= ./output

##########################################################
BUILD_EGKI_NCI:=TRUE
BUILD_CE      :=TRUE
BUILD_EE      :=FALSE
BUILD_P2P     :=FALSE
BUILD_CHO     :=FALSE
BUILD_SNEP    :=FALSE
BUILD_EGKI_HAL:=TRUE

##########################################################
EXTRA_LIB_PATH := extra_lib
NFC_LIB_PATH := libnfc-nci
NFA    := src/nfa
NFC    := src/nfc
HAL    := src/hal
UDRV   := src/udrv
HALIMPL := cxd224x
ADAPT   := src/adaptation
LOCAL_C_INCLUDES:=

D_CFLAGS := -DBUILDCFG=1 
D_CFLAGS += -DNFC_HAL_TARGET=1 #@@@

# CXD224x customize
D_CFLAGS += -DLLCP_WAITING_TIME=8 -DRW_I93_TOUT_RESP=150
D_CFLAGS += -DNFA_EE_MAX_AID_ENTRIES=64 -DNFA_EE_MAX_AID_CFG_LEN=640

# Suppress Warning
COMMON_SUPPRESS_WARN = -DGKI_BUFFER_DEBUG=FALSE -DBTU_STACK_LITE_ENABLED=FALSE -DMMI_INCLUDED=FALSE -DNFCNCI_DEBUGFUNC=FALSE -DGKI_PTHREAD_JOINABLE=FALSE -DVALIDATE_PACKET=0 -DSERIAL_AMBA=FALSE
COMMON_SUPPRESS_WARN += -Wno-unused-value -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unused-function -Wno-shadow
C_SUPPRESS_WARN = -Wno-strict-prototypes -Wno-implicit-function-declaration

# Addtional functions
# add -DNFA_HALCC_INCLUDED=1 for enable HALControlCommand
# add -DNFA_HCIDISCONNECT_INCLUDED=1 for enable HciDisconnect
# add -DNFA_EEDISCONNECTFORCE_INCLUDED=1 for enable NFA_EeDisconnectForce()
COMMON_CFLAGS += -DDEBUG -D_DEBUG -DLINUX_NATIVE -DNFA_DYNAMIC_MEMORY=TRUE -DNFC_DYNAMIC_MEMORY=TRUE -DGKI_DYNAMIC_MEMORY=TRUE -DNDEBUG
COMMON_CFLAGS += -DSPZ_IMPL -DSPZ2_IMPL -DNFA_HALCC_INCLUDED=1 -DNFA_HCIDISCONNECT_INCLUDED=1 -DNFA_EEDISCONNECTFORCE_INCLUDED=1
COMMON_CFLAGS += -DNFC_HAL_USE_TRACES=FALSE -DNFC_GKI_USE_TRACES=TRUE -DNFC_RW_ONLY=TRUE
COMMON_CFLAGS += -DNFC_RW_ONLY_PLUS_CE=TRUE -DNFA_NV_CO_INCLUDED=FALSE -DNFC_HAL_HCI_INCLUDED=FALSE

#NFA_CFLAGS=$(D_CFLAGS)
NFA_CFLAGS=$(D_CFLAGS) 
NCI_PATH=src
HAL_PATH=halimpl
VPATH += libnfc-nci/src
VPATH += libnfc-nci/halimpl

## External GKI nci 

TEMP_BASE_DIR = $(SDKDIR)$(DELIM)modules$(DELIM)nfc$(DELIM)libnfc
ifeq ($(WINTOOL),y)
  BASE_DIR = "${shell cygpath -w $(TEMP_BASE_DIR)}"
else
  BASE_DIR = $(TEMP_BASE_DIR)
endif

INCLUDES += -I$(BASE_DIR)$(DELIM)include
INCLUDES += -I$(BASE_DIR)$(DELIM)..$(DELIM)..$(DELIM)include
INCLUDES += -I$(BASE_DIR)$(DELIM)include$(DELIM)cutils
INCLUDES += -I$(BASE_DIR)$(DELIM)reader_hce$(DELIM)include
INCLUDES += -I$(BASE_DIR)$(DELIM)reader_hce
INCLUDES += -I$(BASE_DIR)$(DELIM)lib

LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(EXTRA_LIB_PATH)/gki_ncihal/include
LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(EXTRA_LIB_PATH)/gki_ncihal/gki/ulinux
LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(EXTRA_LIB_PATH)/gki_ncihal/gki/common

LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/src/include
LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/$(NFA)/include
LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/$(NFA)/int
LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/$(NFC)/include
LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/$(NFC)/int
LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/src/hal/include
LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/src/hal/int
LOCAL_C_INCLUDES += -I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/halimpl/cxd224x/include

ADAPTATION_EXTRA_FILES=

# Card Emulation
ifeq ($(BUILD_CE), TRUE)
	NFA_CE =\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)ce$(DELIM)nfa_ce_api.c \
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)ce$(DELIM)nfa_ce_main.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)ce$(DELIM)nfa_ce_act.c
else
	NFA_CE =
endif

# Embeded SecureElement/ HCI interface 
ifeq ($(BUILD_EE), TRUE)
	NFA_EE =\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)ee/nfa_ee_act.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)ee/nfa_ee_api.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)ee/nfa_ee_main.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)hci/nfa_hci_utils.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)hci/nfa_hci_act.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)hci/nfa_hci_main.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)hci/nfa_hci_ci.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)hci/nfa_hci_api.c
	ADAPTATION_EXTRA_FILES +=
else
	CFLAGS += -DNFC_NFCEE_INCLUDED=FALSE  
	NFA_EE =
endif

# P2P
ifeq ($(BUILD_P2P), TRUE)
	NFA_P2P =\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)p2p/nfa_p2p_act.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)p2p/nfa_p2p_main.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)p2p/nfa_p2p_api.c
else
	CFLAGS += -DNFA_P2P_INCLUDED=FALSE
	NFA_P2P =
endif

# Change handover
ifeq ($(BUILD_CHO), TRUE)
	NFA_CHO =\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)cho/nfa_cho_util.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)cho/nfa_cho_sm.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)cho/nfa_cho_main.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)cho/nfa_cho_api.c
else
	CFLAGS += -DNFA_CHO_INCLUDED=FALSE
	NFA_CHO =
endif

# SNEP
ifeq ($(BUILD_SNEP), TRUE)
	NFA_SNEP =\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)snep/nfa_snep_act.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)snep/nfa_snep_main.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)snep/nfa_snep_api.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)snep/nfa_snep_default.c
else
	CFLAGS += -DNFA_SNEP_INCLUDED=FALSE 
	NFA_SNEP =
endif

NFA_FILES = \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)dm/nfa_dm_cfg.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)dm/nfa_dm_act.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)dm/nfa_dm_api.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)dm/nfa_dm_ndef.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)dm/nfa_dm_main.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)dm/nfa_dm_discover.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)rw/nfa_rw_act.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)rw/nfa_rw_main.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)rw/nfa_rw_api.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)sys/nfa_sys_main.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)sys/nfa_sys_ptim.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)sys/nfa_sys_cfg.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFA)$(DELIM)sys/nfa_sys_cback.c \
	$(NFA_CE) \
	$(NFA_EE) \
	$(NFA_P2P) \
	$(NFA_CHO) \
	$(NFA_SNEP)

# P2P
ifeq ($(BUILD_P2P), TRUE)
	NFC_LLCP =\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)llcp/llcp_main.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)llcp/llcp_api.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)llcp/llcp_dlc.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)llcp/llcp_util.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)llcp/llcp_sdp.c\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)llcp/llcp_link.c
else
	NFC_LLCP =
endif

# Embeded SecureElement/ HCI interface 
ifeq ($(BUILD_EE), TRUE)
	NFC_EE =\
		.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)nfc/nfc_ee.c
else
	NFC_EE =
endif

NFC_FILES = \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)ndef/ndef_utils.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)ndef/ndef_cho_utils.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)nci/nci_hrcv.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)nci/nci_hmsgs.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/ce_main.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/ce_t3t.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/ce_t4t.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/rw_t2t_ndef.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/tags_int.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/rw_t3t.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/rw_t1t.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/rw_t4t.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/rw_t2t.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/rw_i93.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/rw_t1t_ndef.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)tags/rw_main.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)nfc/nfc_test.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)nfc/nfc_vs.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)nfc/nfc_main.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)nfc/nfc_ncif.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)nfc/nfc_utils.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)nfc/nfc_halcc.c \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(NFC)$(DELIM)nfc/nfc_task.c \
	$(NFC_LLCP) \
	$(NFC_EE)

C_ADAPTATION_FILES =\
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(ADAPT)$(DELIM)libmain.c\
	$(ADAPTATION_EXTRA_FILES)

CPP_ADAPTATION_FILES = \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(ADAPT)$(DELIM)config.cpp \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(ADAPT)$(DELIM)android_logmsg.cpp \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(ADAPT)$(DELIM)OverrideLog.cpp \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(ADAPT)$(DELIM)NfcAdaptation.cpp \
	.$(DELIM)$(NFC_LIB_PATH)$(DELIM)$(ADAPT)$(DELIM)CrcChecksum.cpp

C_SRCS_NCI = \
	$(NFA_FILES) \
	$(NFC_FILES) \
	$(C_ADAPTATION_FILES) \
	$(GKI_FILES) \
	nfca_version.c

CPP_SRCS_NCI = \
	$(CPP_ADAPTATION_FILES)

##
CNCIFLAGS= $(COMMON_CFLAGS) $(INCLUDES) $(LOCAL_C_INCLUDES) $(NFA_CFLAGS) $(COMMON_SUPPRESS_WARN) $(C_SUPPRESS_WARN)
CXXNCIFLAGS= $(COMMON_CFLAGS) $(INCLUDES) $(LOCAL_C_INCLUDES) $(NFA_CFLAGS) $(COMMON_SUPPRESS_WARN)

#######################################################################
## External GKI hal ##
HAL_C_INCLUDES+= \
		-I$(BASE_DIR)$(DELIM)$(EXTRA_LIB_PATH)/gki_ncihal/include \
		-I$(BASE_DIR)$(DELIM)$(EXTRA_LIB_PATH)/gki_ncihal/gki/ulinux \
		-I$(BASE_DIR)$(DELIM)$(EXTRA_LIB_PATH)/gki_ncihal/gki/common

HAL_C_INCLUDES += \
	-I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/halimpl/cxd224x/include \
	-I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/$(HAL)/include \
	-I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/$(HAL)/int \
	-I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/src/include \
	-I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/$(NFC)/include \
	-I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/$(NFA)/include \
	-I$(BASE_DIR)$(DELIM)$(NFC_LIB_PATH)/$(UDRV)/include

HAL_EXTRA_FILES=

#if BUILD_TRACE
#    HAL_EXTRA_FILES+=$(HALIMPL)/adaptation/android_logmsg.cpp
#endif

C_SRCS_HAL = \
	$(HALIMPL)/hal/hal/nfc_hal_hci_ci.c \
	$(HALIMPL)/hal/hal/nfc_hal_main.c \
	$(HALIMPL)/hal/hal/nfc_hal_api.c \
	$(HALIMPL)/hal/hal/nfc_hal_nci.c \
	$(HALIMPL)/hal/hal/nfc_hal_dm.c \
	$(HALIMPL)/hal/hal/nfc_hal_prm.c \
	$(HALIMPL)/hal/hal/nfc_hal_dm_cfg.c \
	$(HALIMPL)/adaptation/userial_linux.c  \
	$(GKIH_FILES) \
	$(HAL_EXTRA_FILES) \

CPP_SRCS_HAL = \
	$(HALIMPL)/adaptation/android_logmsg.cpp \
	$(HALIMPL)/adaptation/CondVar.cpp \
	$(HALIMPL)/adaptation/HalAdaptation.cpp \
	$(HALIMPL)/adaptation/Mutex.cpp  \
	$(HALIMPL)/adaptation/OverrideLog.cpp \
	$(HALIMPL)/adaptation/NonVolatileStore.cpp \
	$(HALIMPL)/adaptation/patchram.cpp \
	$(HALIMPL)/adaptation/spdhelper.cpp  \
	$(HALIMPL)/adaptation/StartupConfig.cpp \
	$(HALIMPL)/adaptation/config.cpp

##
CHALFLAGS= $(COMMON_CFLAGS) $(INCLUDES) $(HAL_C_INCLUDES) $(NFA_CFLAGS) $(COMMON_SUPPRESS_WARN) $(C_SUPPRESS_WARN)
CXXHALFLAGS= $(COMMON_CFLAGS) $(INCLUDES) $(HAL_C_INCLUDES) $(NFA_CFLAGS) $(COMMON_SUPPRESS_WARN)

#-----------------------------------------------------------------------#
VPATH += lib
C_SRC_LIB = \
	log.c \
	nfc_nci.c
#-----------------------------------------------------------------------#
VPATH += wrapper
C_SRC_PTHREAD = \
	syscall_wrapper.c

#-----------------------------------------------------------------------#
VPATH += reader_hce
C_SRC_API =

VPATH += reader_hce
VPATH += reader_hce/common
CPP_SRC_API = \
	libnfc_api.cpp \
	libnfc_bt_api.cpp \
	PowerSwitch.cpp \
	NativeNfcManager.cpp \
	NativeNfcTag.cpp \
	NfcTag.cpp

#########################################################################################
OBJS_NCI = \
	$(addprefix $(OUTPUT_PATH)/$(NCI_PATH)/,$(patsubst %.c,%.o,$(C_SRCS_NCI))) \
	$(addprefix $(OUTPUT_PATH)/$(NCI_PATH)/,$(patsubst %.cpp,%.o,$(CPP_SRCS_NCI)))

OBJS_HAL = \
	$(addprefix $(OUTPUT_PATH)/$(HAL_PATH)/,$(patsubst %.c,%.o,$(C_SRCS_HAL))) \
	$(addprefix $(OUTPUT_PATH)/$(HAL_PATH)/,$(patsubst %.cpp,%.o,$(CPP_SRCS_HAL)))

OBJS_LIB= \
	$(addprefix $(OUTPUT_PATH)/,$(patsubst %.c,%.o, $(C_SRC_LIB)))

OBJS_WRP= \
	$(addprefix $(OUTPUT_PATH)/,$(patsubst %.c,%.o, $(C_SRC_PTHREAD)))

OBJS_API= \
	$(addprefix $(OUTPUT_PATH)/,$(patsubst %.c,%.o, $(C_SRC_API))) \
	$(addprefix $(OUTPUT_PATH)/,$(patsubst %.cpp,%.o, $(CPP_SRC_API)))

OBJS := $(OBJS_NCI) $(OBJS_HAL) $(OBJS_LIB) $(OBJS_WRP) $(OBJS_API)

CSRCS := $(C_SRCS_NCI) $(C_SRCS_HAL) $(C_SRC_LIB) $(C_SRC_PTHREAD) $(C_SRC_API)
CXXSRCS := $(CPP_SRCS_NCI) $(CPP_SRCS_HAL) $(CPP_SRC_API)
SRCS := $(CSRCS) $(CXXSRCS)

#CFLAGS += -nostdinc
#CXXFLAGS += -nostdinc -I$(TOPDIR)/libxx

TMP_CFLAGS   := $(CFLAGS)
TMP_CXXFLAGS := $(CXXFLAGS)

#########################################################################################
$(OUTPUT_PATH)/$(NCI_PATH)/%.o : %.c
	$(Q) mkdir -p $(dir $@);
	$(eval CFLAGS := $(TMP_CFLAGS) $(CNCIFLAGS))
	$(call COMPILE, $<, $@)

$(OUTPUT_PATH)/$(NCI_PATH)/%.o : %.cpp
	$(Q) mkdir -p $(dir $@);
	$(eval CXXFLAGS := $(TMP_CXXFLAGS) $(CXXNCIFLAGS))
	$(call COMPILEXX, $<, $@)

#------------------------------------------------------------------------------
$(OUTPUT_PATH)/$(HAL_PATH)/%.o : %.c
	$(Q) mkdir -p $(dir $@)
	$(eval CFLAGS := $(TMP_CFLAGS) $(CHALFLAGS))
	$(call COMPILE, $<, $@)

$(OUTPUT_PATH)/$(HAL_PATH)/%.o : %.cpp
	$(Q) mkdir -p $(dir $@);
	$(eval CXXFLAGS := $(TMP_CXXFLAGS) $(CXXHALFLAGS))
	$(call COMPILEXX, $<, $@)

#------------------------------------------------------------------------------
$(OUTPUT_PATH)/%.o : %.c
	$(Q) mkdir -p $(dir $@)
	$(eval CFLAGS := $(TMP_CFLAGS) $(CNCIFLAGS))
	$(call COMPILE, $<, $@)

$(OUTPUT_PATH)/%.o : %.cpp
	$(Q) mkdir -p $(dir $@);
	$(eval CXXFLAGS := $(TMP_CXXFLAGS) $(CXXNCIFLAGS))
	$(call COMPILEXX, $<, $@)

################################################################################

CXXEXT = .cpp

#ifeq ($(CONFIG_WINDOWS_NATIVE),y)
#  BIN		= ..\..\libapps$(LIBEXT)
#else
#ifeq ($(WINTOOL),y)
#  BIN		= ..\\..\\libapps$(LIBEXT)
#else
#  BIN		= ../../libapps$(LIBEXT)
#endif
#endif

BIN = libnfc$(LIBEXT)

ROOTDEPPATH	= --dep-path .

# Common build

#VPATH		=

all: $(BIN)
.PHONY: context depend clean distclean

# $(AOBJS): %$(OBJEXT): %.S
# 	$(call ASSEMBLE, $<, $@)

# $(COBJS): %$(OBJEXT): %.c
# 	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $(BIN), $(OBJS))
	$(Q) touch .built

install:

context:

.depend: Makefile $(CSRCS)
	$(Q) $(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(C_SRCS_API) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)
	$(call DELDIR,$(OUTPUT_PATH))
	$(call DELFILE, $(OBJS))

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep



























#CSRCS = $(NFA_CE) $(NFA_EE) $(NFA_P2P) $(NFA_CHO) $(NFA_SNEP) $(NFA_FILES) $(NFC_LLCP) $(NFC_EE) $(NFC_FILES) $(C_ADAPTATION_FILES) $(CPP_ADAPTATION_FILES)
#CXXSRCS =
#
#BIN = libnfc$(LIBEXT)
#
#AOBJS = $(ASRCS:.S=$(OBJEXT))
#COBJS = $(CSRCS:.c=$(OBJEXT))
#CXXOBJS = $(CXXSRCS:$(CXXEXT)=$(OBJEXT))
#
#SRCS = $(ASRCS) $(CSRCS) $(CXXSRCS)
#LIB_OBJS = $(AOBJS) $(COBJS) $(CXXOBJS)
#
## Here is a example of adding include paths.
## Note that include path addtion will be affected to build time.
## Don't add unnecessary include paths for module.
##
##SKELETONSRCDIR = $(SDKDIR)$(DELIM)modules$(DELIM)skeleton
##ifeq ($(WINTOOL),y)
##  CFLAGS += -I "${shell cygpath -w $(SKELETONSRCDIR)}"
##else
##  CFLAGS += -I$(SKELETONSRCDIR)
##endif
#
#
#
#all: $(BIN)
#.PHONY: context depend clean distclean
#
#$(AOBJS): %$(OBJEXT): %.S
#	$(call ASSEMBLE, $<, $@)
#
#$(COBJS): %$(OBJEXT): %.c
#	$(call COMPILE, $<, $@)
#
#$(CXXOBJS): %$(OBJEXT): %$(CXXEXT)
#	$(call COMPILE, $<, $@)
#
#$(BIN): $(LIB_OBJS)
#	$(call ARCHIVE, $@, $(LIB_OBJS))
#
#.depend: Makefile $(SRCS)
#	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(ASRCS) $(CSRCS) >Make.dep
#	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CXXFLAGS) -- $(CXXSRCS) >>Make.dep
#	$(Q) touch $@
#
#depend: .depend
#
## context target proceeds before depend. User can be used this rule
## to prerequisite process (e.g. generate header file).
#
#.context:
#	$(Q) touch $@
#
#context:
#
#clean:
#	$(call DELFILE, $(BIN))
#	$(call CLEAN)
#	$(call DELFILE, $(COBJS))
#
#distclean: clean
#	$(call DELFILE, .context)
#	$(call DELFILE, Make.dep)
#	$(call DELFILE, .depend)
#
#-include Make.dep

