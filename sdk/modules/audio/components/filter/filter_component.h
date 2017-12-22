/***********************************************************************
 *
 *      File Name: filter_component.h
 *
 *      Description: Header file of filter component
 *
 *      Notes: (C) Copyright 2015 Sony Corporation
 *
 *      Author: Hsingying Ho
 *
 ***********************************************************************
 */

#ifndef FILTER_COMPONENT_H
#define FILTER_COMPONENT_H

#include "memutils/os_utils/chateau_osal.h"
#include "wien2_common_defs.h"
#include "memutils/memory_manager/MemHandle.h"
#include "memutils/message/Message.h"

#ifdef CONFIG_AUDIOUTILS_SRC
#include "src_filter_component.h"
#endif
#ifdef CONFIG_AUDIOUTILS_MFE
#include "mfe_filter_component.h"
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
#include "mpp_filter_component.h"
#endif

#include "dsp_driver/include/dsp_drv.h"

__WIEN2_BEGIN_NAMESPACE

using namespace MemMgrLite;


typedef bool (*FilterCompCallback)(DspDrvComPrm_t*);

/* This enumertation defines all the types of filter component can provide. 
 * For component's user, it is not necessary to know how this component is
 * built, is it tunnelled or non-tunnelled. User only focus on what kinds of
 * functions they need to use.
 */
enum FilterComponentType
{
  SRCOnly = 0,           /* Use single core to process SRC. */
  MfeOnly,               /* Use MFE-SRC tunnelled modules. */
  MppEax,                /* Use master: XLOUD-SRC tunnelled
                          * and slave: MFE mechenism.
                          */
  FilterComponentTypeNum
};

/* TODO: Consideration for validity of this structure is needed */
struct FilterComponentParam
{
  Apu::ApuFilterType filter_type;
  FilterCompCallback callback;

  union
  {
#ifdef CONFIG_AUDIOUTILS_SRC
    InitSRCParam init_src_param;
    ExecSRCParam exec_src_param;
    StopSRCParam stop_src_param;
#endif
#ifdef CONFIG_AUDIOUTILS_MFE
    InitMFEParam              init_mfe_param;
    ExecMFEParam              exec_mfe_param;
    Apu::ApuSetParamFilterCmd set_mfe_param;
    Apu::ApuTuningFilterCmd   tuning_mfe_param;
#endif
#ifdef CONFIG_AUDIOUTILS_MPP
    InitXLOUDParam            init_xloud_param;
    ExecXLOUDParam            exec_xloud_param;
    Apu::ApuSetParamFilterCmd set_mpp_param;
    Apu::ApuTuningFilterCmd   tuning_mpp_param;
#endif
  };
};

#ifdef CONFIG_AUDIOUTILS_SRC
struct SrcFilterCompCmpltParam
{
  Apu::ApuEventType event_type;

  union
  {
    InitSRCParam init_src_param;
    ExecSRCParam exec_src_param;
    StopSRCParam stop_src_param;
  };
};
#endif

/*--------------------------------------------------------------------*/
extern "C" {

uint32_t AS_filter_activate(FilterComponentType,MsgQueId,PoolId, uint32_t*);
bool AS_filter_deactivate(FilterComponentType);
uint32_t AS_filter_init(FilterComponentParam, uint32_t*);
bool AS_filter_exec(FilterComponentParam);
bool AS_filter_stop(FilterComponentParam);
bool AS_filter_setparam(FilterComponentParam);
bool AS_filter_tuning(FilterComponentParam);
bool AS_filter_recv_done(void);

} /* extern "C" */

__WIEN2_END_NAMESPACE

#endif /* FILTER_COMPONENT_H */
