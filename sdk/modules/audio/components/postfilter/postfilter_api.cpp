/********************************************************************
 *
 *  File Name: postfilter_api.cpp
 *
 *  Description: PostProcess Componet
 *
 *  Notes: (C) Copyright 2018 Sony Corporation
 *
 *  Author: -
 *
 ********************************************************************
 */

#include "postfilter_api.h"

extern "C"
{
/*--------------------------------------------------------------------
    C Interface
  --------------------------------------------------------------------*/
uint32_t AS_postfilter_init(const InitPostfilterParam *param,
                            void *p_instance,
                            uint32_t *dsp_inf)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL || dsp_inf == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Execute */

  return ((PostfilterBase *)p_instance)->init_apu(*param, dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_postfilter_exec(const ExecPostfilterParam *param, void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Execute */

  return ((PostfilterBase *)p_instance)->exec_apu(*param);
}

/*--------------------------------------------------------------------*/
bool AS_postfilter_flush(const FlushPostfilterParam *param, void *p_instance)
{
  /* Parameter check */

  if (param == NULL || p_instance == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Execute */

  return ((PostfilterBase *)p_instance)->flush_apu(*param);
}

/*--------------------------------------------------------------------*/
bool AS_postfilter_recv_done(void *p_instance, PostfilterCmpltParam *output)
{
  /* Parameter check */

  if (p_instance == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Execute */

  if (output == NULL)
    {
      return ((PostfilterBase *)p_instance)->recv_done();
    }
  else
    {
      return ((PostfilterBase *)p_instance)->recv_done(output);
    }
}

/*--------------------------------------------------------------------*/
uint32_t AS_postfilter_activate(void **p_instance,
                                MemMgrLite::PoolId apu_pool_id,
                                MsgQueId apu_mid,
                                uint32_t *dsp_inf,
                                bool through)
{
  if (p_instance == NULL || dsp_inf == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  /* Reply pointer of self instance, which is used for API call. */

  *p_instance = (through) ?
                    (void*)(new PostfilterThrough())
                  : (void*)(new PostfilterComponent(apu_pool_id,apu_mid));

  if (*p_instance == NULL)
    {
      POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_RESOURCE_ERROR);
      return AS_ECODE_COMMAND_PARAM_OUTPUT_DATE;
    }

  return ((PostfilterBase *)*p_instance)->activate(dsp_inf);
}

/*--------------------------------------------------------------------*/
bool AS_postfilter_deactivate(void *p_instance)
{

  if ((PostfilterBase *)p_instance != NULL)
    {
      ((PostfilterBase *)p_instance)->deactivate();
      delete (PostfilterBase*)p_instance;
      return true;
    }

  POSTFILTER_ERR(AS_ATTENTION_SUB_CODE_UNEXPECTED_PARAM);

  return false;
}
} /* extern "C" */

