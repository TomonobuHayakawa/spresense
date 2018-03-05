/**********************************************************************
 *
 *      File Name: postfilter_ctrl.cxx
 *
 *      Description: filter control
 *
 *      Notes: (C) Copyright 2018 Sony Corporation
 *
 *      Author: -
 *
 **********************************************************************
 */
#include "postfilter_ctrl.h"

/*--------------------------------------------------------------------*/
PostFilterCtrl::CtrlProc PostFilterCtrl::CtrlFuncTbl[Wien2::Apu::ApuEventTypeNum][StateNum] =
{
  /* boot */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::illegal,  /* Ready  */
    &PostFilterCtrl::illegal   /* Exec   */
  },

  /* init */
  {
    &PostFilterCtrl::init,     /* Booted */
    &PostFilterCtrl::init,     /* Ready  */
    &PostFilterCtrl::illegal   /* Exec   */
  },

  /* exec */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::exec,     /* Ready  */
    &PostFilterCtrl::exec      /* Exec   */
  },

  /* flush */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::illegal,  /* Ready  */
    &PostFilterCtrl::flush     /* Exec   */
  },

  /* setParam */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::setparam, /* Ready  */
    &PostFilterCtrl::setparam  /* Exec   */
  },

  /* tuning */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::illegal,  /* Ready  */
    &PostFilterCtrl::illegal   /* Exec   */
  },

  /* error */
  {
    &PostFilterCtrl::illegal,  /* Booted */
    &PostFilterCtrl::illegal,  /* Ready  */
    &PostFilterCtrl::illegal   /* Exec   */
  },
};

/*--------------------------------------------------------------------*/
void PostFilterCtrl::parse(Wien2::Apu::Wien2ApuCmd *cmd)
{
  (this->*CtrlFuncTbl[cmd->header.event_type][m_state])(cmd);
}

/*--------------------------------------------------------------------*/
void PostFilterCtrl::init(Wien2::Apu::Wien2ApuCmd *cmd)
{
  Wien2::Apu::ApuInitPostFilterCmd *init_cmd = &cmd->init_postfilter_cmd;

  (void)init_cmd;
  cmd->result.exec_result = Wien2::Apu::ApuExecOK;

  m_state = ReadyStatus;
}

/*--------------------------------------------------------------------*/
void PostFilterCtrl::exec(Wien2::Apu::Wien2ApuCmd *cmd)
{
  Wien2::Apu::ApuExecPostFilterCmd *exec_cmd = &cmd->exec_postfilter_cmd;

  /* !!tentative!! simply copy from input to output */

  memcpy(exec_cmd->output_buffer.p_buffer,
         exec_cmd->input_buffer.p_buffer,
         exec_cmd->input_buffer.size);

  //{
  //  /* RC filter example */

  //  int16_t *ls = (int16_t*)exec_cmd->output_buffer.p_buffer;
  //  int16_t *rs = ls + 1;

  //  static int16_t ls_l = 0;
  //  static int16_t rs_l = 0;

  //  if (!ls_l && !rs_l)
  //    {
  //      ls_l = *ls;
  //      rs_l = *rs;
  //    }

  //  for (uint32_t cnt = 0; cnt < exec_cmd->input_buffer.size; cnt += 4)
  //    {
  //      *ls = (ls_l * 99 / 100) + (*ls * 1 / 100);
  //      *rs = (rs_l * 99 / 100) + (*rs * 1 / 100);

  //      ls_l = *ls;
  //      rs_l = *rs;

  //      ls += 2;
  //      rs += 2;
  //    }
  //}

  cmd->result.exec_result = Wien2::Apu::ApuExecOK;

  m_state = ExecStatus;
}

/*--------------------------------------------------------------------*/
void PostFilterCtrl::flush(Wien2::Apu::Wien2ApuCmd *cmd)
{
  Wien2::Apu::ApuFlushPostFilterCmd *flush_cmd = &cmd->flush_postfilter_cmd;

  (void)flush_cmd;
  cmd->result.exec_result = Wien2::Apu::ApuExecOK;

  m_state = ReadyStatus;
}

/*--------------------------------------------------------------------*/
void PostFilterCtrl::setparam(Wien2::Apu::Wien2ApuCmd *cmd)
{
  cmd->result.exec_result = Wien2::Apu::ApuExecOK;
}

/*--------------------------------------------------------------------*/
void PostFilterCtrl::illegal(Wien2::Apu::Wien2ApuCmd *cmd)
{
  cmd->result.exec_result = Wien2::Apu::ApuExecError;
  /* Error */
}

