/**********************************************************************
 *
 *      File Name: fil_ctrl.h
 *
 *      Description: filter Control Class
 *
 *      Notes: (C) Copyright 2018 Sony Corporation
 *
 *      Author: -
 *
 **********************************************************************
 */

#ifndef __POSTFILTER_CTRL_H__
#define __POSTFILTER_CTRL_H__

#include <string.h>

#include "apus/apu_cmd.h"
#include "postfilter_command.h"

class PostFilterCtrl
{
public:
  void parse(Wien2::Apu::Wien2ApuCmd *cmd);

  PostFilterCtrl()
    : m_state(BootedStatus)
  {}

private:
  enum StateType
  {
    BootedStatus = 0,
    ReadyStatus,
    ExecStatus,
    StateNum
  };

  StateType m_state;
  typedef void (PostFilterCtrl::*CtrlProc)(Wien2::Apu::Wien2ApuCmd *cmd);
  static CtrlProc CtrlFuncTbl[Wien2::Apu::ApuEventTypeNum][StateNum];

  void init(Wien2::Apu::Wien2ApuCmd *cmd);
  void exec(Wien2::Apu::Wien2ApuCmd *cmd);
  void flush(Wien2::Apu::Wien2ApuCmd *cmd);
  void setparam(Wien2::Apu::Wien2ApuCmd *cmd);
  void illegal(Wien2::Apu::Wien2ApuCmd *cmd);
};

#endif /* __POSTFILTER_CTRL_H__ */

