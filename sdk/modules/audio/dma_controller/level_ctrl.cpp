/****************************************************************************
 * modules/audio/components/renderer/level_ctrl.cpp
 *
 *   Copyright (C) 2016-2017 Sony Corporation. All rights reserved.
 *   Author: Ryuuta Sakane <Ryuuta.Sakane@sony.com>
 *           Hayakawa Tomonobu <Tomonobu.Hayakawa@sony.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor Sony nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
    Include
 ****************************************************************************/
#include "debug/dbg_log.h"
#include "level_ctrl.h"

/*--------------------------------------------------------------------*/
LevelCtrl::LevelCtrlState LevelCtrl::chgMuteState[CmdNum][StatusNum] =
{
  /* Request: OFF  */
  {                /* Fate state: */
    StatusMuteOff, /*   OFF       */
    StatusFadeIn,  /*   OUT       */
    StatusFadeIn,  /*   ON        */
    StatusMuteOff  /*   IN        */
  },

  /* Request: ON   */
  {                /* Fate state: */
    StatusFadeOut, /*   OFF       */
    StatusMuteOn,  /*   OUT       */
    StatusMuteOn,  /*   ON        */
    StatusFadeOut  /*   IN        */
  },

  /* Request: STAY */
  {                /* Fate state: */
    StatusMuteOff, /*   OFF       */
    StatusFadeOut, /*   OUT       */
    StatusMuteOn,  /*   ON        */
    StatusFadeIn   /*   IN        */
  },
};

LevelCtrl::LevelCtrlState LevelCtrl::chgNoFadeMuteState[CmdNum][StatusNum] =
{
  /* Request: OFF  */
  {                /* Fate state: */
    StatusMuteOff, /*   OFF       */
    StatusMuteOff, /*   OUT       */
    StatusMuteOff, /*   ON        */
    StatusMuteOff  /*   IN        */
  },

  /* Request: ON   */
  {                /* Fate state: */
    StatusMuteOn,  /*   OFF       */
    StatusMuteOn,  /*   OUT       */
    StatusMuteOn,  /*   ON        */
    StatusMuteOn   /*   IN        */
  },

  /* Request: STAY */
  {                /* Fate state: */
    StatusMuteOff, /*   OFF       */
    StatusMuteOn,  /*   OUT       */
    StatusMuteOn,  /*   ON        */
    StatusMuteOff  /*   IN        */
  },
};

LevelCtrl::ActProc LevelCtrl::ActProcTbl[StatusNum][StatusNum] =
{
  /* Next status: OFF */
  {                            /* Current status: */
    &LevelCtrl::no_action,     /*   OFF           */
    &LevelCtrl::unMuteSdinVol, /*   OUT           */
    &LevelCtrl::unMuteSdinVol, /*   ON            */
    &LevelCtrl::no_action      /*   IN            */
  },

  /* Next status: OUT */
  {                            /* Current status: */
    &LevelCtrl::muteSdinVol,   /*   OFF           */
    &LevelCtrl::no_action,     /*   OUT           */
    &LevelCtrl::no_action,     /*   ON            */
    &LevelCtrl::muteSdinVol    /*   IN            */
  },

  /* Next status: ON  */
  {                            /* Current status: */
    &LevelCtrl::muteSdinVol,   /*   OFF           */
    &LevelCtrl::no_action,     /*   OUT           */
    &LevelCtrl::no_action,     /*   ON            */
    &LevelCtrl::muteSdinVol    /*   IN            */
  },

  /* Next status: IN  */
  {                            /* Current status: */
    &LevelCtrl::no_action,     /*   OFF           */
    &LevelCtrl::unMuteSdinVol, /*   OUT           */
    &LevelCtrl::unMuteSdinVol, /*   ON            */
    &LevelCtrl::no_action      /*   IN            */
  },
};

/*--------------------------------------------------------------------*/
bool LevelCtrl::init(asDmacSelId dmac_id, bool auto_fade, bool fade_enable)
{
  /* select state transition table and initial mute state request */

  if (fade_enable == true)
    {
      this->m_tablePtr = chgMuteState;
    }
  else
    {
      this->m_tablePtr = chgNoFadeMuteState;
    }

  /* initialize target mixer id */

  this->m_sdin = GetDmacPathToMixerId(dmac_id);

  /* store auto_fade info */

  this->m_auto_fade = auto_fade;

  /* set initial muting state according to auto_fade */

  if (auto_fade == true)
    {
      this->muteSdinVol(true);
      this->m_state = StatusMuteOn;
    }
  else
    {
      this->unMuteSdinVol(true);
      this->m_state = StatusMuteOff;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool LevelCtrl::setFadeRamp(uint32_t* samples_for_fade)
{
  uint32_t rate           = 0;
  uint32_t dig_sft        = 0;
  E_AS     rtcd           = E_AS_OK;

  /* if mute is not enabled, DIG_SFT register is 0 (fade not support) */

  dig_sft = (this->m_tablePtr == chgMuteState) ? 1 : 0;

  /* calc fade required samples */

  *samples_for_fade = 996;

  /* set to baseband driver */

  rtcd = asAc_SetDigSft(dig_sft);
  if (rtcd != E_AS_OK)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return false;
    }

  rtcd = asAc_SetDsrRate(rate);
  if (rtcd != E_AS_OK)
    {
      DMAC_ERR(AS_ATTENTION_SUB_CODE_BASEBAND_ERROR);
      return false;
    }

  return true;
}

/*--------------------------------------------------------------------*/
bool LevelCtrl::exec(LevelCtrlCmd request, bool is_wait)
{
  LevelCtrlState next_state;

  /* parameter check */

  if (request >= CmdNum)
    {
      request = CmdStay;
    }

  if (this->m_tablePtr == NULL)
    {
      return false;
    }

  /* get next state */

  next_state = this->m_tablePtr[request][this->m_state];

  /* action */

  (this->*ActProcTbl[next_state][this->m_state])(is_wait);

  /* state change */

  this->m_state = next_state;

  return true;
}

/*--------------------------------------------------------------------*/
void LevelCtrl::muteSdinVol(bool wait_fade_term)
{
  switch (this->m_sdin)
    {
      case AS_MIXER1_OUT:
        muteVolumeFade(AS_VOLUME_INPUT1, wait_fade_term);
        break;

      case AS_MIXER2_OUT:
        muteVolumeFade(AS_VOLUME_INPUT2, wait_fade_term);
        break;

      default:
        /* do nothing */
        break;
    }
}

/*--------------------------------------------------------------------*/
void LevelCtrl::unMuteSdinVol(bool wait_fade_term)
{
  switch (this->m_sdin)
    {
      case AS_MIXER1_OUT:
        unMuteVolumeFade(AS_VOLUME_INPUT1, wait_fade_term);
        break;

      case AS_MIXER2_OUT:
        unMuteVolumeFade(AS_VOLUME_INPUT2, wait_fade_term);
        break;

      default:
        /* do nothing */
        break;
    }
}

void LevelCtrl::no_action(bool wait_flg)
{
  /* do nothing */
}

