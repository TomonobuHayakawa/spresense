/****************************************************************************
 *
 *      File Name: MsgWrapper.h
 *
 *      Description: Message wrapper API implementation.
 *
 *      Notes: (C) Copyright 2013 Sony Corporation
 *
 *      Author: Satoru AIZAWA
 *
 ****************************************************************************
 */
#ifndef MSG_WRAPPER_H_INCLUDED
#define MSG_WRAPPER_H_INCLUDED

#ifdef _ITRON4
#include "kernel_id.h"	/* TNUM_TICK */
const uint32_t TickPrecision = TNUM_TICK;
#else
const uint32_t TickPrecision = 1;
#endif
#include "memutils/message/Message.h"

/*****************************************************************
 * メッセージラッパクラス (OS有無などによりCPU毎にカスタマイズが必要)
 *****************************************************************/
class MsgWrapper {
public:
  /* メッセージパケットの送信(タスクコンテキスト、パラメタなし) */
  static err_t sendWithTimeout(MsgQueId dest, MsgPri pri, MsgType type, MsgQueId reply, size_t ms)
    {
      /* 送信失敗かつタイムアウト時間が指定されていれば、リトライを行う */
      err_t ret = MsgLib::send(dest, pri, type, reply);
      if (ret != ERR_OK && ms > 0)
        {
          err_t        err_code;
          MsgQueBlock* que;
          err_code = MsgLib::referMsgQueBlock(dest, &que);
          if (err_code != ERR_OK)
            {
              return err_code;
            }
          DMP_MSGLIB_WARN(MsgRetryLog('b', type, dest, pri, reply, GET_CPU_ID(), que->getNumMsg(pri), 0, 0x00));
          for (size_t i = 0; ms == TIME_FOREVER || i < ms; i += TickPrecision)
            {
              SleepTask(1);
              ret = MsgLib::send(dest, pri, type, reply);
              if (ret == ERR_OK)
                {
                  break;
                }
            }
          DMP_MSGLIB_WARN(MsgRetryLog((ret == ERR_OK) ? 'n' : 'e',
            type, dest, pri, reply, GET_CPU_ID(), que->getNumMsg(pri), 0, 0x00));
        }
      return ret;
    }

  /* メッセージパケットの送信(タスクコンテキスト、パラメタあり) */
  template<typename T>
  static err_t sendWithTimeout(MsgQueId dest, MsgPri pri, MsgType type, MsgQueId reply, const T& param, size_t ms)
    {
      /* 送信失敗かつタイムアウト時間が指定されていれば、リトライを行う */
      err_t ret = MsgLib::send(dest, pri, type, reply, param);
      if (ret != ERR_OK && ms > 0)
        {
          err_t        err_code;
          MsgQueBlock* que;
          err_code = MsgLib::referMsgQueBlock(dest, &que);
          if (err_code != ERR_OK)
            {
              return err_code;
            }
          DMP_MSGLIB_WARN(MsgRetryLog('b', type, dest, pri, reply,
            GET_CPU_ID(), que->getNumMsg(pri), sizeof(T), reinterpret_cast<const uint32_t&>(param)));
          for (size_t i = 0; ms == TIME_FOREVER || i < ms; i += TickPrecision)
            {
              SleepTask(1);
              ret = MsgLib::send(dest, pri, type, reply, param);
              if (ret == ERR_OK)
                {
                  break;
                }
            }
          DMP_MSGLIB_WARN(MsgRetryLog((ret == ERR_OK) ? 'n' : 'e', type, dest, pri, reply,
            GET_CPU_ID(), que->getNumMsg(pri), sizeof(T), reinterpret_cast<const uint32_t&>(param)));
        }
      return ret;
    }
}; /* class MsgWrapper */

#endif /* MSG_WRAPPER_H_INCLUDED */
