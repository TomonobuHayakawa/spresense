/*
 * Sony Advanced Instrument of Libraries
 *
 * This program is subject to copyright protection in accordance with the
 * applicable law. It must not, except where allowed by law, by any means or
 * in any form be reproduced, distributed or lent. Moreover, no part of the
 * program may be used, viewed, printed, disassembled or otherwise interfered
 * with in any form, except where allowed by law, without the express written
 * consent of the copyright holder.
 *
 * Copyright 2013,2014 Sony Corporation
 */

#include <time.h>
#include "nf_common.h"

#define NFC_TIMER_SIGNAL (SIGUSR1)

timer_t g_timerid;

/*******************************************************************************
**
** Function         nf_timeout_handler
**
** Description      This function is handler called timeout. control a PON pin low.
**
** Parameters:      info - (input) AlermHandler information structure.
**
** Returns          none
**
*******************************************************************************/

void nf_timeout_handler(int signo, siginfo_t *info, void *ucontext)
{
    struct itimerspec timer;
    int ret;

    DBG_LOGF_DEBUG("nf_timeout_handler() start.\n");

    pthread_mutex_lock(&_mutex_i2c);

    /* if(timer_status.status != pdFALSE) */
    /* { */
    /*     /\* timer is active, do nothing *\/ */
    /* } */
    /* else */

    if( signo == NFC_TIMER_SIGNAL )
    {
        /* timer is not active */
        nfc_i2c_ctrl_io(I2C_IO_WAKE_CTL, I2C_IO_HI);

        if(wait_rsp_flg == true)
        {
            wait_rsp_flg = false;
            err_ntf_flg = true;
            sem_post(&sem_cb_wait);
            sem_post(&sem_recv);
        }
    }

    timer.it_value.tv_sec = 0;
    timer.it_value.tv_nsec = 0;
    ret = timer_settime(g_timerid, 0, &timer, NULL);
    if (ret != OK)
    {
      DBG_LOGF_ERROR("timer_settime failed. %d\n", ret);
    }

    pthread_mutex_unlock(&_mutex_i2c);

    DBG_LOGF_DEBUG("nf_timeut_handler() end.\n");
}

/*******************************************************************************
**
** Function         nf_create_timer
**
** Description      This function is create timer.
**
** Parameters:      none
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
int nf_create_timer(void)
{
  sigset_t mask;
  struct sigaction sa;
  struct sigevent sev;
  int ret;

  DBG_LOGF_DEBUG("nf_create_timer() start.\n");

  sigemptyset(&mask);
  sigaddset(&mask, NFC_TIMER_SIGNAL);
  ret = sigprocmask(SIG_UNBLOCK, &mask, NULL);
  if (ret != OK)
    {
      DBG_LOGF_ERROR("sigprocmask failed. %d\n", ret);
      return E_FAIL;
    }

  sa.sa_sigaction = nf_timeout_handler;
  sa.sa_flags = SA_SIGINFO;
  sigfillset(&sa.sa_mask);
  sigdelset(&sa.sa_mask, NFC_TIMER_SIGNAL);
  ret = sigaction(NFC_TIMER_SIGNAL, &sa, NULL);
  if (ret != OK)
    {
      DBG_LOGF_ERROR("sigaction failed. %d\n", ret);
      return E_FAIL;
    }

  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = NFC_TIMER_SIGNAL;
  sev.sigev_value.sival_int = 200; //Any value
  sev.sigev_value.sival_ptr = NULL;
  ret = timer_create(CLOCK_REALTIME, &sev, &g_timerid);
  if (ret != OK)
    {
      DBG_LOGF_ERROR("timer_create failed. %d\n", ret);
      return E_FAIL;
    }

  DBG_LOGF_DEBUG("nf_create_timer() end.\n");
  
  return(E_OK);
}

/*******************************************************************************
**
** Function         nf_start_timer
**
** Description      This function is start timer.
**
** Parameters:      none
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
int nf_start_timer(void)
{
  int ret;
  struct itimerspec timer;

  DBG_LOGF_DEBUG("nf_start_timer() start.\n");
    
  int first_ms = 200;
  timer.it_value.tv_sec = first_ms / 1000;
  timer.it_value.tv_nsec = (first_ms % 1000) * 1000 * 1000;

  ret = timer_settime(g_timerid, 0, &timer, NULL);
  if (ret != OK)
    {
      DBG_LOGF_ERROR("timer_settime failed. %d\n", ret);
      return E_FAIL;
    }

  DBG_LOGF_DEBUG("nf_start_timer() end.\n");

  return(E_OK);
}

/*******************************************************************************
**
** Function         nf_stop_timer
**
** Description      This function is stop timer.
**
** Parameters:      void
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
int nf_stop_timer(void)
{
      sigset_t mask;

    DBG_LOGF_DEBUG("nf_stop_timer() start.\n");


    timer_delete(g_timerid);

    sigfillset(&mask);
    sigprocmask(SIG_SETMASK, &mask, NULL);

    DBG_LOGF_DEBUG("nf_stop_timer() end.\n");

    return(E_OK);
}

/*******************************************************************************
**
** Function         nf_delete_timer
**
** Description      This function is delete timer.
**
** Parameters:      void
**
** Returns          E_OK
**                  E_FAIL
**
*******************************************************************************/
int nf_delete_timer(void)
{
//@@@    int ret;

    DBG_LOGF_DEBUG("nf_delete_timer() start.\n");

    /* ret = SYS_DeleteAlarmHandler(desc); */
    /* if(ret != 0) */
    /* { */
    /*     DBG_LOGF_ERROR("SYS_DeleteAlarmHandler() failed ret=%d.\n", ret); */
    /*     return(E_FAIL); */
    /* } */

    DBG_LOGF_DEBUG("nf_delete_timer() end.\n");

    return(E_OK);
}

