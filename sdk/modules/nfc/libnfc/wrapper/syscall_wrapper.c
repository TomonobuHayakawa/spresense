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
 * Copyright 2014 Sony Corporation
 */

#include <semaphore.h>
#include <termios.h>
#include "time.h"

#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <nuttx/clock.h>

extern int cxd224x_i2c_ResetSig(void);
extern int cxd224x_i2c_WakeupSig(void);

extern int clock_time2ticks(FAR const struct timespec *reltime, FAR int *ticks);

struct pollfd *g_fds; //SPZ2_IMPL TODO check how to share with driver
nfds_t g_nfds;        //SPZ2_IMPL TODO check how to share with driver

/*******************************************************************************
**
** Function         clock
**
** Description      
**
** Parameters:      
**
** Returns          0
**
*******************************************************************************/
clock_t clock(void)
{
  int ret;
  
  int clk;
  struct timespec ts;
 
  ret = clock_gettime(CLOCK_REALTIME, &ts);
  if( ret != OK )
    {
      return (time_t)ERROR;
    } 

  clock_time2ticks(&ts,&clk);
  
  return clk;
}

/*******************************************************************************
**
** Function         times
**
** Description      
**
** Parameters:      
**
** Returns          0
**
*******************************************************************************/
time_t times( void *buf )
{
  return clock();
}

/*******************************************************************************
**
** Function         recv
**
** Description      
**
** Parameters:      
**
** Returns          0
**
*******************************************************************************/
ssize_t recv(int sockfd, void *buf, size_t len, int flags)
{
    /* i2c driver recv */
    cxd224x_i2c_ResetSig();

    return 0;
}

/*******************************************************************************
**
** Function         send
**
** Description      
**
** Parameters:      
**
** Returns          0
**
*******************************************************************************/
ssize_t send(int sockfd, const void* buf, size_t len, int flags)
{
    int i;

    for(i = 0; i < g_nfds; i++)
    {
        if(sockfd == (g_fds[i].fd + 1))
        {
            g_fds[i].revents = POLLIN;
        }
    }

    /* i2c driver send */
    cxd224x_i2c_WakeupSig();

    return 1;
}

/*******************************************************************************
**
** Function         socketpair
**
** Description      
**
** Parameters:      
**
** Returns          0
**
*******************************************************************************/
int socketpair(int domain, int type, int protocol, int sv[2])
{
    sv[0] = -1;
    sv[1] = -2;

    return 0;
}
