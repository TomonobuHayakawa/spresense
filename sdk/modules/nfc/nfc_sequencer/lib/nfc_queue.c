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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nf_common.h"

QUEUE *que;

/*******************************************************************************
**
** Function         create_list
**
** Description      This function is
**
** Parameters:
**
** Returns
**
*******************************************************************************/
BUF_LIST *create_list(unsigned char *in_dat, BUF_LIST *next)
{
    BUF_LIST *list;

    if((list = malloc(sizeof(BUF_LIST))) != NULL)
    {
        memcpy(list->item, in_dat, 255);
        list->next = next;
    }
    return list;
}

/*******************************************************************************
**
** Function         create_queue
**
** Description      This function is
**
** Parameters:
**
** Returns
**
*******************************************************************************/
void create_queue(void)
{
    if((que = malloc(sizeof(QUEUE))) != NULL)
    {
        que->front = NULL;
        que->rear  = NULL;
    }

    return;
}

/*******************************************************************************
**
** Function         delete_list
**
** Description      This function is
**
** Parameters:
**
** Returns
**
*******************************************************************************/
void delete_list(BUF_LIST *list)
{
    BUF_LIST *temp;

    while (list != NULL)
    {
      temp = list->next;
      free(list);
      list = temp;
    }

    return;
}

/*******************************************************************************
**
** Function         delete_queue
**
** Description      This function is
**
** Parameters:
**
** Returns
**
*******************************************************************************/
void delete_queue(void)
{
    delete_list(que->front);
    free(que);

    return;
}

/*******************************************************************************
**
** Function         is_empty
**
** Description      This function is
**
** Parameters:
**
** Returns
**
*******************************************************************************/
bool is_empty(void)
{
    return ((que->front == NULL)? true: false);
}

/*******************************************************************************
**
** Function         enqueue
**
** Description      This function is
**
** Parameters:
**
** Returns
**
*******************************************************************************/
bool enqueue(unsigned char *in_dat)
{
    BUF_LIST *list;

    if((list = create_list(in_dat, NULL)) != NULL)
    {
        if(is_empty()) 
            que->front = que->rear = list;
        else
        {
            que->rear->next = list;
            que->rear = list;
        }
        return true;
    }
    return false;
}

/*******************************************************************************
**
** Function         dequeue
**
** Description      This function is
**
** Parameters:
**
** Returns
**
*******************************************************************************/
unsigned char *dequeue(unsigned char *out_dat, bool *err)
{
    BUF_LIST *list;

    if(is_empty())
    {
        *err = false;
        return 0;
    }

    list = que->front;
    memcpy(out_dat, list->item, 255);
    *err = true;
    que->front = list->next;
    free(list);

    if(que->front == NULL)
        que->rear = NULL;

    return out_dat;
}

