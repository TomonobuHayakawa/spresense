/**********************************************************************
 *
 *      File Name: queue.h
 *      
 *      Description: Queue Library for C Language
 *
 *      Notes: (C) Copyright 2005 Sony Corporation
 *
 *      Author: Yasuhi Motoyama
 *
 **********************************************************************
 */
#ifndef QUEUE_H
#define QUEUE_H

#include "buffer.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 *	Queueのデータ構造定義
 *******************************************************************************
 */
typedef struct {
	RingBuffer	data_buf;	/* RingBufferでQueue機能を実現	      */
//	unsigned int	*cnt;		/* 現在登録データ数		      */
} Queue;


/*******************************************************************************
 *	ライブラリプロトタイプ宣言
 *******************************************************************************
 */
extern Queue *Queue_new(unsigned int, unsigned int);
extern int Queue_push(unsigned char *, Queue *);
extern int Queue_pop(Queue *);
extern unsigned char *Queue_top(Queue *);
extern int Queue_empty(Queue *);
extern int Queue_full(Queue *);
extern void Queue_clear(Queue *);

#ifdef __cplusplus
}
#endif


#endif // QUEUE_H

