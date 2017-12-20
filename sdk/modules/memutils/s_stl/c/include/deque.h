/**********************************************************************
 *
 *      File Name: deque.h
 *      
 *      Description: Deque Library for C Language
 *
 *      Notes: (C) Copyright 2005 Sony Corporation
 *
 *      Author: Yasuhi Motoyama
 *
 **********************************************************************
 */
#ifndef DEQUE_H
#define DEQUE_H

#include "buffer.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 *	Dequeのデータ構造定義
 *******************************************************************************
 */
typedef struct {
	RingBuffer	data_buf;	/* RingBufferでDeque機能を実現	      */
//	unsigned int	*cnt;		/* 現在登録データ数		      */
} Deque;


/*******************************************************************************
 *	ライブラリプロトタイプ宣言
 *******************************************************************************
 */
extern Deque *Deque_new(unsigned int, unsigned int);
extern int Deque_push_front(unsigned char *, Deque *);
extern int Deque_push_back(unsigned char *, Deque *);
extern int Deque_pop_front(Deque *);
extern int Deque_pop_back(Deque *);
extern unsigned char *Deque_front(Deque *);
extern unsigned char *Deque_back(Deque *);
extern int Deque_empty(Deque *);
extern int Deque_full(Deque *);
extern void Deque_clear(Deque *);

#ifdef __cplusplus
}
#endif


#endif // DEQUE_H

