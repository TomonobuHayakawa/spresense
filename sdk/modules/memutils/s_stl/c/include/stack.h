/**********************************************************************
 *
 *      File Name: stack.h
 *      
 *      Description: Stack Library for C Language
 *
 *      Notes: (C) Copyright 2005 Sony Corporation
 *
 *      Author: Yasuhi Motoyama
 *
 **********************************************************************
 */
#ifndef STACK_H
#define STACK_H

#include "buffer.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 *	stackのデータ構造定義
 *******************************************************************************
 */
typedef struct {
	RingBuffer	data_buf;	/* RingBufferでstack機能を実現	      */
//	unsigned int	*cnt;		/* 現在登録データ数		      */
} Stack;


/*******************************************************************************
 *	ライブラリプロトタイプ宣言
 *******************************************************************************
 */
extern Stack *Stack_new(unsigned int, unsigned int);
extern int Stack_push(unsigned char *, Stack *);
extern int Stack_pop(Stack *);
extern unsigned char *Stack_top(Stack *);
extern int Stack_empty(Stack *);
extern int Stack_full(Stack *);
extern void Stack_clear(Stack *);

#ifdef __cplusplus
}
#endif


#endif // STACK_H

