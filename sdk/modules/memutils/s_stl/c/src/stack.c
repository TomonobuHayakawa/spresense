/**********************************************************************
 *
 *      File Name: stack.c
 *      
 *      Description: Stack Library for C Language
 *
 *      Notes: (C) Copyright 2005 Sony Corporation
 *
 *      Author: Yasuhi Motoyama
 *
 **********************************************************************
 */
#include "s_stl_config.h"
#include "buffer.h"
#include "stack.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 *	Queueのオブジェクト生成
 *******************************************************************************
 */
Stack *Stack_new(unsigned int size, unsigned int depth)
{
	return (Stack *)RingBuffer_new(size, depth);
}


/*******************************************************************************
 *	Stackのpush処理
 *******************************************************************************
 */
int Stack_push(unsigned char *new_data, Stack *this)
{
	return RingBuffer_alloc_front(new_data, &this->data_buf);
}


/*******************************************************************************
 *	Stackのpop処理
 *******************************************************************************
 */
int Stack_pop(Stack *this)
{
	return RingBuffer_free_front(&this->data_buf);
}


/*******************************************************************************
 *	Stackの先頭データ参照
 *******************************************************************************
 */
unsigned char *Stack_top(Stack *this)
{
	return RingBuffer_get_front(&this->data_buf);
}


/*******************************************************************************
 *	Stackのデータ有無確認
 *******************************************************************************
 */
int Stack_empty(Stack *this)
{
	return RingBuffer_empty(&this->data_buf);
}


/*******************************************************************************
 *	Stackのデータfull確認
 *******************************************************************************
 */
int Stack_full(Stack *this)
{
	return RingBuffer_full(&this->data_buf);
}


/*******************************************************************************
 *	Stackのデータクリア
 *******************************************************************************
 */
void Stack_clear(Stack *this)
{
	RingBuffer_clear(&this->data_buf);
}


#ifdef __cplusplus
}
#endif

