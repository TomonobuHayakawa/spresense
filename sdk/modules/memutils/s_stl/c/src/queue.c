/**********************************************************************
 *
 *      File Name: queue.c
 *      
 *      Description: Queue Library for C Language
 *
 *      Notes: (C) Copyright 2005 Sony Corporation
 *
 *      Author: Yasuhi Motoyama
 *
 **********************************************************************
 */
#include "s_stl_config.h"
#include "buffer.h"
#include "queue.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 *	Queueのオブジェクト生成
 *******************************************************************************
 */
Queue *Queue_new(unsigned int size, unsigned int depth)
{
	return (Queue *)RingBuffer_new(size, depth);
}


/*******************************************************************************
 *	Queueのpush処理
 *******************************************************************************
 */
int Queue_push(unsigned char *new_data, Queue *this)
{
	return RingBuffer_alloc_front(new_data, &this->data_buf);
}


/*******************************************************************************
 *	Queueのpop処理
 *******************************************************************************
 */
int Queue_pop(Queue *this)
{
	return RingBuffer_free_back(&this->data_buf);
}


/*******************************************************************************
 *	Queueの先頭データ参照
 *******************************************************************************
 */
unsigned char *Queue_top(Queue *this)
{
	return RingBuffer_get_back(&this->data_buf);
}


/*******************************************************************************
 *	Queueのデータ有無確認
 *******************************************************************************
 */
int Queue_empty(Queue *this)
{
	return RingBuffer_empty(&this->data_buf);
}


/*******************************************************************************
 *	Queueのデータfull確認
 *******************************************************************************
 */
int Queue_full(Queue *this)
{
	return RingBuffer_full(&this->data_buf);
}


/*******************************************************************************
 *	Queueのデータクリア
 *******************************************************************************
 */
void Queue_clear(Queue *this)
{
	RingBuffer_clear(&this->data_buf);
}


#ifdef __cplusplus
}
#endif

