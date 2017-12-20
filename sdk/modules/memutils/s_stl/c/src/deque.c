/**********************************************************************
 *
 *      File Name: deque.c
 *      
 *      Description: Deque Library for C Language
 *
 *      Notes: (C) Copyright 2005 Sony Corporation
 *
 *      Author: Yasuhi Motoyama
 *
 **********************************************************************
 */
#include "s_stl_config.h"
#include "buffer.h"
#include "deque.h"

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 *	Dequeのオブジェクト生成
 *******************************************************************************
 */
Deque *Deque_new(unsigned int size, unsigned int depth)
{
	return (Deque *)RingBuffer_new(size, depth);
}


/*******************************************************************************
 *	Dequeのフロントへのpush処理
 *******************************************************************************
 */
int Deque_push_front(unsigned char *new_data, Deque *this)
{
	return RingBuffer_alloc_front(new_data, &this->data_buf);
}


/*******************************************************************************
 *	Dequeのバックへのpush処理
 *******************************************************************************
 */
int Deque_push_back(unsigned char *new_data, Deque *this)
{
	return RingBuffer_alloc_back(new_data, &this->data_buf);
}


/*******************************************************************************
 *	Dequeのフロントデータのpop処理
 *******************************************************************************
 */
int Deque_pop_front(Deque *this)
{
	return RingBuffer_free_front(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのバックデータのpop処理
 *******************************************************************************
 */
int Deque_pop_back(Deque *this)
{
	return RingBuffer_free_back(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのフロントデータの参照
 *******************************************************************************
 */
unsigned char *Deque_front(Deque *this)
{
	return RingBuffer_get_front(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのバックデータの参照
 *******************************************************************************
 */
unsigned char *Deque_back(Deque *this)
{
	return RingBuffer_get_back(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのデータ有無確認
 *******************************************************************************
 */
int Deque_empty(Deque *this)
{
	return RingBuffer_empty(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのデータfull確認
 *******************************************************************************
 */
int Deque_full(Deque *this)
{
	return RingBuffer_full(&this->data_buf);
}


/*******************************************************************************
 *	Dequeのデータクリア
 *******************************************************************************
 */
void Deque_clear(Deque *this)
{
	RingBuffer_clear(&this->data_buf);
}


#ifdef __cplusplus
}
#endif

