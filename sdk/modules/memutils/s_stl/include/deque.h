/**********************************************************************
 *
 *      File Name: deque.h
 *      
 *      Description: Deque
 *
 *      Notes: (C) Copyright 2004 Sony Corporation
 *
 *      Author: Tomonobu Hayakawa
 *
 **********************************************************************
 */

#ifndef DEQUE_H_INCLUDED
#define DEQUE_H_INCLUDED

#include "s_stl_config.h"
#include "buffer.h"

__STL_BEGIN_NAMESPACE

/*-------------------------------------------------------------------*/
template<class T, int N> class Deque{

private:

	RingBuffer<T, N> data_buf;
	int cnt;
public:
	Deque() : cnt(0){}

	bool push_front(const T& new_data){
		if(cnt>=N){ return FALSE; } else {cnt++;}
		return data_buf.alloc_front(new_data);
	}

	bool push_back(const T& new_data){
		if(cnt>=N){ return FALSE; } else {cnt++;}
		return data_buf.alloc_back(new_data);
	}

	bool pop_front(void){
		if(cnt<=0){ return FALSE; } else {cnt--;}
		return data_buf.free_front();
	}

	bool pop_back(void){
		if(cnt<=0){ return FALSE; } else {cnt--;}
		return data_buf.free_back();
	}

	const T& front(void) const {
		return data_buf.get_front();
	}

	const T& back(void) const {
		return data_buf.get_back();
	}

	int size(void) const { return cnt; }
	int rest(void) const { return (N-cnt); }

	bool empty(void) const { return (cnt==0); }
	bool full(void) const { return (cnt==N); }


	const T& at(const int n) const { return at_front(n); }

	const T& at_front(const int n) const {
		return data_buf.at_front(n);
	}

	const T& at_back(const int n) const {
		return data_buf.at_back(n);
	}

	void clear(void){
		cnt = 0;
		data_buf.clear();
	}
};

__STL_END_NAMESPACE
		
#endif // DEQUE_H_INCLUDED

