/**********************************************************************
 *
 *      File Name: stack.h
 *      
 *      Description: Stack
 *
 *      Notes: (C) Copyright 2002 Sony Corporation
 *
 *      Author: Tomonobu Hayakawa
 *
 **********************************************************************
 */

#ifndef STACK_H_INCLUDED
#define STACK_H_INCLUDED

#include "s_stl_config.h"
#include "buffer.h"

__STL_BEGIN_NAMESPACE

template<class T, int N> class Stack{

private:

	RingBuffer<T, N> data_buf;
	int cnt;
public:
	Stack() : cnt(0){}

	bool push(const T& new_data){
		if(cnt>=N){ return FALSE; } else {cnt++;}
		data_buf.alloc_front(new_data);
		return TRUE;
	}

	bool pop(void){
		if(cnt<=0){ return FALSE; } else {cnt--;}
		return data_buf.free_front();
	}
	const T& top(void) const {
		return data_buf.get_front();
	}

	int size(void) const { return cnt; }
	int rest(void) const { return (N-cnt); }

	bool empty(void) const { return (cnt==0); }
	bool full(void) const { return (cnt==N); }

	void clear(void){
		cnt=0;
		data_buf.clear();
	}
};
		
__STL_END_NAMESPACE

#endif // STACK_H_INCLUDED

