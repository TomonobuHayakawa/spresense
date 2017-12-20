/**********************************************************************
 *
 *      File Name: m_stack.h
 *      
 *      Description: Stack
 *
 *      Notes: (C) Copyright 2002 Sony Corporation
 *
 *      Author: Tomonobu Hayakawa
 *
 **********************************************************************
 */

#ifndef M_STACK_H_INCLUDED
#define M_STACK_H_INCLUDED

#include "s_stl_config.h"
#include "buffer.h"

__USING_S_STL;

__M_STL_BEGIN_NAMESPACE

template<class T, int N> class Stack{

private:

	RingBuffer<T, N> data_buf;
	int cnt;
	ID  mtx,sem;

public:
	Stack() : cnt(0){
		CREATE_MTX(mtx,0);
		if(mtx < E_OK) _fatal;
		CREATE_SEM(sem,N);
		if(sem < E_OK){ _fatal; }
	}

	bool push(const T& new_data,TMO tim_out){
		ER err = TAKE_SEM(sem,tim_out);
		if(err == E_TMOUT){	return FALSE; }
		if(err != E_OK){ _fatal; }
		if(LOCK_MTX(mtx)){ _fatal; }
		if(cnt>=N){
			if(GIVE_MTX(mtx)){ _fatal; }
			return FALSE;
		}
		cnt++;
		bool rst=data_buf.alloc_front(new_data);
		if(GIVE_MTX(mtx)){ _fatal; }
		return rst;
	}

	bool pop(void){
		if(LOCK_MTX(mtx)){ _fatal; }
		if(cnt<=0){
			if(GIVE_MTX(mtx)){ _fatal; }
			return FALSE;
		} else {cnt--;}
		bool rst=data_buf.free_front();
		if(GIVE_MTX(mtx)){ _fatal; }
		if(GIVE_SEM(sem)){ _fatal; }
		return rst;
	}

	bool tpop(T* data){
		if(LOCK_MTX(mtx)){ _fatal; }
		if(cnt<=0){
			if(GIVE_MTX(mtx)){ _fatal; }
			return FALSE;
		} else { cnt--; }
		*data = top();
		bool rst = data_buf.free_front();
		if(GIVE_MTX(mtx)){ _fatal; }
		if(GIVE_SEM(sem)){ _fatal; }
		return rst;
	}

	const T& top(void) const {
		return data_buf.get_front();
	}

	bool empty(void) const { return (cnt==0); }
	bool full(void) const { return (cnt==N); }

	void clear(void){
		cnt=0;
		data_buf.clear();
	}
};
		
__M_STL_END_NAMESPACE

#endif // M_STACK_H_INCLUDED

