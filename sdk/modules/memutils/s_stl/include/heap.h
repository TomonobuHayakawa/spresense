/**********************************************************************
 *
 *      File Name: heap.h
 *      
 *      Description: Heap (Data Structure)
 *
 *      Notes: (C) Copyright 2002 Sony Corporation
 *
 *      Author: Tomonobu Hayakawa
 *
 **********************************************************************
 */
#ifndef HEAP_H_INCLUDED
#define HEAP_H_INCLUDED

#include "s_stl_config.h"
#include <new>

__STL_BEGIN_NAMESPACE

template<class T, int N> class Heap{

 private:
	int cnt;
	unsigned char buf[sizeof(T)*N];

 public:

	Heap():cnt(0){}

	bool add(const T& new_data){

		T* ptr = reinterpret_cast<T*>(buf);

		if(cnt >= N){
			cnt = N;
			return FALSE;
		}
		for(int i=cnt;i>0;){
			int j=(i-1)/2;
			ptr = reinterpret_cast<T*>(buf+j*sizeof(T));
			if(*ptr > new_data){
				T* child_ptr = reinterpret_cast<T*>(buf+i*sizeof(T));
//				*child_ptr = *ptr;
				child_ptr->~T();
				new(child_ptr) T(*ptr);
			}else{
				ptr = reinterpret_cast<T*>(buf+i*sizeof(T));
				break;
			}
			i=j;
		}
		cnt++;
//		new(ptr) T;
//		*ptr = new_data;
		new(ptr) T(new_data);
		return TRUE;
	}

	const T& top() const {
		if(cnt < 1){ _fatal; }
		T* ptr = reinterpret_cast<T*>(buf);
		return *ptr;
	}

	bool remove(void){
		if(--cnt < 0){
			cnt = 0;
			return FALSE;
		}
		T* ptr = reinterpret_cast<T*>(buf);
		T* last_ptr = reinterpret_cast<T*>(buf+cnt*sizeof(T));

		ptr->~T();

		for(int i=0;i<cnt;){
			int child=i*2+1;
			ptr = (T*)(buf+(i*sizeof(T)));

			if(child > cnt) break;
			T* child_ptr = reinterpret_cast<T*>(buf+(child)*sizeof(T));
				
			if(child+1 < cnt &&  *child_ptr > *(child_ptr+1)){
				child_ptr++;
				child++;
			}

			if(*last_ptr > *child_ptr) {
//				*ptr = *child_ptr;
				new(ptr) T(*child_ptr);
				child_ptr->~T();
				i=child;
			}else{
				break;
			}
		}
//		*ptr=*last_ptr;
		new(ptr) T(*last_ptr);
		last_ptr->~T();
		return TRUE;
	}
	

	int size(void) const { return cnt; }
	int rest(void) const { return (N-cnt); }

	bool full(void) const { return ( cnt == N ); }
	bool empty(void) const { return ( cnt == 0 ); }

	void clear(void){
		for(T* ptr=(T*)buf;ptr < reinterpret_cast<T*>(buf+cnt*sizeof(T));ptr++){
			ptr->~T();
		}
		cnt = 0;
	}
};

__STL_END_NAMESPACE

#endif // HEAP_H_INCLUDED

