/**********************************************************************
 *
 *      File Name: buffer.h
 *      
 *      Description: Memory IO Library
 *
 *      Notes: (C) Copyright 2004 Sony Corporation
 *
 *      Author: Tomonobu Hayakawa
 *
 **********************************************************************
 */
#ifndef BUFFER_H_INCLUDED
#define BUFFER_H_INCLUDED

#include "memutils/s_stl/s_stl_config.h"
#include <new>

__STL_BEGIN_NAMESPACE

/*-------------------------------------------------------------------*/
template<class T,int N> class Buffer{

private:

	struct SEG {
		unsigned char data[sizeof(T)];
		uint index;

		SEG() : index(EMPTY){}
	};

	SEG  buffer[N];
	SEG* search_top;

public:
	Buffer(): search_top(buffer) {}

	T* alloc(void){
		for(SEG* area_p = search_top;area_p<&buffer[N];area_p++){
			if(area_p->index == EMPTY){
				area_p->index = FULL;
				search_top = area_p+1;
//				new (area_p->data) T;
				return reinterpret_cast<T*>(&(area_p->data));
			}
		}
		return NULL;
	}

	bool free(T* data_p){
		SEG* area_p = reinterpret_cast<SEG*>(data_p);
		if((area_p < &buffer[0]) || (area_p >= &buffer[N])) return FALSE;
		if(area_p->index == EMPTY) return FALSE;
		data_p->~T();
		area_p->index = EMPTY;
		if(search_top > area_p){ search_top = area_p;}
		return TRUE;
	}

	void clear(void){
		for(int i=0;i<N;i++){
			T* area_p = reinterpret_cast<T*>(buffer[i].data);
			if(buffer[i].index == FULL) area_p->~T();
			buffer[i].index = EMPTY;
		}
		search_top = buffer;
	}
};

/*-------------------------------------------------------------------*/
template<class T,int N> class RingBuffer{

private:

	unsigned char buffer[sizeof(T)*N];
	T* head_p;
	T* tail_p;

	int cnt;

public:
	RingBuffer(): head_p(static_cast<T*>(static_cast<void*>(buffer))), tail_p(head_p), cnt(0){}

	bool alloc_front(const T& new_data){
		if(cnt>=N){ return FALSE; } else { cnt++; }
		if(cnt != 1){
			if(head_p+1 > static_cast<T*>(static_cast<void*>(&buffer[sizeof(T)*(N-1)])) ){
				head_p = static_cast<T*>(static_cast<void*>(&buffer[0]));
			}else{
				head_p++;
			}
		}
		new(head_p) T(new_data);
		return TRUE;
	}

	bool alloc_back(const T& new_data){
		if(cnt>=N){ return FALSE; } else { cnt++; }
		if(cnt != 1){
			if(tail_p-1 < static_cast<T*>(static_cast<void*>(&buffer[0]))){
				tail_p = static_cast<T*>(static_cast<void*>(&buffer[sizeof(T)*(N-1)]));
			}else{
				tail_p--;
			}
		}
		new(tail_p) T(new_data);
		return TRUE;
	}

	bool free_front(void){
		if(cnt<=0){ return FALSE; } else { cnt--; }
		head_p->~T();
		if(cnt != 0){
			if(head_p-1 < static_cast<T*>(static_cast<void*>(&buffer[0]))){
				head_p = static_cast<T*>(static_cast<void*>(&buffer[sizeof(T)*(N-1)]));
			}else{
				head_p--;
			}
		}
		return TRUE;
	}

	bool free_back(void){
		if(cnt<=0){ return FALSE; } else {cnt--;}
		tail_p->~T();
		if(cnt != 0){
			if(tail_p+1 > static_cast<T*>(static_cast<void*>(&buffer[sizeof(T)*(N-1)]))){
				tail_p = static_cast<T*>(static_cast<void*>(&buffer[0]));
			}else{
				tail_p++;
			}
		}
		return TRUE;
	}

	const T& get_front(void) const {
		if(cnt <= 0) _fatal;
		return *head_p;
	}

	const T& get_back(void) const {
		if(cnt <= 0) _fatal;
		return *tail_p;
	}


	const T& at_front(const int n) const {
		if(cnt <= n) _fatal;
		if(head_p-n+N > reinterpret_cast<const T*>(&buffer[sizeof(T)*(N-1)])){
			return *(head_p-n);
		}else{
			return *(head_p-n+N);
		}
	}

	const T& at_back(const int n) const {
		if(cnt <= n) _fatal;
		if(tail_p+n > reinterpret_cast<const T*>(&buffer[sizeof(T)*(N-1)])){
			return *(tail_p+n-N);
		}else{
			return *(tail_p+n);
		}
	}


	bool empty(void) const { return (cnt==0); }
	bool full(void) const { return (cnt==N); }

	void clear(void){
		while(cnt>0){
			free_back();
		}
	}
};

__STL_END_NAMESPACE

#endif /* BUFFER_H_INCLUDED */

