/**********************************************************************
 *
 *      File Name: pqueue.h
 *      
 *      Description: Priority Queue
 *
 *      Notes: (C) Copyright 2002 Sony Corporation
 *
 *      Author: Tomonobu Hayakawa
 *
 **********************************************************************
 */

#ifndef PQUEUE_H_INCLUDED
#define PQUEUE_H_INCLUDED

#include "s_stl_config.h"
#include "node.h"
#include "buffer.h"

#include "list.h"
#include "heap.h"
#include "sheap.h"

__STL_BEGIN_NAMESPACE

template<class T, int N> class PQueue{

private:

#if PQUEUE_STR == LIST
        OneWayList<T>		pqueue;
        Buffer<Node<T>, N>	data_buf;
	int 			cnt;
#elif PQUEUE_STR == HEAP
	Heap<T, N>		heap;
#elif PQUEUE_STR == SHEAP
	SHeap<T>		pqueue;
        Buffer<Node<T>, N>	data_buf;
	int 			cnt;
#elif PQUEUE_STR == PHEAP
	Heap<NodePtr<T>, N>	heap;
	Buffer<Node<T>, N>	data_buf;
#endif

public:

#if PQUEUE_STR == PHEAP

	PQueue(){}

	bool push(const T& new_data){

		NodePtr<T> new_node;

		if( heap.full() ) return FALSE;
		new_node.ptr = data_buf.alloc();
//		if(new_node.ptr == NULL_NODE) return FALSE;

		(new_node.ptr)->right = NULL_NODE;
		(new_node.ptr)->left  = NULL_NODE;
//		(new_node.ptr)->data  = new_data;
		new(&((new_node.ptr)->data)) T(new_data);

		return heap.add(new_node);
	}

	bool pop(void){
		if( heap.empty() ) return FALSE;
		data_buf.free(heap.top().ptr);
		return heap.remove();
	}

	const T& top(void) const {
		NodePtr<T> top_ptr = heap.top();
		return  (top_ptr.ptr)->data;
	}

	int size(void) const { return cnt; }
	int rest(void) const { return (N-cnt); }

	bool full(void) const { return heap.full(); }
	bool empty(void) const { return heap.empty(); }

	void clear(void){
		data_buf.clear();
		heap.clear();
	}

#elif PQUEUE_STR == HEAP

	PQueue(){}

	bool push(const T& new_data){ return heap.add(new_data); }
	const T& top(void) const { return heap.top(); }
	bool pop(void){ return heap.remove(); }
	int size(void) const { return heap.size(); }
	int rest(void) const { return heap.rest(); }
	bool full(void) const { return heap.full(); }
	bool empty(void) const { return heap.empty(); }
	void clear(void){ heap.clear(); return; }

#else

	PQueue() : cnt(0){}

	bool push(const T& new_data){

		if(cnt>=N){ return FALSE; } else { cnt++; }
		Node<T>* new_node = data_buf.alloc();
//		if(new_node == NULL_NODE) return FALSE;

		new_node->right = NULL_NODE;
		new_node->left  = NULL_NODE;
//		new_node->data  = new_data;
		new(&(new_node->data)) T(new_data);

		return pqueue.add(new_node);
	}

	const T& top(void) const {
		if(!empty()) return pqueue.top();
		else return reinterpret_cast<const T&>(data_buf);
	}

	bool pop(void){

		if(cnt<=0){ return FALSE; } else { cnt--; }
		Node<T>* free_area = pqueue.remove();
//		if(free_area == NULL_NODE) return FALSE;

		data_buf.free(free_area);
		return TRUE;
	}

	int size(void) const { return cnt; }
	int rest(void) const { return (N-cnt); }

	bool empty(void) const { return (cnt==0); }
	bool full(void) const { return (cnt==N); }

	void clear(void){
		data_buf.clear();
		pqueue.clear();
		cnt = 0;
	}
#endif

};

__STL_END_NAMESPACE

#endif // PQUEUE_H_INCLUDED

