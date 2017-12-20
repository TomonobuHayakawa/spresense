/**********************************************************************
 *
 *      File Name: list.h
 *      
 *      Description: List (Data Structure)
 *
 *      Notes: (C) Copyright 2002 Sony Corporation
 *
 *      Author: Tomonobu Hayakawa
 *
 **********************************************************************
 */
#ifndef LIST_H_INCLUDED
#define LIST_H_INCLUDED

#include "s_stl_config.h"
#include "node.h"

__STL_BEGIN_NAMESPACE

/*-------------------------------------------------------------------*/
template<class T> class OneWayList {

private:

	Node<T>* list_top;

public:
	OneWayList() : list_top(NULL_NODE){}

	bool add(Node<T>* new_node)
	{
		Node<T>* list_p;

		if(empty()) {
			list_top = new_node;
			list_top->right = NULL_NODE;
			return TRUE;
		} else {
			if(*list_top > *new_node){
				new_node->right = list_top;
				list_top = new_node;
				return TRUE;
			}
			for(list_p = list_top;list_p->right != NULL_NODE;
			    list_p = list_p->right){
				if((*list_p->right) > *new_node){
					new_node->right = list_p->right;
					list_p->right = new_node;
					return TRUE;
				}
			}
		}
		list_p->right =  new_node;
		new_node->right = NULL_NODE;
		return TRUE;
	}

	Node<T>* remove(void){
		if(empty()) {
			_fatal;
			return NULL_NODE;
		}
		Node<T>* free_node = list_top;
		list_top = list_top->right;
		return free_node;
	}

	const T& top(void) const {
		if(empty()) { _fatal; }
		return list_top->data;
	}

	void clear(void){ list_top = NULL_NODE; }

	bool empty(void) const { return (list_top == NULL_NODE); }

};

__STL_END_NAMESPACE

#endif // LIST_H_INCLUDED

