/**********************************************************************
 *
 *      File Name: sheap.h
 *      
 *      Description: Skew Heap (Data Structure)
 *
 *      Notes: (C) Copyright 2002 Sony Corporation
 *
 *      Author: Tomonobu Hayakawa
 *
 **********************************************************************
 */
#ifndef SHEAP_H_INCLUDED
#define SHEAP_H_INCLUDED

#include "node.h"

__STL_BEGIN_NAMESPACE

template<class T> class SHeap{

private:
	Node<T>* tree_top;

public:

	SHeap():tree_top(NULL_NODE){}

	bool add(Node<T>* new_node){
		if(empty()) {
			tree_top = new_node;
			return TRUE;
		}
		tree_top = marge(tree_top, new_node);
		return TRUE;
	}

	const T& top(void) const {
		if(empty()) { _fatal; }
		return tree_top->data;
	}

	Node<T>* remove(void){
		if(empty()) { return NULL_NODE; }

		Node<T>* free_node = tree_top;
		tree_top = marge(tree_top->left,tree_top->right);
		return free_node;
	}

	bool empty(void) const {
		return (tree_top == NULL_NODE);
	}

	void clear(void){
		tree_top = NULL_NODE;
	}

private:
	Node<T>* marge(Node<T>* tr1,Node<T>* tr2){
		if(!tr1) return tr2;
		if(!tr2) return tr1;

		if(*tr1 > *tr2)
			return marge(tr2,tr1);

		Node<T>* child = tr1->left;
		if(child){
			tr1->left = marge(tr1->right,tr2);
			tr1->right = child;
		}else{
			tr1->left = tr2;
		}
		return tr1;
	}
};

__STL_END_NAMESPACE

#endif // SHEAP_H_INCLUDED

