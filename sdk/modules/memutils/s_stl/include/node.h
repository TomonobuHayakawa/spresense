/**********************************************************************
 *
 *      File Name: node.h
 *      
 *      Description: Node
 *
 *      Notes: (C) Copyright 2002 Sony Corporation
 *
 *      Author: Tomonobu Hayakawa
 *
 **********************************************************************
 */
#ifndef NODE_H_INCLUDED
#define NODE_H_INCLUDED

#include "s_stl_config.h"

__STL_BEGIN_NAMESPACE

/*-------------------------------------------------------------------*/
template<class T> struct Node{
	T      data;
	Node*  left;
	Node*  right;

	Node() : left(NULL_NODE),right(NULL_NODE) {}

        bool operator>(const Node& x) const {return data > x.data;}
  //        bool operator<(const Node& x) const {return data < x.data;}

};

/*-------------------------------------------------------------------*/
template<class T> struct NodePtr{
	Node<T>*  ptr;
        bool operator>(const NodePtr& x) const {return *ptr > *(x.ptr);}
};

__STL_END_NAMESPACE

#endif // NODE_H_INCLUDED
