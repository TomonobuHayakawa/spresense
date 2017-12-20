/***********************************************************************
 *
 *      File Name: cpp_util.h
 *
 *      Description: C++ utility class and templates.
 *
 *      Notes: (C) Copyright 2010 Sony Corporation
 *
 *      Author: Satoru AIZAWA
 *
 ***********************************************************************
 */
#ifndef CPP_UTIL_H_INCLUDED
#define CPP_UTIL_H_INCLUDED
#ifdef __cplusplus

//#include "chateau_assert.h"

////////////////////////////////////////////////////////////
// Copy guard class
//
class CopyGuard {
protected:
	CopyGuard() {}
	~CopyGuard() {}
private:
	CopyGuard(const CopyGuard&);
	const CopyGuard& operator=(const CopyGuard&);
}; // class CopyGuard


////////////////////////////////////////////////////////////
// Range checkable array class
// 範囲チェックを行うには、-DARRAY_ASSERT=CHATEAU_FATAL_ASSERTオプションでビルドする
//
#ifndef ARRAY_ASSERT
#define ARRAY_ASSERT(exp)
#endif

template<typename T, size_t N>
class Array {
public:
	// 配列同様の初期化を可能にするため、publicとする
	T m_elems[N];
public:
	typedef T elem_type_t;

	static size_t capacity() { return N; }

	T& operator[](size_t index) {
		ARRAY_ASSERT(index < N);
		return m_elems[index];
	}

	const T& operator[](size_t index) const {
		ARRAY_ASSERT(index < N);
		return m_elems[index];
	}
}; // class Array

////////////////////////////////////////////////////////////
// Template functions
//
template<typename T>
inline const T& Min(const T& a, const T& b) { return (a < b) ? a : b; }

template<typename T>
inline const T& Max(const T& a, const T& b) { return (a > b) ? a : b; }

#endif /* __cplusplus */
#endif /* CPP_UTIL_H_INCLUDED */
/*
 * $Log: $
 */
