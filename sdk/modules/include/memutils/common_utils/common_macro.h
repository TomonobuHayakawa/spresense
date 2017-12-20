/***********************************************************************
 *
 *      File Name: common_macro.h
 *
 *      Description: Common macros definitions.
 *
 *      Notes: (C) Copyright 2010 Sony Corporation
 *
 *      Author: Satoru AIZAWA
 *
 * $Id: $
 ***********************************************************************
 */
#ifndef COMMON_MACRO_H_INCLUDED
#define COMMON_MACRO_H_INCLUDED

#include <stddef.h>	/* size_t, offsetof */

#ifndef __cplusplus
#ifndef __bool_true_false_are_defined
#define __bool_true_false_are_defined
typedef enum { false, true } bool;
#endif /* __bool_true_false_are_defined */
#endif /* __cplusplus */

#define ALIGN_OF(type) offsetof(struct { char x; type y; }, y)
#define COUNT_OF(array) (sizeof(array) / sizeof(array[0]))

#define ROUND_DOWN(n, power2) ((n) & ~((power2) - 1))
#define ROUND_UP(n, power2) ROUND_DOWN(((n) + ((power2) - 1)), power2)

#define JOIN_MACRO(x, y) JOIN_TOKEN(x, y)
#define JOIN_TOKEN(x, y) x ## y

#if defined(__CC_ARM)
#define INLINE __inline
#else
#define INLINE inline
#endif

#endif /* COMMON_MACRO_H_INCLUDED */
/*
 * $Log: $
 */
