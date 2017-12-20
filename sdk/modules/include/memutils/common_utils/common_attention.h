/**********************************************************************
 *
 *      File Name: common_attention.h
 *
 *      Description: Attention Header
 *
 *      Notes: (C) Copyright 2016 Sony Corporation
 *
 *      Author: Takuya OHASHI
 *
 **********************************************************************
 */
#ifndef __COMMON_ATTENTION_H_INCLUDED__
#define __COMMON_ATTENTION_H_INCLUDED__

typedef enum {
	INFORMATION_ATTENTION_CODE = 0, /* Just Information */
	WARNING_ATTENTION_CODE,         /* Warning. 自律復帰可能 */
	ERROR_ATTENTION_CODE,           /* 上位からの制御次第で復帰可能 */
	FATAL_ATTENTION_CODE,           /* 致命的な状態. リセットが必要 */
} ErrorAttensionCode;

#ifdef __cplusplus
extern "C" {
#endif

/** 各プロジェクト毎に下記 IF で実装する
 **/
#ifdef ATTENTION_USE_FILENAME_LINE
extern void _Attention(uint8_t module_id, uint8_t attention_code, uint8_t sub_code,
		       const char* filename, uint32_t line);
#else
extern void _Attention(uint8_t module_id, uint8_t attention_code, uint8_t sub_code);
#endif


#ifdef ATTENTION_USE_FILENAME_LINE
#define INFORMATION_ATTENTION(module_id, sub_code) \
	_Attention((module_id), INFORMATION_ATTENTION_CODE, (sub_code), __FILE__, __LINE__)
#define WARNING_ATTENTION(module_id, sub_code) \
	_Attention((module_id), WARNING_ATTENTION_CODE, (sub_code), __FILE__, __LINE__)
#define ERROR_ATTENTION(module_id, sub_code) \
	_Attention((module_id), ERROR_ATTENTION_CODE, (sub_code), __FILE__, __LINE__)
#define FATAL_ATTENTION(module_id, sub_code) \
	_Attention((module_id), FATAL_ATTENTION_CODE, (sub_code), __FILE__, __LINE__)
#else /* ATTENTION_USE_FILENAME_LINE */
#define INFORMATION_ATTENTION(module_id, sub_code) \
	_Attention((module_id), INFORMATION_ATTENTION_CODE, (sub_code))
#define WARNING_ATTENTION(module_id, sub_code) \
	_Attention((module_id), WARNING_ATTENTION_CODE, (sub_code))
#define ERROR_ATTENTION(module_id, sub_code) \
	_Attention((module_id), ERROR_ATTENTION_CODE, (sub_code))
#define FATAL_ATTENTION(module_id, sub_code) \
	_Attention((module_id), FATAL_ATTENTION_CODE, (sub_code))
#endif /* ATTENTION_USE_FILENAME_LINE */

#ifdef __cplusplus
}
#endif

#endif /* __COMMON_ATTENTION_H_INCLUDED__ */
