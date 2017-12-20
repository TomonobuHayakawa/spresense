/**********************************************************************
 *
 *      File Name: buffer.h
 *      
 *      Description: RingBuffer Library for C Language
 *
 *      Notes: (C) Copyright 2005 Sony Corporation
 *
 *      Author: Yasuhi Motoyama
 *
 **********************************************************************
 */
#ifndef BUFFER_H
#define BUFFER_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 *	RingBufferのデータ構造定義
 *******************************************************************************
 */
typedef struct {
	unsigned char	*buffer;	/* データ格納領域先頭		      */
	unsigned char	*head_p;	/* データ格納領域書込み可能先頭位置   */
	unsigned char	*tail_p;	/* データ格納領域書込み可能最後尾位置 */
	unsigned int	size;		/* データ型サイズ		      */
	unsigned int	depth;		/* データ格納数			      */
	unsigned int	cnt;		/* 現在登録データ数		      */
} RingBuffer;


/*******************************************************************************
 *	ライブラリプロトタイプ宣言
 *******************************************************************************
 */
extern RingBuffer *RingBuffer_new(unsigned int, unsigned int);
extern int RingBuffer_alloc_front(unsigned char *, RingBuffer *);
extern int RingBuffer_alloc_back(unsigned char *, RingBuffer *);
extern int RingBuffer_free_front(RingBuffer *);
extern int RingBuffer_free_back(RingBuffer *);
extern unsigned char *RingBuffer_get_front(RingBuffer *);
extern unsigned char *RingBuffer_get_back(RingBuffer *);
extern int RingBuffer_empty(RingBuffer *);
extern int RingBuffer_full(RingBuffer *);
extern void RingBuffer_clear(RingBuffer *);

#ifdef __cplusplus
}
#endif


#endif // BUFFER_H

