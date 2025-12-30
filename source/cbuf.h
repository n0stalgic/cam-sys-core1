/*
 * cbuf.h
 *
 *  Created on: Dec 20, 2025
 *      Author: chris
 */

#ifndef CBUF_H_
#define CBUF_H_

#include "sysdef.h"

#define MAX_BUF_SIZE 128

typedef struct
{
	u8 buf[MAX_BUF_SIZE];
	u32 head;
	u32 tail;
	u32 count;

} cbuf_t;


cbuf_t* cbuf_init(cbuf_t* cbuf);
void    cbuf_put(cbuf_t* cbuf, u32 data);
u8      cbuf_get(cbuf_t* cbuf, u32* data);
u8      cbuf_peek(cbuf_t* cbuf, u32* data);
u8      cbuf_isfull(cbuf_t* cbuf);

#endif /* CBUF_H_ */
