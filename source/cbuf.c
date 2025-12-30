/*
 * cbuf.c
 *
 *  Created on: Dec 20, 2025
 *      Author: chris
 */


#include "cbuf.h"
#include "sysdef.h"


cbuf_t* cbuf_init(cbuf_t* cbuf)
{
	if (cbuf == NULL)
	{
		return NULL;
	}

	cbuf->head = 0U;
	cbuf->tail = 0U;
	cbuf->count = 0U;

	return cbuf;
}

void    cbuf_put(cbuf_t* cbuf, u32 data)
{
	if (cbuf == NULL)
	{
		return;
	}

	cbuf->buf[cbuf->head] = data;
	cbuf->head = (cbuf->head + 1) % MAX_BUF_SIZE;

	if (cbuf->count == MAX_BUF_SIZE)
	{
		cbuf->tail = (cbuf->tail + 1) % MAX_BUF_SIZE;
	}
	else
	{
		cbuf->count++;
	}
}

u8      cbuf_get(cbuf_t* cbuf, u32* data)
{
	if (data == NULL || cbuf == NULL || cbuf->count == 0)
	{
		return FALSE;
	}

	*data = cbuf->buf[cbuf->tail];
	cbuf->tail = (cbuf->tail + 1) % MAX_BUF_SIZE;

	cbuf->count--;

	return TRUE;

}

u8      cbuf_peek(cbuf_t* cbuf, u32* data)
{
	if (data == NULL || cbuf == NULL || cbuf->count == 0)
	{
		return FALSE;
	}


	*data = cbuf->buf[cbuf->tail];

	return TRUE;
}

u8      cbuf_isfull(cbuf_t* cbuf)
{
	return (cbuf != NULL && cbuf->count == MAX_BUF_SIZE);
}

