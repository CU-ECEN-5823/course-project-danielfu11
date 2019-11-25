/**
 * @file 	circbuf.c
 * @author 	Guanxiong Fu
 * @date	Nov 23 2019
 * @brief	Implementation of a circular buffer
 *
 */

#include "circbuf.h"

/**
 * @brief  Initialize circular buffer
 * @param  buf       Pointer to circbuf struct
 *         capacity  Size of the buffer
 * @return CB_SUCCESS or CB_ERROR
 *
 */
cb_status_e cb_initialize_buffer(circbuf_s * buf, uint32_t capacity)
{
	// Handle invalid inputs
	if (!buf || capacity <= 0)
	{
		return CB_ERROR;
	}
	buf->base = (item_s *)malloc(sizeof(item_s) * capacity);
	buf->capacity = capacity;
	buf->size = 0;
	buf->head = buf->base;
	buf->tail = buf->base;
	// Initialize all items to NULL
	int i;
	item_s * temp = buf->base;
	for (i=0; i < capacity; ++i)
	{
		temp->item = NULL;
		++temp;
	}
	return CB_SUCCESS;
}


/**
 * @brief  Free allocated memory for circular buffer
 * @param  buf Pointer to circbuf struct
 * @return CB_SUCCESS or CB_ERROR
 *
 */
cb_status_e cb_delete_buffer(circbuf_s * buf)
{
	if (!buf || !buf->head || !buf->tail || !buf->base)
	{
		return CB_ERROR;
	}
	while (cb_is_buffer_empty(buf) == CB_NOT_EMPTY)
	{
		cb_remove_item_from_buffer(buf);
	}
	free(buf->base);
	return CB_SUCCESS;
}


/**
 * @brief  Check if buffer is full
 * @param  buf Pointer to circbuf struct
 * @return CB_NOT_FULL or CB_FULL or CB_ERROR
 *
 */
cb_status_e cb_is_buffer_full(circbuf_s * buf)
{
	if (!buf || !buf->head || !buf->tail || !buf->base)
	{
		return CB_ERROR;
	}
	return (buf->size == buf->capacity) ? CB_FULL : CB_NOT_FULL;
}


/**
 * @brief  Check if buffer is empty
 * @param  buf Pointer to circbuf struct
 * @return CB_NOT_EMPTY or CB_EMPTY or CB_ERROR
 *
 */
cb_status_e cb_is_buffer_empty(circbuf_s * buf)
{
	if (!buf || !buf->head || !buf->tail || !buf->base)
	{
		return CB_ERROR;
	}
	return (buf->size == 0) ? CB_EMPTY : CB_NOT_EMPTY;
}

/**
 * @brief  Add item to buffer
 * @param  buf  Pointer to circbuf struct
 *         item Generic item to add to buffer
 * @return CB_SUCCESS or CB_ERROR or CB_FULL
 *
 */
cb_status_e cb_add_item_to_buffer(circbuf_s * buf, void * item)
{
	if (!buf || !buf->head || !buf->tail || !buf->base)
	{
		return CB_ERROR;
	}
	if (cb_is_buffer_full(buf) == CB_FULL)
	{
		return CB_FULL;
	}
	buf->head->item = item;
	buf->size++;
	// If head is at top of buffer
	if (buf->head == buf->base + buf->capacity)
	{
		if (buf->tail != buf->base)
		{
			// Move head to base
			buf->head = buf->base;
		}
	}
	// Move head up the buffer
	else
	{
		buf->head++;
	}
	return CB_SUCCESS;
}


/**
 * @brief  Remove item from buffer and return the removed item
 * @param  buf  Pointer to circbuf struct
 * @return The removed item
 *
 */
void * cb_remove_item_from_buffer(circbuf_s * buf)
{
	if (!buf || !buf->head || !buf->tail || !buf->base)
	{
		return NULL;
	}
	if (cb_is_buffer_empty(buf) == CB_EMPTY)
	{
		return NULL;
	}
	void * item = buf->tail->item;
	buf->tail->item = NULL;
	buf->size--;
	// If tail is at top of buffer
	if (buf->tail == buf->base + buf->capacity)
	{
		buf->tail = buf->base;
	}
	// Move tail up the buffer
	else
	{
		buf->tail++;
	}
	return item;
}
