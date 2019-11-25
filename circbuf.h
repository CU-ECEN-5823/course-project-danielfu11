/**
 * @file 	circbuf.h
 * @author 	Guanxiong Fu
 * @date	Nov 23 2019
 * @brief	Header file for circular buffer implementation
 *
 */

#ifndef __CIRCBUF_H__
#define __CIRCBUF_H__

#include <stdint.h>
#include <stdlib.h>

// Generic buffer item structure
typedef struct
{
	void * item;
} item_s;

// Circular buffer structure
typedef struct
{
	item_s * base; 	// Pointer to the base of buffer
	item_s * head;	// Pointer to the first domain name in the buffer
	item_s * tail;	// Pointer to the last domain name in the buffer
	int	size;		// Number of domain names in the buffer
	int capacity;	// Max number of domain names in the buffer
} circbuf_s;

// Enums for circular buffer status
typedef enum
{
	CB_SUCCESS 		= 0,
	CB_ERROR 		= 1,
	CB_NOT_FULL 	= 2,
	CB_FULL 		= 3,
	CB_NOT_EMPTY 	= 4,
	CB_EMPTY 		= 5,
} cb_status_e;


cb_status_e cb_initialize_buffer(circbuf_s * buf, uint32_t capacity);

cb_status_e cb_delete_buffer(circbuf_s * buf);

cb_status_e cb_is_buffer_full(circbuf_s * buf);

cb_status_e cb_is_buffer_empty(circbuf_s * buf);

cb_status_e cb_add_item_to_buffer(circbuf_s * buf, void * item);

void * cb_remove_item_from_buffer(circbuf_s * buf);

#endif /* __CIRCBUF_H__ */
