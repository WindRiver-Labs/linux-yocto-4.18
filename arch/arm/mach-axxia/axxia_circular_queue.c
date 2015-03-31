/*
 * axxia_circular_queue.c
 *
 *  Created on: Sep 30, 2014
 *      Author: z8cpaul
 */


#include <asm/exception.h>
#include "axxia_circular_queue.h"


void axxia_initialize_queue(struct circular_queue_t *queue)
{
	int i;

	queue->valid_items = 0;
	queue->first = 0;
	queue->last = 0;

	for (i = 0; i < MAX_ITEMS; i++)
		queue->data[i] = NULL;

	return;
}

bool axxia_is_empty(struct circular_queue_t *queue)
{

	if (queue->valid_items == 0)
		return true;
	else
		return false;
}

int axxia_put_item(struct circular_queue_t *queue, void *item_value)

{
	if (queue->valid_items >= MAX_ITEMS) {
		pr_err("ERROR: queue is full\n");
		return -EINVAL;
	} else {
		queue->valid_items++;
		queue->data[queue->last] = item_value;
		queue->last = (queue->last + 1) % MAX_ITEMS;
	}
	return 0;
}


int axxia_get_item(struct circular_queue_t *queue, void **item_value)
{

	if (axxia_is_empty(queue)) {
		return -1;
	} else {
		*item_value = queue->data[queue->first];
		queue->first = (queue->first + 1) % MAX_ITEMS;
		queue->valid_items--;
	}
	return 0;

}
