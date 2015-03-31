/*
 * axxia_circular_queue.h
 *
 *  Created on: Sep 30, 2014
 *      Author: z8cpaul
 */

#ifndef AXXIA_CIRCULAR_QUEUE_H_
#define AXXIA_CIRCULAR_QUEUE_H_

#define MAX_ITEMS    1020

struct circular_queue_t

{
	int first;
	int last;
	int valid_items;
	void *data[MAX_ITEMS];
};

void axxia_initialize_queue(struct circular_queue_t *queue);

bool axxia_is_empty(struct circular_queue_t *queue);

int axxia_put_item(struct circular_queue_t *queue, void *item_value);

int axxia_get_item(struct circular_queue_t *queue, void **item_value);

#endif
