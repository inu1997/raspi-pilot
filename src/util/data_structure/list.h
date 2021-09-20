/**
 * @file list.h
 * @author LIN 
 * @brief Object List data structure.
 * Keep address of variable as object in linked list.
 * 
 * 
 * @version 0.1
 * @date 2021-09-06
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef _LIST_H_
#define _LIST_H_

#include <stdbool.h>

struct List;

struct List *list_init();

int list_destroy(struct List *l);

int list_add(struct List *l, void *object);

int list_remove(struct List *l, void *object);

int list_find(struct List *l, void *object);

void *list_iterate(struct List *l, int i);

int list_get_count(struct List *l);

#endif // _LIST_H_
