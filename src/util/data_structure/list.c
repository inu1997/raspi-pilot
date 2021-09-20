#include "list.h"

#include "util/logger.h"
#include "util/debug.h"

#include <stdlib.h>
#include <stdarg.h>

struct Node {
    void *object;
    struct Node *next;
};

struct List {
    struct Node *head;
};

/**
 * @brief Initiator of list.
 * 
 * @return Newly created list structure.
 */
struct List *list_init() {
    struct List *l = malloc(sizeof(struct List));
    l->head = NULL;
    return l;
}

/**
 * @brief Destroyer of list.
 * 
 * @param l
 *      The list.
 * @return Count of objects the list used to hold.
 */
int list_destroy(struct List *l) {
    int cnt = 0;
    struct Node *tmp;
    struct Node *node = l->head;
    while(node != NULL) {
        tmp = node;
        node = node->next;
        free(tmp);
        cnt++;
    }
    free(l);
    return cnt;
}

/**
 * @brief Add object to list.
 * 
 * @param l 
 *      The list.
 * @param object 
 *      The object.
 * @return Count of objects after adding to list.
 */
int list_add(struct List *l, void *object) {
    struct Node *new_node = malloc(sizeof(struct Node));
    new_node->object = object;
    new_node->next = NULL;
    int cnt = 1;
    if (l->head == NULL) {
        // Handle first add.
        l->head = new_node;
    } else {
        struct Node *prev;
        struct Node *node = l->head;
        while(node != NULL) {
            prev = node;
            node = node->next;
            cnt++;
        }
        prev->next = new_node;
    }
    return cnt;
}

/**
 * @brief Remove object(recognized by address) from list.
 * 
 * @param l
 *      The list.
 * @param object 
 *      The object.
 * @return 0 if success else -1.
 */
int list_remove(struct List *l, void *object) {
    int ret = -1;
    if (l->head->object == object) {
        // Handle remove first object.
        struct Node *orig_head = l->head;
        l->head = l->head->next;
        ret = 0;
        free(orig_head);
    } else {
        struct Node *node = l->head;
        struct Node *prev;
    
        while(node != NULL) {
            prev = node;
            node = node->next;
            if (node != NULL) {
                if (node->object == object) {
                    // Found object.
                    prev->next = node->next;
                    free(node);
                    ret = 0;
                    break;
                }
            }
        }
    }
    return ret;
}

/**
 * @brief Find the first object from list.
 * 
 * @param l
 *      The list.
 * @param object
 *      The object.
 * @return Index of object else -1(Not found).
 */
int list_find(struct List *l, void *object) {
    struct Node *node = l->head;
    int i = 0;
    int ret = -1;
    while(node != NULL) {
        if (node->object == object) {
            ret = i;
            break;
        }
        node = node->next;
        i++;
    }
    return ret;
}

/**
 * @brief Work as iternation.
 * 
 * @param l
 *      The list.
 * @param i
 *      Index.
 * @return Address of object at i index. NULL if it doesn't exist. 
 */
void *list_iterate(struct List *l, int i) {
    struct Node *node = l->head;
    while(node != NULL && i > 0) {
        node = node->next;
        i--;
    }

    return node == NULL ? NULL : node->object;
}

/**
 * @brief Get how many objects are in the list.
 * 
 * @param l
 *      The list.
 * @return Count of objects.
 */
int list_get_count(struct List *l) {
    struct Node *node = l->head;
    int cnt = 0;
    while (node != NULL) {
        node = node->next;
        cnt++;
    }
    return cnt;
}