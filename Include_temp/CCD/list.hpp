//
// Created by Chenfei Wang on 7/7/2023.
//

#ifndef TCCPPLIB_LIST_HPP
#define TCCPPLIB_LIST_HPP


//#include <cstring>
#include "basic.hpp"

namespace CCD {
    struct _ccd_list_t {
        struct _ccd_list_t *next, *prev;
    };
    typedef struct _ccd_list_t ccd_list_t;



/**
 * Get the struct for this entry.
 * @ptr:	the &ccd_list_t pointer.
 * @type:	the type of the struct this is embedded in.
 * @member:	the name of the list_struct within the struct.
 */
#define ccdListEntry(ptr, type, member) \
    ccd_container_of(ptr, type, member)

/**
 * Iterates over list.
 */
#define ccdListForEach(list, item) \
        for (item = (list)->next; \
             _ccd_prefetch((item)->next), item != (list); \
             item = (item)->next)

/**
 * Iterates over list safe against remove of list entry
 */
#define ccdListForEachSafe(list, item, tmp) \
        for (item = (list)->next, tmp = (item)->next; \
             item != (list); \
             item = tmp, tmp = (item)->next)

/**
 * Iterates over list of given type.
 * @pos:	the type * to use as a loop cursor.
 * @head:	the head for your list.
 * @member:	the name of the list_struct within the struct.
 */
#define ccdListForEachEntry(head, pos, postype, member)                 \
    for (pos = ccdListEntry((head)->next, postype, member);    \
         _ccd_prefetch(pos->member.next), &pos->member != (head);    \
         pos = ccdListEntry(pos->member.next, postype, member))

/**
 * Iterates over list of given type safe against removal of list entry
 * @pos:	the type * to use as a loop cursor.
 * @n:		another type * to use as temporary storage
 * @head:	the head for your list.
 * @member:	the name of the list_struct within the struct.
 */
#define ccdListForEachEntrySafe(head, pos, postype, n, ntype, member)         \
    for (pos = ccdListEntry((head)->next, postype, member),             \
         n = ccdListEntry(pos->member.next, postype, member);    \
         &pos->member != (head);                    \
         pos = n, n = ccdListEntry(n->member.next, ntype, member))


/**
 * Initialize list.
 */
    _ccd_inline void ccdListInit(ccd_list_t *l);

    _ccd_inline ccd_list_t *ccdListNext(ccd_list_t *l);

    _ccd_inline ccd_list_t *ccdListPrev(ccd_list_t *l);

/**
 * Returns true if list is empty.
 */
    _ccd_inline int ccdListEmpty(const ccd_list_t *head);

/**
 * Appends item to end of the list l.
 */
    _ccd_inline void ccdListAppend(ccd_list_t *l, ccd_list_t *item);

/**
 * Removes item from list.
 */
    _ccd_inline void ccdListDel(ccd_list_t *item);



///
/// INLINES:
///

    _ccd_inline void ccdListInit(ccd_list_t *l) {
        l->next = l;
        l->prev = l;
    }

    _ccd_inline ccd_list_t *ccdListNext(ccd_list_t *l) {
        return l->next;
    }

    _ccd_inline ccd_list_t *ccdListPrev(ccd_list_t *l) {
        return l->prev;
    }

    _ccd_inline int ccdListEmpty(const ccd_list_t *head) {
        return head->next == head;
    }

    _ccd_inline void ccdListAppend(ccd_list_t *l, ccd_list_t *n) {
        n->prev = l->prev;
        n->next = l;
        l->prev->next = n;
        l->prev = n;
    }

    _ccd_inline void ccdListDel(ccd_list_t *item) {
        item->next->prev = item->prev;
        item->prev->next = item->next;
        item->next = item;
        item->prev = item;
    }


}


#endif //TCCPPLIB_LIST_HPP
