#ifndef __LINKED_LIST_H__
#define __LINKED_LIST_H__

/*+=======================+*/
/*| Linked-list functions |*/
/*+=======================+*/

#include <stdio.h>

/*! Structure for linked lists. Structure variable used to represent the element of a linked list of integer data.  List nodes are employed primarily to
store lists of subtree leaves for each node in an RRT data structure. */
struct list_node {
	int data;						/*!< Integer data associated with the list node */
	struct list_node *next;			/*!< Pointer to the next linked list element (or NULL at end of list) */
};

/*! Print the elements of a linked list to file
	\param[in] filename	Pointer to file, generated using `fopen()`
	\param[in] format	Format string compatible with the `fprintf()` command
	\param[in] ptr		Pointer to the first element of the linked list */
void PrintListToFile(FILE* filename, char* format, struct list_node *ptr);

/*! Insert an element to the front of a linked list
	\param[in] root		Current root node of the linked list
	\param[in] new_data	Integer data to be inserted
	\return new_root	Pointer to newly-added root node */
struct list_node* AddListElement(struct list_node *root, int new_data );

/*! Remove the non-unique elements of the reverse-sorted leaf list *B* from the reverse-sorted leaf list *A*
	\param[in] T		Pointer to RRT
	\param[in] A_index	Index of list A in tree T
	\param[in] B_index	Index of list B in tree T */
void SetDiffLeafListBfromA( struct tree *T, int A_index, int B_index );

/*! Merge the elements of reverse-sorted leaf list *B* into the reverse-sorted leaf list *A*
	\param[in] T		Pointer to RRT
	\param[in] A_index	Index of list A in tree T
	\param[in] B_index	Index of list B in tree T */
void MergeLeafListBwithA( struct tree *T, int A_index, int B_index );

/*! Free the memory stored in a linked list given the pointer *root* to its root node */
void FreeList(struct list_node *root);

#endif
