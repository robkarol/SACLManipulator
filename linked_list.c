#include "linked_list.h"
#include <stdlib.h>

/*+=======================+*/
/*| Linked-list functions |*/
/*+=======================+*/

/*! Structure for Rapidly-exploring Random Trees (RRT's).  Structure variable used to represent an RRT.  The particular data structure
used by Manipulator RRT is augmented to include indicators of node safety, linked lists for each tree node, and other elements in
order to improve the efficiency of replanning and closed-loop control. */
struct tree_t {
	double** nodes;					/*!< Array of pointers to the vectors q of joint angles, whose k-th element corresponds to node k */
	double* costs;					/*!< Array of costs-to-come (for a forward tree) or costs-to-go (for a reverse tree), whose k-th element corresponds to node k */
	int* parents;					/*!< An array whose k-th index corresponds to the parent node to node k (or a -1 for the root node) */
	int* connections;				/*!< An array whose k-th index corresponds to the index of the leaf node in the partner tree that is connected to node k (or a -1 if no connection was made) */
	struct list_node** leaf_lists;	/*!< An array of linked list pointers, the k-th element of which is used to find the "leaves" to which node k is connected */
	int* safety;					/*!< Array of booleans used to identify if saved nodes are safe to use after tree construction (tracks if they violate new constraints) */
	int* indices;					/*!< An array whose k-th index is the index for node k (necessary for the KD-tree algorithm) */
	struct kdtree* kd_tree;			/*!< Pointer to the tree's associated KD-tree, used for NearestNeighbor selection */
};

/*! Print the elements of a linked list to file
	\param[in] filename	Pointer to file, generated using `fopen()`
	\param[in] format	Format string compatible with the `fprintf()` command
	\param[in] ptr		Pointer to the first element of the linked list */
void PrintListToFile(FILE* filename, char* format, struct list_node *ptr) {
	while ( ptr != NULL ) {
		fprintf( filename, format, ptr->data );
		ptr = ptr->next;
	}
}

/*! Insert an element to the front of a linked list
	\param[in] root		Current root node of the linked list
	\param[in] new_data	Integer data to be inserted
	\return new_root	Pointer to newly-added root node */
struct list_node* AddListElement(struct list_node *root, int new_data ) {

	struct list_node *temp;
	temp		= (struct list_node*) malloc( sizeof(struct list_node) );
    temp->data	= new_data;
	temp->next	= root;
	root		= temp;

	return root;
}

/*! Remove the non-unique elements of the reverse-sorted leaf list *B* from the reverse-sorted leaf list *A*
	\param[in] T		Pointer to RRT
	\param[in] A_index	Index of list A in tree T
	\param[in] B_index	Index of list B in tree T */
void SetDiffLeafListBfromA( struct tree *T, int A_index, int B_index ) {

	struct list_node *temp, *previous, *ptrA, *ptrB;

	previous	= NULL;
	ptrA		= T->leaf_lists[ A_index ];
	ptrB		= T->leaf_lists[ B_index ];

	while ( (ptrA != NULL) && (ptrB != NULL) ) {
		if ( ptrA->data > ptrB->data ) {
			/* If ptrA->data is greater than ptrB->data, advance ptrA (ptrA->data cannot be contained in B since all remaining elements
			of B are less than ptrB->data) until ptrA->data <= ptrB->data */
			previous	= ptrA;
			ptrA		= ptrA->next;
		} else if (ptrA->data == ptrB->data) {
			/* If equal to, and there is no previous element in A (ptrA points to the head of the list), redefine the head of the list to the
			next element since the head will be deleted.  If not, set the previous element of A to point to the next element.  */
			if ( previous == NULL ) T->leaf_lists[A_index] = ptrA->next;
			else previous->next = ptrA->next;

			/* Delete the element of A */
			temp		= ptrA;
			ptrA		= ptrA->next;
			ptrB		= ptrB->next;
			free(temp);
		}
		else {
			/* If ptrA->data < ptrB->data, ptrB->data is no longer a possible element in A.  Advance ptrB to its next element
			until ptrB points to an element less than or equal to that of ptrA */
			ptrB	= ptrB->next;
		}
	}
}

/*! Merge the elements of reverse-sorted leaf list *B* into the reverse-sorted leaf list *A*
	\param[in] T		Pointer to RRT
	\param[in] A_index	Index of list A in tree T
	\param[in] B_index	Index of list B in tree T */
void MergeLeafListBwithA( struct tree *T, int A_index, int B_index ) {

	struct list_node *temp, *previous, *ptrA, *ptrB;

	previous	= NULL;
	ptrA		= T->leaf_lists[ A_index ];
	ptrB		= T->leaf_lists[ B_index ];

	while ( ptrB != NULL ) {
		if ( (ptrA == NULL) || (ptrA->data < ptrB->data) ) {
			/* If ptrA is NULL, add the remaining elements of B to A.  Otherwise, add all elements of B in front of ptrA
			until ptrA->data is >= ptrB->data again.  In both cases, be careful to preserve the reverse-sorted order of A. */
			temp		= (struct list_node*) malloc( sizeof(struct list_node) );
			temp->data	= ptrB->data;
			temp->next	= ptrA;
			ptrB		= ptrB->next;

			/* If no element exists before ptrA, the new element added to A is the head of the list.  Otherwise make the
			previous element point to the newly added element */
			if ( previous == NULL ) T->leaf_lists[ A_index ] = temp;
			else previous->next = temp;

			previous	= temp;
		}
		else if (ptrA->data == ptrB->data) {
			/* If the two elements are equal, skip over that element of A since it is already in A.  Advance A and B to their next elements.  */
			previous	= ptrA;
			ptrA		= ptrA->next;
			ptrB		= ptrB->next;
		}
		else {
			/* If ptrA->data > ptrB->data, there is nothing to be added to A from B in front of ptrA (ptrB->data belongs somewhere behind ptrA) */
			previous	= ptrA;
			ptrA		= ptrA->next;
		}
	}
}

/*! Free the memory stored in a linked list given the pointer *root* to its root node */
void FreeList(struct list_node *root) {
	struct list_node *ptr = root;
	struct list_node *temp;

	/* While not at the end of the list, free elements starting at the root element */
	while ( ptr != NULL ) {
		temp = ptr;
		ptr = ptr->next;
		free(temp);
	}
	root = NULL;
}
