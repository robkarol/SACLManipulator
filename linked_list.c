/*! Structure for linked lists. Structure variable used to represent the element of a linked list of integer data.  List nodes are employed primarily to
store lists of subtree leaves for each node in an RRT data structure. */
struct list_node {
	int data;						/*!< Integer data associated with the list node */
	struct list_node *next;			/*!< Pointer to the next linked list element (or NULL at end of list) */
};
