#ifndef __TREE_H__
#define __TREE_H__

/*! Structure for Rapidly-exploring Random Trees (RRT's).  Structure variable used to represent an RRT.  The particular data structure
used by Manipulator RRT is augmented to include indicators of node safety, linked lists for each tree node, and other elements in
order to improve the efficiency of replanning and closed-loop control. */
struct tree {
	double** nodes;					/*!< Array of pointers to the vectors q of joint angles, whose k-th element corresponds to node k */
	double* costs;					/*!< Array of costs-to-come (for a forward tree) or costs-to-go (for a reverse tree), whose k-th element corresponds to node k */
	int* parents;					/*!< An array whose k-th index corresponds to the parent node to node k (or a -1 for the root node) */
	int* connections;				/*!< An array whose k-th index corresponds to the index of the leaf node in the partner tree that is connected to node k (or a -1 if no connection was made) */
	struct list_node** leaf_lists;	/*!< An array of linked list pointers, the k-th element of which is used to find the "leaves" to which node k is connected */
	int* safety;					/*!< Array of booleans used to identify if saved nodes are safe to use after tree construction (tracks if they violate new constraints) */
	int* indices;					/*!< An array whose k-th index is the index for node k (necessary for the KD-tree algorithm) */
	struct kdtree* kd_tree;			/*!< Pointer to the tree's associated KD-tree, used for NearestNeighbor selection */
};

#endif

