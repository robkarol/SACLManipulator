#include "sorting.h"
#include "utils.h"

void InsertionSort( double A[], int length, int I[] ) {
	
	/* Store original ordering of indices */
	for (int i = 0; i < length; i++) {
		I[i] = i;
	}

	/* Sort A, starting with its second element and moving right.  Compare v = A[i] with elements
	A[j] for j < i, moving left, which contains the previously-sorted array A[0:1:i-1].  Insert v where
	appropriate, creating the sorted list A[0:1:i].  Continue until the end of A. */
	for (int i = 1; i < length; i++) {
		/* Save the values v and c, the elements to be inserted into the sorted sublist A[0:1:i-1] and I[0:1:i-1] */
		double v = A[i];
		int c = I[i];

		/* Sort A and I, using values of A as keys.  Insert v = A[i] into the sorted sublist.
		Any values larger than v should be shifted to the right by 1 to make room for v. */
		int j = i;
		while (j > 0 && A[j-1] > v) {
			A[j] = A[j-1];
			I[j] = I[j-1];
			j = j - 1;
		}

		/* If at the beginning of the array (j = 0), insert v and c.  Otherwise the loop exited
		where the element A[j-1] < v.  Since this is the largest element among A[0:1:j-1], v must
		belong to A[j].  All larger elements were already shifted to the right, so overwrite A[j] with v. */
		A[j] = v;
		I[j] = c;
	}
}


void SiftDown( double A[], int root, int bottom, int I[] ) {
	// Considering A as a binary heap, the given "root" node is a parent to up to two children 
	// at indices 2*root + 1 and 2*root + 2 if they exist.  If neither exists, root is a leaf node.
	int maxChild = root * 2 + 1;
 
	// Find the biggest of the two children and set it equal to maxChild
	if ( maxChild < bottom ) {
		int otherChild = maxChild + 1;
		maxChild = ( A[otherChild] > A[maxChild] ) ? otherChild : maxChild;		// Reversed for stability
	} else {
		if ( maxChild > bottom ) return;	// Don't overflow; root is a leaf node and therefore in the right place.
	}
 
	// If we have the correct ordering, we are done ( A[parent] > A[both children] ).
	if ( A[root] >= A[maxChild] ) return;
 
	// Otherwise, swap the parent with the larger of the two children.
	double temp	= A[root];			int tmp_int = I[root];
	A[root]		= A[maxChild];		I[root]		= I[maxChild];
	A[maxChild]	= temp;				I[maxChild]	= tmp_int;
 
	// Tail queue recursion (propagate the parent down since it may still be out of place).
	// Will be compiled as a loop with correct compiler switches.
	SiftDown( A, maxChild, bottom, I );
}

void HeapSort( double A[], int length, int I[] ) {
	int i, tmp_int; double temp;
 
	/* Store original ordering of indices */
	for (int i = 0; i < length; i++) {
		I[i] = i;
	}

	/* Re-order A as a binary heap ("heapify A").  Only need to sift first half of A, where all possible parent nodes reside.  
	SiftDown will float each improperly placed parent down and re-order their subtrees to satisfy the heap property. */
	for (i = (length / 2); i >= 0; i--) {
		SiftDown( A, i, length - 1, I );
	}
	
	/* A is now a binary heap, with Amax = A[0].  Swap it and the last element and remove the last element from consideration.
	The only node out of place now is A[0].  Sift it down to make A[0:1:length-2] a heap.  Now A[0] is the max. of this new heap.  Swap it
	with the second-to-last element and remove it from consideration.  Sift A[0] down to make A[0:1:length-3] a heap.  Repeat until fully-sorted!  */
	for (i = length-1; i >= 1; i--) {
		temp = A[0];	tmp_int = I[0];		// Swap
		A[0] = A[i];	I[0]	= I[i];
		A[i] = temp;	I[i]	= tmp_int;
 
		SiftDown( A, 0, i-1, I );
	}
}

void Merge( double L[], double R[], int I_L[], int I_R[], int length_L, int length_R, double B[], int J[] ) {
	
	// L[] = left sublist of size length_L, R[] = right sublist of size length_R,	B[] = output array
	// I_L[] = indices of left sublist,		I_R[] = indices of right sublist,		J[] = output indices
	int i = 0, j = 0, k = 0;

	/* Iterate over the left sublist and right sublist, comparing the two and saving the smallest indices to B. */
	while (i < length_L && j < length_R) {
		if ( L[i] < R[j] ) {
			B[k]	= L[i];
			J[k]	= I_L[i];
			i		+= 1;
		}
		else {
			B[k]	= R[j];
			J[k]	= I_R[j];
			j		+= 1;
		}
		k += 1;
	}
	
	/* If i < length_L, all elements of R were used up and therefore the remaining elements of L are larger.
	Iterate over the rest of the left sublist, adding the remaining (sorted!) elements to B */
	while ( i < length_L ) {
		B[k]	= L[i];
		J[k]	= I_L[i];
		k += 1;	i += 1;
	}
	
	/* If j < length_R, all elements of L were used up and therefore the remaining elements of R are larger.
	Iterate over the rest of the right sublist, adding the remaining (sorted!) elements to B */
	while ( j < length_R ) {
		B[k]	= R[j];
		J[k]	= I_R[j];
		k += 1;	j += 1;
	}
}

void MergeSort( double A[], int length, int I[] ) {
	
	/* Store original ordering of indices */
	for (int i = 0; i < length; i++) {
		I[i] = i;
	}

	if (length == 1) return;
	
	/* Allocate placeholder array(s) for merge operations */
	double* B	= (double*) malloc( length*sizeof(double) );
	int* J		= (int*) malloc( length*sizeof(int) );
	int j;

	/* Bottom-up merge sort iteratively divides the array into sub-arrays, sorts them and then merges them back together.
	Work on sublists of size k = 1, 2, 4, 8, 16, ..., last power of 2 smaller than length */
	for (int k = 1; k < length; k *= 2 ) {

		/* Create a sorted merged list B[j + (0:(2k-1))] from each pair of sorted sublists L = A[j + (0:(k-1))] and 
		R = A[j + (k:(2k-1))].  Ensure that the leftmost index, j, satisfies j + (k-1) < length so L is always full, clipping R when 
		their difference satisfies length-(j+k)) < k (R will be empty at the end of its first iteration, k = 1, of sorting an odd-length array) */
		j = 0;
		while ( (j+(k-1)) < length ) {
			Merge( &(A[j]), &(A[j + k]), &(I[j]), &(I[j+k]), k, min(k,(length-1)-(j+(k-1))), &(B[j]), &(J[j]) );
			j = j + 2*k;
		}

		/* Copy B over to A, enforcing the loop invariant that every sublist is already sorted
		(the combined pairs of sublists in B, sorted, join to become a single sublist for the next value of k) */
		for (int i = 0; i < length; i++) {
			A[i] = B[i];
			I[i] = J[i];
		}
	}

	free(B); free(J);
}

void RearrangeIntVector( int A[], int length, int I[] ) {
	
	/* Create a temporary vector B for the values of A */
	int* B = (int*) malloc(length*sizeof(int));
	for (int i = 0; i < length; i++) {
		B[i] = A[i];
	}

	/* The j-th entry of A should be its I[j]-th entry.  Overwrite using the copied vector B of the original A. */
	for (int j = 0; j < length; j++) {
		A[j] = B[ I[j] ];
	}

	free(B);
}

void RearrangeIntPtrVector( int* A[], int length, int I[] ) {
	
	/* Create a temporary vector B for the pointers stored in A */
	int** B = (int**) malloc(length*sizeof(int*));
	for (int i = 0; i < length; i++) {
		B[i] = A[i];
	}

	/* The j-th entry of A should be its I[j]-th entry.  Overwrite using the copied vector B of the original A. */
	for (int j = 0; j < length; j++) {
		A[j] = B[ I[j] ];
	}

	free(B);
}
