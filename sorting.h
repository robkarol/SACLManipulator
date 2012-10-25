#ifndef __SORTING_H__
#define __SORTING_H__

/*! Insertion sort (stable) a `double` vector *A* (in-place) and return the re-ordered indices *I* <BR>
	(\f$O(n)\f$ **best-case** = *already sorted*, \f$O(n^2)\f$ **worst-case** = reverse-sorted, \f$O(n^2)\f$ **average time**) 
	\param[in,out] A	Array of elements to be sorted
	\param[in] length	Number of elements in A
	\param[in,out] I	Empty `int` array of same size as A.  Returned with the rearranged indices of A. */
void InsertionSort( double A[], int length, int I[] );

/*! Auxiliary function for `HeapSort` used to float down elements of *A* into their appropriate place in a heap subtree \see HeapSort
	\param[in,out] A	Array of heap elements
	\param[in] root		Index of subtree root whose element should be floated down
	\param[in] bottom	Index of the last element in the heap
	\param[in,out] I	Rearranged indices of A */
void SiftDown( double A[], int root, int bottom, int I[] );

/*! Heap sort (unstable) a `double` vector *A* (in-place) and return the re-ordered indices *I* <BR>
	(\f$\Omega(n)\f$, \f$O(n \log{n})\f$ **best-case**, \f$O(n \log{n})\f$ **worst-case**, \f$O(n \log{n})\f$ **average time**) <BR>
	Modified from the source code found here: http://www.algorithmist.com/index.php/Heap_sort.c \see SiftDown
	\param[in,out] A	Array of elements to be sorted
	\param[in] length	Number of elements in A
	\param[in,out] I	Empty `int` array of same size as A.  Returned with the rearranged indices of A. */
void HeapSort( double A[], int length, int I[] );

/*! Auxiliary function for `MergeSort` used to merge two sorted sublists into a combined sorted list \see MergeSort
	\param[in] L,R					Left and right subarrays to be merged 
	\param[in] I_L,I_R				Indices of the left and right subarrays
	\param[in] length_L,length_R	Lengths of each subarray
	\param[out] B					Combined, sorted array of the elements of L and R (stable merge)
	\param[out] J					Indices of A corresponding to the rearrangement of elements in B */
void Merge( double L[], double R[], int I_L[], int I_R[], int length_L, int length_R, double B[], int J[] );
	

/*! Merge sort (stable) a `double` vector *A* (using \f$O(2n)\f$ memory) and return the re-ordered indices *I* <BR>
	(\f$\Omega(n)\f$, \f$O(n \log{n})\f$ **best-case**, \f$O(n \log{n})\f$ **worst-case**, \f$O(n \log{n})\f$ **average time**) <BR>
	Modified from the source code found here: http://www.algorithmist.com/index.php/Merge_sort.c \see Merge
	\param[in,out] A	Array of elements to be sorted
	\param[in] length	Number of elements in A
	\param[in,out] I	Empty `int` array of same size as A.  Returned with the rearranged indices of A. */
void MergeSort( double A[], int length, int I[] );

/*! Rearrange integer vector *A* according to the indices in `int` vector *I* */
void RearrangeIntVector( int A[], int length, int I[] );

/*! Rearrange integer pointer vector *A* according to the indices in `int` vector *I* */
void RearrangeIntPtrVector( int* A[], int length, int I[] );

#endif