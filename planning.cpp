#include "planning.h"
#include "utils.h"
#include "kdtree.h"
#include "tree_data_structure.h"
#include "matlab_interface.h"
#include "sorting.h"
#include "obstacles.h"
#include "collision_detection.h"
#include "log.h"

void Halton( int* sequence, int length, int D, double** h ) {
	/* Returns "num" vectors of double pointers of length D to h, listed according to the Halton sequence */
	/* CAUTION: if D >= 12, this code will reuse primes (Dec2BaseInts is restricted to primes between 2 and 36) */

	int primes[] = { 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 
		53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131 };
	int p, N;
	int *a;

	/* The m-th row of h corresponds to the m-th value in a particular Halton sequence */
	/* The n-th column of h corresponds to the n-th prime number (up to the 11-th prime number, then they are re-used) */
	for (int j = 0; j < D; j++) {
		p = primes[j % 11];

		/* If integer sequence[i] = a0 + a1*p + a2*p^2 + ... (base-p representation, summed as least-to-most significant bit),
		then the sequence[i]-th number in the Halton sequence corresponding to prime p is:  a0/p + a1/(p^2) + a2/(p^3) + ... */
		for (int i = 0; i < length; i++) {
			
			/* Find the base-p representation of integer sequence[i].  Note that "a" stores coefficients 
			in the reverse order of the sum above (most-to-least significant bit) */
			a = Dec2BaseInts(sequence[i], p, &N);

			/* Stores the sequence[i]-th value of the Halton sequence corresponding to the (j+1)-th prime number */
			h[i][j] = 0;
			for (int k = 0; k < N; k++) {
				h[i][j] += ((double) a[k])*pow((double) p, (double) -(N-k));
			}

			free( a );
		}
	}
}

void Sample( int feedback_mode, char* sampling, int n, double** Q, double* q_max, double* q_min, int* iter, double* q ) {
	
	if (feedback_mode == 0) {
		for (int j = 0; j < n; j++) {
			q[j] = Q[*iter-1][j];
		}
	}
	else {
		if ( strncmp(sampling, "halton", 6) == 0 ) {
			Halton( iter, 1, n, &(q) );
			for (int j = 0; j < n; j++) {
				q[j] = q[j] * ( q_max[j] - q_min[j] ) + q_min[j];
			}
		}
		else if ( strncmp(sampling, "pseudorandom", 12) == 0 ) {
			for (int j = 0; j < n; j++) {
				q[j] = ( rand() % (int) floor(q_max[j] - q_min[j]) ) + q_min[j];
			}
		}
	}
}

double Steer( double* q, double* q_near, int n, double epsilon, double* q_new, double* w ) {

	/* Distance from q_near to q */
	double dist = Sqrt( DistSq( q, q_near, n, w ) );

	/* Parameter corresponding to the advance along the line from q_near to q ( 0 <= s <= 1, s = epsilon/dist unless dist is smaller than epsilon ) */
	double s = min( epsilon/dist, 1 );

	/* Advance towards q to the new point, which is at most epsilon away from q_near */
	for ( int i = 0; i < n; i++ ) {
		q_new[i] = q_near[i] +  s * ( q[i]-q_near[i] );
	}

	/* Return the cost as the distance increment ( cost = epsilon unless dist is smaller ) */
	return s*dist;		
}

void Nearest(struct tree *T, int num_nodes, double* q, int n, double* w, int cost_type, int* nearest_index, double* q_near, char* NN_alg, Engine* matlab) {
	
	double cost, min_cost = DBL_MAX;

	// kdtree Code (does not check node safety)
	if ( strncmp(NN_alg, "kd_tree", 7) == 0 ) {
		struct kdres *kd_set	= kd_nearest(T->kd_tree, q);
		int *node_ptr			= (int*) kd_res_item_data( kd_set );
		*nearest_index			= *node_ptr;
		kd_res_free(kd_set);
	}

	// Brute Force Code
	else if ( strncmp(NN_alg, "brute_force", 11) == 0 ) {

		/* For each node in tree T, compute the cost function between q_near and q, and save the min(max_neighbors,num_nodes) most optimal connections */
		for (int i = 0; i < num_nodes; i++) {
			cost = Sqrt(DistSq(q, T->nodes[i], n, w ) );

			/* Compute the cost of connection q_near to q (1 = globally optimal, 2 = locally optimal) */
			if (cost_type == 1) cost = cost + T->costs[i];

			/* Save the best (safe) index as the nearest_index, returning q_near after all nodes have been searched */
			if ( cost < min_cost && T->safety[i] == 1 ) {
				*nearest_index	= i;
				min_cost		= cost;
			}
		}
	}

	/* Output q_near and nearest_index (already pointing towards the minimum-cost node index) */
	for (int i = 0; i < n; i++) {
		q_near[i] = T->nodes[ *nearest_index ][i];
	}

	/* Highlight the sample q and the nearest node q_near in the RRT figure */
	PlotNearestInMATLAB( T, n, q, q_near, matlab );
}

void NearestNeighbors(struct tree *T, int num_nodes, double* q, int n, double* w, int max_neighbors, int cost_type, double eta_RRT, double gamma_RRT,
	int* neighbors, double* costs, int* n_neighbors, char* NN_alg) {
	
	int *I				= (int*) malloc(max_neighbors*sizeof(int));
	int num_neighbors	= 0;

	// kdtree Code (does not enforce maximum number of neighbors)
	if ( strncmp(NN_alg, "kd_tree", 7) == 0 ) {
		int *node_ptr;
		double search_radius	= min( gamma_RRT * pow( log( (double) num_nodes ) / ((double) num_nodes),  1.0/n),  eta_RRT );
		struct kdres *kd_set	= kd_nearest_range( T->kd_tree, q, search_radius );
		
		num_neighbors			= 0;
		double *q_NN			= (double*) malloc(n*sizeof(double));

		for (int i = 0; i < kd_res_size(kd_set); i++ ) {
			node_ptr = (int*) kd_res_item( kd_set, q_NN );
			if ( T->safety[*node_ptr] == 1 ) {
				costs			= (double*) realloc( costs,		(num_neighbors+1)*sizeof(double) );
				neighbors		= (int*)	realloc( neighbors, (num_neighbors+1)*sizeof(int) );
				I				= (int*)	realloc( I,			(num_neighbors+1)*sizeof(int) );
				neighbors[num_neighbors]	= *node_ptr;
				costs[num_neighbors]		= Sqrt( DistSq(q,q_NN,n,w) );
				num_neighbors	+= 1;
			}
			kd_res_next( kd_set );
		}

		if ( num_neighbors < SORT_SWITCH )	InsertionSort( costs, num_neighbors, I );
		else								MergeSort( costs, num_neighbors, I );
		RearrangeIntVector( neighbors, num_neighbors, I );
		kd_res_free(kd_set);
		free(q_NN);
	}

	// Brute Force Code
	else if ( strncmp(NN_alg, "brute_force", 11) == 0 ) {

		/* For each node in tree T, compute the cost function between q_near and q, and save the min(max_neighbors,num_nodes) most optimal connections */
		double cost_to_go;
		for (int i = 0; i < num_nodes; i++) {
			cost_to_go = Sqrt( DistSq(q, T->nodes[i], n, w) );

			/* Compute the cost of connection q_near to q (1 = globally optimal, 2 = locally optimal) */
			if (cost_type == 1)		cost_to_go = cost_to_go + T->costs[i];

			/* If the node is unsafe (only applies during feedback), do not include it in the list of nearest neighbors */
			if ( T->safety[i] != 1 ) {
				continue;
			}

			/* Maintain up to "max_neighbors" number of the best connections; store their costs and index values in "costs" and "neighbors" respectively */
			if ( num_neighbors < max_neighbors ) {
				neighbors[num_neighbors]	= i;
				costs[num_neighbors]		= cost_to_go;
				num_neighbors += 1;

				/* If on the last node of T or "max_neighbors" has been reached, sort the list of costs, which may be in any order, using the best-suited sort */
				if (( num_neighbors == max_neighbors ) || ( i == num_nodes - 1 )) {
					if ( num_neighbors < SORT_SWITCH )	InsertionSort( costs, num_neighbors, I );
					else								MergeSort( costs, num_neighbors, I );
					RearrangeIntVector( neighbors, num_neighbors, I );
				}
			}
			else {
				/* If neighbor node i has a lower cost than any of the stored neighbors (sorted in ascending order by cost, and hence the maximum is at the last
					element at index num_neighbors-1), then substitute it for the maximum-cost neighbor and re-order the list. */
				if ( cost_to_go < costs[num_neighbors-1] ) {
					neighbors[num_neighbors-1]	= i;
					costs[num_neighbors-1]		= cost_to_go;
					
					/* InsertionSort (O(n) time) the almost-sorted list so that the last element is in its proper position. */
					InsertionSort( costs, num_neighbors, I );
					RearrangeIntVector( neighbors, num_neighbors, I );
				}
			}
		}
	}

	/* Output the number of neighbors found, n_neighbors */
	*n_neighbors = num_neighbors;

	free(I);
}

int Extend(double* q, double* q_near, double epsilon, int n, double* w, double* q_new, struct obstacles* obs, struct geom *G, struct DHparams *DH, double* cost_to_go, int indicator) {

	/* Steer from q_near to q, returning new state q_new (which lies within a distance-squared of epsilon from q_near) */
	*cost_to_go += Steer( q, q_near, n, epsilon, q_new, w );

	/* Test whether the node q_new violates the constraints.  It is implicitly assumed by this algorithm that, provided epsilon is small enough,
		we can take the incremental segment between q_near and q_new as safe given that their endpoints are safe. */
	int obs_num = NULL;
	if ( ConstraintViolation( q_new, n, obs, G, DH, indicator, &obs_num ) == SAFE ) {
		if ( DistSq(q,q_new,n,w) <= pow(epsilon,2) ) {
			return 2;		/* Status = "Reached" */
		}
		else {
			return 1;		/* Status = "Advanced" */
		}
	}
	else {
		return 0;			/* Status = "Trapped" */
	}
}

int Connect(double* q, double* q_near, double epsilon, int n, double* w, double* q_new, struct obstacles* obs, struct geom *G, struct DHparams *DH, double* cost_to_go, int indicator) {
	
	/* Copy the original q_near to a temporary placeholder (so q_near is not overwritten after each call to Extend). */
	double* q_nearest = (double*) malloc( n*sizeof(double) );
	for (int i = 0; i < n; i++) q_nearest[i] = q_near[i];

	/* Attempt to extend all the way from q_near to q by repeatedly calling Extend until q is reached or a collision occurs.  The cost_to_go
		is set within the Extend algorithm, in which the cost_to_go is incremented with every output from Steer() */
	int S;
	do {
		S = Extend(q, q_nearest, epsilon, n, w, q_new, obs, G, DH, cost_to_go, indicator);

		/* Reset q_nearest as the "closest" node to q so that we may incrementally test the safety of the entire line segment from the original q_near to q */
		for (int i = 0; i < n; i++) {
			q_nearest[i] = q_new[i];
		}
	} while (S == 1);

	free( q_nearest );
	return S;		/* Return the status of Connect, which must be either "Trapped" (0) or "Reached" (2) */
}

void InsertNode( struct tree* T, int n, int* num_nodes, double* q_new, int parent_index, double cost_to_go, 
	int connection, list_node* leaf_list, int safety, char* NN_alg, Engine* matlab ) {

	/* Allocate new memory */
	T->nodes		= (double**)			realloc(T->nodes,		(*num_nodes+1)*sizeof(double*));
	T->costs		= (double*)				realloc(T->costs,		(*num_nodes+1)*sizeof(double));
	T->parents		= (int*)				realloc(T->parents,		(*num_nodes+1)*sizeof(int));
	T->connections	= (int*)				realloc(T->connections, (*num_nodes+1)*sizeof(int));
	T->safety		= (int*)				realloc(T->safety,		(*num_nodes+1)*sizeof(int));	
	T->leaf_lists	= (struct list_node**)	realloc(T->leaf_lists,	(*num_nodes+1)*sizeof(struct list_node*));
	T->nodes[*num_nodes] = (double*) malloc(n*sizeof(double));

	/* Assign node data to the new fields of T */
	for (int i = 0; i < n; i++) {
		T->nodes[*num_nodes][i]	= q_new[i];
	}
	T->parents[*num_nodes]		= parent_index;
	T->costs[*num_nodes]		= cost_to_go;
	T->connections[*num_nodes]	= connection;
	T->leaf_lists[*num_nodes]	= leaf_list;
	T->safety[*num_nodes]		= safety;

	/* If using a kd-tree nearest neighbor algorithm, insert node information into the kd-tree data structure */
	if ( strncmp( NN_alg, "kd_tree", 7) == 0 ) {
		T->indices					= (int*) realloc(T->indices, (*num_nodes+1)*sizeof(int));
		T->indices[*num_nodes]		= *num_nodes;
		assert( kd_insert( T->kd_tree, T->nodes[*num_nodes], const_cast<int*>(&(T->indices[*num_nodes])) ) == NULL );	// kdtree Code
	}

	/* If connected to the MATLAB Engine, plot the newly added node and the edge to its parent */
	PlotEdgeInMATLAB( T, n, *num_nodes, matlab );

	/* Increment the number of nodes in the tree */
	*num_nodes += 1;
}

void ReWire( int feedback_mode, struct tree *T, int n, double* w, int rewire_node_index, int* neighbors, double* costs_to_neighbors, int n_neighbors, double epsilon, 
	struct obstacles* obs, struct geom *G, struct DHparams *DH, int indicator, Engine* matlab ) {

	int original_parent, former_ancestor, new_ancestor;
	double cost_to_go_temp	= 0.0;
	double* q_near_temp		= (double*) malloc(n*sizeof(double));
	double* q_new_temp		= (double*) malloc(n*sizeof(double));

	/* Send the rewire node to MATLAB (the following MATLAB code assumes "tree" is unchanged from its setting in the just-called InsertNode). */
	if (matlab != NULL) {
		assert( SendArraysToMATLAB( __LINE__, matlab, 1, "q_rewire", Double,1, 1,1,n, T->nodes[rewire_node_index] )  == EXIT_SUCCESS );		
	}

	/* Scan through the nearest neighbors and for any that would have a better path leading from the rewire node rather than their parent, update them.
	Starts at index j = 1 in order to skip the first value, which should always be the just-added node of tree T and therefore should not be rewired
	(keep it though, so that replanning during feedback will attempt to connect to its original path if possible). */
	for (int j = 1; j < n_neighbors; j++) {
		
		if ( T->costs[rewire_node_index] + costs_to_neighbors[j] < T->costs[ neighbors[j] ] ) {
			
			/* Create a temporary copy of the neighbor node (q_near_temp will be altered during Connect, hence cannot use neighbor node directly). */
			for (int i = 0; i < n; i++) {
				q_near_temp[i] = T->nodes[ neighbors[j] ][i];
			}

			if (Connect( T->nodes[rewire_node_index], q_near_temp, epsilon, n, w, q_new_temp, obs, G, DH, &cost_to_go_temp, indicator) == 2) {
				original_parent				= T->parents[ neighbors[j] ];
				T->parents[ neighbors[j] ]	= rewire_node_index;
				T->costs[ neighbors[j] ]	= T->costs[rewire_node_index] + costs_to_neighbors[j];
				
				/* If the trees are new, no leaf lists have been generated yet.  If they're not, i.e. during feedback, then update ancestor leaf lists
				(up to and excluding the root, since its leaf list is invariant except under the addition of a new node to the tree) */
				if (feedback_mode == 1) {

					/* neighbors[j]'s former ancestors: new list = current list - neighbors[j]'s list [each is already reverse-sorted, Minkowski diff] */
					former_ancestor = original_parent;
					while ( former_ancestor != 0 ) {
						SetDiffLeafListBfromA( T, former_ancestor, neighbors[j] );
						former_ancestor = T->parents[ former_ancestor ];
					}

					/* neighbors[j]'s new ancestors:	new list = current list + neighbors[j]'s list [each is already reverse-sorted, Union] */
					new_ancestor	= rewire_node_index;
					while ( new_ancestor != 0 ) {
						MergeLeafListBwithA( T, new_ancestor, neighbors[j] );
						new_ancestor = T->parents[ new_ancestor ];
					}
					
					/* Also reset safety to 1 since the new wiring may make the new path valid again (this will be reset later if found to be otherwise) */
					T->safety[ neighbors[j] ] = 1;
				}

				/* Delete the old edge in the MATLAB RRT construction figure and add in the newly rewired one instead. */
				PlotRewiringInMATLAB( T, n, neighbors[j], matlab );
			}
		}
	}

	free(q_new_temp); free(q_near_temp);
}

void AddLeafToLists(struct tree *T, int leaf_index) {
	int a	= T->parents[leaf_index];
	while (a != -1) {
		T->leaf_lists[a] = AddListElement(T->leaf_lists[a], leaf_index);
		a			= T->parents[a];
	}
}

int BuildRRTs(struct tree *Ta, struct tree *Tb, int* n_nodesA, int* n_nodesB, int n, double* w, int* iter, int max_iter, char* soln, char* sampling, 
	double** Q, double* q_max, double* q_min, char* NN_alg, int max_neighbors, double eta_RRT, double gamma_RRT, double epsilon, int obs_indicator,
	struct obstacles* obs, struct geom *G, struct DHparams *DH, int* node_star_index, char load_trees, char* filename, int I, fpos_t *fpos, Engine* matlab ) {

	/* 
	Input:	Ta				- pointer to the forward tree
			Tb				- pointer to the reverse tree
			n_nodesA		- pointer to the number of nodes in tree A
			n_nodesB		- pointer to the number of nodes in tree B
			n				- dimension of the configuration space (C-space), i.e. the number of elements in each node of each tree
			w				- array of distance weights for the weighted distance-squared cost function
			iter			- current iteration value (used to continue sampling unique points during closed-loop feedback)
			max_iter		- maximum number of iterations to use for pre-computation (open-loop, before executing motion plans)
			soln			- user specification of the type of solution to seek during pre-computation (closed-loop feedback exits after one pass): 
										"feasible"		(exit on first feasible path found)
										"suboptimal"	(continue to improve solutions up through "max_iter" iterations)
			sampling		- user specification of the type of sampling sequence to use (pre-computation and closed-loop):
										"halton"		(recommended - a deterministic but provably optimal sampling sequence)
										"pseudorandom"	(relies on the default C pseudorandom number generator)
			Q				- pointer to the list of samples pre-computed for the initial open-loop tree generation (pre-computation only)
			q_max			- upper bounds on each of the joint angles:		q1, q2, ..., qn
			q_min			- lower bounds on each of the joint angles:		q1, q2, ..., qn
			NN_alg			- user specification of the Nearest-Neighbor algorithm to use (pre-computation and closed-loop):
										"kd_tree"		(currently unsupported, as issues appear to exist with the kdtree library)
										"brute_force"	(brute force search over all nodes, selecting the node of lowest Euclidean distance from the target)
			max_neighbors	- maximum number of neighbors to use when attempting to rewire the trees AND when searching for new safe paths (see FindSafePaths)
			eta_RRT			- maximum radius within which at least one node must always exist, for kd_tree fixed-radius search ( computed in main() )
			gamma_RRT		- decay coefficient used for the kd_tree fixed-radius search ( computed in main() )
										See "Sampling-Based Algorithms for Optimal Motion Planning" by Karaman and Frazzoli
			epsilon			- maximum error allowed for termination of Steer() and convergence during closed-loop control ( see main() )
			obs_indicator	- integer which indicates which types of constraints to check during RRT construction:
										0		Check static obstacles only (e.g., for pre-computation)
										1		Check dynamic (temperature) obstacles only (useful for testing node safety after adding a new temp zone)
										2		Check both static and dynamic obstacles (useful for feedback and replanning from an untested node)
			obs				- obstacles structure containing all necessary information about static obstacles (cylinders, planes, and cuboids*)
								and dynamic obstacles (during closed-loop motion) (temperature obstacle cones, the cuboid object stored in cuboids)
			G				- geometry structure containing all necessary information about body-fixed coordinates used to represent the manipulator
			DH				- DH parameters structure containing manipulator joint and link information (used to form transformation matrices between frames)
			node_star_index - pointer to the "leaf" nodes in each tree through which the most-optimal path passes:
										node_star_index[0] = best leaf node in Ta (lowest cost-to-go path from the root)
										node_star_index[1] = best leaf node in Tb (lowest cost-to-come from the node to the root)
			load_trees		- character indicating whether to load previously computed trees ('y') or generate new ones ('n').  Set to 'n' during motion execution.
			filename		- filename root to which the following file extension is appended: "trees.dat", which contains all tree information.
								This data file is used to load trees from previous runs and thus should NOT be modified.
			I				- index of the current motion plan for which this function was called; used at the end of the first plan to delete a previous copy
								of "trees.dat" if it already exists and to title the plans written to file.
			fpos			- pointer to file position; used to append motion plans to "trees.dat" where the last iteration left off
			matlab			- pointer to MATLAB Engine, which generates an RRT construction figure if "disp_matlab_plots" = 'a' for "all".  Very slow but
								useful for debugging in order to verify the correctness of the algorithm

	Output:		- Constructed trees, which may be accessed by the pointers Ta and Tb
				- The number of nodes contained in each tree, indicated by the pointers n_nodesA and n_nodesB
				- The current sampling iteration, given by the pointer "iter"
				- The RRT tree construction figure, if enabled by the call to BuildRRTs, and located in the MATLAB handle RRTfig
				- The integer "feasible" which indicates whether any feasible solution has been found (1 on success, 0 on failure)
	*/

	int num_nodesA = *n_nodesA, num_nodesB = *n_nodesB, num_nodes[2] = {num_nodesA, num_nodesB}, n_neighbors = 0;
	int feasible = 0, temp_int, q_near_index;
	double cost_to_go = DBL_MAX, J_star = DBL_MAX;
	struct tree *temp_tree_ptr, *trees[2] = {Ta, Tb};
	FILE *treefile;

	int *neighbors  = (int*) malloc(max_neighbors*sizeof(int));
	double *costs   = (double*) malloc(max_neighbors*sizeof(double));
	double *q		= (double*) malloc(n*sizeof(double));
	double *q_near	= (double*) malloc(n*sizeof(double));
	double *q_new   = (double*) malloc(n*sizeof(double));
	assert( neighbors != NULL && costs != NULL && q != NULL && q_near != NULL && q_new != NULL);

	/* Identify if the trees to-be-built are new based on the current value of "iter".  Determines whether currently executing plans or not. */
	int feedback_mode	= 0;
	if (*iter > 1) feedback_mode = 1;

	/* Set the RRTs to search for nearest-neighbors by the locally greedy choice of closest Euclidean neighbors */
	int cost_type = 2;

	/* If connected to the MATLAB Engine, initialize a plot for the RRT's. */
	if ( (matlab != NULL) && (feedback_mode == 0) ) {
		assert( SendArraysToMATLAB( __LINE__, matlab, 8,	"I", Int,0, 1,1,1, I+1,		"n", Int,0, 1,1,1, n,		"epsilon", Double,0, 1,1,1, epsilon,	
			"max_iter", Int,0, 1,1,1, max_iter,		"q_min", Double,1, 1,1,n, q_min,		"q_max", Double,1, 1,1,n, q_max,
			"q_init", Double,1, 1,1,n, Ta->nodes[0],		"q_goal", Double,1, 1,1,n, Tb->nodes[0] )  == EXIT_SUCCESS );
		engEvalString( matlab, "scrsz = get(0,'ScreenSize'); figdims = [0.5, 0.75];		fontsize = 12;		titlesize = 14;								\
			RRTfig(I) = figure('OuterPosition', [(0.75-figdims(1)/2)*scrsz(3), (1-figdims(2))*scrsz(4), figdims(1)*scrsz(3), figdims(2)*scrsz(4)]); 	\
			set(gca,'FontSize', fontsize);	hold on; box on; grid on; plothandles = cell(2, max_iter+1);												\
			sample_format = 'om';			nearest_format = 'sk';			nearest_size = 10;			nearest_linewidth = 1.5;						\
			edge_format = {'-b','-r'};		node_format = {'vb','.r'};		node_color = {'none','none'};		node_size = [6, 6];						\
			title({['RRT Tree Development for Motion Plan #', num2str(I), ', n = ', num2str(n), ', epsilon = ', num2str(epsilon), ' deg'];				\
				['(Note some dimensions may be hidden)']}, 'FontSize', titlesize);"	);
		if (n >= 3) {
			engEvalString(matlab, "xlabel('q_1 (deg)'); ylabel('q_2 (deg)'); zlabel('q_3 (deg)'); axis([q_min(1), q_max(1), q_min(2), q_max(2), q_min(3), q_max(3)]); view(3); \
				figure(RRTfig(I)); plot3(q_init(1), q_init(2), q_init(3), 'og', q_goal(1), q_goal(2), q_goal(3), 'or', 'MarkerSize', 6, 'LineWidth', 2 );"	);
		} else if (n == 2) {
			engEvalString(matlab, "xlabel('q_1 (deg)'); ylabel('q_2 (deg)'); axis([q_min(1), q_max(1), q_min(2), q_max(2)]);		\
				figure(RRTfig(I)); plot(q_init(1), q_init(2), 'og', q_goal(1), q_goal(2), 'or', 'MarkerSize', 6, 'LineWidth', 2 );");
		} else {
			engEvalString(matlab, "xlabel('q_1 (deg)'); xlim([q_min(1), q_max(1)]);								\
				figure(RRTfig(I)); plot(q_init, 0, 'og', q_goal, 0, 'or', 'MarkerSize', 6, 'LineWidth', 2 );"	);
		}
	}

	/* Begin tree construction.  If in the pre-computation phase (feedback_mode = false) and soln = "suboptimal", attempt to build up to "max_iter" samples
		per tree.  If in feedback, enter the loop but exit after one iteration so that checks on the temperature sensors can be made. */
	if (strncmp(&load_trees, "y", 1) != 0) {
		while ( (*iter <= max_iter) || (feedback_mode == 1) ) {
			/* Determine next sample */
			Sample( feedback_mode, sampling, n, Q, q_max, q_min, iter, q );

			/* Determine the nearest node in the forward tree (Ta) to q and attempt to extend towards it, yielding q_new */
			cost_to_go = 0.0;
			Nearest(Ta, num_nodesA, q, n, w, cost_type, &q_near_index, q_near, NN_alg, matlab);
			
			if (Connect(q, q_near, epsilon, n, w, q_new, obs, G, DH, &cost_to_go, obs_indicator) != 0) {
				/* Tree Ta successfully extended to or reached q_near, returning q_new = Ta.nodes[ neighbors[0] ].  Add this node to Ta. */
				InsertNode(Ta, n, &num_nodesA, q_new, q_near_index, Ta->costs[q_near_index] + cost_to_go, -1, NULL, 1, NN_alg, matlab);

				/* If successful, re-wire the forward tree to the newly added node q_new = Ta->nodes[num_nodesA-1] */
				NearestNeighbors(Ta, num_nodesA, Ta->nodes[num_nodesA-1], n, w, max_neighbors, cost_type, eta_RRT, gamma_RRT, neighbors, costs, &n_neighbors, NN_alg);
				ReWire( feedback_mode, Ta, n, w, num_nodesA-1, neighbors, costs, n_neighbors, epsilon, obs, G, DH, 0, matlab );

				/* Determine the nearest node in the reverse tree (Tb) to the newly added node in Ta and attempt to connect to it, yielding q_new */
				cost_to_go = 0.0;
				Nearest(Tb, num_nodesB, Ta->nodes[num_nodesA-1], n, w, cost_type, &q_near_index, q_near, NN_alg, matlab);

				if (Connect(Ta->nodes[num_nodesA-1], q_near, epsilon, n, w, q_new, obs, G, DH, &cost_to_go, obs_indicator) == 2) {
					/* Both trees connected to the same node (within epsilon)! Add this node to Tb */
					InsertNode(Tb, n, &num_nodesB, q_new, q_near_index, Tb->costs[q_near_index] + cost_to_go, -1, NULL, 1, NN_alg, matlab);

					/* If successful, re-wire the reverse tree to the newly added node q_new = Tb->nodes[num_nodesB] */
					NearestNeighbors(Tb, num_nodesB, Tb->nodes[num_nodesB-1], n, w, max_neighbors, cost_type, eta_RRT, gamma_RRT, neighbors, costs, &n_neighbors, NN_alg);
					ReWire( feedback_mode, Tb, n, w, num_nodesB-1, neighbors, costs, n_neighbors, epsilon, obs, G, DH, 0, matlab );

					/* For the connected leaf nodes, save the index of its respective leaf in the other tree.  Set feasible to 1 and increment
					the total number of nodes in each tree.  */
					Ta->connections[num_nodesA-1] = num_nodesB-1;
					Tb->connections[num_nodesB-1] = num_nodesA-1;
					feasible = 1;

					/* If executing feedback and the trees are not new, update the leaf lists with the newly-added leaf nodes prior to exiting the function */
					if (feedback_mode == 1) {
						AddLeafToLists(Ta, num_nodesA-1);
						AddLeafToLists(Tb, num_nodesB-1);
					}
				}
			}

			/* Increment "iter". If in feedback mode, exit the tree construction phase after a single iteration. */
			*iter = *iter + 1;
			if (feedback_mode == 1) {
				num_nodes[0]	= num_nodesA;
				num_nodes[1]	= num_nodesB;
				goto END_OF_FUNCTION;
			}

			/* Swap tree pointers to alternate between "extending" or "connecting" for each tree */
			temp_tree_ptr = Ta;			temp_int	= num_nodesA;
			Ta		= Tb;				num_nodesA	= num_nodesB;
			Tb		= temp_tree_ptr;	num_nodesB	= temp_int;

			/* Report every 10% of progress to the console window */
			if ( ((10*(*iter)) % max_iter) == 0 ) {
				printf("\t%02d%s\n", (100*(*iter))/max_iter, "% complete...");
			}

			/* Break the loop if a feasible solution has been found and only one is desired. */
			if ( (feasible == 1 && strncmp(soln, "feasible", 8) == 0) ) {
				break;
			}
		}

		/* If "iter" is even, an odd-number of loops was conducted (and therefore an odd number of swaps), and vice versa. If this is so, 
		Ta and Tb are now mixed-up from their original order.  Swap them back into place (to correspond with n_nodes and treefiles again). */
		if ((*iter % 2) == 0) {
			temp_tree_ptr = Ta;			temp_int	= num_nodesA;
			Ta		= Tb;				num_nodesA	= num_nodesB;
			Tb		= temp_tree_ptr;	num_nodesB	= temp_int;
		}

		/* For all parent nodes of connected leaves, add to their "leaf_lists" the indices of the leaves they are connected to.
		Thus, all nodes directly connected to the other tree have in their "leaf_list" the index of the node in the other 
		tree to which they are connected.  All nodes not directly connected to the other tree have in their "leaf_list" a list of
		all their descendent nodes that do directly connect to the other tree. */
		LOG(logINFO) << "Saving connected leaf nodes to leaf lists...";
		trees[0]	= Ta;		num_nodes[0]	= num_nodesA;
		trees[1]	= Tb;		num_nodes[1]	= num_nodesB;
		for (int k = 0; k < 2; k++) {
			for (int i = 1; i < num_nodes[k]; i++) {
				if (trees[k]->connections[i] != -1) {
					AddLeafToLists(trees[k], i);
				}
			}
		}

		/* Output the two trees to file */
		LOG(logINFO) << "Writing tree data to files...";
		if (I == 0) {
			remove( strcat(filename, "trees.dat") );
			filename[strlen(filename)-9] = NULL;
		}
		treefile = fopen( strcat(filename, "trees.dat"), "a+" );
		for (int k = 0; k < 2; k++) {
			if (k == 0) {
				fprintf(treefile, "%s%02d%s\n", "Motion Plan #", I+1, ": Forward Tree");
			}
			else {
				fprintf(treefile, "%s%02d%s\n", "Motion Plan #", I+1, ": Reverse Tree");
			}
			fprintf(treefile, "%s\t", "Node");
			for (int j = 0; j < n; j++) {
				fprintf(treefile, "%s%d\t", "q", j+1);
			}
			fprintf(treefile, "%s\t%s\t%s\t%s\t%s\n", "Parent", "Cost-to-Come", "Safety", "Connection", "Leaves");
			for (int i = 0; i < num_nodes[k]; i++) {
				fprintf(treefile, "%i\t", i);
				for (int j = 0; j < n; j++) {
					fprintf(treefile, "%5.5Lf\t", trees[k]->nodes[i][j]);
				}
				fprintf(treefile, "%i\t%5.5Lf\t%i\t%i", trees[k]->parents[i], trees[k]->costs[i], trees[k]->safety[i], trees[k]->connections[i]);
				PrintListToFile(treefile, "\t%i", trees[k]->leaf_lists[i]);
				fprintf(treefile, "%s", "\n");
			}
			fprintf(treefile, "%s", "\n");
		}
		fclose(treefile);
		filename[strlen(filename)-9] = NULL;
	}
	else if (strncmp(&load_trees, "y", 1) == 0) {
		int parent_index, safety, connection, index = 0, leaf_index, n_leaves, *leaf_indices = NULL, return_val = 1;
		list_node** leaf_list_ptr;
		char c;

		/* Load the two trees from file */
		LOG(logINFO) << "Loading tree data from files...";
		treefile = fopen(strcat(filename, "trees.dat"), "r");		/* Open tree file for reading */

		if(treefile != NULL)
		{
			if (I != 0) {
				fsetpos(treefile, fpos);
			}
			for (int k = 0; k < 2; k++) {
				num_nodes[k] = 0;
				fscanf(treefile, "%*[^\n]\n%*[^\n]\n");			/* Skip header lines */
				while ( (return_val != 0) && (return_val != EOF) ) {
					return_val			= fscanf(treefile, "%i", &index);
					for (int j = 0; j < n; j++) {
						return_val		= fscanf(treefile, "%Lf", &(q_new[j]) );
					}
					return_val			= fscanf(treefile, "%i%Lf%i%i%c", &(parent_index), &(cost_to_go), &(safety), &(connection), &c );

					n_leaves = 0;
					while (strncmp(&c, "\t", 1) == 0) {
						return_val		= fscanf(treefile, "%i%c", &leaf_index, &c);
						n_leaves		+= 1;
						leaf_indices				= (int*) realloc( leaf_indices, n_leaves*sizeof(int) );
						leaf_indices[n_leaves-1]	= leaf_index;
					}

					/* Leaves were read in reverse-sorted order, hence if they are pushed sequentially to the head of a new leaf list then the leaf list
					will be in forward-sorted order, which is not consistent with their original ordering.  Enter them into the list in reverse.
					(This ordering is important so that newly-added nodes during feedback maintain the ordering, and so merging and differencing lists operates correctly). */
					leaf_list_ptr		= (struct list_node**)	malloc((1)*sizeof(struct list_node*));
					*leaf_list_ptr		= NULL;
					for (int j = 0; j < n_leaves; j++) {
						*leaf_list_ptr	= AddListElement(*leaf_list_ptr, leaf_indices[(n_leaves-1)-j]);
					}

					/* With all of the info saved from the file, send to InsertNode to allocate memory for it in trees[k] and add in the node information */
					InsertNode( trees[k], n, &(num_nodes[k]), q_new, parent_index, cost_to_go, connection, *leaf_list_ptr, safety, NN_alg, NULL );
				}

				/* Now that the tree is fully-built, we can plot all edges (any nodes rewired to higher-index parents would cause errors if plotted during construction) */
				for (int j = 1; j < num_nodes[k]; j++) {
					PlotEdgeInMATLAB( trees[k], n, j, matlab );
				}

				/* Reset return_val to 1 for the next tree */
				return_val	= 1;
			}


			fgetpos(treefile, fpos);
			fclose(treefile);
		}

		else
		{
			LOG(logERROR) << "Could not open tree file.";
		}
		filename[strlen(filename)-9] = NULL;

		*iter = max_iter;  // max_iter, loaded from the previous simulation's "input.dat" file, contains the number of iterations used to generate its "trees.dat" file
		free(leaf_indices);
	}

	/* Determine the minimum-cost path among the two trees */
	for (int i = 0; i < num_nodes[0]; i++) {
		if ( (Ta->connections[i] != -1) && (Ta->costs[i] + Tb->costs[ Ta->connections[i] ] < J_star) ) {
			feasible			= 1;
			J_star				= Ta->costs[i] + Tb->costs[ Ta->connections[i] ];
			node_star_index[0]	= i;
			node_star_index[1]	= Ta->connections[i];
		}
	}

	END_OF_FUNCTION:
	/* Save the number of nodes in each tree and return the "feasible" boolean */
	*n_nodesA = num_nodes[0];
	*n_nodesB = num_nodes[1];

	free(neighbors); free(costs);
	free(q); free(q_near); free(q_new);
	
	return feasible;
}

int* TracePath(struct tree *T, int node1, int node2, int *pathlen) {
	int a		= node1;
	int* path	= NULL;
	int index	= 0;

	while ( (a != -1) && (a != T->parents[node2]) ) {
		path		= (int*) realloc (path, (index+1)*sizeof(int));
		path[index] = a;
		a			= T->parents[a];
		index		= index + 1;
	}
	*pathlen	= index;

	return path;
}

void StorePath(struct tree *Ta, struct tree *Tb, int n, int* pathA, int* pathB, int pathlenA, int pathlenB, char* filename, int I, double **path) {

	/* Save the nodes along pathA in the forward direction and concatenate those with pathB in the reverse direction */
	for (int i = 0; i < (pathlenA + pathlenB); i++) {
		if (i < pathlenA) {
			for (int j = 0; j < n; j++) {
				path[i][j] = Ta->nodes[ pathA[(pathlenA-1)-i] ][j];
			}
		}
		else {
			for (int j = 0; j < n; j++) {
				path[i][j] = Tb->nodes[ pathB[i-pathlenA] ][j];
			}
		}
	}

	/* Save results to file if the index, I, is not given as -1 (prevents infinite re-writes during motion plan execution) */
	if (I != -1) {
		FILE *datafile;
		if (I == 0) {
			remove( strcat(filename, "output.dat") );
			filename[strlen(filename)-10] = NULL;
		}
		datafile = fopen( strcat(filename, "output.dat"), "a+");
		fprintf(datafile, "%s%02i%s\n", "Motion Plan #", I+1, ":");
		fprintf(datafile, "%s\t", "Node");
		for (int j = 0; j < n; j++) {
			fprintf(datafile, "%s%d\t", "q", j+1);
		}
		fprintf(datafile, "%s", "\n");

		for (int i = 0; i < (pathlenA + pathlenB); i++) {
			fprintf(datafile, "%i\t", i);
			for (int j = 0; j < n; j++) {
				fprintf(datafile, "% 5.6Lf\t", path[i][j]);
			}
			fprintf(datafile, "%s", "\n");
		}
		fprintf(datafile, "%s", "\n");
		fclose(datafile);
		filename[strlen(filename)-10] = NULL;
	}
}

void SplitPathAtNode( double** path, double* q, double *w, int* pathA, int* pathB, int pathlenA, int index, struct tree** T_ptrs, int* n_nodes, int n, char* NN_alg, Engine* matlab ) {
	
	double* q_prev_node = path[index-1];
	double* q_next_node = path[index];
	double cost_to_come, cost_to_go;
	int prev_node, new_node, next_node;

	if ( (index > 0) && (index < pathlenA) ) {
		/* Add q to Ta: parent = previous node index
						cost = cost of previous node + cost from previous
						copy the leaves of the next node as its leaves */
		prev_node	= pathA[ (pathlenA-1)-(index-1) ];
		new_node	= n_nodes[0];
		next_node	= pathA[ (pathlenA-1)-(index) ];
		cost_to_come = Sqrt( DistSq(q, q_prev_node, n, w) );

		/* Insert q.  Cost of descendent nodes is unchanged, provided q is on the line segment between q_prev and q_next.  Since q has no leaves
		already, there is no need to merge its leaf list with its new ancestors. */
		InsertNode( T_ptrs[0], n, &(n_nodes[0]), q, prev_node, T_ptrs[0]->costs[prev_node] + cost_to_come, -1, NULL, 1, NN_alg, matlab );
		MergeLeafListBwithA( T_ptrs[0], new_node, next_node );
		T_ptrs[0]->parents[ next_node ] = new_node;  
	}
	else if (index >= pathlenA) {
		/* Add q to Tb: parent = next node index, 
						cost = cost of next node + cost to next node
						no leaves */
		new_node	= n_nodes[1];
		next_node	= pathB[ index - pathlenA ];
		cost_to_go  = Sqrt( DistSq(q, q_next_node, n, w) );

		/* Insert q.  Cost of descendent nodes is unchanged, provided q is on the line segment between q_prev and q_next.  Since q has no leaves
		already, there is no need to merge its leaf list with its new ancestors. */
		InsertNode( T_ptrs[1], n, &(n_nodes[1]), q, next_node, T_ptrs[1]->costs[next_node] + cost_to_go, -1, NULL, 1, NN_alg, matlab );
		if (index > pathlenA) {
			prev_node	= pathB[ index - pathlenA - 1 ];
			MergeLeafListBwithA( T_ptrs[1], new_node, prev_node );
			T_ptrs[1]->parents[ prev_node ] = new_node;
		}
	}
}

void SaveBestPlans(int *num_replans, int max_replans, double cost_to_go, int tree_index, int newpath_start_node, int newpath_end_node, double *replan_costs, int** replan_indices ) {
	
	int *I	= (int*) malloc(max_replans*sizeof(int));

	if (*num_replans < max_replans) {
		replan_costs[*num_replans]		= cost_to_go;
		replan_indices[*num_replans][0]	= tree_index;			// Save as the first index the type of tree (0 = forward tree, 1 = reverse tree)
		replan_indices[*num_replans][1]	= newpath_start_node;	// Save as the second index the starting node of the path to follow (depends on type of tree)
		replan_indices[*num_replans][2]	= newpath_end_node;		// Save as the third index the ending node (=leaf for forward tree, =root for reverse tree)
		*num_replans += 1;

		if ( *num_replans == max_replans ) {

			/* Sort the list of replan costs, which may be in any order, using the best-suited sort. */
			if ( max_replans < SORT_SWITCH )	InsertionSort( replan_costs, max_replans, I );
			else								MergeSort( replan_costs, max_replans, I );
			RearrangeIntPtrVector( replan_indices, max_replans, I );
		}
	}
	else {
		if (cost_to_go < replan_costs[ max_replans-1 ]) {
			replan_costs[max_replans-1]			= cost_to_go;
			replan_indices[max_replans-1][0]	= tree_index;
			replan_indices[max_replans-1][1]	= newpath_start_node;
			replan_indices[max_replans-1][2]	= newpath_end_node;
			
			/* InsertionSort (O(n) time) the almost-sorted list so that the last element is in its proper position. */
			InsertionSort( replan_costs, max_replans, I );
			RearrangeIntPtrVector( replan_indices, max_replans, I );
		}
	}

	free(I);
}

int RePlan(struct tree **T_ptrs, int* n_nodes, double* q, int n, double* w, int max_replans, double max_replan_neighbors, double eta_RRT, double gamma_RRT, int** replan_indices, char* NN_alg) {
	
	double cost_to_go;
	int num_replans = 0, n_neighbors = 0, cost_type = 2, n_neighborsA, n_neighborsB;
	n_neighborsA = (int) ceil( (max_replan_neighbors/100)*n_nodes[0]);
	n_neighborsB = (int) ceil( (max_replan_neighbors/100)*n_nodes[1]);

	struct list_node *node_ptr;
	double *replan_costs	= (double*) malloc(max_replans*sizeof(double));
	int *neighbors			= (int*) malloc( max(n_neighborsA, n_neighborsB)*sizeof(int));
	double *costs			= (double*) malloc( max(n_neighborsA, n_neighborsB)*sizeof(double));
	int *I					= (int*) malloc(max_replans*sizeof(int));

	// Cost-Priority Search
	/* Find minimum-cost paths back to the forward tree.  Search the nearest neighbors in the forward tree from q, obtaining the number of candidates
	n_neighbors, the cost to connect to each one, costs, and the location of each, neighbors.  Search according to NN_alg and the cost_type above. */
	NearestNeighbors( T_ptrs[0], n_nodes[0], q, n, w, n_neighborsA, cost_type, eta_RRT, gamma_RRT, neighbors, costs, &n_neighbors, NN_alg );
	for (int k = 0; k < n_neighbors; k++) {

		/* If the nearest neighbor node is safe, attempt to save the paths to which it connects */
		if ( T_ptrs[0]->safety[ neighbors[k] ] == 1 ) {

			/* If the nearest neighbor is a parent node, find best paths among its safe leaves by storing a list of up to "max_replans" number of
			possible connections in "replan_indices" and repeatedly sorting them so that the neighbors are listed in best order according to "replan_costs" */
			node_ptr = T_ptrs[0]->leaf_lists[ neighbors[k] ];
			while ( node_ptr != NULL ) {
				if ( T_ptrs[0]->safety[ node_ptr->data ] == 1 ) {
					cost_to_go = costs[k] + ( T_ptrs[0]->costs[ node_ptr->data ] - T_ptrs[0]->costs[ neighbors[k] ] ) + T_ptrs[1]->costs[ T_ptrs[0]->connections[ node_ptr->data ] ];
					SaveBestPlans(&num_replans, max_replans, cost_to_go, 0, neighbors[k], node_ptr->data, replan_costs, replan_indices );
				}
				node_ptr = node_ptr->next;
			}

			/* If the nearest neighbor is also a leaf node, attempt to save it as a best path */
			if ( T_ptrs[0]->connections[ neighbors[k] ] != -1 ) {		
				cost_to_go = costs[k] + T_ptrs[1]->costs[ T_ptrs[0]->connections[ neighbors[k] ] ];
				SaveBestPlans(&num_replans, max_replans, cost_to_go, 0, neighbors[k], neighbors[k], replan_costs, replan_indices );
			}
		}
	}

	/* Find minimum-cost paths back to the reverse tree */
	NearestNeighbors( T_ptrs[1], n_nodes[1], q, n, w, n_neighborsB, cost_type, eta_RRT, gamma_RRT, neighbors, costs, &n_neighbors, NN_alg );
	for (int k = 0; k < n_neighbors; k++) {
		if ( T_ptrs[1]->safety[ neighbors[k] ] == 1 ) {
			cost_to_go = costs[k] + T_ptrs[1]->costs[ neighbors[k] ];
			SaveBestPlans(&num_replans, max_replans, cost_to_go, 1, 0, neighbors[k], replan_costs, replan_indices );
		}
	}

	/* In case fewer than "max_replans" number of replans was available, sort the unsorted costs and indices prior to output */
	if (num_replans < max_replans) {
		if ( num_replans < SORT_SWITCH )	InsertionSort( replan_costs, num_replans, I );
		else								MergeSort( replan_costs, num_replans, I );
		RearrangeIntPtrVector( replan_indices, num_replans, I );
	}

	free(replan_costs); free(neighbors); free(costs); free(I);
	return num_replans;
}

int FindSafePath(struct tree **T_ptrs, int** replan_indices, int num_replans, int* pathlenA, int* pathlenB, int** pathA, int** pathB,
	double*** path, double* q, double epsilon, int n, double* w, struct obstacles* obs, struct geom *G, struct DHparams *DH, char* filename) {

	int k = 0, unsafe = 0;
	double cost_to_go, *q_temp = (double*) malloc(n*sizeof(double)), *q_new_temp = (double*) malloc(n*sizeof(double));

	while ( k < num_replans ) {
		/* Determine the path corresponding to plan k, resetting pathA, pathB, pathlenA, and pathlenB */
		for (int k1 = *pathlenA + *pathlenB - 1; k1 >= 0; k1--) {
			free(path[0][k1]);
		}
		free(path[0]); free(pathA[0]); free(pathB[0]);

		if (replan_indices[k][0] == 0) {
			pathA[0]	= TracePath( T_ptrs[0], replan_indices[k][2], replan_indices[k][1], pathlenA );
			pathB[0]	= TracePath( T_ptrs[1], T_ptrs[0]->connections[ replan_indices[k][2] ], 0, pathlenB );
		}
		else if (replan_indices[k][0] == 1) {
			pathA[0]	= TracePath( T_ptrs[0], -1, -1, pathlenA );
			pathB[0]	= TracePath( T_ptrs[1], replan_indices[k][2], replan_indices[k][1], pathlenB );
		}
		path[0] = Make2DDoubleArray( *pathlenA + *pathlenB, n );
		StorePath( T_ptrs[0], T_ptrs[1], n, pathA[0], pathB[0], *pathlenA, *pathlenB, filename, -1, path[0] );

		/* Assume the path is safe.  If found otherwise, repeat the loop with the next available plan or else exit with unsafe = 1. */
		unsafe = 0;

		/* Test forward tree path vertices for safety */ 
		for (int k1 = *pathlenA-1; k1 >= 0; k1--) {
			if ( T_ptrs[0]->safety[ pathA[0][k1] ] == 0 ) {
				unsafe = 1;
				goto END_OF_LOOP;
			}
		}

		/* If still safe, test reverse tree path vertices for safety. */
		for (int k2 = 0; k2 < *pathlenB; k2++) {
			if ( T_ptrs[1]->safety[ pathB[0][k2] ] == 0 ) {
				unsafe = 1;
				goto END_OF_LOOP;
			}
		}

		/* If still safe, test entire path (including connections from q to start of path and in between path vertices). */
		//memcpy( q_temp, q, n*sizeof(q[0]) );
		if ( Connect( path[0][0], q, epsilon, n, w, q_new_temp, obs, G, DH, &cost_to_go, 2 ) == 2 ) {		
			for ( int k3 = 0; k3 < *pathlenA + *pathlenB - 1; k3++ ) {
				//memcpy( q_temp, path[0][k3], n*sizeof(q[0]) );
				if ( Connect( path[0][k3+1], path[0][k3], epsilon, n, w, q_new_temp, obs, G, DH, &cost_to_go, 1 ) != 2 ) {
					unsafe = 1;
					if ( k3+1 < *pathlenA ) {
						T_ptrs[0]->safety[ pathA[0][(*pathlenA-1) - (k3+1)] ] = 0;	// Since only one edge leads into path[0][k3+1] in treeA, mark this node as unsafe
					} else if ( k3 >= *pathlenA ) {
						T_ptrs[1]->safety[ pathB[0][k3 - *pathlenA] ] = 0;			// Since only one edge emanates from path[0][k3] in treeB, mark this node as unsafe
					}
					goto END_OF_LOOP;
				}
			}
		}
		else {
			unsafe = 1;
			goto END_OF_LOOP;
		}

		/* If unsafe is still 0, the current plan is safe.  Exit the loop and return unsafe = 0; otherwise keep searching. */
		if (unsafe == 0) break;

		END_OF_LOOP:
		k = k + 1;
	}

	free(q_temp); free(q_new_temp);
	return unsafe;
}

int ExhaustiveRePlan(struct tree **T_ptrs, int* n_nodes, double eta_RRT, double gamma_RRT, char* NN_alg, int* pathlenA, int* pathlenB, int** pathA, int** pathB,
	double*** path, double* q, double epsilon, int n, double* w, struct obstacles* obs, struct geom *G, struct DHparams *DH, char* filename) {

		int n_neighbors = 0, cost_type = 2, unsafe = 1, k;
		int *neighbors	= (int*) malloc( (max(n_nodes[0], n_nodes[1]))*sizeof(int) );
		double *costs	= (double*) malloc( (max(n_nodes[0], n_nodes[1]))*sizeof(double) );
		int **replan_indices	= Make2DIntArray(1,3);
		struct list_node* node_ptr	= NULL;

		// Feasibility-Priority Search
		/* In the case that the cost-priority search from RePlan was unsuccessful, conduct an exhaustive search with focus not on costs but on
		feasibility. First search for feasible nodes over each tree, then accept the first feasible solution found. */
		for (int t = 1; t >= 0; t--) {
			NearestNeighbors( T_ptrs[t], n_nodes[t], q, n, w, n_nodes[t], cost_type, eta_RRT, gamma_RRT, neighbors, costs, &n_neighbors, NN_alg );

			k = 0;
			while (unsafe == 1 && k < n_neighbors) {
				if ( T_ptrs[t]->safety[ neighbors[k] ] == 1 ) {

					node_ptr = T_ptrs[t]->leaf_lists[ neighbors[k] ];
					while ( node_ptr != NULL && unsafe == 1 ) {
						if ( T_ptrs[t]->safety[ node_ptr->data ] == 1 ) {
							if (t == 0) {
								replan_indices[0][0] = 0;
								replan_indices[0][1] = neighbors[k];
								replan_indices[0][2] = node_ptr->data;
							} else {
								replan_indices[0][0] = 1;
								replan_indices[0][1] = 0;
								replan_indices[0][2] = neighbors[k];
							}
							unsafe = FindSafePath( T_ptrs, replan_indices, 1, pathlenA, pathlenB, pathA, pathB, path, q, epsilon, n, w, obs, G, DH, filename );
						}
						node_ptr = node_ptr->next;
					}
				}

				k = k + 1;
			}

			if ( unsafe == 0 ) {
				break;
			}
		}

		free(replan_indices[0]); free(replan_indices);
		free(neighbors); free(costs);
		return unsafe;
}


void EmergencyPlan( double* q, double* q_emergency, double* q_waypoints, int n, int n_emergency, double* w,
	double epsilon_sq, CPhidgetAdvancedServoHandle servo, int* channels, int plan_num ) {

	double minDist = DBL_MAX, dist;
	int minDist_index = -1, increment = FORWARD;

	for (int i = 0; i < n_emergency; i++) {
		dist = DistSq(q, &q_emergency[i*n], n, w);
		if (dist < minDist) {
			minDist = dist;
			minDist_index = i;
		}
	}

	if (plan_num == 0) {
		q_emergency = &(q_waypoints[n]);
		n_emergency = 1;
		minDist_index = 0;
	} else if (plan_num == 1) {
		increment = REVERSE;
	} else {
		increment = FORWARD;
	}

	if( minDist_index >= 0) {
		for (int j = minDist_index; (j >= 0 && j < n_emergency); j += increment ) {
			for (int k = 0; k < n; k++) {
				CPhidgetAdvancedServo_setPosition(servo, channels[k], Deg2Command((&q_emergency[n*j])[k]));
			}
			do {
				for (int k = 0; k < n; k++) {
					CPhidgetAdvancedServo_getPosition(servo, channels[k], &(q[k]));
					q[k] = Command2Deg(q[k]);
				}
			} while (DistSq(q, &q_emergency[n*j], n, w) > epsilon_sq);
		}
	}
	
}

void ResetSimulation( char reset_trees, struct tree* trees, int n_trees, int n_plans, int* num_nodes, int* num_nodes_precomputed,
	int* feasible, int** node_star_index, struct obstacles* obs, Engine* matlab ) {
	
	/* If requesting a tree reset after each run, return to the original trees by truncating the number of nodes to their original values */
	if (strncmp(&reset_trees, "y", 1) == 0) {
		for (int i = 0; i < n_trees; i++) num_nodes[i] = num_nodes_precomputed[i];
	}
	
	/* Reset node safety properties back to 1 and remove any temperature obstacles from the previous run */
	obs->n_temp_zones = 0;
	for (int i = 0; i < n_trees; i++) {
		for (int j = 0; j < num_nodes[i]; j++) {
			trees[i].safety[j] = 1;
		}
	}

	/* Determine the new best plan in the trees due to the introduction of nodes during the last simulation */
	double J_star = DBL_MAX;
	for (int i = 0; i < n_plans; i++) {
		for (int j = 0; j < num_nodes[2*i]; j++) {
			if ( ( trees[2*i].connections[j] != -1) && ( trees[2*i].costs[j] + trees[2*i+1].costs[ trees[2*i].connections[j] ] < J_star) ) {
				feasible[i]				= 1;
				J_star					= trees[2*i].costs[j] + trees[2*i+1].costs[ trees[2*i].connections[j] ];
				node_star_index[i][0]	= j;
				node_star_index[i][1]	= trees[2*i].connections[j];
			}
		}
	}

	if ( matlab != NULL ) {
		engEvalString( matlab, "close( TRAJfig, PLANfig ); clear all;");
	}
}

void GenerateSamples(double** Q, char* sampling, int n, int max_iter, double* q_max, double* q_min, char* filename) {
	
	FILE *samplefid;
	char* sampleFile = strcat(filename, "samples.dat");
	samplefid = fopen( sampleFile, "w+");

	if(samplefid != 0)	// check if file could be opened
	{
		fprintf(samplefid, "%s\t", "Index");
		for (int j = 0; j < n; j++) {
			fprintf(samplefid, "%s%d\t", "q", j+1);
		}
		fprintf(samplefid, "%s", "\n");

		/* If "pseudorandom" is chosen, use rand() to generate joint angles between q_max and q_min */
		if ( strncmp(sampling, "pseudorandom", 10) == 0 ) {
			for (int i = 0; i < max_iter; i++) {
				fprintf(samplefid, "%i\t", i);
				for (int j = 0; j < n; j++) {
					Q[i][j] = ( rand() % (int) floor(q_max[j] - q_min[j]) ) + q_min[j];
					fprintf(samplefid, "% 5.6Lf\t", Q[i][j]);
				}
				fprintf(samplefid, "%s", "\n");
			}
		}
		/* If "halton" is chosen, use the Halton sequence to generate joint angles and scale to the appropriate range */
		int index = 0;
		if ( strncmp(sampling, "halton", 10) == 0 ) {
			for (int i = 0; i < max_iter; i++) {
				index = i + 1;
				Halton( &index, 1, n, &(Q[i]) );
				fprintf(samplefid, "%i\t", i);
				for (int j = 0; j < n; j++) {
					Q[i][j] = Q[i][j] * ( q_max[j] - q_min[j] ) + q_min[j];
					fprintf(samplefid, "% 5.6Lf\t", Q[i][j]);
				}
				fprintf(samplefid, "%s", "\n");
			}
		}
		fclose(samplefid);
	}
	else
	{
		LOG(logERROR) << "Could not open sample file ["<<sampleFile<<"]";
	}
	filename[strlen(filename)-11] = NULL;

	assert(Q != NULL);
}
