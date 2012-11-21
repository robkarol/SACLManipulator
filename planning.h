#ifndef __PLANNING_H__
#define __PLANNING_H__

#include "engine.h"
#include "linked_list.h"
#include "phidget_interface.h"

#define FORWARD 1
#define REVERSE -1

/*! Generates the specified values of the D-dimensional Halton sequence.  Computes the members of the Halton sequence corresponding to the elements of *sequence*,
returning a *length* \f$\times D\f$ array of `doubles` to array *h*.  Each row *m* corresponds to the \f$m^{th}\f$ element of *sequence*, while each column *n*
corresponds to the dimension, \f$n = 1,\ldots,D\f$.  See p.207 of "Planning Algorithms" by Steven LaValle. \see Dec2BaseInts, Sample, GenerateSamples
	\param[in]	sequence	Array of the desired members in the Halton sequence
	\param[in]	length		Number of elements in *sequence*
	\param[in]	D			Dimension of hypercube used for sequencing (requires \f$D \leq 32\f$)
	\param[in,out]	h		Pre-allocated array of size *length* \f$\times D\f$, populated and returned with Halton samples (by row) */
void Halton( int* sequence, int length, int D, double** h );

/*! Determines the next sample joint-angle vector to use for RRT construction.  Uses sample array *Q* during tree
pre-computation, or else the method specified by *sampling* during closed-loop motion plan execution. \see GenerateSamples, Halton, Extend, Connect
	\param[in]	feedback_mode	Boolean specifying whether or not tree-construction is taking place during feedback
	\param[in]	sampling		User specification of sampling method ("pseudorandom" or "Halton")
	\param[in]	Q				Sample array output from `GenerateSamples`
	\param[in]	q_min, q_max	Joint angle bounds
	\param[in]	iter			Pointer to the current iteration number, used to prevent redundant samples
	\param[in,out] q			Pre-allocated `double` vector of length *n*, returned as the next sample */
void Sample( int feedback_mode, char* sampling, int n, double** Q, double* q_max, double* q_min, int* iter, double* q );

/*! Navigation function from configuration *q_near* to configuration *q*.  Yields the new state *q_new* within weighted distance *epsilon* from *q_near*
in the direction of *q*.  Currently chooses new configurations along the straight line segment connecting the near and goal states. \see DistSq, Extend, Connect
	\param[in]		q			Target joint-angle vector, of length *n*
	\param[in]		q_near		Nearest joint-angle vector to *q* in the current tree, of length *n*
	\param[in]		epsilon		Maximum weighted incremental distance to travel from *q* to *q_near* (terminates if squared-distance is less than \f$\epsilon^2\f$)
	\param[in,out]	q_new		Pre-allocated vector of length *n*, used to store new joint-angle configuration
	\param[in]		w			Vector of weighting factors as defined by the user
	\returns Cost-to-go from *q_near* to *q_new* as `double` */
double Steer( double* q, double* q_near, int n, double epsilon, double* q_new, double* w );


/*! Searches the tree *T* for the nearest node to *q*.  Determines the single nearest-neighbor to joint-angle vector *q* among all *num_nodes* nodes within tree *T*.
Proximity is determined by weighted squared-Euclidean-distance for brute-force search or unweighted distance for KD-tree search.  Uses either a brute-force search or KD-tree 
search depending on the specification of the user.  The selection is also illustrated by a call to the `PlotNearestInMATLAB` command if the MATLAB Engine pointer *matlab* is 
not NULL. \see BuildRRTs, DistSq, PlotNearestInMATLAB
	\param[in]	T					Pointer to the tree to be searched
	\param[in]	q					Query node, often a sample generated by `Sample`, of length *n*
	\param[in]	cost_type			Indicator of cost-function type (enter 1 for global cost function (*brute-force only*) or 2 for local (greedy) cost function)
	\param[in,out]	nearest_index	Returned as pointer to index of nearest-neighbor node
	\param[in,out]	q_near			Returned as nearest-neighbor node
	\param[in]		NN_alg			User-specification of NearestNeighbor method (either "brute_force" for search over all nodes or "kd_tree" for KD-tree nearest search) */
void Nearest(struct tree *T, int num_nodes, double* q, int n, double* w, int cost_type, int* nearest_index, double* q_near, char* NN_alg, Engine* matlab);

/*! Searches the tree *T* for up to *max_neighbors* neighbors near *q*.  Determines at most *max_neighbors* nearest-neighbors to joint-angle vector *q* among all *num_nodes* nodes 
within tree *T*.  Proximity is determined by weighted squared-Euclidean-distance for brute-force search or unweighted distance for KD-tree search.  Uses either a brute-force search 
or KD-tree search depending on the specification of the user.  KD-trees currently return all nodes within a variable radius, and are unconstrained by *max_neighbors*. \see BuildRRTs, DistSq
	\param[in]	T					Pointer to the tree to be searched
	\param[in]	q					Query node, often a sample generated by `Sample`, of length *n*
	\param[in]	cost_type			Indicator of cost-function type (enter 1 for global cost function (*brute-force only*) or 2 for local (greedy) cost function)
	\param[in]	eta_RRT				Maximum radius possible between any two samples produced by `Steer` and stored in the trees
	\param[in]	gamma_RRT			Must choose \f$\gamma_{RRT} > 2*\left[ (1 + \frac{1}{n})*(\frac{\mu(C_{free})}{\mu(V_{B_n})}) \right]^(1/n)\f$, where \f$\mu\f$ is the Lebesgue measure, 
									\f$C_{free}\f$ is the free configuration space, and \f$V_{B_n}\f$ is the volume of a normed ball of dimension *n*
	\param[in,out]	neighbors		Pre-allocated `int` array of length *max_neighbors*. Returned as pointer to the indices of nearest-neighbor nodes, of length *n_neighbors*
	\param[in,out]	costs			Pre-allocated `double` array of length *max_neighbors*. Returned as array of costs to/from *q* from/to each neighbor, of length *n_neighbors*
	\param[in,out]	n_neighbors		Returned as pointer to the number of discovered nearest-neighbors
	\param[in]		NN_alg			User-specification of NearestNeighbor method (either "brute_force" for search over all nodes or "kd_tree" for KD-tree variable radius search) */
void NearestNeighbors(struct tree *T, int num_nodes, double* q, int n, double* w, int max_neighbors, int cost_type, double eta_RRT, double gamma_RRT,
	int* neighbors, double* costs, int* n_neighbors, char* NN_alg);


/*! Implements one iteration of `Steer` and tests for constraint violation, attempting to extend from node *q_near* towards *q*. \see BuildRRTs, Connect, Steer, DistSq, ConstraintViolation
	\param[in]		q				Target joint-angle vector, of length *n*
	\param[in]		q_near			Nearest joint-angle vector to *q* in the current tree, of length *n*
	\param[in,out]	q_new			Pre-allocated vector of length *n*, used to store new joint-angle configuration
	\param[in,out]	cost_to_go		Returned as pointer to cost-to-go from *q_near* to *q_new*
	\param[in]		indicator		Indicator of obstacle test type (see `ConstraintViolation`)
	\returns		The status of `Extend` (either *Trapped* (0), *Advanced* (1), or *Reached* (2)) */
int Extend(double* q, double* q_near, double epsilon, int n, double* w, double* q_new, struct obstacles* obs, struct geom *G, struct DHparams *DH, double* cost_to_go, int indicator);

/*! Repeatedly calls `Extend` to incrementally test path safety and attempt to grow a branch from node *q_near* to *q*. \see BuildRRTs, Extend, DistSq, ConstraintViolation
	\param[in]		q				Target joint-angle vector, of length *n*
	\param[in]		q_near			Nearest joint-angle vector to *q* in the current tree, of length *n*
	\param[in,out]	q_new			Pre-allocated vector of length *n*, used to store new joint-angle configuration
	\param[in,out]	cost_to_go		Returned as pointer to cost-to-go from *q_near* to *q_new*
	\param[in]		indicator		Indicator of obstacle test type (see `ConstraintViolation`)
	\returns		The status of `Connect` (must be either *Trapped* (0), or *Reached* (2)) */
int Connect(double* q, double* q_near, double epsilon, int n, double* w, double* q_new, struct obstacles* obs, struct geom *G, struct DHparams *DH, double* cost_to_go, int indicator);


/*! Adds a new node to an RRT structure.  Re-allocates memory and inserts new node *q_new* into the tree pointed to by *T*, increasing the number of nodes by 1 and
setting the nodes properties.  Depending on the NearestNeighbor algorithm indicated by *NN_alg*, also adds the new node to the corresponding KD-tree.  If the MATLAB
Engine pointer *matlab* is not NULL, adds the node and the edge from its parent to RRTfig. \see ::tree, BuildRRTs, PlotPathInMATLAB
	\param[in,out]	T			Pointer to the tree in which the new node should be inserted.  Returned with updated tree.
	\param[in,out]	num_nodes	Pointer to the number of nodes in the tree. Returned as pointer to updated node count value.
	\param[in]	q_new		New node to insert into the tree
	\param[in]	parent_index, cost_to_go, connection, leaf_list, safety		Properties of the new node (typically initially assumed to be unconnected (-1), with a NULL leaf list, and safe (1)) */
void InsertNode( struct tree* T, int n, int* num_nodes, double* q_new, int parent_index, double cost_to_go, int connection, list_node* leaf_list, int safety, char* NN_alg, Engine* matlab );

/*! Updates tree branches by re-wiring them into more efficient connections.  As the sampling sequence is relatively arbitrary, new nodes introduced to the tree at
later times may turn out to be more efficient parents than previous ones.  This means that suboptimal edges must be removed and re-wired to the most recently-added nodes.
It can be shown that, so long as searching within an appropriate radius, this re-wiring leads to convergence towards the optimal path.  `ReWire` implements this routine.
Given *rewire_node_index* as the most-recently added node index, its list of neighbors, and costs of connecting to each one, the edges of each are tested and replaced if the
path through the rewire node is found to be more efficient.  \see BuildRRTs, SetDiffLeafListBfromA, MergeLeafListBwithA, PlotRewiringInMATLAB, ConstraintViolation
	\param[in,out]	T				Pointer to tree.  Returned with re-wirings and cost updates.
	\param[in]	feedback_mode		Boolean specifying whether or not tree-construction is taking place during motion plan execution.  If not, leaf list updates need not be made,
									as those are reserved for after pre-computation.  If so, leaf list merges and set differences are required.
	\param[in]	rewire_node_index	Index of the node through which re-wiring takes place.  Tested as potential new parent.
	\param[in]	neighbors			Indices of nearest-neighbor nodes, of length *n_neighbors*
	\param[in]	costs_to_neighbors	Costs to nearest-neighbor nodes from the rewire node, of length *n_neighbors*
	\param[in]	indicator			Indicator of obstacle test type (see `ConstraintViolation`) */
void ReWire( int feedback_mode, struct tree *T, int n, double* w, int rewire_node_index, int* neighbors, double* costs_to_neighbors, int n_neighbors, double epsilon, struct obstacles* obs, struct geom *G, struct DHparams *DH, int indicator, Engine* matlab );


	/*! Adds *leaf_index* to all ancestors' leaf lists up to and including the root node */
void AddLeafToLists(struct tree *T, int leaf_index);

/*! Build the Rapidly-Exploring Random Trees (RRT's).  Constructs RRTs for motion plan *I* using a modified form of the bi-directional RRT-Connect algorithm.  Operates in two
separate phases, depending on whether it has been called previously or not (as indicated by the value pointed to by *iter*).  If unrun before, uses the sample array *Q* to generate 
samples and waits until the end of all iterations to update node leaf lists.  If not, assumes the manipulator is executing closed-loop motion control and continuing to add samples.  In this
case, one node is added to each tree at a time, and leaf lists must be updated.  If it is requested to load trees from a previous simulation, calls <*filename*>trees.dat to upload previous
values instead.  If the MATLAB Engine pointer *matlab* is not NULL, also generates a plot with handle RRTfig illustrating tree construction. 
\see Sample, Nearest, NearestNeighbors, Extend, Connect, ReWire, InsertNode, ConstraintViolation, SendArraysToMATLAB
	\param[in,out]	Ta, Tb				Pointers to the forward and reverse trees.  Each must be initialized with the initial and goal joint-angle vectors, respectively.
										Returned as fully-constructed trees.
	\param[in,out]	n_nodesA, n_nodesB	Pointers to the number of nodes in each tree.  Returned with new values prior to function exit.
	\param[in]		n					Dimensionality of tree nodes, i.e.\ the number of joint-angles
	\param[in,out]	iter				Pointer to the current sampling iteration
	\param[in,out]	node_star_index		Pointer to each tree's node leaf indices through which the most optimal path passes
	\param[in]		load_trees			User-specification indicating whether to load old trees (Enter 'y' to read values from file, or 'n' to compute using current user settings)
	\param[in,out]	fpos				Pointer to current file position of <*filename*>trees.dat (used to properly load all motion plan trees)
	\returns Boolean indicating whether a feasible path has been found */
int BuildRRTs(struct tree *Ta, struct tree *Tb, int* n_nodesA, int* n_nodesB, int n, double* w, int* iter, int max_iter, char* soln, char* sampling, 
	double** Q, double* q_max, double* q_min, char* NN_alg, int max_neighbors, double eta_RRT, double gamma_RRT, double epsilon, int obs_indicator,
	struct obstacles* obs, struct geom *G, struct DHparams *DH, int* node_star_index, char load_trees, char* filename, int I, fpos_t *fpos, Engine* matlab );


/*! Traces the indices along a connected node path.  Traces the path from a node *node1* in the tree *T* up to its ancestor *node2* (or root, if it comes first) and 
returns a pointer to the index path array.  Note this path is traced in the backwards direction from what is intended for a forward tree. \see StorePath
	\param[in]		node1		Origin node at which to begin the traversal
	\param[in]		node2		Ancestor node at which to stop
	\param[in,out]	pathlen		Pointer to the number of nodes along the tree path */
int* TracePath(struct tree *T, int node1, int node2, int *pathlen);


/*! Stores the total sequence of nodes along a connected pair of forward-tree and reverse-tree paths.  Traverses from the initial point in *pathA* to the final point
in *pathB*, in correct order, saving the nodes encountered along the way. Saves the resulting sequence of nodes to <*filename*>output.dat.  \see TracePath
	\param[in]		Ta, Tb				Pointers to forward and reverse trees, respectively
	\param[in]		pathA, pathB		Index path arrays from Ta and Tb to-be-merged (see `TracePath`)
	\param[in]		pathlenA, pathlenB	Lengths of each *pathA* and *pathB*
	\param[in]		I					Index of current motion plan (used to label saved motion plans. Enter -1 to suppress file output).
	\param[in,out]	path				Pre-allocated `double` array of size \f$(\f$(*pathlenA*+*pathlenB*)\f$\times n)\f$, returned with node path */
void StorePath(struct tree *Ta, struct tree *Tb, int n, int* pathA, int* pathB, int pathlenA, int pathlenB, char* filename, int I, double **path);

/*! Split a path edge at node *q* and insert it into the appropriate tree. \see TracePath, StorePath, InsertNode, MergeLeafListBwithA */
void SplitPathAtNode( double** path, double* q, double *w, int* pathA, int* pathB, int pathlenA, int index, struct tree** T_ptrs, int* n_nodes, int n, char* NN_alg, Engine* matlab );

/*! Saves an index representation of the re-plan path to a list of best re-plan paths.  Given the *cost_to_go* and the tree, starting index, and ending index characterizing
the re-plan path, saves its values to the list of best paths found so far.  If fewer than *max_replans* number of re-plans have been found, saves the path by default.  On the other
hand, if *max_replans* number have indeed already been found, maintains the *replan_costs* vector in sorted order by optimality, along with its corresponding *replan_indices* array.
\see RePlan, FindSafePath, ExhaustiveRePlan
	\param[in,out]	num_replans		Current number of re-plans saved.  Increments by 1 until at most *max_replans* number of paths have been found.
	\param[in]	cost_to_go			Cost-to-go for the current re-plan path.
	\param[in]	tree_index			Index representing the type of tree (0 = forward tree, 1 = reverse tree)
	\param[in]	newpath_start_node	Index for the replan path starting node (depends on type of tree)
	\param[in]	newpath_end_node	Index for the replan path ending node (=leaf for forward tree, =root for reverse tree)
	\param[in,out]	replan_costs	Vector of length *num_replans* of the costs-to-go for each replan path.
	\param[in,out]	replan_indices	List of size *num_replans*\f$\times3\f$, each row corresponding to a different set of replan indices: tree_index, newpath_start_node, newpath_end_node. */
void SaveBestPlans(int *num_replans, int max_replans, double cost_to_go, int tree_index, int newpath_start_node, int newpath_end_node, double *replan_costs, int** replan_indices );
	

/*! Replan according to the shortest-distance paths that do not currently violate obstacle constraints.  *Cost-Priority Search*: Generates a sorted list of potential re-plan paths
according to the shortest-paths among all possible paths through the set of *max_replan_neighbors* percent of nearest-neighbors in each tree.  Note this does not conduct 
a full search (unless set to 100%), nor does it necessarily compute feasible paths (but merely the globally shortest *unconstrained* path among the set of locally closest safe nodes).
\see SaveBestPlans, FindSafePath, ExhaustiveRePlan
	\param[in]		max_replans				Maximum number of cost-priority re-plan paths to search for
	\param[in]		max_replan_neighbors	Percent of tree nodes to use during a replan search (same for each tree, forward and reverse)
	\param[in,out]	replan_indices			Pre-allocated list of size *max_replans*\f$\times3\f$, used to store indices of re-plan paths (see `SaveBestPlans`).
	\returns The number of re-plan paths found and stored in *replan_indices* */
int RePlan(struct tree **T_ptrs, int* n_nodes, double* q, int n, double* w, int max_replans, double max_replan_neighbors, double eta_RRT, double gamma_RRT, int** replan_indices, char* NN_alg);


/*! Reconstructs the best safe path produced by `RePlan` and/or `ExhaustiveRePlan`. Finds a safe path (that satisfies obstacle constraints) given the 
sorted locally-optimal motion plans in *replan_indices* (if one exists). Tests plans sequentially, returning the first feasible path found.  Hierarchically
determines path safety, first checking node safety, then checking safety of connection from node *q* to the first element of the path, and finally checking
the safety of edges within the new path (only requires check against temperature obstacles as the edges already satisfy static obstacle constraints).
\see SaveBestPlans, RePlan, ExhaustiveRePlan, StorePath, ConstraintViolation
	\param[in]		replan_indices		List of indices of re-plan paths (see `SaveBestPlans` and `RePlan`)
	\param[in,out]	pathlenA, pathlenB	Current forward and reverse tree pathlengths.  Returned with new pathlengths if feasible path found.
	\param[in,out]	pathA, pathB		Current forward and reverse tree index paths.  Returned with new index paths if feasible path found.
	\param[in,out]	path				Current array of nodes.  Returned with new path if feasible path found. (see `StorePath`)
	\returns Boolean indicating whether a new safe path has been found */
int FindSafePath(struct tree **T_ptrs, int** replan_indices, int num_replans, int* pathlenA, int* pathlenB, int** pathA, int** pathB,
	double*** path, double* q, double epsilon, int n, double* w, struct obstacles* obs, struct geom *G, struct DHparams *DH, char* filename);


/*! Replan by searching over all safe nearest neighbors until a feasible solution is discovered (if one exists). *Feasibility-Priority Search*: In the case that the 
cost-priority search from `RePlan` was unsuccessful, conduct an exhaustive search with focus not on costs but on feasibility.  First search for feasible nodes
over each tree (listed in order of proximity), then accept the first feasible solution found. Calls `FindSafePath` one path at a time until successful. \see SaveBestPlans, RePlan, FindSafePath */
int ExhaustiveRePlan(struct tree **T_ptrs, int* n_nodes, double eta_RRT, double gamma_RRT, char* NN_alg, int* pathlenA, int* pathlenB, int** pathA, int** pathB,
	double*** path, double* q, double epsilon, int n, double* w, struct obstacles* obs, struct geom *G, struct DHparams *DH, char* filename);
	

/*! Reset the simulation for a new run.  Resets node saftey properties to 1, removes any temperature obstacles from previous run,
re-defines the new "best" plan by finding the new shortest paths that result from any node and edge additions made during the previous simulation,
and closes old plots so that they may be regenerated during the next simulation. */
void ResetSimulation( char reset_trees, struct tree* trees, int n_trees, int n_plans, int* num_nodes, int* num_nodes_precomputed,
	int* feasible, int** node_star_index, struct obstacles* obs, Engine* matlab );


/*! Generate array of samples, *Q*.  Computes the sample array used during construction of pre-computed RRT's (prior to motion plan execution).  Used so that
samples can be sent in batch rather than generated individually up to *max_iter* times during the call to `BuildRRTs`.  Generates <*filename*>samples.dat \see Sample, Halton
	\param[in,out]	Q			Pre-allocated `double` array of size \f$(\f$*max_iter*\f$ \times n)\f$, used to store samples
	\param[in]	sampling		User specification of sampling method ("pseudorandom" or "halton")
	\param[in]	q_min, q_max	Joint angle bounds
	\param[in] filename			String containing the full path + root of input filename */
void GenerateSamples(double** Q, char* sampling, int n, int max_iter, double* q_max, double* q_min, char* filename);
	

/*! Execute emergency motion plan. Searches nearest point of the motion plan and execute all folllowing nodes. */ 
void EmergencyPlan( double* q, double* q_emergency, double* q_waypoints, int n, int n_emergency, double* w, double epsilon_sq, 
	CPhidgetAdvancedServoHandle servo, int* channels, int plan_num );

#endif