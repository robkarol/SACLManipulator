#ifndef __COLLISION_DETECTION_H__
#define __COLLISION_DETECTION_H__

#define SAFE 0
#define PLANAR_COLLISION 1
#define CUBOIDAL_COLLISION 2
#define CYLINDRICAL_COLLISION 3
#define CONICAL_COLLISION 4
#define TEMPOBS_COLLISION 5

/*! Tests a manipulator configuration *q_new* for violation of constraints.  Determines whether a node *q_new* is safe to add to a tree by testing the manipulator configuration
for violation of obstacle constraints.  First generates all world-frame coordinates corresponding the new manipulator configuration using `WorldCoords`.  Then, depending on the value
specified by *obs_indicator*, examines whether any point intersects one of the obstacles stored within *obs*.  Conducts tests in order of increasing computational complexity, starting
with planes and ending with truncated cones. \see ::obs, ::geom, ::DHparams, BuildRRTs, WorldCoords
	\param[in]	q_new			Query node, of length *n*
	\param[in]	obs_indicator	Indicator of obstacle test type (Enter 0 to test static constraints, 1 to test dynamic constraints, or 2 to test both)
	\returns	A boolean indicating violation (1) or not (0) */
int ConstraintViolation( double* q_new, int n, struct obstacles* obs, struct geom *G, struct DHparams *DH, int obs_indicator, int* obs_num );

/*! Identify and mark as unsafe any temperature obstacle violators.  Modifies the "safety" property of tree nodes that are found to reside in or result in inevitable collision with
temperature obstacles pointed to by *obs*.  Scans and marks the nodes of trees T[0] (forward tree) and T[1] (reverse tree) one-by-one.  If the tree is a reverse tree, all decendents
leading to an unsafe node are also unsafe, as well as any leaves in the other tree directly connected to it.  This is where the benefits of the augmented data "connections" and 
"leaf_lists" come into play.  If the goal node, i.e.\ root of T[1], is found to be unsafe, then abort the program, as nothing can be done to recover the arm and find a new safe path 
(all paths lead to collision). \see ::tree, ::obs, WorldCoords, ConstructTempObstacle */
void TempObsViolation(struct tree** T, int* num_nodes, int n, struct obstacles* obs, struct geom *G, struct DHparams *DH, int num_plan );

#endif