#ifndef __MATLAB_INTERFACE_H__
#define __MATLAB_INTERFACE_H__


#include <stdarg.h>
#include <stdio.h>
#include <engine.h>
#include <stdlib.h>
#include "utils.h"


/*! Sends *arg_count* number of `double` variables to the MATLAB Engine, entered as a list of names followed by values, <BR> e.g.\ "v1", v1, "v2", v2 (CURRENTLY UNUSED) */
void SendDoublesToMATLAB(Engine* matlab, int arg_count, ... );

/*! Sends *arg_count* number of real 2D `double` array variables to the MATLAB Engine, entered as a list of names followed by values, row dimension, 
	and column dimension, <BR> e.g.\ "v1", m1, n1, v1, "v2", m2, n2, v2, etc...\ <BR> (NOTE: Assumes rows of v1, v2, ...\ are each contiguous blocks of memory. 
	Can handle `double` vectors (`double*` or `double[]`) as well, provided they are entered with m = 1. ) (CURRENTLY UNUSED) */
void Send2DDoubleArraysToMATLAB(Engine* matlab, int arg_count, ... );

/*! Sends real \f$N\f$-D numeric arrays up to \f$N=3\f$ to the MATLAB Engine, saving the variables as formatted numeric matrices.  
	Returns an `int` on success/failure. \see ::datatypes */
int SendArraysToMATLAB( int line, Engine* matlab, int arg_count, ... );

/*! Adds an illustration of sample node *q* and the selected node *q_near* in tree *T* to the RRT construction figure with handle RRTfig. <BR>
	(Nothing is done if the MATLAB Engine pointer *matlab* is NULL.) \see BuildRRTs, Nearest */
void PlotNearestInMATLAB( struct tree* T, int n, double* q, double* q_near, Engine* matlab );

/*! Adds a straight line plot between a node (index *node_index*) and its parent to the RRT construction figure with handle RRTfig. <BR> 
	(Use C convention, i.e.\ starting from 0, for *node_index*.  Nothing is done if the MATLAB Engine pointer *matlab* is NULL.) \see BuildRRTs, InsertNode */
void PlotEdgeInMATLAB( struct tree* T, int n, int node_index, Engine* matlab );

/*! Adds a straight line plot between a neighbor node (index *neighbor_index*) and its new parent to the RRT construction figure with handle RRTfig, 
	deleting the old edge.	Assumes \f$q_{rewire}\f$ has already been sent to MATLAB from `ReWire` and that "tree" is unchanged from its former definition 
	from `InsertNode` and `PlotEdgeInMATLAB`. (Nothing is done if the MATLAB Engine pointer *matlab* is NULL.) \see BuildRRTs, InsertNode, PlotEdgeInMATLAB */
void PlotRewiringInMATLAB( struct tree* T, int n, int neighbor_index, Engine* matlab );

/*! Plot a new path segment in MATLAB to figure handle PLANfig.  Sends *path_new* and *pathlen_new* to MATLAB, merging path[1:1:(pathlen - pathlen_old + current_path_index)] 
with the new path, and displays the result in figure with handle PLANfig (defined in `main`).  "path_old" is the previously planned trajectory for indices 
beyond "current_path_index", which is instead replaced by path_new. (Use C convention for indices.  Nothing is done if the MATLAB Engine pointer *matlab* is NULL.) \see main
	\param[in]	plan_index			Index of the current motion plan. Enter as -1 to finalize the plot after motion is over.
	\param[in]	current_path_index	Entering 0 erases the old plan, while entering "pathlen_oldplan" appends the new one.
	\param[in]	pathlen_new			Length of new path to be added to the plot
	\param[in]	path_new			Nodes along the new path */
void PlotPathInMATLAB( int plan_index, int current_path_index, int pathlen_new, int n, double** path_new, Engine* matlab );

/*! Plots the current manipulator configuration to figure handle TRAJfig.  Plots the manipulator configuration corresponding to the coordinate structure C 
returned by `WorldCoords`, adding a visualization of the arm OBB's and point representation to TRAJfig (Requires previous definition of "coord_format", "coord_color", 
"link_colors", "link_alpha", and "N_coords". Nothing is done if the MATLAB Engine pointer *matlab* is NULL.) \see main, PlotEndEffectorPathInMATLAB
	\param[in]	C			Coordinates in the world-frame of the manipulator arm
	\param[in]	n_points	Total number of points in C
	\param[in]	opacity		Face opacity. Enter 0 for transparent or 1 for full. */
void PlotRobotConfigInMATLAB( struct coords* C, int n_points, double opacity, Engine* matlab );

/*! Plot the temperature obstacles to figure handle TRAJfig. */
void PlotTempObstaclesInMATLAB( struct obstacles* obs, Engine* matlab );

/*! Plots the end effector trajectory corresponding to a given path in figure handle TRAJfig.  Generates the points traced out by the end-effector position along a given 
joint-angle path using `Steer`, adding a curve of the traced-path plus the planned and former planned paths to TRAJfig.  Also overlays a plot of planar obstacles (updated 
with each plot in order to properly span the axis dimensions), the current temperature obstacles, and removes the grasped object (last cuboidal obstacle) if "plan_index"
exceeds the grasp maneuver index.  Must be called immediately after `PlotPathInMATLAB` (relies in the MATLAB Engine environment on many of the same variables). 
\see main, PlotRobotConfigInMATLAB, PlotPathInMATLAB, Steer
	\param[in]	n_cuboids_total		Total number of cuboids, including the grasped object.  Used to test whether the grasped object has already been eliminated from plots.
	\param[in]	pathlen_new			Length of new path to be added to the plot
	\param[in]	path_new			Nodes along the new path */
void PlotEndEffectorPathInMATLAB( int n, double epsilon, double* w, int n_cuboids_total, struct obstacles* obs, struct geom *G, struct DHparams *DH,
		int pathlen_new, double** path_new, Engine* matlab );

#endif