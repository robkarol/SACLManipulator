#ifndef __MANIP_GEOM_H__
#define __MANIP_GEOM_H__

/*! Structure for coordinate storage.  Structure variable used to point to a set of 3D Cartesian coordinates of type double.  Often used 
to save the world coordinates of a transformed set of manipulator geometry points, corresponding to a particular joint angle configuration vector. */
struct coords {
	double* x;						/*!< Array of x-coordinates */
	double* y;						/*!< Array of y-coordinates */
	double* z;						/*!< Array of z-coordinates */
};

/*! Structure for manipulator geometry.  Structure variable encapsulating manipulator geometry information, including link dimensions, Oriented-Bounding Box
(OBB) positioning, and body-fixed OBB coordinates. */
struct geom {
	double* L;						/*!< Array of link lengths */
	double* W;						/*!< Array of link widths */
	double* H;						/*!< Array of link heights */
	double* rho_x;					/*!< x-coordinates in each link's body-fixed frame to the back bottom-left corner of their respective Oriented Bounding Boxes (OBB's) */
	double* rho_y;					/*!< y-coordinates in each link's body-fixed frame to the back bottom-left corner of their respective Oriented Bounding Boxes (OBB's) */
	double* rho_z;					/*!< z-coordinates in each link's body-fixed frame to the back bottom-left corner of their respective Oriented Bounding Boxes (OBB's) */
	int* N_coords;					/*!< Array of the numbers of coordinates used to represent each link (at least 8, one for each OBB corner, plus any face points) */
	double*** Body_coords;			/*!< An array of the x-y-z body-fixed OBB coordinate arrays, one for each link */
};

/*! Structure of Denavit-Hartenberg parameters.  Structure variable containing the Denavit-Hartenberg (DH) parameters of the manipulator arm.  DH parameters are a particular 
manipulator parametrization that models robotic arm joints as two "screw" operations - one rotation and translation about the original x-axis followed by a rotation and translation
about the intermediate z-axis.  This can be used to represent any 2-DOF joint; more complicated joints can be represented by several degenerate-case sets of DH parameters 
(refer to p.103 of "Planning Algorithms" by Steven LaValle).  The "theta" parameters about z are not included as each corresponds to a node stored in the RRTs. */
struct DHparams {
	double* d;						/*!< Vector of translations along the \f$z_i\f$ (rotation) axes */
	double* a;						/*!< Vector of translations along the \f$x_{i-1}\f$ (perp) axes */
	double* alpha;					/*!< Vector of rotation angles about the \f$x_{i-1}\f$ (perp) axes */
};


/*! Compute the homogeneous transformation matrix given yaw-pitch-roll Euler angles \see InvTransformMatrix, HomTransformMatrix, InvHomTransformMatrix
	\param[in] yaw, pitch, roll	Euler angles of rotation about the original z-, y-, and x-axes, respectively [deg]
	\param[in]	trans			\f$3\times1\f$ translation vector (applied after rotations), done w.r.t.\ original axes directions
	\param[in,out]	T			Pre-allocated array of size \f$4\times4\f$, returned as the output matrix */
void TransformMatrix( double yaw, double pitch, double roll, double* trans, double** T );

/*! Computes the inverse of the transform matrix corresponding to the given yaw-pitch-roll Euler angles and translation vector, transforming the 
coordinates back to their original frame by reversing the translation and rotation sequence \see TransformMatrix, HomTransformMatrix, InvHomTransformMatrix
	\param[in] yaw, pitch, roll	Euler angles of rotation for the original transform about the z-, y-, and x-axes, respectively [deg]
	\param[in]	trans			\f$3\times1\f$ translation vector for the original transform (applied after rotations), done w.r.t.\ original axes directions
	\param[in,out]	T			Pre-allocated array of size \f$4\times4\f$, returned as the output matrix */
void InvTransformMatrix( double yaw, double pitch, double roll, double* trans, double** T );


/*! Compute the homogeneous transformation matrix for the \f$i^{ th}\f$ link: 
	\f$\left. (x,y,z,1) \right|_{i-1} = T_i \left. (x,y,z,1) \right|_{i}\f$ \see ::DHparams, TransformMatrix, InvTransformMatrix, InvHomTransformMatrix
	\param[in] a,d,alpha	DH-parameters for the manipulator arm (constant)
	\param[in] q			Manipulator joint-angle configuration
	\param[in,out]	T		Pre-allocated array of size \f$4\times4\f$, returned as the output matrix */
void HomTransformMatrix( double a, double d, double q, double alpha, double** T );


/*! Compute the inverse homogeneous transformation matrix for the \f$i^{ th}\f$ link: 
	\f$\left. (x,y,z,1) \right|_{i} = T_i^{-1} \left. (x,y,z,1) \right|_{i-1}\f$ \see ::DHparams, TransformMatrix, InvTransformMatrix, HomTransformMatrix
	\param[in] a,d,alpha	DH-parameters for the manipulator arm (constant)
	\param[in] q			Manipulator joint-angle configuration
	\param[in,out]	T		Pre-allocated array of size \f$4\times4\f$, returned as the output matrix */
void InvHomTransformMatrix( double a, double d, double q, double alpha, double** T );


/*! Compute the body-fixed coordinates of manipulator Oriented Bounding Boxes (OBB's), including the 8 corner points
for each box, the facepoints specified by *n_facepts*, and the end effector point given by *grip_pos* \see ::geom, WorldCoords, main
	\param[in]	n_facepts	Vector containing the numbers of points to use for each pair of OBB faces, given in groups of 3 for each link (faces 
							parallel to xy-, yz-, and xz-planes, respectively). Computed using the Halton sequence, scaled to the link face dimensions.
	\param[in]	grip_pos	\f$1\times3\f$ vector in the body-fixed frame of link *n* of the representative end-effector position */
void BodyFixedOBBcoords( struct geom *G, int* n_facepts, double* grip_pos, int n );

/*! Output the coordinates, *C*, w.r.t.\ the world frame of the corners and face points of each link's OBB \see ::geom, ::DHparams, ::coords, BodyFixedOBBcoords, main
	\param[in]	q			Manipulator joint-angle configuration
	\param[in]	C			Pointer to coordinates structure, pre-allocated to hold sum(G->N_coords) of \f$(x,y,z)\f$ ordered-pairs */
void WorldCoords( struct geom *G, struct DHparams *DH, double* q, int n, struct coords *C );

#endif