#ifndef __OBSTACLES_H__
#define __OBSTACLES_H__


/*! Structure for planar obstacles.  Structure variable used to store planar obstacle data, including the vector normal, \f$\hat{n} = (a,b,c)\f$, and distance, \f$d\f$,
to the plane from the origin.  The plane equation, \f$f = ax + by + cz + d\f$, corresponds to that of the world-frame. */
struct planar_obs {
	double* a;						/*!< Vector of unit-normal vector x-component values, one for each plane */
	double* b;						/*!< Vector of unit-normal vector y-component values, one for each plane */
	double* c;						/*!< Vector of unit-normal vector z-component values, one for each plane */
	double* d;						/*!< Vector of distances from the origin, \f$d = -ax - by - cz\f$, for any point \f$(x,y,z)\f$ in each plane */
};

/*! Structure for cuboidal obstacles.  Structure variable used to store cuboidal obstacle data, including the vector normals, \f$\hat{n} = (a,b,c)\f$, and distances, \f$d\f$,
for the set of 6 planes defining each cuboid.  cuboid_obs is distinguished from planar_obs by the fact that sets of 6 planes at a time correspond to an obstacle,
meaning \f$f = ax + by + cz + d < 0\f$ must hold for a group of all 6 planes at once for a collision whereas in planar_obs only one violation is required. */
struct cuboid_obs {
	double* a;						/*!< Vector of unit-normal vector x-component values, in groups of 6, one for each cuboid */
	double* b;						/*!< Vector of unit-normal vector y-component values, in groups of 6, one for each cuboid */
	double* c;						/*!< Vector of unit-normal vector z-component values, in groups of 6, one for each cuboid */
	double* d;						/*!< Vector of distances from the origin, \f$d = -ax - by - cz\f$, in groups of 6, for any point \f$(x,y,z)\f$ in the planes defining each cuboid */
	double*** T;					/*!< Vector of transformation matrices from the cuboid frame to the world frame (only used for plots; not required for collision-checking) */
};

/*! Structure for cylindrical obstacles.  Structure variable used to store cylindrical obstacle data, including the cylinder radii, \f$r\f$, the heights, \f$H\f$, and the
inverse transformation matrices, \f$T^{-1}\f$, that resolve coordinates from the world frame into the frame of the cylinder (z-axis aligned, origin at geometric center).  
Used during constraint checking to test whether the inequalities \f$f_1 = x^2 + y^2 - r^2 < 0\f$, \f$f_2 = z - \frac{H}{2} < 0\f$, and \f$f_3 = z + \frac{H}{2} > 0\f$ hold, indicating a collision. */
struct cylindrical_obs {
	double* r;						/*!< Vector of cylinder radii */
	double* H;						/*!< Vector of cylinder heights */
	double*** Tinv;					/*!< Vector of transformation matrices from the world frame to the cylinder frame */
};

/*! Structure for conical obstacles.  Structure variable used to store conical obstacle data, including cone half-angles, \f$\beta\f$, cone lower boundaries, \f$z = h_1\f$, cone
upper boundaries, \f$z = h_2\f$, and the inverse transformation matrices, \f$T^{-1}\f$, that resolve coordinates from the world frame into the frame of the cone (z-axis aligned, 
origin at cone apex).  Currently unimplemented. */
struct cone_obs {
	double* beta;					/*!< Vector of cone half-angles (deg) */
	double* h1;						/*!< Vector of cone heights from apex of truncated cone bottoms */
	double* h2;						/*!< Vector of cone heights from apex of truncated cone tops */
	double*** Tinv;					/*!< Vector of transformation matrices from the world frame to the cone frame */
};

/*! Structure for conical temperature obstacles.  Structure variable used to store temperature obstacles, which are modelled as truncated circular cones (frustra).  Includes cone 
half-angles, \f$\beta\f$, cone lower boundaries, \f$z = h_1\f$, cone upper boundaries, \f$z = h_2\f$, and the inverse transformation matrices, \f$T^{-1}\f$, that resolve coordinates
from the world frame into the frame of the cone (z-axis aligned, origin at cone apex).  Used to generate temperature obstacles along sensor unit normals during motion plan execution,
and during constraint checking to test whether the inequalities \f$f_1 = x^2 + y^2 - (z \beta)^2 < 0\f$, \f$f_2 = z - h_2 < 0\f$, and \f$f_3 = z - h_1 > 0\f$ hold, indicating a collision. */
struct temp_obs {
	double* beta;					/*!< Vector of cone half-angles (deg) */
	double* h1;						/*!< Vector of cone heights from apex of truncated cone bottoms */
	double* h2;						/*!< Vector of cone heights from apex of truncated cone tops */
	double*** Tinv;					/*!< Vector of transformation matrices that transform from the world frame to the temp obs frame */
};

/*! Structure for all obstacle data.  Structure variable representing the compilation of all obstacle information, including conical obstacles, cylindrical obstacles, 
cuboidal obstacles, planar obstacles (all considered "static") and finally temperature obstacles (considered "dynamic", though static once generated).  
Only temperature obstacles can increase in number during motion plan execution; all else must be prescribed prior to the simulation.  The final 
element of the cuboidal obstacle structure is taken to be the cuboidal obstacle, eliminated from consideration once grasped by the manipulator. */
struct obstacles {
	struct cone_obs *cones;				/*!< All conical obstacle information */
	struct cylindrical_obs *cylinders;	/*!< All cylindrical obstacle information */
	struct cuboid_obs *cuboids;			/*!< All cuboidal obstacle information */
	struct planar_obs *planes;			/*!< All planar obstacle information */
	struct temp_obs *temp_zones;		/*!< All current temperature obstacle information */
	int n_cones;						/*!< Number of conical obstacles */
	int n_cylinders;					/*!< Number of cylindrical obstacles */
	int n_cuboids;						/*!< Number of cuboidal obstacles */
	int n_planes;						/*!< Number of planar obstacles */
	int n_temp_zones;					/*!< Number of temperature obstacles */
};


/*! Generate new temperature obstacle.  Computes new temperature obstacle primitives from the circular sensor region currently centered at pos = \f$(x,y,z)\f$ [in]
with radius *radius* [in] in the skin surface-normal direction \f$\hat{n} = (n_x,n_y,n_z)\f$.  All quantities must be defined in the body-fixed frame of link *I* 
(currently oriented by joint-angle vector *q*).  Adds the new obstacle to *obs* as a `temp_obs` struct. \see ::DHparams, ::temp_obs, ::obstacles, TempObsViolation
	\param[in]	I				Link frame in which the obstacle, position vector, and normal vector are defined
	\param[in]	pos				Position of maximum-temperature sensor from which the obstacle should emanate
	\param[in]	radius, H, beta	Temperature obstacle cone parameters (radius of frustrum floor [in], total height \f$H\f$ [in], and cone half-angle \f$\beta\f$ [deg])
	\param[in]	offset			Height offset above the sensor along its surface normal at which to start the truncated cone
	\param[in]	n_hat			Direction \f$\hat{n} = (n_x,n_y,n_z)\f$ of the skin surface normal to be used for the cone axis */
void ConstructTempObstacle( int I, double* pos, double radius, double H, double offset, double* n_hat, double beta, struct DHparams *DH, double* q, struct obstacles *obs );


/*! Generate obstacle primitives from parameters determined by `GenerateInput` \see ::obstacles, GenerateInput, main */
void GenerateObstacles( struct obstacles *obs, int n_planes, double* nhat_planes, double* xyz_planes, 
	int n_cylinders, double* r_cylinders, double* H_cylinders, double* xyz_cylinders, double* YPR_cylinders,
	int n_cuboids, double* xyz_cuboids, double* LWH_cuboids, double* YPR_cuboids, int i_grip_obs );
	


#endif