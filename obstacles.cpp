#include "obstacles.h"
#include "utils.h"
#include "manipulator_geometry.h"


void ConstructTempObstacle( int I, double* pos, double radius, double H, double offset, double* n_hat, double beta, struct DHparams *DH, double* q, struct obstacles *obs )  {

	int i, j, k;
	double** T_inv			= Make2DDoubleArray(4,4);
	double** Timinus1_inv	= Make2DDoubleArray(4,4);
	double** Ti_inv			= Make2DDoubleArray(4,4);
	double** T_inv_cone		= Make2DDoubleArray(4,4);
	double** T_inv_total	= Make2DDoubleArray(4,4);

	/* Find the transformation matrix, T_inv, that transforms coordinates from the world space to the cone frame */
	IdentityMatrix(Timinus1_inv,4);
	for (i = 0; i < I; i++) {
		InvHomTransformMatrix( DH->a[i], DH->d[i], q[i], DH->alpha[i], Ti_inv );
		MatrixMultiply( Ti_inv, Timinus1_inv, 4, 4, 4, T_inv );

		for (j = 0; j < 4; j++) {
			for (k = 0; k < 4; k++) {
				Timinus1_inv[j][k] = T_inv[j][k];
			}
		}
	}

	/* Find the distance from the sensor position to the cone apex, according to the radius at pos and the half-angle beta */
	assert( (beta > 0) && (beta < 90) );
	double h1 = radius / tan( beta*(PI/180) );

	/* Determine the rotation matrix from link I's current body-fixed frame to the cone frame */
	double cone_axis[3] = {0, 0, 1};
	double** R = Make2DDoubleArray(3,3);
	FindRotMat( n_hat, cone_axis, R );

	/* Compute the transformation matrix from link I's current body-fixed frame to the cone frame, with origin at the cone apex */
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			if ((i < 3) && (j < 3)) {
				T_inv_cone[i][j] = R[i][j];
			}
			else if ((i == 3) && (j < 3)) {
				T_inv_cone[i][j] = 0;
			}
			else if ((i < 3) && (j == 3)) {
				T_inv_cone[i][j] = -( T_inv_cone[i][0]*(pos[0] - h1 * n_hat[0]) + T_inv_cone[i][1]*(pos[1] - h1 * n_hat[1]) + T_inv_cone[i][2]*(pos[2] - h1 * n_hat[2]) );
				//T_inv_cone[i][j] = pos[i] - h1 * n_hat[i];
			}
			else {
				T_inv_cone[i][j] = 1;
			}
		}
	}

	/* Append the cone transformation to the transformation to link I to obtain the total transformation from the world space to the cone */
	MatrixMultiply( T_inv_cone, T_inv, 4, 4, 4, T_inv_total );

	/* Store the results in the "temp obstacle" structure, temp_zones */
	obs->n_temp_zones		= obs->n_temp_zones + 1;
	obs->temp_zones->beta	= (double*) realloc(obs->temp_zones->beta, (obs->n_temp_zones)*sizeof(double));
	obs->temp_zones->h1		= (double*) realloc(obs->temp_zones->h1, (obs->n_temp_zones)*sizeof(double));
	obs->temp_zones->h2		= (double*) realloc(obs->temp_zones->h2, (obs->n_temp_zones)*sizeof(double));
	obs->temp_zones->Tinv	= (double***) realloc(obs->temp_zones->Tinv, (obs->n_temp_zones)*sizeof(double**));

	obs->temp_zones->beta[obs->n_temp_zones-1]	= beta;
	obs->temp_zones->h1[obs->n_temp_zones-1]	= h1 + offset;
	obs->temp_zones->h2[obs->n_temp_zones-1]	= h1 + H + offset;
	obs->temp_zones->Tinv[obs->n_temp_zones-1]	= T_inv_total;

	/* Free un-needed memory (do not free T_inv_total!) */
	for (i = 2; i >= 0; i--) {
		free(R[i]);
	}
	free(R);
	for (i = 3; i >= 0; i--) {
		free(T_inv[i]); free(Timinus1_inv[i]); free(Ti_inv[i]); free(T_inv_cone[i]);
	}
	free(T_inv); free(Timinus1_inv); free(Ti_inv); free(T_inv_cone);
}


void GenerateObstacles( struct obstacles *obs, int n_planes, double* nhat_planes, double* xyz_planes, 
	int n_cylinders, double* r_cylinders, double* H_cylinders, double* xyz_cylinders, double* YPR_cylinders,
	int n_cuboids, double* xyz_cuboids, double* LWH_cuboids, double* YPR_cuboids, int i_grip_obs ) {
	
	/* Save planar obstacles in the form: f = (a*x + b*y + c*z + d) < 0 */
	struct planar_obs* planes = (struct planar_obs*) malloc( sizeof(struct planar_obs) );
	planes->a	= (double*) malloc(n_planes*sizeof(double));
	planes->b	= (double*) malloc(n_planes*sizeof(double));
	planes->c	= (double*) malloc(n_planes*sizeof(double));
	planes->d	= (double*) malloc(n_planes*sizeof(double));
	for (int i = 0; i < n_planes; i++) {
		planes->a[i] = nhat_planes[3*i];
		planes->b[i] = nhat_planes[3*i+1];
		planes->c[i] = nhat_planes[3*i+2];
		planes->d[i] = -1.0*( planes->a[i]*xyz_planes[3*i] + planes->b[i]*xyz_planes[3*i+1] + planes->c[i]*xyz_planes[3*i+2] );
	}

	/* Save cylindrical obstacles as: f = (r - R) < 0 */
	struct cylindrical_obs* cylinders = (struct cylindrical_obs*) malloc( sizeof(struct cylindrical_obs) );
	double CG[3];
	cylinders->r = (double*) malloc(n_cylinders*sizeof(double));
	cylinders->H = (double*) malloc(n_cylinders*sizeof(double));
	cylinders->Tinv = (double***) malloc(n_cylinders*sizeof(double**));
	for (int i = 0; i < n_cylinders; i++) {
		cylinders->r[i] = r_cylinders[i];
		cylinders->H[i] = H_cylinders[i];
		cylinders->Tinv[i] = Make2DDoubleArray(4, 4);
		CG[0] = xyz_cylinders[3*i];
		CG[1] = xyz_cylinders[3*i+1];
		CG[2] = xyz_cylinders[3*i+2];
		InvTransformMatrix( YPR_cylinders[3*i], YPR_cylinders[3*i+1], YPR_cylinders[3*i+2], CG, cylinders->Tinv[i]);
	}

	/* Save cuboidal obstacles as groups of 6 planes, all 6 of which must be satisfied as f < 0 for collision */
	struct cuboid_obs* cuboids = (struct cuboid_obs*) malloc( sizeof(struct cuboid_obs) );
	cuboids->a = (double*) malloc(6*n_cuboids*sizeof(double));
	cuboids->b = (double*) malloc(6*n_cuboids*sizeof(double));
	cuboids->c = (double*) malloc(6*n_cuboids*sizeof(double));
	cuboids->d = (double*) malloc(6*n_cuboids*sizeof(double));
	cuboids->T = (double***) malloc(n_cuboids*sizeof(double**));
	for (int i = 0; i < n_cuboids; i++) {
		CG[0]		= xyz_cuboids[3*i];
		CG[1]		= xyz_cuboids[3*i+1];
		CG[2]		= xyz_cuboids[3*i+2];
		double** T	= Make2DDoubleArray(4, 4);
		InvTransformMatrix( YPR_cuboids[3*i], YPR_cuboids[3*i+1], YPR_cuboids[3*i+2], CG, T );
		cuboids->T[i] = T;

		/* Intend a transformation to the cuboid frame by yaw-pitch-roll of the world frame, followed 
		by translation v = CG of coordinates to cuboid center.  To transform cuboid planes a'x' + b'y' + c'z' + d' = 0
		into the world frame (ax + by + cz + d = 0), we need to apply an inverse transformation to the vector [x' y' z'].
		Note to invert the first rotation of the FRAME and not the coordinates, we must take the transpose of the usual result
		Tinv = [R^-1, v^-1; 0, 1] --> need Tinv = [(R^-1)^-1, v^-1; 0, 1] = [R, v^-1; 0, 1]. */

		/* Front face											Back face */
		cuboids->a[6*i] = T[0][0];								cuboids->a[6*i+1] = -T[0][0];	
		cuboids->b[6*i] = T[1][0];								cuboids->b[6*i+1] = -T[1][0];	
		cuboids->c[6*i] = T[2][0];								cuboids->c[6*i+1] = -T[2][0];	
		cuboids->d[6*i] = T[0][3] - LWH_cuboids[3*i]/2.0;		cuboids->d[6*i+1] = -T[0][3] - LWH_cuboids[3*i]/2.0;

		/* Right face											Left face */
		cuboids->a[6*i+2] = T[0][1];							cuboids->a[6*i+3] = -T[0][1];
		cuboids->b[6*i+2] = T[1][1];							cuboids->b[6*i+3] = -T[1][1];
		cuboids->c[6*i+2] = T[2][1];							cuboids->c[6*i+3] = -T[2][1];
		cuboids->d[6*i+2] = T[1][3] - LWH_cuboids[3*i+1]/2.0;	cuboids->d[6*i+3] = -T[1][3] - LWH_cuboids[3*i+1]/2.0;

		/* Top face												Bottom face */
		cuboids->a[6*i+4]  = T[0][2];							cuboids->a[6*i+5]  = -T[0][2];
		cuboids->b[6*i+4]  = T[1][2];							cuboids->b[6*i+5]  = -T[1][2];
		cuboids->c[6*i+4]  = T[2][2];							cuboids->c[6*i+5]  = -T[2][2];
		cuboids->d[6*i+4]  = T[2][3] - LWH_cuboids[3*i+2]/2.0;	cuboids->d[6*i+5]  = -T[2][3] - LWH_cuboids[3*i+2]/2.0;
	}

	/* Re-position the cuboidal obstacle corresponding to the grasped object to the final element of "cuboids" (for easy removal later) */
	double temp_dbl;
	if ( i_grip_obs != (n_cuboids-1) ) {
		for (int j = 0; j < 6; j++) {
			temp_dbl						= cuboids->a[6*i_grip_obs+j];
			cuboids->a[6*i_grip_obs+j]		= cuboids->a[6*(n_cuboids-1)+j];
			cuboids->a[6*(n_cuboids-1)+j]	= temp_dbl;
			temp_dbl						= cuboids->b[6*i_grip_obs+j];
			cuboids->b[6*i_grip_obs+j]		= cuboids->b[6*(n_cuboids-1)+j];
			cuboids->b[6*(n_cuboids-1)+j]	= temp_dbl;
			temp_dbl						= cuboids->c[6*i_grip_obs+j];
			cuboids->c[6*i_grip_obs+j]		= cuboids->c[6*(n_cuboids-1)+j];
			cuboids->c[6*(n_cuboids-1)+j]	= temp_dbl;
			temp_dbl						= cuboids->d[6*i_grip_obs+j];
			cuboids->d[6*i_grip_obs+j]		= cuboids->d[6*(n_cuboids-1)+j];
			cuboids->d[6*(n_cuboids-1)+j]	= temp_dbl;
		}
	}

	/* Save conical obstacles */
	struct cone_obs* cones = (struct cone_obs*) malloc( sizeof(struct cone_obs) );

	/* Save a structure for temperature obstacles, to be added individually during closed-loop control if necessary */
	struct temp_obs* temp_zones = (struct temp_obs*) malloc( sizeof(struct temp_obs) );
	temp_zones->beta	= (double*) malloc( sizeof(double) );
	temp_zones->h1		= (double*) malloc( sizeof(double) );
	temp_zones->h2		= (double*) malloc( sizeof(double) );
	temp_zones->Tinv	= (double***) malloc( sizeof(double**) );

	/* Save everything to the obstacles structure */
	obs->cuboids	= cuboids;			obs->n_cuboids		= n_cuboids;
	obs->cylinders	= cylinders;		obs->n_cylinders	= n_cylinders;
	obs->planes		= planes;			obs->n_planes		= n_planes;
	obs->cones		= cones;			obs->n_cones		= 0;
	obs->temp_zones	= temp_zones;		obs->n_temp_zones	= 0;
}