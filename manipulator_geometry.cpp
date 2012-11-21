#include "manipulator_geometry.h"
#include "utils.h"
#include "planning.h"  // --> probably critical dependency (comment by W.Pointner)

void TransformMatrix( double yaw, double pitch, double roll, double* trans, double** T ) {
	yaw		= yaw*(PI/180);
	pitch	= pitch*(PI/180);
	roll	= roll*(PI/180);

	T[0][0] = cos(pitch) * cos(yaw);
	T[0][1] = cos(pitch) * sin(yaw);
	T[0][2] = -sin(pitch);
	T[0][3] = trans[0];
	T[1][0] = sin(roll)*sin(pitch)*cos(yaw) - cos(roll)*sin(yaw);
	T[1][1] = sin(roll)*sin(pitch)*sin(yaw) + cos(roll)*cos(yaw);
	T[1][2] = sin(roll)*cos(pitch);
	T[1][3] = trans[1];
	T[2][0] = cos(roll)*sin(pitch)*cos(yaw) + sin(roll)*sin(yaw);
	T[2][1] = cos(roll)*sin(pitch)*sin(yaw) - sin(roll)*cos(yaw);
	T[2][2] = cos(roll)*cos(pitch);
	T[2][3] = trans[2];
	T[3][0] = 0.0;
	T[3][1] = 0.0;
	T[3][2] = 0.0;
	T[3][3] = 1.0;
}

void InvTransformMatrix( double yaw, double pitch, double roll, double* trans, double** T ) {
	yaw		= yaw*(PI/180);
	pitch	= pitch*(PI/180);
	roll	= roll*(PI/180);

	T[0][0] = cos(pitch) * cos(yaw);
	T[0][1] = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
	T[0][2] = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll);
	T[0][3] = -( T[0][0]*trans[0] + T[0][1]*trans[1] + T[0][2]*trans[2] );
	T[1][0] = sin(yaw)*cos(pitch);
	T[1][1] = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
	T[1][2] = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);
	T[1][3] = -( T[1][0]*trans[0] + T[1][1]*trans[1] + T[1][2]*trans[2] );
	T[2][0] = -1.0*sin(pitch);
	T[2][1] = cos(pitch)*sin(roll);
	T[2][2] = cos(pitch)*cos(roll);
	T[2][3] = -( T[2][0]*trans[0] + T[2][1]*trans[1] + T[2][2]*trans[2] );
	T[3][0] = 0.0;
	T[3][1] = 0.0;
	T[3][2] = 0.0;
	T[3][3] = 1.0;
}

void HomTransformMatrix( double a, double d, double q, double alpha, double** T ) {
	q		= q*(PI/180);
	alpha	= alpha*(PI/180);

	T[0][0] = cos( q );
	T[0][1] = -1.0*sin( q );
	T[0][2] = 0.0;
	T[0][3] = a;
	T[1][0] = sin( q )*cos( alpha );
	T[1][1] = cos( q )*cos( alpha );
	T[1][2] = -1.0*sin( alpha );
	T[1][3] = -1.0*sin( alpha )*d;
	T[2][0] = sin( q )*sin( alpha );
	T[2][1] = cos( q )*sin( alpha );
	T[2][2] = cos( alpha );
	T[2][3] = cos( alpha )*d;
	T[3][0] = 0.0;
	T[3][1] = 0.0;
	T[3][2] = 0.0;
	T[3][3] = 1.0;
}

void InvHomTransformMatrix( double a, double d, double q, double alpha, double** T ) {
	q		= q*(PI/180);
	alpha	= alpha*(PI/180);

	T[0][0] = cos( q );
	T[0][1] = sin( q )*cos( alpha );
	T[0][2] = sin( q )*sin( alpha );
	T[0][3] = -1.0*a*cos( q );
	T[1][0] = -1.0*sin( q );
	T[1][1] = cos( q )*cos( alpha );
	T[1][2] = cos( q )*sin( alpha );
	T[1][3] = a*sin( q );
	T[2][0] = 0.0;
	T[2][1] = -1.0*sin( alpha );
	T[2][2] = cos( alpha );
	T[2][3] = -1.0*d;
	T[3][0] = 0.0;
	T[3][1] = 0.0;
	T[3][2] = 0.0;
	T[3][3] = 1.0;
}

void BodyFixedOBBcoords( struct geom *G, int* n_facepts, double* grip_pos, double grip_sep, int n ) {

	/* Use the Halton sampling sequence to optimally cover the OBB faces with the given number of face points */
	int maxval = -1;
	for (int i = 0; i < 3*n; i++) { if (n_facepts[i] > maxval) { maxval = n_facepts[i]; } }

	int* sequence = (int*) malloc( maxval*sizeof(int) );	// Will not cause an error if maxval is 0 (only if we attempt to access it)
	for (int i = 0; i < maxval; i++) {
		sequence[i] = i;
	}
	double** U = Make2DDoubleArray(maxval, 2);
	Halton( sequence, maxval, 2, U );
	
	/* Determine the total number of points for all OBB faces of each link i and the end-effector tips */
	int* N_facepts = (int*) malloc( (n+2)*sizeof(int) );
	for (int i = 0; i <= n+1; i++) {
		N_facepts[i] = 2*(n_facepts[3*i] + n_facepts[3*i+1] + n_facepts[3*i+2]);
	}

	/* Loop over all links of the manipulator, plus the two 
	end effector tips */

	double **T, rho[3], L, W, H, dx[8], dy[8], dz[8];
	
	// TODO: check if this should really be i <= (n+1)
	for (int i = 0; i <= n+1; i++) {
		rho[0]	= G->rho_x[i];
		rho[1]	= G->rho_y[i];
		rho[2]	= G->rho_z[i];
		L		= G->L[i];
		W		= G->W[i];
		H		= G->H[i];

		/* Apply end-effector gripper separation to end-effector tip OBB's */
		if (i == n) {			rho[2] += grip_sep/2.0;		/* Right end-effector tip */
		} else if (i == n+1) {	rho[2] -= grip_sep/2.0;		/* Left end-effector tip */
		}

		/* Initialize an OBB coordinate array for each link/end-effector-tip i (the last term 
		allocates an additional unit on the last iteration for the end effector position -- see below) */
		T		= Make2DDoubleArray(3, 8 + N_facepts[i] + (i/(n+1)) );

		/* Add the body-fixed OBB corner points of link i to T */
		dx[0] = 0; dx[1] = 0; dx[2] = 0; dx[3] = 0; dx[4] = L; dx[5] = L; dx[6] = L; dx[7] = L;
		dy[0] = 0; dy[1] = 0; dy[2] = W; dy[3] = W; dy[4] = W; dy[5] = W; dy[6] = 0; dy[7] = 0;
		dz[0] = 0; dz[1] = H; dz[2] = 0; dz[3] = H; dz[4] = 0; dz[5] = H; dz[6] = 0; dz[7] = H;

		for (int j = 0; j < 8; j++) {
			T[0][j] = rho[0] + dx[j];
			T[1][j] = rho[1] + dy[j];
			T[2][j] = rho[2] + dz[j];
		}

		/* Add the body-fixed OBB face points of link i to T */
		for (int j = 0; j < (N_facepts[i]/2); j++) {
			if ( j < n_facepts[3*i] ) {
				T[0][8 + 2*j]	= rho[0] + L*U[j][0];
				T[1][8 + 2*j]	= rho[1] + W*U[j][1];
				T[2][8 + 2*j]	= rho[2];
				T[0][8 + 2*j+1]	= T[0][8 + 2*j];
				T[1][8 + 2*j+1]	= T[1][8 + 2*j];
				T[2][8 + 2*j+1]	= T[2][8 + 2*j] + H;
			}
			else if ( j < (n_facepts[3*i] + n_facepts[3*i+1]) ) {
				T[0][8 + 2*j]	= rho[0];
				T[1][8 + 2*j]	= rho[1] + W*U[j-n_facepts[3*i]][0];
				T[2][8 + 2*j]	= rho[2] + H*U[j-n_facepts[3*i]][1];
				T[0][8 + 2*j+1]	= T[0][8 + 2*j] + L;
				T[1][8 + 2*j+1]	= T[1][8 + 2*j];
				T[2][8 + 2*j+1]	= T[2][8 + 2*j];
			}
			else {
				T[0][8 + 2*j]	= rho[0] + L*U[j-n_facepts[3*i]-n_facepts[3*i+1]][0];
				T[1][8 + 2*j]	= rho[1];
				T[2][8 + 2*j]	= rho[2] + H*U[j-n_facepts[3*i]-n_facepts[3*i+1]][1];
				T[0][8 + 2*j+1]	= T[0][8 + 2*j];
				T[1][8 + 2*j+1]	= T[1][8 + 2*j] + W;
				T[2][8 + 2*j+1]	= T[2][8 + 2*j];
			}
		}

		/* Save the points and total number of coordinates to geometry structure G for each link i */
		G->Body_coords[i]	= T;
		// TODO: T is allocated every time and then body_coords are set to this data
		// memory for G->Body_coords is still allocated but not used anymore
		G->N_coords[i]		= 8 + N_facepts[i];

		/* If on the last iteration, add the end effector point as the final body-fixed coordinate 
		(used for visualization only, not collision detection; added here for ease of implementation with PlotEndEffectorPathInMATLAB) */
		if (i == (n+1)) {
			G->Body_coords[i][0][ G->N_coords[i] ] = grip_pos[0];
			G->Body_coords[i][1][ G->N_coords[i] ] = grip_pos[1];
			G->Body_coords[i][2][ G->N_coords[i] ] = grip_pos[2];
			G->N_coords[i] += 1;
		}
	}

	/* Free memory (do not free T!) */
	for (int i = maxval-1; i >= 0; i--) {
		free(U[i]);
	}
	free(U); free(N_facepts); free(sequence);
}

void WorldCoords( struct geom *G, struct DHparams *DH, double* q, int n, struct coords *C ) {
	
	double** T			= Make2DDoubleArray(4,4);
	double** Timinus1	= Make2DDoubleArray(4,4);
	double** Ti			= Make2DDoubleArray(4,4);
	double **v			= Make2DDoubleArray(4,1);
	double **v_new		= Make2DDoubleArray(4,1);
	int sum = 0;

	IdentityMatrix(Timinus1,4);

	for (int i = 0; i < n+2; i++) {
		/* Find the transformation matrix from the link i to the world frame.  If i >= n, i corresponds to an end-effector tip (do not update T). */
		if (i < n) {
			HomTransformMatrix( DH->a[i], DH->d[i], q[i], DH->alpha[i], Ti );
			MatrixMultiply( Timinus1, Ti, 4, 4, 4, T );
		}
		
		/* Transform the body-fixed OBB coords of link i to the world frame and save them to C */
		for (int j = 0; j < G->N_coords[i]; j++) {
			v[0][0] = G->Body_coords[i][0][j];
			v[1][0] = G->Body_coords[i][1][j];
			v[2][0] = G->Body_coords[i][2][j];
			v[3][0] = 1;

			MatrixMultiply( T, v, 4, 4, 1, v_new );
			C->x[sum + j] = v_new[0][0];
			C->y[sum + j] = v_new[1][0];
			C->z[sum + j] = v_new[2][0];
		}
		sum = sum + G->N_coords[i];

		/* Update T_(i-1) if the next index corresponds to a link.  Otherwise, stop updating (end-effector tips use the matrix T of link n). */
		if (i+1 < n) {
			for (int j = 0; j < 4; j++) {
				for (int k = 0; k < 4; k++) {
					Timinus1[j][k] = T[j][k];
				}
			}
		}
	}

	for (int i = 3; i >= 0; i--) {
		free(T[i]); free(Timinus1[i]); free(Ti[i]); free(v[i]); free(v_new[i]);
	}
	free(T); free(Timinus1); free(Ti); free(v); free(v_new);
}

