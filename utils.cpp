#include "utils.h"
#include "planning.h" // critical reference here

double ElapsedTime( clock_t start, clock_t stop ) {
	return ((stop - start)*1000)/CLOCKS_PER_SEC;
}


double** Make2DDoubleArray(int nx, int ny) {  
	double** A = (double**) malloc(nx*sizeof(double*));
	for (int i = 0; i < nx; i++)
		A[i] = (double*) malloc(ny*sizeof(double));
	return A;
}

int** Make2DIntArray(int nx, int ny) {  
	int** A	= (int**) malloc(nx*sizeof(int*));
	for (int i = 0; i < nx; i++)
		A[i] = (int*) malloc(ny*sizeof(int));
	return A;
}

double*** Make3DDoubleArray(int nx, int ny, int nz) {
	double*** A = (double***) malloc(nx*sizeof(double**));
	for (int i = 0; i < nx; i++) {
		A[i] = Make2DDoubleArray(ny,nz);
	}
	return A;
}

void VectorDiff(double* v_left, double* v_right, double* v_out, int n) {
	for (int i = 0; i < n; i++) {
		v_out[i] = v_left[i] - v_right[i];
	}
}

double Norm(double* v, int n, double p) {
	double result = 0;
	assert(p >= 1);

	/* Infinity-norm finds the maximum of v */
	if (p == DBL_MAX) {
		result = fabs( v[0] );
		for (int i = 1; i < n; i++) {
			if ( fabs(v[i]) > result ) {
				result = fabs( v[i] );
			}
		}
	}
	/* All other p-norms (p >= 1) return ( v[0]^p + v[1]^p + ... v[end]^p )^(1/p) */
	else {
		for (int i = 0; i < n; i++) {
			result = result + pow( fabs(v[i]), p );
		}
		result = pow( result, 1.0/p );
	}

	return result;
}

double DistSq( double* v1, double* v2, int n, double* w ) {
	double dsq = 0;
	for (int i = 0; i < n; i++)
		dsq += w[i]*(v2[i] - v1[i])*(v2[i] - v1[i]);
	return dsq;
}



int SumInts( int* A, int n ) {
	int sum = 0;
	for (int i = 0; i < n; i++) {
		sum += A[i];
	}

	return sum;
}

void Cross( double* u, double* v, double* w ) {
	w[0] = u[1] * v[2] - u[2] * v[1];
	w[1] = u[2] * v[0] - u[0] * v[2];
	w[2] = u[0] * v[1] - u[1] * v[0];
}

void MatrixMultiply(double** A, double** B, int m, int n, int p, double** C) {
	double sum = 0;

	for ( int i = 0 ; i < m ; i++ ) {
		for ( int j = 0 ; j < p ; j++ ) {
			for ( int k = 0 ; k < n ; k++ ) {
				sum = sum + A[i][k]*B[k][j];
			}
			C[i][j] = sum;
			sum = 0;
		}
	}
}

void Matrix3by3Inverse( double** M, double** M_inv ) {
	double determinant	= 0;		/* Variable for the matrix determinant of M */
	double C_ji;					/* Co-factor C_ji of element M_ji, related to M_inv by:  M_inv_ij = C_ji/det(M) */
	double det_inv;					/* Variable for the inverse of the determinant */
	double epsilon		= 0.0001;	/* Acceptable tolerance for concluding that a determinant equals 0 (and hence that M is non-invertible) */

	/* Compute the determinant */
	for ( int i = 0; i < 3; i++ ) {
		determinant = determinant + M[0][i] * ( M[1][(i+1)%3] * M[2][(i+2)%3] - M[1][(i+2)%3] * M[2][(i+1)%3] );
	}
	
	/* If the determinant is too small, abort */
	assert( determinant > epsilon );
	det_inv = 1.0 / determinant;

	/* Find the inverse of the matrix */
	for ( int i = 0; i < 3; i++ ) {
		for ( int j = 0; j < 3; j++ ) {
			C_ji		= ( M[(j+1)%3][(i+1)%3] * M[(j+2)%3][(i+2)%3] ) - ( M[(j+1)%3][(i+2)%3] * M[(j+2)%3][(i+1)%3] );
			M_inv[i][j] = C_ji * det_inv;
		}
	}

	/* Check that the identity matrix results for M*M^-1 */
	double** Product = Make2DDoubleArray(3,3);
	MatrixMultiply(M,M_inv,3,3,3,Product);
	for ( int i = 0; i < 3; i++ ) {
		for ( int j = 0; j < 3; j++ ) {
			if (i == j) assert( fabs(Product[i][j] - 1) < epsilon );
			else assert( fabs(Product[i][j]) < epsilon );
		}
	}
	free(Product);
}

void ScalarMultiply(double** A, double* c, int m, int n) {

	for ( int i = 0 ; i < m ; i++ ) {
		for ( int j = 0 ; j < n ; j++ ) {
			A[i][j] = (*c)*A[i][j];
		}
	}
}

void IdentityMatrix(double** A, int n) {
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			if (i == j) {
				A[i][j] = 1;
			}
			else {
				A[i][j] = 0;
			}
		}
	}
}

void FindRotMat( double* v, double* v_out, double** R ) {
	
	double a[3], a_prime[3], b[3], b_prime[3], c[3], c_norm;	/* a = v_hat, a' = v_out_hat, c = c' is perp to both, and b, b' each complete the RHR */
	double** R1			= Make2DDoubleArray(3,3);				/* The matrix R1 rotates from the reference frame to a frame x-aligned with v */
	double** R1_inv		= Make2DDoubleArray(3,3);				/* Inverse of R1 */
	double** R3			= Make2DDoubleArray(3,3);				/* The matrix R3 rotates from the reference frame to a frame x-aligned with v_out */
	double v_norm		= Norm(v,3,2.0);						/* Magnitude of v */
	double v_out_norm	= Norm(v_out,3,2.0);					/* Magnitude of v_out */
	double epsilon		= 0.0001;								/* Tolerance for testing non-zero values of v and v_out (for testing degenerate cases) */

	/* Make sure v and v_out are non-zero vectors */
	assert( fabs(v[0]) > epsilon || fabs(v[1]) > epsilon || fabs(v[2]) > epsilon );
	assert( fabs(v_out[0]) > epsilon || fabs(v_out[1]) > epsilon || fabs(v_out[2]) > epsilon );
	
	/* Check if v and v_out are parallel.  If so, output the identity matrix.  If not, continue. */
	if ( fabs( (v[0]*v_out[0] + v[1]*v_out[1] + v[2]*v_out[2])/(v_norm*v_out_norm) - 1 ) < epsilon ) {
		IdentityMatrix( R, 3 );
	}
	else {

		/* Define a and a' */
		for (int i = 0; i < 3; i++) {
			a[i]		= v[i]/v_norm;
			a_prime[i]	= v_out[i]/v_out_norm;
		}
		
		/* Define c and c' (c' = c).  If v and v_out are anti-parallel, use SVD instead of the cross product to find an orthogonal vector. */
		if ( fabs( a[0]*a_prime[0] + a[1]*a_prime[1] + a[2]*a_prime[2] + 1 ) < epsilon ) {
			if (fabs(a[0]) > epsilon) {
				c[2] = pow( 1.0/ (1.0 + (a[2]/a[0])), 0.5 );
				c[0] = -(a[2]/a[0])*c[2];
				c[1] = 0;
			}
			else if (fabs(a[1]) > epsilon) {
				c[0] = 0;
				c[2] = pow( 1.0/ (1.0 + (a[2]/a[1])), 0.5 );
				c[1] = -(a[2]/a[1])*c[2];
			}
			else {
				c[0] = 0;
				c[1] = pow( 1.0/ (1.0 + (a[1]/a[2])), 0.5 );
				c[2] = -(a[1]/a[2])*c[1];
			}
		}
		else {
			Cross(a,a_prime,c);
			c_norm = Norm(c,3,2.0);
			for (int i = 0; i < 3; i++) {
				c[i] = c[i]/c_norm;
			}
		}

		/* Define b and b' (no normalization needed since c _|_ a and c' _|_ a' by definition) */
		Cross(c,a,b);
		Cross(c,a_prime,b_prime);

		/* Define R1 = [a, b, c] and R3 = [a', b', c'] */
		for (int i = 0; i < 3; i++) {
			R1[i][0] = a[i];
			R1[i][1] = b[i];
			R1[i][2] = c[i];
			R3[i][0] = a_prime[i];
			R3[i][1] = b_prime[i];
			R3[i][2] = c[i];
		}

		/* Now, R = R3*R1^-1 */
		Matrix3by3Inverse(R1, R1_inv);
		MatrixMultiply(R3, R1_inv, 3, 3, 3, R);
	}

	for (int i = 2; i >= 0; i--) {
		free(R1[i]); free(R1_inv[i]); free(R3[i]);
	}
	free(R1); free(R1_inv); free(R3);
}

char* Dec2Base(int num, int base, int *ptr_N) {
	/* For bases < 10, use atoi( conversion ) to return integer representation */
	/* Also finds the maximum number of elements, *N, in the converted number ( smallest integer n such that base^n > num ) */
	assert( num >= 0 );
	assert( base >= 2 && base <= 36 );

	/* Allocate memory for the conversion string (extra 1 added for NULL terminator).  Define "digits" for numbers above 9. */
	int n;
	char* conversion = (char*) malloc((1+1)*sizeof(char));
	char digits[] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f', //
			'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'};

	/* Determine the string form of the converted integer in the new base.  If 0 or 1, output the trivial result [necessary to avoid taking log(0) or log(1)]. */
	if ( (num == 0) || (num == 1) ) {
		n = 1;
		conversion[0] = digits[num];
		conversion[1] = NULL;
	}
	else {
		n = (int) floor( ( log((double)num) / log((double)base) ) + 1 );	/* Determines the number of characters needed to represent num in the new base */
		conversion = (char*) realloc(conversion, (n+1)*sizeof(char));		/* Re-allocates memory for the conversion string up to its appropriate size */
		assert(conversion != NULL);

		/* Repeatedly divide "num" by "base", saving the remainders in a string (which form the new representation).
		Builds the converted number from its least significant bit to its most significant bit, padding the front with zeros. */
		int remainder, quotient;
		quotient		= num;
		conversion[n]	= NULL;
		for (int i = 0; i < n; i++) {
			if ( quotient > 0 ) {
				remainder			= quotient % base;		/* (modulus division returns the remainder of a division) */
				quotient			= quotient/base;		/* (integer division <--> floor ) */
				conversion[n-1-i]	= digits[remainder];
			}
			else {
				conversion[n-1-i]	= digits[0];
			}
		}
	}

	*ptr_N = n;
	return conversion;
}

int* Dec2BaseInts(int num, int base, int *ptr_N) {
	assert( num >= 0 );
	assert( base >= 2 && base <= 36 );

	/* Determine the string representation of the decimal-to-base conversion, and allocate memory for each of its N corresponding integers */
	char* conversion_string = Dec2Base( num, base, ptr_N );
	int* conversion_ints = (int*) malloc( (*ptr_N)*sizeof(int));
	
	/* Given the same digit representation as used by Dec2Base, store the integer form corresponding to each character in the string */
	int i, j;
	char digits[] = {'0','1','2','3','4','5','6','7','8','9','a','b','c','d','e','f', //
		'g','h','i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z'};
	for (i = 0; i < (*ptr_N); i++) {
		j = 0;
		while ( j < (signed) strlen(digits) && strncmp( &(conversion_string[i]), &(digits[j]), 1 ) != 0 ) {
			j += 1;
		}
		conversion_ints[i] = j;
	}

	free( conversion_string );
	return conversion_ints;
}


void GenerateInput( char* filename, char* soln, char* sampling, int* max_iter, int* max_neighbors, double* epsilon, int* n, int* n_waypoints, double* q_waypoints, 
	double* q_min, double* q_max, int* grip_actions, double* grip_angles, int* n_facepts, double* L, double* W, double* H, double* rho_x, double* rho_y, 
	double* rho_z, double* d, double* a, double* alpha, int* n_planes, double* nhat_planes, double* xyz_planes, int* n_cylinders, double* YPR_cylinders, 
	double* xyz_cylinders, double* r_cylinders, double* H_cylinders, int* n_cuboids, double* YPR_cuboids, double* LWH_cuboids, double* xyz_cuboids, char load_input ) {
	
	/* Output input data if "load_input" = 'n', otherwise load input values from previous run */
	FILE *infilefid;
	time_t rawtime;

	if (strncmp(&load_input, "n", 1) == 0) {
		printf("Printing input and obstacle data to file...\n");
		infilefid = fopen( strcat(filename, "input.dat"), "w+" );
		time(&rawtime);
		fprintf(infilefid, "%s\n%s\t%s%s\t%s\n%s\t%s\n", "Manipulator RRT Input Data", "Last updated:    ", ctime(&rawtime), "Solution type:   ", soln, "Sampling method: ", sampling);
		fprintf(infilefid, "%s\t%i\n%s\t%i\n%s\t%5.5Lf\n%s\t%i\n%s\t%i\n", "Max. iterations: ", *max_iter, "Max. neighbors:  ", *max_neighbors, "epsilon [deg]:   ", *epsilon, "n:               ", *n, "n_waypoints:     ", *n_waypoints);
		fprintf(infilefid, "%s\t%i\t%i\n%s\t%5.5Lf\t%5.5Lf\t%5.5Lf\n\n", "grip_actions:    ", grip_actions[0], grip_actions[1], "grip_angles [deg]: ", grip_angles[0], grip_angles[1], grip_angles[2]);
		for (int i = 0; i < *n_waypoints; i++) {
			fprintf(infilefid, "%s%d%s\t", "q_WP", i, " ");
		}
		fprintf(infilefid, "%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n", 
			"q_min ", "q_max ", "L", "W", "H", "rho_x ", "rho_y ", "rho_z ", "d", "a", "alpha", "n_ptsXY", "n_ptsYZ", "n_ptsXZ");
		for (int i = 0; i < *n; i++) {
			for (int j = 0; j < *n_waypoints; j++) {
				fprintf(infilefid, "%5.5Lf\t", q_waypoints[(*n)*j + i]);
			}
			fprintf(infilefid, "%5.5Lf\t%5.5Lf\t%5.5Lf\t%5.5Lf\t%5.5Lf\t%5.5Lf\t%5.5Lf\t%5.5Lf\t%5.5Lf\t%5.5Lf\t%5.5Lf\t%i\t%i\t%i\n",
				q_min[i], q_max[i], L[i], W[i], H[i], rho_x[i], rho_y[i], rho_z[i], 
				d[i], a[i], alpha[i], n_facepts[3*i], n_facepts[3*i+1], n_facepts[3*i+2]);
		}
	}
	else if (strncmp(&load_input, "y", 1) == 0) {
		printf("Loading input parameters and settings from file...\n");
		infilefid = fopen( strcat(filename, "input.dat"), "r" );

		//TODO : CHECK if infilfid == NULL

		if(infilefid != NULL)
		{
			fscanf(infilefid, "%*[^\n]\n%*[^\n]\n%*[^\t]\t%s\n%*[^\t]\t%s\n", soln, sampling );
			fscanf(infilefid, "%*[^\t]\t%i\n%*[^\t]\t%i\n%*[^\t]\t%Lf\n%*[^\t]\t%i\n%*[^\t]\t%i\n", max_iter, max_neighbors, epsilon, n, n_waypoints );
			fscanf(infilefid, "%*[^\t]\t%i\t%i\n%*[^\t]\t%Lf\t%Lf\t%Lf\n", &(grip_actions[0]), &(grip_actions[1]), &(grip_angles[0]), &(grip_angles[1]), &(grip_angles[2]) );
			fscanf(infilefid, "\n%*[^\n]\n");
			for (int i = 0; i < *n; i++) {
				for (int j = 0; j < *n_waypoints; j++) {
					fscanf(infilefid, "%Lf\t", &(q_waypoints[(*n)*j + i]) );
				}
				fscanf(infilefid, "%Lf\t%Lf\t%Lf\t%Lf\t%Lf\t%Lf\t%Lf\t%Lf\t%Lf\t%Lf\t%Lf\t%i\t%i\t%i\n",
					&(q_min[i]), &(q_max[i]), &(L[i]), &(W[i]), &(H[i]), &(rho_x[i]), &(rho_y[i]), &(rho_z[i]), 
					&(d[i]), &(a[i]), &(alpha[i]), &(n_facepts[3*i]), &(n_facepts[3*i+1]), &(n_facepts[3*i+2]));
			}
		}

		else
		{
			printf("ERROR: could not open input file.\n");
		}
	}
	if(infilefid != NULL)
	{
		fclose(infilefid);
	}
	filename[strlen(filename)-9] = NULL;

	/* Output obstacles data if "load_input" = 'n', otherwise load obstacle values from previous run */
	FILE *obsfilefid;
	int index = 0, return_val = 1;
	char c;

	if (strncmp(&load_input, "n", 1) == 0) {
		obsfilefid = fopen( strcat(filename, "obstacles.dat"), "w+");
		if(obsfilefid != NULL)
		{
			fprintf(obsfilefid, "%s\n", "Manipulator RRT Obstacle Data");
			fprintf(obsfilefid, "%s\t%s\n", "nhat_planes", "xyz_planes");
			for (int i = 0; i < 3*(*n_planes); i++) {
				fprintf(obsfilefid, "%5.4Lf\t%5.4Lf\n", nhat_planes[i], xyz_planes[i]);
			}
			fprintf(obsfilefid, "\n%s\t%s\t%s\t%s\n", "YPR_cylinders", "xyz_cylinders", "r_cylinders", "H_cylinders"); 
			for (int i = 0; i < 3*(*n_cylinders); i++) {
				if (i < *n_cylinders) {
					fprintf(obsfilefid, "%5.4Lf\t%5.4Lf\t%5.4Lf\t%5.4Lf\n", YPR_cylinders[i], xyz_cylinders[i], r_cylinders[i], H_cylinders[i]);
				}
				else {
					fprintf(obsfilefid, "%5.4Lf\t%5.4Lf\n", YPR_cylinders[i], xyz_cylinders[i]);
				}
			}
			fprintf(obsfilefid, "\n%s\t%s\t%s\n", "YPR_cuboids", "LWH_cuboids", "xyz_cuboids");
			for (int i = 0; i < 3*(*n_cuboids); i++) {
				fprintf(obsfilefid, "%5.4Lf\t%5.4Lf\t%5.4Lf\n", YPR_cuboids[i], LWH_cuboids[i], xyz_cuboids[i]);
			}
		}

		else
		{
			printf("ERROR: could not open obstacle file.\n");
		}
	}
	else if (strncmp(&load_input, "y", 1) == 0) {
		obsfilefid = fopen( strcat(filename, "obstacles.dat"), "r");

		if(obsfilefid != NULL)
		{
			fscanf(obsfilefid, "%*[^\n]\n%*[^\n]\n");
			while (return_val != 0) {
				return_val = fscanf(obsfilefid, "%Lf\t%Lf\n", &(nhat_planes[index]), &(xyz_planes[index]) );
				index += 1;
			}
			*n_planes = (index-1)/3;		index = 0;			return_val = 1;
			fscanf(obsfilefid, "\n%*[^\n]\n");
			while (return_val != 0) {
				return_val = fscanf(obsfilefid, "%Lf\t%Lf%c", &(YPR_cylinders[index]), &(xyz_cylinders[index]), &c );
				if (strncmp(&c, "\t", 1) == 0) {
					fscanf(obsfilefid, "%Lf\t%Lf\n", &(r_cylinders[index]), &(H_cylinders[index]) );
				}
				index += 1;
			}
			*n_cylinders = (index-1)/3;		index = 0;			return_val = 1;
			fscanf(obsfilefid, "\n%*[^\n]\n");
			while ((return_val != 0) && (return_val != EOF)) {
				return_val = fscanf(obsfilefid, "%Lf\t%Lf\t%Lf\n", &(YPR_cuboids[index]), &(LWH_cuboids[index]), &(xyz_cuboids[index]) );
				index += 1;
			}
			*n_cuboids = (index-1)/3;		index = 0;			return_val = 1;
		}
		else
		{
			printf("ERROR: could not open obstacle file.\n");
		}
	}
	if(obsfilefid != NULL)
	{
		fclose(obsfilefid);
	}
	filename[strlen(filename)-13] = NULL;
}
