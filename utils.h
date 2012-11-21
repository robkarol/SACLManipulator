#ifndef __UTILS_H__
#define __UTILS_H__


#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <float.h>
#include <windows.h>
#include <time.h>

#ifndef PI
	#define PI 3.1415926535897932385
#endif
/*! Array size above which an \f$O(n \log{n})\f$ sorting algorithm should be used instead of InsertionSort */
#ifndef SORT_SWITCH
	#define SORT_SWITCH 100
#endif
/*! Determines the number of elements in a non-dynamically allocated array */
#ifndef Numel
	#define Numel(x)  (sizeof(x) / sizeof(x[0]))
#endif
/*! Round a number to the nearest integer depending on its sign and fractional part (CURRENTLY UNUSED) */
#ifndef Round
	#define Round(number) (number >= 0) ? (int)(number + 0.5) : (int)(number - 0.5);
#endif
/*! Rounds a double to a desired scale, e.g.\ resolution = 0.5 rounds to nearest half-integer (CURRENTLY UNUSED) */
#ifndef Round2Res
	#define Round2Res(value, resolution) ( ((double) Round(value/resolution)) * resolution )
#endif
/*! Returns the maximum of two values \f$a\f$ and \f$b\f$ */
#ifndef Max
	#define Max( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif
/*! Returns the minimum of two values \f$a\f$ and \f$b\f$ */
#ifndef Min
	#define Min( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif
/*! Returns the square root of a value (no error-checking - value must be non-negative) */
#ifndef Sqrt
	#define Sqrt(x) ( pow(x,0.5) )
#endif
/*! Copies a `va_list` ("variable argument" list) pointer to another `va_list` pointer (CURRENLTY UNUSED) */
#ifndef va_copy
	#define va_copy(dest,src) ((dest) = (src))
#endif

/*! Enumerated list of datatypes.  Enumerated list of allowable datatypes for the custom function `SendArraysToMATLAB` that transfers 
C/C++ variables to the MATLAB Engine environment.  This is used to send data to MATLAB prior to plotting.  Only the datatypes listed
here have been coded for transfer.  Note some of the datatypes in this list are automatically promoted to alternate datatypes as required by `va_arg`. */
typedef enum datatypes {
	Long,								/*!< Long integer (saved in MATLAB as uint64) */
	Short, 								/*!< Short integer (saved in MATLAB as uint32, promoted to int) */
	Char,								/*!< Char (saved in MATLAB as uint32, promoted to int) */
	Int,								/*!< Signed integer (saved in MATLAB as uint32) */
	Float,								/*!< Float (saved in MATLAB as double, promoted to double) */
	Double								/*!< Double (saved in MATLAB as double) */
} Type;


/*! Return the elapsed time between two clock timers in milliseconds
	\param[in] start Beginning timer, generated using `clock()`
	\param[in] stop  Ending timer, generated using `clock()` */
double ElapsedTime( clock_t start, clock_t stop );

/*! Creates a dynamic 2D `double` array of pointers of size *nx* rows by *ny* cols */
double** Make2DDoubleArray(int nx, int ny);

/*! Creates a dynamic 2D `int` array of pointers of size *nx* rows by *ny* cols */
int** Make2DIntArray(int nx, int ny);

/*! Creates a dynamic 3D `double` array of pointers of size *nx* x *ny* x *nz* */
double*** Make3DDoubleArray(int nx, int ny, int nz);

/*! Compute the vector difference, *v_left* - *v_right*, for two `double` vectors of length *n* */
void VectorDiff(double* v_left, double* v_right, double* v_out, int n);

/*! Compute the *p*-norm of a `double` vector *v* of size *n* (Ex: *p* = 1 for Manhattan-norm, *p* = 2 for Euclidean-norm, or *p* = `DBL_MAX` for \f$\infty\f$-norm) */
double Norm(double* v, int n, double p);

/*! Compute the weighted Euclidean distance-squared, \f$d = \left(\sqrt{w \cdot (v_2 - v_1)} \right)^2\f$ between two vectors of dimension *n*
	\param[in] v1,v2	Vectors used in difference
	\param[in] n		Number of elements in each vector
	\param[in] w		Vector of weighting factors, one for each dimension */
double DistSq( double* v1, double* v2, int n, double* w );


/*! Sum integers in `int` array \f$A\f$ of length \f$n\f$ */
int SumInts( int* A, int n );

/*! Compute the vector cross product, \f$w = u \times v\f$, for 3-D `double` vectors *u* and *v* */
void Cross( double* u, double* v, double* w );

/*! Compute the matrix product of two 2-D `double` arrays, \f$A*B\f$, where \f$A\f$ is size \f$(m \times n)\f$ and \f$B\f$ is size \f$(n \times p)\f$ */
void MatrixMultiply(double** A, double** B, int m, int n, int p, double** C);

/*! Compute the inverse of a \f$3\times3\f$ matrix \f$M\f$ through calculation of its adjugate, adj\f$(M) = C^T\f$, where \f$C\f$ is the matrix of co-factors of \f$M\f$ */
void Matrix3by3Inverse( double** M, double** M_inv );

/*! Compute the scalar multiplication of a 2-D `double` array, \f$A\f$ (size \f$m \times n\f$), with `double` \f$c\f$ */
void ScalarMultiply(double** A, double* c, int m, int n);

/*! Initialize the values of matrix \f$A\f$ (size \f$n \times n\f$) as an identity matrix \f$I_n\f$ */
void IdentityMatrix(double** A, int n);

/*! Find a rotation matrix \f$R\f$ from its operand, \f$v\f$, and output, \f$v_{out}\f$, \f$3\times1\f$ column vectors */
void FindRotMat( double* v, double* v_out, double** R );

/*! Convert a base-10 integer *num* to arbitrary base between 2 and 36, returning its string representation and its length *N* \see Dec2BaseInts */
char* Dec2Base(int num, int base, int *ptr_N);

/*! Convert a base-10 integer *num* to arbitrary base between 2 and 36, returning its vector-of-integers representation and its length *N* \see Dec2Base */
int* Dec2BaseInts(int num, int base, int *ptr_N);


/*! Print user input values to file or load values from previous run.  Generates/calls <*filename*>input.dat and <*filename*>obstacles.dat depending
on the setting *load_input* specified by the user.  Requires that *filename* fully-specify the path and filename root.  \see main
	\param[in] filename		String containing the full path + root of input filename
	\param[in] load_input	`char` indicating whether to load a previous file ('y') or save a new one ('n') */
void GenerateInput( char* filename, char* soln, char* sampling, int* max_iter, int* max_neighbors, double* epsilon, 
					int* n, int* n_waypoints, double* q_waypoints, double* q_min, double* q_max, int* grip_actions, 
					double* grip_angles, double* grip_sep, double* grip_pos, int* n_facepts, double* L, double* W, double* H, 
					double* rho_x, double* rho_y, double* rho_z, double* d, double* a, double* alpha, int* n_planes, 
					double* nhat_planes, double* xyz_planes, int* n_cylinders, double* YPR_cylinders, double* xyz_cylinders, 
					double* r_cylinders, double* H_cylinders, int* n_cuboids, double* YPR_cuboids, double* LWH_cuboids, 
					double* xyz_cuboids, char load_input );
	

#endif