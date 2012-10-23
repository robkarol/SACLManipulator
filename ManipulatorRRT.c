/*! \file ManipulatorRRT.cpp
    \brief Source file for the control code of the Stanford Structures and Composites Lab manipulator arm.

Code for controlling the Stanford Structures and Composites Lab manipulator arm.  Generates a closed-loop motion plan using a
variant of the `RRT*-Connect` Algorithm that has been adapted for fast re-planning and efficient motion plan constraint evaluation.

**Written by:**			Joseph A. Starek, Aero/Astro PhD Student, Autonomous Systems Lab (ASL) <BR>
**Last updated:**		10/15/2012 <BR>
**Original use:**		Windows 7 x64 OS, Microsoft Visual Studio 2010 Express, MATLAB 2011a Student Version <BR>

**Assumptions:**
  - A manipulator arm is tasked with traversing a sequence of known joint angle ("configuration") waypoints
  - A single cuboidal object is grasped and dropped during the course of the sequence, in between two of the specified waypoints
  - A weighted Euclidean distance-squared metric in joint space (configuration space) is used to represent the cost of motion
	- Arm servos are controlled by Phidget 21 control boards
	- At most one temperature violation can occur per iteration of a feedback cycle
	- Temperature obstacles do not move from their original position and are not removed once introduced.

**Units:**
	- All angles are input in degrees and converted to radians where necessary
	- Currently, lengths are given in units of "inches"
	- The Phidget controllers rely on internal scales for units, dimensions unknown (standard ranges are given when possible)

**Output:**
	- Data files, determined by the *filename* variable representing the *full path* + *filename*, specified at the top of the `int main()` function
	- <*filename*>input.dat		- List of major variables specified by the user, including waypoints, C-space dimension, and run-time parameters
    - <*filename*>obstacles.dat - Static obstacle parameters specified by the user
    - <*filename*>samples.dat	- Coordinate file of all sample points used during the generation of the pre-computed RRT's (prior to motion control)
		- <*filename*>trees.dat		- Detailed list of contents stored in all pairs of pre-computed RRT's (prior to motion control)
		- <*filename*>output.dat	- Reference joint angle commands determined during pre-computation (prior to motion control)
	- Various MATLAB plots generated during runtime to provide visualization into the progress and accuracy of the code (can be repressed)
	- Feedback motion control of the SACL manipulator arm through a joint angle command history sent to the Phidget 21 control boards

**Portability Concerns:**
	- `SendArraysToMATLAB` uses structured error handling (\_\_try/\_\_except statements) to read data appropriately (catches access violations), which is Windows-specific
	- The variables *filename* and *tempsensor_file* defined at the top of `main` must be redefined for any new computer running this code
	- For the most part, C-style comments are used; however, some C++ commenting remains, which may be in conflict with some compilers
	- Most but not all switch statements use a "default" statement
	- Function names are must longer than 6 characters, which can conflict with some systems
	- Includes "windows.h", which is compatible only with Windows systems

**Instructions for Setup:**
	1. Install a copy of MATLAB with MATLAB Engine capability (should likely need to be 2010 or later)
		- Set Windows Path variable in Advanced System Settings \f$\rightarrow\f$ Environment Variables:<BR>
			**C:\\Program Files\\MATLAB\\R2011a Student\\bin\\win64** (or the path where "libeng.dll" is located)
	2. Install the Phidgets21 Library
		- Go to http://www.phidgets.com/ \f$\rightarrow\f$ Programming tab \f$\rightarrow\f$ C/C++ link \f$\rightarrow\f$ Quick Downloads \f$\rightarrow\f$
		**64 bit Windows Driver and Library Files (Zipped)** (or 32 bit if your OS is 32 bit)
		- You should confirm that C: \f$\rightarrow\f$ Program Files \f$\rightarrow\f$ Phidgets contains "phidget21.h" and
		\f$\rightarrow\f$ x86 contains "phidget21.lib"
	3. Install a copy of the latest KD-tree code (http://code.google.com/p/kdtree/)
		- Save the folder "kdtree-x.y.z", where x, y, and z are the version numbers, to any directory you like
		- Let's call the directory with this folder *kdtree-dir*
		- Create a "kdtree.obj" file, a static library of KD-tree functions recognizable by MS Visual Studio
			+ Open a session of Microsoft Visual Studio 2010 or above.
			+ Go to File \f$\rightarrow\f$ New \f$\rightarrow\f$ Project. Select "Win32 Console Application".
			+ For "Name", enter "kdtree" and click OK.
			+ On the Win32 Application Wizard, click Next and select "DLL" for the Application Type.  Check "Empty Project" and click "Finish".
			+ In the Solution Explorer pane on the left, right click Header Files \f$\rightarrow\f$ Add \f$\rightarrow\f$ New Item.
			Browse to *kdtree-dir*\\kdtree-x.y.z\\kdtree.h and add it to the project.
			+ In the Solution Explorer pane on the left, right click Source Files \f$\rightarrow\f$ Add \f$\rightarrow\f$ New Item.
			Browse to *kdtree-dir*\\kdtree-x.y.z\\kdtree.c and add it to the project.
			+ Under the Solution Configurations bar next to the green arrow, switch "Debug" to "Release".  Press F5 or click the green arrow to
			build the library file.
			+ When finished (even if there are some warnings), navigate to your kdtree solution folder for the MS Visual Studio project.  Go to
				\f$\rightarrow\f$ kdtree_DLL \f$\rightarrow\f$ Release and check that the "kdtree.obj" was generated.
		- Move the "kdtree.obj" file to *kdtree-dir*\\kdtree-x.y.z
	4. Unzip and save the ManipulatorRRT folder to your Documents folder \f$\rightarrow\f$ Visual Studio 2010 \f$\rightarrow\f$ Projects
		(or wherever the default Projects folder is for your version of MS Visual Studio)
	5. Open ManipulatorRRT.sln in Microsoft Visual Studio 2010 or above.
		- In the "Solution Explorer" pane on the left, right-click the ManipulatorRRT project and go to "Properties"
	6. Go to Configuration Properties \f$\rightarrow\f$ C/C++ \f$\rightarrow\f$ General \f$\rightarrow\f$ Additional Include Directories
		- Click the drop-down button on the right and select <Edit...>
		- Enter the following in their own fields and select OK:
			- <B> $(SystemDrive)\\Program Files (x86)\\MATLAB\\R2011a Student\\extern\\include </B>	(or wherever "engine.h" is located)
			- <B> $(SystemDrive)\\Program Files\\Phidgets </B>										(or wherever "phidget21.h" is located)
			- <B> *kdtree-dir*\\kdtree-x.y.z </B>													(or wherever "kdtree.h" is located)
	7. Go to Configuration Properties \f$\rightarrow\f$ Linker \f$\rightarrow\f$ General \f$\rightarrow\f$ Additional Library Directories
		- Click the drop-down button on the right and select <Edit...>
		- Enter the following in their own fields and select OK:
			- <B> $(SystemDrive)\\Program Files (x86)\\MATLAB\\R2011a Student\\extern\\lib\\win32\\microsoft </B>	(or wherever "libeng.lib" and "libmx.lib" are located)
			- <B> $(SystemDrive)\\Program Files\\Phidgets\\x86 </B>													(or wherever "phidget21.lib" is located)
			- <B> *kdtree-dir*\\kdtree-x.y.z </B> 																	(or wherever "kdtree.obj" is located)
	8. Add the following libraries to Configuration Properties \f$\rightarrow\f$ Linker \f$\rightarrow\f$ Additional Dependencies:
		- Click the drop-down button on the right and select <Edit...>
		- Enter the following in their own fields and select OK:
			- **libeng.lib**
			- **libmx.lib**
			- **phidget21.lib**
			- **kdtree.obj**
	9. The code should now be prepared.  Plug in the power cord to the Phidget servo control board into an outlet, plug in
	the USB from the control board into your computer and allow drivers to install.  You should see an icon in your taskbar
	that looks like a "Ph" entitled Phidget Control Panel.  If double-clicked, you should see that the device is connected.  You can
	manually adjust servo values here, if necessary, so long as the code is not currently engaging the control board (in which case
	all you will see is a blank screen with no access to the servo parameters).
	10. Set all static user input values in the first section of the `main` function.  Select the green button or press F5 to run.
	Or press CTRL + F5 to run and keep the command window open once finished.

			You should see the arm developing a motion plan and completing its task!

**Useful URL's:**
	- [Stanford Autonomous Systems Lab](http://aa.stanford.edu/research/index.php#asl)
	- [C/C++ Code Reference](http://www.cplusplus.com/reference/)
	- [kdtree Library](http://code.google.com/p/kdtree/)
	- [MATLAB Engine Setup](http://stackoverflow.com/questions/8800439/problems-including-matlab-engine-h-for-c-code)
	- [Phidget Advanced Servo Reference](http://www.phidgets.com/documentation/web/flashdoc/files/phidgets/PhidgetAdvancedServo-as.html)
	- [Phidget Interface Kit Reference](http://www.phidgets.com/documentation/web/flashdoc/files/phidgets/PhidgetInterfaceKit-as.html)
*/

#include <stdlib.h>		/* Standard function library */
#include <stdio.h>		/* Standard input/output library (e.g. `setbuf`, `fopen`, `fclose`, `fflush`, `fscanf`, `fprintf`, etc) */
#include <stdio.h>		/* Standard input/output library for functions using streams (necessary for `fpos` template) */
#include <math.h>		/* Required for trigonometric functions (e.g. `sin()` and `cos()`, `log()`, and `power()` commands, etc) */
#include <assert.h>		/* Required for `assert` commands, used to validate correctness of output at key points in the code */
#include <string.h>		/* Required for `strncmp`, `strlen`, and `strcpy` commands */
#include <float.h>		/* Required for definition of `DBL_MAX` */
#include <windows.h>	/* Required for the `Sleep()` command */
#include <time.h>		/* Required for the `clock()` function, `clock` template, and `CLOCKS_PER_SEC` definition */
#include <engine.h>		/* Library for communicating with MATLAB (via the MATLAB Engine) */
#include <kdtree.h>		/* Library for handling KD-trees */

#pragma warning (disable : 4996)
/*! Suppress compiler warnings about deprecated functions, e.g.\ `strcpy` (warning C4996) */
#define _CRT_SECURE_NO_DEPRECATE
/* If any of the following declarations are not already defined, define them for use here */
/*! Numerical approximation of \f$\pi\f$ */
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
	#define Min( a , b ) ( ((a) < (b)) ? (a) : (b) )
#endif
/*! Returns the square root of a value (no error-checking - value must be non-negative) */
#ifndef Sqrt
	#define Sqrt(x) ( pow(x,0.5) )
#endif
/*! Copies a `va_list` ("variable argument" list) pointer to another `va_list` pointer (CURRENLTY UNUSED) */
#ifndef va_copy
	#define va_copy(dest,src) ((dest) = (src))
#endif


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

/*+=================+*/
/*| Basic functions |*/
/*+=================+*/

/*! Return the elapsed time between two clock timers in milliseconds
	\param[in] start Beginning timer, generated using `clock()`
	\param[in] stop  Ending timer, generated using `clock()` */
double ElapsedTime( clock_t start, clock_t stop ) {
	return ((stop - start)*1000)/CLOCKS_PER_SEC;
}

/*! Convert a joint angle in degrees to its appropriate command value */
double Deg2Command(double q_deg) {
	double q_cmd = q_deg + 20.0;
	return q_cmd;
}

/*! Convert a joint angle command back to degrees */
double Command2Deg(double q_cmd) {
	double q_deg = q_cmd - 20.0;
	return q_deg;
}

/*! Creates a dynamic 2D `double` array of pointers of size *nx* rows by *ny* cols */
double** Make2DDoubleArray(int nx, int ny) {
	double** A = (double**) malloc(nx*sizeof(double*));
	for (int i = 0; i < nx; i++)
		A[i] = (double*) malloc(ny*sizeof(double));
	return A;
}

/*! Creates a dynamic 2D `int` array of pointers of size *nx* rows by *ny* cols */
int** Make2DIntArray(int nx, int ny) {
	int** A	= (int**) malloc(nx*sizeof(int*));
	for (int i = 0; i < nx; i++)
		A[i] = (int*) malloc(ny*sizeof(int));
	return A;
}

/*! Creates a dynamic 3D `double` array of pointers of size *nx* x *ny* x *nz* */
double*** Make3DDoubleArray(int nx, int ny, int nz) {
	double*** A = (double***) malloc(nx*sizeof(double**));
	for (int i = 0; i < nx; i++) {
		A[i] = Make2DDoubleArray(ny,nz);
	}
	return A;
}

/*! Compute the vector difference, *v_left* - *v_right*, for two `double` vectors of length *n* */
void VectorDiff(double* v_left, double* v_right, double* v_out, int n) {
	for (int i = 0; i < n; i++) {
		v_out[i] = v_left[i] - v_right[i];
	}
}

/*! Compute the *p*-norm of a `double` vector *v* of size *n* (Ex: *p* = 1 for Manhattan-norm, *p* = 2 for Euclidean-norm, or *p* = `DBL_MAX` for \f$\infty\f$-norm) */
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

/*! Compute the weighted Euclidean distance-squared, \f$d = \left(\sqrt{w \cdot (v_2 - v_1)} \right)^2\f$ between two vectors of dimension *n*
	\param[in] v1,v2	Vectors used in difference
	\param[in] n		Number of elements in each vector
	\param[in] w		Vector of weighting factors, one for each dimension */
double DistSq( double* v1, double* v2, int n, double* w ) {
	double dsq = 0;
	for (int i = 0; i < n; i++)
		dsq += w[i]*(v2[i] - v1[i])*(v2[i] - v1[i]);
	return dsq;
}


/*+===================+*/
/*| Sorting functions |*/
/*+===================+*/

/*! Insertion sort (stable) a `double` vector *A* (in-place) and return the re-ordered indices *I* <BR>
	(\f$O(n)\f$ **best-case** = *already sorted*, \f$O(n^2)\f$ **worst-case** = reverse-sorted, \f$O(n^2)\f$ **average time**)
	\param[in,out] A	Array of elements to be sorted
	\param[in] length	Number of elements in A
	\param[in,out] I	Empty `int` array of same size as A.  Returned with the rearranged indices of A. */
void InsertionSort( double A[], int length, int I[] ) {

	/* Store original ordering of indices */
	for (int i = 0; i < length; i++) {
		I[i] = i;
	}

	/* Sort A, starting with its second element and moving right.  Compare v = A[i] with elements
	A[j] for j < i, moving left, which contains the previously-sorted array A[0:1:i-1].  Insert v where
	appropriate, creating the sorted list A[0:1:i].  Continue until the end of A. */
	for (int i = 1; i < length; i++) {
		/* Save the values v and c, the elements to be inserted into the sorted sublist A[0:1:i-1] and I[0:1:i-1] */
		double v = A[i];
		int c = I[i];

		/* Sort A and I, using values of A as keys.  Insert v = A[i] into the sorted sublist.
		Any values larger than v should be shifted to the right by 1 to make room for v. */
		int j = i;
		while (j > 0 && A[j-1] > v) {
			A[j] = A[j-1];
			I[j] = I[j-1];
			j = j - 1;
		}

		/* If at the beginning of the array (j = 0), insert v and c.  Otherwise the loop exited
		where the element A[j-1] < v.  Since this is the largest element among A[0:1:j-1], v must
		belong to A[j].  All larger elements were already shifted to the right, so overwrite A[j] with v. */
		A[j] = v;
		I[j] = c;
	}
}

/*! Auxiliary function for `HeapSort` used to float down elements of *A* into their appropriate place in a heap subtree \see HeapSort
	\param[in,out] A	Array of heap elements
	\param[in] root		Index of subtree root whose element should be floated down
	\param[in] bottom	Index of the last element in the heap
	\param[in,out] I	Rearranged indices of A */
void SiftDown( double A[], int root, int bottom, int I[] ) {
	// Considering A as a binary heap, the given "root" node is a parent to up to two children
	// at indices 2*root + 1 and 2*root + 2 if they exist.  If neither exists, root is a leaf node.
	int maxChild = root * 2 + 1;

	// Find the biggest of the two children and set it equal to maxChild
	if ( maxChild < bottom ) {
		int otherChild = maxChild + 1;
		maxChild = ( A[otherChild] > A[maxChild] ) ? otherChild : maxChild;		// Reversed for stability
	} else {
		if ( maxChild > bottom ) return;	// Don't overflow; root is a leaf node and therefore in the right place.
	}

	// If we have the correct ordering, we are done ( A[parent] > A[both children] ).
	if ( A[root] >= A[maxChild] ) return;

	// Otherwise, swap the parent with the larger of the two children.
	double temp	= A[root];			int tmp_int = I[root];
	A[root]		= A[maxChild];		I[root]		= I[maxChild];
	A[maxChild]	= temp;				I[maxChild]	= tmp_int;

	// Tail queue recursion (propagate the parent down since it may still be out of place).
	// Will be compiled as a loop with correct compiler switches.
	SiftDown( A, maxChild, bottom, I );
}

/*! Heap sort (unstable) a `double` vector *A* (in-place) and return the re-ordered indices *I* <BR>
	(\f$\Omega(n)\f$, \f$O(n \log{n})\f$ **best-case**, \f$O(n \log{n})\f$ **worst-case**, \f$O(n \log{n})\f$ **average time**) <BR>
	Modified from the source code found here: http://www.algorithmist.com/index.php/Heap_sort.c \see SiftDown
	\param[in,out] A	Array of elements to be sorted
	\param[in] length	Number of elements in A
	\param[in,out] I	Empty `int` array of same size as A.  Returned with the rearranged indices of A. */
void HeapSort( double A[], int length, int I[] ) {
	int i, tmp_int; double temp;

	/* Store original ordering of indices */
	for (int i = 0; i < length; i++) {
		I[i] = i;
	}

	/* Re-order A as a binary heap ("heapify A").  Only need to sift first half of A, where all possible parent nodes reside.
	SiftDown will float each improperly placed parent down and re-order their subtrees to satisfy the heap property. */
	for (i = (length / 2); i >= 0; i--) {
		SiftDown( A, i, length - 1, I );
	}

	/* A is now a binary heap, with Amax = A[0].  Swap it and the last element and remove the last element from consideration.
	The only node out of place now is A[0].  Sift it down to make A[0:1:length-2] a heap.  Now A[0] is the max. of this new heap.  Swap it
	with the second-to-last element and remove it from consideration.  Sift A[0] down to make A[0:1:length-3] a heap.  Repeat until fully-sorted!  */
	for (i = length-1; i >= 1; i--) {
		temp = A[0];	tmp_int = I[0];		// Swap
		A[0] = A[i];	I[0]	= I[i];
		A[i] = temp;	I[i]	= tmp_int;

		SiftDown( A, 0, i-1, I );
	}
}

/*! Auxiliary function for `MergeSort` used to merge two sorted sublists into a combined sorted list \see MergeSort
	\param[in] L,R					Left and right subarrays to be merged
	\param[in] I_L,I_R				Indices of the left and right subarrays
	\param[in] length_L,length_R	Lengths of each subarray
	\param[out] B					Combined, sorted array of the elements of L and R (stable merge)
	\param[out] J					Indices of A corresponding to the rearrangement of elements in B */
void Merge( double L[], double R[], int I_L[], int I_R[], int length_L, int length_R, double B[], int J[] ) {

	// L[] = left sublist of size length_L, R[] = right sublist of size length_R,	B[] = output array
	// I_L[] = indices of left sublist,		I_R[] = indices of right sublist,		J[] = output indices
	int i = 0, j = 0, k = 0;

	/* Iterate over the left sublist and right sublist, comparing the two and saving the smallest indices to B. */
	while (i < length_L && j < length_R) {
		if ( L[i] < R[j] ) {
			B[k]	= L[i];
			J[k]	= I_L[i];
			i		+= 1;
		}
		else {
			B[k]	= R[j];
			J[k]	= I_R[j];
			j		+= 1;
		}
		k += 1;
	}

	/* If i < length_L, all elements of R were used up and therefore the remaining elements of L are larger.
	Iterate over the rest of the left sublist, adding the remaining (sorted!) elements to B */
	while ( i < length_L ) {
		B[k]	= L[i];
		J[k]	= I_L[i];
		k += 1;	i += 1;
	}

	/* If j < length_R, all elements of L were used up and therefore the remaining elements of R are larger.
	Iterate over the rest of the right sublist, adding the remaining (sorted!) elements to B */
	while ( j < length_R ) {
		B[k]	= R[j];
		J[k]	= I_R[j];
		k += 1;	j += 1;
	}
}

/*! Merge sort (stable) a `double` vector *A* (using \f$O(2n)\f$ memory) and return the re-ordered indices *I* <BR>
	(\f$\Omega(n)\f$, \f$O(n \log{n})\f$ **best-case**, \f$O(n \log{n})\f$ **worst-case**, \f$O(n \log{n})\f$ **average time**) <BR>
	Modified from the source code found here: http://www.algorithmist.com/index.php/Merge_sort.c \see Merge
	\param[in,out] A	Array of elements to be sorted
	\param[in] length	Number of elements in A
	\param[in,out] I	Empty `int` array of same size as A.  Returned with the rearranged indices of A. */
void MergeSort( double A[], int length, int I[] ) {

	/* Store original ordering of indices */
	for (int i = 0; i < length; i++) {
		I[i] = i;
	}

	if (length == 1) return;

	/* Allocate placeholder array(s) for merge operations */
	double* B	= (double*) malloc( length*sizeof(double) );
	int* J		= (int*) malloc( length*sizeof(int) );
	int j;

	/* Bottom-up merge sort iteratively divides the array into sub-arrays, sorts them and then merges them back together.
	Work on sublists of size k = 1, 2, 4, 8, 16, ..., last power of 2 smaller than length */
	for (int k = 1; k < length; k *= 2 ) {

		/* Create a sorted merged list B[j + (0:(2k-1))] from each pair of sorted sublists L = A[j + (0:(k-1))] and
		R = A[j + (k:(2k-1))].  Ensure that the leftmost index, j, satisfies j + (k-1) < length so L is always full, clipping R when
		their difference satisfies length-(j+k)) < k (R will be empty at the end of its first iteration, k = 1, of sorting an odd-length array) */
		j = 0;
		while ( (j+(k-1)) < length ) {
			Merge( &(A[j]), &(A[j + k]), &(I[j]), &(I[j+k]), k, min(k,(length-1)-(j+(k-1))), &(B[j]), &(J[j]) );
			j = j + 2*k;
		}

		/* Copy B over to A, enforcing the loop invariant that every sublist is already sorted
		(the combined pairs of sublists in B, sorted, join to become a single sublist for the next value of k) */
		for (int i = 0; i < length; i++) {
			A[i] = B[i];
			I[i] = J[i];
		}
	}

	free(B); free(J);
}

/*! Rearrange integer vector *A* according to the indices in `int` vector *I* */
void RearrangeIntVector( int A[], int length, int I[] ) {

	/* Create a temporary vector B for the values of A */
	int* B = (int*) malloc(length*sizeof(int));
	for (int i = 0; i < length; i++) {
		B[i] = A[i];
	}

	/* The j-th entry of A should be its I[j]-th entry.  Overwrite using the copied vector B of the original A. */
	for (int j = 0; j < length; j++) {
		A[j] = B[ I[j] ];
	}

	free(B);
}

/*! Rearrange integer pointer vector *A* according to the indices in `int` vector *I* */
void RearrangeIntPtrVector( int* A[], int length, int I[] ) {

	/* Create a temporary vector B for the pointers stored in A */
	int** B = (int**) malloc(length*sizeof(int*));
	for (int i = 0; i < length; i++) {
		B[i] = A[i];
	}

	/* The j-th entry of A should be its I[j]-th entry.  Overwrite using the copied vector B of the original A. */
	for (int j = 0; j < length; j++) {
		A[j] = B[ I[j] ];
	}

	free(B);
}


/*+=======================+*/
/*| Mathematics functions |*/
/*+=======================+*/

/*! Sum integers in `int` array \f$A\f$ of length \f$n\f$ */
int SumInts( int* A, int n ) {
	int sum = 0;
	for (int i = 0; i < n; i++) {
		sum += A[i];
	}

	return sum;
}

/*! Compute the vector cross product, \f$w = u \times v\f$, for 3-D `double` vectors *u* and *v* */
void Cross( double* u, double* v, double* w ) {
	w[0] = u[1] * v[2] - u[2] * v[1];
	w[1] = u[2] * v[0] - u[0] * v[2];
	w[2] = u[0] * v[1] - u[1] * v[0];
}

/*! Compute the matrix product of two 2-D `double` arrays, \f$A*B\f$, where \f$A\f$ is size \f$(m \times n)\f$ and \f$B\f$ is size \f$(n \times p)\f$ */
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

/*! Compute the inverse of a \f$3\times3\f$ matrix \f$M\f$ through calculation of its adjugate, adj\f$(M) = C^T\f$, where \f$C\f$ is the matrix of co-factors of \f$M\f$ */
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

/*! Compute the scalar multiplication of a 2-D `double` array, \f$A\f$ (size \f$m \times n\f$), with `double` \f$c\f$ */
void ScalarMultiply(double** A, double* c, int m, int n) {

	for ( int i = 0 ; i < m ; i++ ) {
		for ( int j = 0 ; j < n ; j++ ) {
			A[i][j] = (*c)*A[i][j];
		}
	}
}

/*! Initialize the values of matrix \f$A\f$ (size \f$n \times n\f$) as an identity matrix \f$I_n\f$ */
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

/*! Find a rotation matrix \f$R\f$ from its operand, \f$v\f$, and output, \f$v_{out}\f$, \f$3\times1\f$ column vectors */
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

/*! Convert a base-10 integer *num* to arbitrary base between 2 and 36, returning its string representation and its length *N* \see Dec2BaseInts */
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

/*! Convert a base-10 integer *num* to arbitrary base between 2 and 36, returning its vector-of-integers representation and its length *N* \see Dec2Base */
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

/*! Generates the specified values of the D-dimensional Halton sequence.  Computes the members of the Halton sequence corresponding to the elements of *sequence*,
returning a *length* \f$\times D\f$ array of `doubles` to array *h*.  Each row *m* corresponds to the \f$m^{th}\f$ element of *sequence*, while each column *n*
corresponds to the dimension, \f$n = 1,\ldots,D\f$.  See p.207 of "Planning Algorithms" by Steven LaValle. \see Dec2BaseInts, Sample, GenerateSamples
	\param[in]	sequence	Array of the desired members in the Halton sequence
	\param[in]	length		Number of elements in *sequence*
	\param[in]	D			Dimension of hypercube used for sequencing (requires \f$D \leq 32\f$)
	\param[in,out]	h		Pre-allocated array of size *length* \f$\times D\f$, populated and returned with Halton samples (by row) */
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

/*! Compute the homogeneous transformation matrix given yaw-pitch-roll Euler angles \see InvTransformMatrix, HomTransformMatrix, InvHomTransformMatrix
	\param[in] yaw, pitch, roll	Euler angles of rotation about the original z-, y-, and x-axes, respectively [deg]
	\param[in]	trans			\f$3\times1\f$ translation vector (applied after rotations), done w.r.t.\ original axes directions
	\param[in,out]	T			Pre-allocated array of size \f$4\times4\f$, returned as the output matrix */
void TransformMatrix( double yaw, double pitch, double roll, double* trans, double** T ) {
	yaw		= yaw*(PI/180);
	pitch	= pitch*(PI/180);
	roll	= roll*(PI/180);

	T[0][0] = cos(pitch) * cos(roll);
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

/*! Computes the inverse of the transform matrix corresponding to the given yaw-pitch-roll Euler angles and translation vector, transforming the
coordinates back to their original frame by reversing the translation and rotation sequence \see TransformMatrix, HomTransformMatrix, InvHomTransformMatrix
	\param[in] yaw, pitch, roll	Euler angles of rotation for the original transform about the z-, y-, and x-axes, respectively [deg]
	\param[in]	trans			\f$3\times1\f$ translation vector for the original transform (applied after rotations), done w.r.t.\ original axes directions
	\param[in,out]	T			Pre-allocated array of size \f$4\times4\f$, returned as the output matrix */
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

/*! Compute the homogeneous transformation matrix for the \f$i^{ th}\f$ link:
	\f$\left. (x,y,z,1) \right|_{i-1} = T_i \left. (x,y,z,1) \right|_{i}\f$ \see ::DHparams, TransformMatrix, InvTransformMatrix, InvHomTransformMatrix
	\param[in] a,d,alpha	DH-parameters for the manipulator arm (constant)
	\param[in] q			Manipulator joint-angle configuration
	\param[in,out]	T		Pre-allocated array of size \f$4\times4\f$, returned as the output matrix */
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

/*! Compute the inverse homogeneous transformation matrix for the \f$i^{ th}\f$ link:
	\f$\left. (x,y,z,1) \right|_{i} = T_i^{-1} \left. (x,y,z,1) \right|_{i-1}\f$ \see ::DHparams, TransformMatrix, InvTransformMatrix, HomTransformMatrix
	\param[in] a,d,alpha	DH-parameters for the manipulator arm (constant)
	\param[in] q			Manipulator joint-angle configuration
	\param[in,out]	T		Pre-allocated array of size \f$4\times4\f$, returned as the output matrix */
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

/*! Compute the body-fixed coordinates of manipulator Oriented Bounding Boxes (OBB's), including the 8 corner points
for each box, the facepoints specified by *n_facepts*, and the end effector point given by *grip_pos* \see ::geom, WorldCoords, main
	\param[in]	n_facepts	Vector containing the numbers of points to use for each pair of OBB faces, given in groups of 3 for each link (faces
							parallel to xy-, yz-, and xz-planes, respectively). Computed using the Halton sequence, scaled to the link face dimensions.
	\param[in]	grip_pos	\f$1\times3\f$ vector in the body-fixed frame of link *n* of the representative end-effector position */
void BodyFixedOBBcoords( struct geom *G, int* n_facepts, double* grip_pos, int n ) {

	/* Use the Halton sampling sequence to optimally cover the OBB faces with the given number of face points */
	int maxval = -1;
	for (int i = 0; i < 3*n; i++) {
		if (n_facepts[i] > maxval) {
			maxval = n_facepts[i];
		}
	}
	int* sequence = (int*) malloc( maxval*sizeof(int) );	// Will not cause an error if maxval is 0 (only if we attempt to access it)
	for (int i = 0; i < maxval; i++) {
		sequence[i] = i;
	}
	double** U = Make2DDoubleArray(maxval, 2);
	Halton( sequence, maxval, 2, U );

	/* Determine the total number of points for all OBB faces of each link i */
	int* N_facepts = (int*) malloc( n*sizeof(int) );
	for (int i = 0; i < n; i++) {
		N_facepts[i] = 2*(n_facepts[3*i] + n_facepts[3*i+1] + n_facepts[3*i+2]);
	}

	/* Loop over all links of the manipulator */
	double** T;
	double rho[3], L, W, H, dx[8], dy[8], dz[8];
	for (int i = 0; i < n; i++) {
		rho[0]	= G->rho_x[i];
		rho[1]	= G->rho_y[i];
		rho[2]	= G->rho_z[i];
		L		= G->L[i];
		W		= G->W[i];
		H		= G->H[i];

		/* Initialize an OBB coordinate array for link i (the last term allocates an additional unit on the last iteration for the end effector) */
		T		= Make2DDoubleArray(3, 8 + N_facepts[i] + (i/(n-1)) );

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
		G->N_coords[i]		= 8 + N_facepts[i];

		/* If on the last link n, add the end effector point as the final body-fixed coordinate */
		if (i == (n-1)) {
			G->Body_coords[i][0][ G->N_coords[i] ] = grip_pos[0];
			G->Body_coords[i][1][ G->N_coords[i] ] = grip_pos[1];
			G->Body_coords[i][2][ G->N_coords[i] ] = grip_pos[2];
			G->N_coords[i]	+= 1;
		}
	}

	/* Free memory (do not free T!) */
	for (int i = maxval-1; i >= 0; i--) {
		free(U[i]);
	}
	free(U); free(N_facepts); free(sequence);
}

/*! Output the coordinates, *C*, w.r.t.\ the world frame of the corners and face points of each link's OBB \see ::geom, ::DHparams, ::coords, BodyFixedOBBcoords, main
	\param[in]	q			Manipulator joint-angle configuration
	\param[in]	C			Pointer to coordinates structure, pre-allocated to hold sum(G->N_coords) of \f$(x,y,z)\f$ ordered-pairs */
void WorldCoords( struct geom *G, struct DHparams *DH, double* q, int n, struct coords *C ) {

	double** T			= Make2DDoubleArray(4,4);
	double** Timinus1	= Make2DDoubleArray(4,4);
	double** Ti			= Make2DDoubleArray(4,4);
	double **v			= Make2DDoubleArray(4,1);
	double **v_new		= Make2DDoubleArray(4,1);
	int sum = 0;

	IdentityMatrix(Timinus1,4);

	for (int i = 0; i < n; i++) {
		/* Find the transformation matrix from the link i to the world frame */
		HomTransformMatrix( DH->a[i], DH->d[i], q[i], DH->alpha[i], Ti );
		MatrixMultiply( Timinus1, Ti, 4, 4, 4, T );

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

		/* Update T_(i-1) */
		for (int j = 0; j < 4; j++) {
			for (int k = 0; k < 4; k++) {
				Timinus1[j][k] = T[j][k];
			}
		}
	}

	for (int i = 3; i >= 0; i--) {
		free(T[i]); free(Timinus1[i]); free(Ti[i]); free(v[i]); free(v_new[i]);
	}
	free(T); free(Timinus1); free(Ti); free(v); free(v_new);
}

/*! Generate new temperature obstacle.  Computes new temperature obstacle primitives from the circular sensor region currently centered at pos = \f$(x,y,z)\f$ [in]
with radius *radius* [in] in the skin surface-normal direction \f$\hat{n} = (n_x,n_y,n_z)\f$.  All quantities must be defined in the body-fixed frame of link *I*
(currently oriented by joint-angle vector *q*).  Adds the new obstacle to *obs* as a `temp_obs` struct. \see ::DHparams, ::temp_obs, ::obstacles, TempObsViolation
	\param[in]	I				Link frame in which the obstacle, position vector, and normal vector are defined
	\param[in]	pos				Position of maximum-temperature sensor from which the obstacle should emanate
	\param[in]	radius, H, beta	Temperature obstacle cone parameters (radius of frustrum floor [in], total height \f$H\f$ [in], and cone half-angle \f$\beta\f$ [deg])
	\param[in]	offset			Height offset above the sensor along its surface normal at which to start the truncated cone
	\param[in]	n_hat			Direction \f$\hat{n} = (n_x,n_y,n_z)\f$ of the skin surface normal to be used for the cone axis */
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

/*! Propagate the temperature map forwards in time by *deltaT* according to the heat equation (CURRENTLY UNUSED) */
void PropagateTemperatures( double*** T, double dx, double dy, double dz, int nx, int ny, int nz, double deltaT, double alpha ) {

	double d2T_dx2, d2T_dy2, d2T_dz2;
	double*** T_temp = Make3DDoubleArray(nx,ny,nz);
	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			for (int k = 0; k < nz; k++) {
				T_temp[i][j][k] = T[i][j][k];
			}
		}
	}

	double dx_inv_sq = pow(1/dx,2.0), dy_inv_sq = pow(1/dy,2.0), dz_inv_sq = pow(1/dz,2.0);

	for (int i = 0; i < nx; i++) {
		for (int j = 0; j < ny; j++) {
			for (int k = 0; k < nz; k++) {
				if ( i == 0 ) {
					d2T_dx2 = dx_inv_sq * ( T[i+2][j][k] - 2*T[i+1][j][k] + T[i][j][k] );
				}
				else if ( i == nx-1 ) {
					d2T_dx2 = dx_inv_sq * ( T[i][j][k] - 2*T[i-1][j][k] + T[i-2][j][k] );
				}
				else {
					d2T_dx2 = dx_inv_sq * ( T[i+1][j][k] - 2*T[i][j][k] + T[i-1][j][k] );
				}

				if ( j == 0 ) {
					d2T_dy2 = dy_inv_sq * ( T[i][j+2][k] - 2*T[i][j+1][k] + T[i][j][k] );
				}
				else if ( j == ny-1 ) {
					d2T_dy2 = dy_inv_sq * ( T[i][j][k] - 2*T[i][j-1][k] + T[i][j-2][k] );
				}
				else {
					d2T_dy2 = dy_inv_sq * ( T[i][j+1][k] - 2*T[i][j][k] + T[i][j-1][k] );
				}

				if ( k == 0 ) {
					d2T_dz2 = dz_inv_sq * ( T[i][j][k+2] - 2*T[i][j][k+1] + T[i][j][k] );
				}
				else if ( k == nz-1 ) {
					d2T_dz2 = dz_inv_sq * ( T[i][j][k] - 2*T[i][j][k-1] + T[i][j][k-2] );
				}
				else {
					d2T_dz2 = dz_inv_sq * ( T[i][j][k+1] - 2*T[i][j][k] + T[i][j][k-1] );
				}

				T[i][j][k] = T_temp[i][j][k] + ( alpha*(d2T_dx2 + d2T_dy2 + d2T_dz2) ) * deltaT;
			}
		}
	}

	for (int i = nx-1; i >= 0; i--) {
		for (int j = ny-1; j >= 0; j--) {
			free(T_temp[i][j]);
		}
		free(T_temp[i]);
	}
	free(T_temp);
}


/*+==========================+*/
/*| Initialization functions |*/
/*+==========================+*/

/*! Print user input values to file or load values from previous run.  Generates/calls <*filename*>input.dat and <*filename*>obstacles.dat depending
on the setting *load_input* specified by the user.  Requires that *filename* fully-specify the path and filename root.  \see main
	\param[in] filename		String containing the full path + root of input filename
	\param[in] load_input	`char` indicating whether to load a previous file ('y') or save a new one ('n') */
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
	fclose(infilefid);
	filename[strlen(filename)-9] = NULL;

	/* Output obstacles data if "load_input" = 'n', otherwise load obstacle values from previous run */
	FILE *obsfilefid;
	int index = 0, return_val = 1;
	char c;

	if (strncmp(&load_input, "n", 1) == 0) {
		obsfilefid = fopen( strcat(filename, "obstacles.dat"), "w+");
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
	else if (strncmp(&load_input, "y", 1) == 0) {
		obsfilefid = fopen( strcat(filename, "obstacles.dat"), "r");
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
	fclose(obsfilefid);
	filename[strlen(filename)-13] = NULL;
}

/*! Generate obstacle primitives from parameters determined by `GenerateInput` \see ::obstacles, GenerateInput, main */
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
		TransformMatrix( YPR_cuboids[3*i], YPR_cuboids[3*i+1], YPR_cuboids[3*i+2], CG, T );
		cuboids->T[i] = T;

		/* Front face											Back face */
		cuboids->a[6*i] = T[0][0];								cuboids->a[6*i+1] = -T[0][0];
		cuboids->b[6*i] = T[0][1];								cuboids->b[6*i+1] = -T[0][1];
		cuboids->c[6*i] = T[0][2];								cuboids->c[6*i+1] = -T[0][2];
		cuboids->d[6*i] = T[0][3] - LWH_cuboids[3*i]/2.0;		cuboids->d[6*i+1] = -( T[0][3] + LWH_cuboids[3*i]/2.0 );

		/* Right face											Left face */
		cuboids->a[6*i+2] = T[1][0];							cuboids->a[6*i+3] = -T[1][0];
		cuboids->b[6*i+2] = T[1][1];							cuboids->b[6*i+3] = -T[1][1];
		cuboids->c[6*i+2] = T[1][2];							cuboids->c[6*i+3] = -T[1][2];
		cuboids->d[6*i+2] = T[1][3] - LWH_cuboids[3*i+1]/2.0;	cuboids->d[6*i+3] = -( T[1][3] + LWH_cuboids[3*i+1]/2.0 );

		/* Top face												Bottom face */
		cuboids->a[6*i+4]  = T[2][0];							cuboids->a[6*i+5]  = -T[2][0];
		cuboids->b[6*i+4]  = T[2][1];							cuboids->b[6*i+5]  = -T[2][1];
		cuboids->c[6*i+4]  = T[2][2];							cuboids->c[6*i+5]  = -T[2][2];
		cuboids->d[6*i+4]  = T[2][3] - LWH_cuboids[3*i+2]/2.0;	cuboids->d[6*i+5]  = -( T[2][3] + LWH_cuboids[3*i+2]/2.0 );
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

/*! Generate array of samples, *Q*.  Computes the sample array used during construction of pre-computed RRT's (prior to motion plan execution).  Used so that
samples can be sent in batch rather than generated individually up to *max_iter* times during the call to `BuildRRTs`.  Generates <*filename*>samples.dat \see Sample, Halton
	\param[in,out]	Q			Pre-allocated `double` array of size \f$(\f$*max_iter*\f$ \times n)\f$, used to store samples
	\param[in]	sampling		User specification of sampling method ("pseudorandom" or "halton")
	\param[in]	q_min, q_max	Joint angle bounds
	\param[in] filename			String containing the full path + root of input filename */
void GenerateSamples(double** Q, char* sampling, int n, int max_iter, double* q_max, double* q_min, char* filename) {

	FILE *samplefid;
	samplefid = fopen( strcat(filename, "samples.dat"), "w+");
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
	filename[strlen(filename)-11] = NULL;

	assert(Q != NULL);
}


/*+============================+*/
/*| RRT and Plotting functions |*/
/*+============================+*/

/*! Determines the next sample joint-angle vector to use for RRT construction.  Uses sample array *Q* during tree
pre-computation, or else the method specified by *sampling* during closed-loop motion plan execution. \see GenerateSamples, Halton, Extend, Connect
	\param[in]	feedback_mode	Boolean specifying whether or not tree-construction is taking place during feedback
	\param[in]	sampling		User specification of sampling method ("pseudorandom" or "Halton")
	\param[in]	Q				Sample array output from `GenerateSamples`
	\param[in]	q_min, q_max	Joint angle bounds
	\param[in]	iter			Pointer to the current iteration number, used to prevent redundant samples
	\param[in,out] q			Pre-allocated `double` vector of length *n*, returned as the next sample */
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

/*! Navigation function from configuration *q_near* to configuration *q*.  Yields the new state *q_new* within weighted distance *epsilon* from *q_near*
in the direction of *q*.  Currently chooses new configurations along the straight line segment connecting the near and goal states. \see DistSq, Extend, Connect
	\param[in]		q			Target joint-angle vector, of length *n*
	\param[in]		q_near		Nearest joint-angle vector to *q* in the current tree, of length *n*
	\param[in]		epsilon		Maximum weighted incremental distance to travel from *q* to *q_near* (terminates if squared-distance is less than \f$\epsilon^2\f$)
	\param[in,out]	q_new		Pre-allocated vector of length *n*, used to store new joint-angle configuration
	\param[in]		w			Vector of weighting factors as defined by the user
	\returns Cost-to-go from *q_near* to *q_new* as `double` */
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

/*! Sends *arg_count* number of `double` variables to the MATLAB Engine, entered as a list of names followed by values, <BR> e.g.\ "v1", v1, "v2", v2 (CURRENTLY UNUSED) */
void SendDoublesToMATLAB(Engine* matlab, int arg_count, ... ) {

	double d;
	char* s;
	mxArray* mxArrayPtr;
	va_list args;
	va_start( args, arg_count );

	for (int i = 0; i < 2*arg_count; i += 2) {
		s			= va_arg( args, char* );				/* 1st argument: Variable name (string) */
		d			= va_arg( args, double );				/* 2nd argument: Value (double) */
		mxArrayPtr	= mxCreateDoubleScalar(d);
		engPutVariable( matlab, s, mxArrayPtr );
		mxDestroyArray( mxArrayPtr );
	}
	va_end( args );
}

/*! Sends *arg_count* number of real 2D `double` array variables to the MATLAB Engine, entered as a list of names followed by values, row dimension,
	and column dimension, <BR> e.g.\ "v1", m1, n1, v1, "v2", m2, n2, v2, etc...\ <BR> (NOTE: Assumes rows of v1, v2, ...\ are each contiguous blocks of memory.
	Can handle `double` vectors (`double*` or `double[]`) as well, provided they are entered with m = 1. ) (CURRENTLY UNUSED) */
void Send2DDoubleArraysToMATLAB(Engine* matlab, int arg_count, ... ) {

	double **M_ptr, *v_ptr, *mem_ptr;
	char* s;
	int m, n;
	mxArray* mxArrayPtr;
	va_list args;
	va_start( args, arg_count );

	for (int i = 0; i < 4*arg_count; i += 4) {
		s			= va_arg( args, char* );				/* 1st argument: Variable name (string) */
		m			= va_arg( args, int );					/* 2nd argument: Number of rows (int) */
		n			= va_arg( args, int );					/* 3rd argument: Number of columns (int) */
		mxArrayPtr	= mxCreateDoubleMatrix( m, n, mxREAL );
		mem_ptr		= mxGetPr(mxArrayPtr);

		if ( m == 1 ) {										/* 4th argument: Pointer to data (double* if m = 1, double** if m > 1) */
			/* Copy vector data to the mxArray */
			v_ptr	= va_arg( args, double* );
			memcpy( (void*) mem_ptr, (void*) v_ptr, n*sizeof(v_ptr[0]) );

		} else if ( m > 1 ) {
			/* Copy matrix data to the mxArray.  Note MATLAB copies memory by column, while C uses rows, hence memcpy cannot be used here.
				(an alternative would be to memcpy (M^T)_ptr[j] to mem_ptr[j*n], j = 1..m.  The transpose must occur before sending to MATLAB unless m = n.) */
			M_ptr	= va_arg( args, double** );
			for (int j = 0; j < m; j++) {
				for (int k = 0; k < n; k++) {
					mem_ptr[j + k*m] = M_ptr[j][k];
				}
			}
		}
		engPutVariable( matlab, s, mxArrayPtr );
		mxDestroyArray( mxArrayPtr );
	}
	va_end( args );
}

/*! Sends real \f$N\f$-D numeric arrays up to \f$N=3\f$ to the MATLAB Engine, saving the variables as formatted numeric matrices.
	Returns an `int` on success/failure. \see ::datatypes */
int SendArraysToMATLAB( int line, Engine* matlab, int arg_count, ... ) {
	/*
	Enter data as: 	`SendArraysToMATLAB(` \_\_LINE\_\_, *matlab*, *arg_count*,	"\f$v_1\f$", Type1,\f$d_1, p_1, q_1, r_1, v_1\f$,
																				"\f$v_2\f$", Type2,\f$d_2, p_2, q_2, r_2, v_2\f$,	etc... `)`, where
		- *arg_count* is the total number of variables to save (the argument list must consist of tuples of 7, one for each variable)
		- *Type* refers to the enumerated list *datatypes* (typedef *Type*), i.e.\ any of `Long`, `Double`, `Int`, etc.
		- *d* refers to the dimension of the datatype (scalar = 0, vector = 1, matrix = 2, 3D-tensor = 3)
		- \f$(p \times q \times r)\f$, i.e.\ the dimension values, reflect the index order used in C, \f$v[p][q][r]\f$ (though MATLAB uses \f$v[q][r][p]\f$!)
		- "v" is the variable containing the data, which must reflect the dimensions given (e.g.\ Type = `Int`, d = 2, \f$(p,q,r)\f$ = \f$(1,3,4)\f$ --> \f$v\f$ must be int** )

	Be warned that `char` and `short` are always promoted to `int`, while `float` is always promoted to `double`. <BR>
	Assumes the 1D rows of each variable is a contiguous block of memory, but otherwise the data may be fragmented.

	(NOTE: Data MUST be input correctly, or else errors (detected or otherwise) will result.  If receiving an "unknown type" error on variable \f$k\f$,
	and yet it seems to have been entered correctly, the error is likely due to variable \f$k-1\f$.  If there is any chance that a variable's dimensions can be 0,
	this must be tested before outside of `SendArraysToMATLAB` to prevent a call to the function!)

	Example input:		`SendArraysToMATLAB(` \_\_LINE\_\_, matlab, arg_count,	"v1", Int,2, 1,3,2, v1,		"v2", Double,3, 5,3,3, v2,	"v3", Char,1, 1,1,1, v3 `)`
		- Saves v1 as a 3x2 matrix of `Ints`
		- Saves v2 as a 3x3x5 array of `doubles`
		- Saves v3 as a scalar `int` corresponding to the `char` v3
	*/
	double	***T_dbl_ptr = NULL,	**M_dbl_ptr = NULL,		*v_dbl_ptr = NULL,		s_dbl = NULL,		*mem_ptr;
	long	***T_long_ptr = NULL,	**M_long_ptr = NULL,	*v_long_ptr = NULL,		s_long = NULL;
	int		***T_int_ptr = NULL,	**M_int_ptr = NULL,		*v_int_ptr = NULL,		s_int = NULL,		d, p, q, r, errorflag; //m, n;
	char	*name;
	enum datatypes type;
	mxArray* mxArrayPtr;
	mwSize mxdims[3];

	va_list args, args_copy;
	va_start( args, arg_count );

	for (int i = 0; i < 7*arg_count; i += 7) {
		name = va_arg( args, char* );				/* 1st argument: Variable name (string) */
		type = va_arg( args, Type );				/* 2nd argument: Variable type (enum datatypes) */
		d	= va_arg( args, int );					/* 3rd argument: Dimension of datatype */
		p   = va_arg( args, int );					/* 4th argument: Number of 2D matrices (int) */
		q	= va_arg( args, int );					/* 5th argument: Number of rows (int) */
		r	= va_arg( args, int );					/* 6th argument: Number of columns (int) */
													/* 7th argument: the variable itself */
		if ( p <= 0 || q <= 0 || r <= 0 ) {
			fprintf(stderr, "ERROR: (Line %d) Zero or negative dimension(s) detected for variable #%i.\n", line, i/6 + 1 ); va_end(args); return EXIT_FAILURE;
		}

		mxdims[0]	= (mwSize) q;		mxdims[1]	= (mwSize) r;		mxdims[2]	= (mwSize) p;
		switch ( type ) {
			case Long:		mxArrayPtr	= mxCreateNumericArray( 3, mxdims, mxINT64_CLASS, mxREAL );		break;
			case Short:		/* Short integers	are automatically promoted to int when passed to va_arg */
			case Char:		/* Chars			are automatically promoted to int when passed to va_arg */
			case Int:		mxArrayPtr	= mxCreateNumericArray( 3, mxdims, mxINT32_CLASS, mxREAL );		break;
			case Float:		/* Floats			are automatically promoted to double when passed to va_arg */
			case Double:	mxArrayPtr	= mxCreateNumericArray( 3, mxdims, mxDOUBLE_CLASS, mxREAL );	break;
			default:		fprintf(stderr, "ERROR: (Line %d) Unknown type for variable #%i. Exiting...\n", line, i/6 + 1);
							va_end(args);		va_end(args_copy);		return EXIT_FAILURE;
		}

		if (mxArrayPtr == NULL) {
			fprintf(stderr, "ERROR: (Line %d) Could not create mxArray for variable #%i. Exiting...\n", line, i/6 + 1);
			va_end(args);		va_end(args_copy);		return EXIT_FAILURE;
		}
		mem_ptr		= mxGetPr(mxArrayPtr);

		/* Given the datatype dimension, d, read in the pointer to the variable and write its contents to the mxArray */
		errorflag = 0;
		switch ( d ) {
			case 0:		__try {
							switch ( type ) {
								case Long:		s_long	= va_arg(args, long);		break;
								case Short:
								case Char:
								case Int:		s_int	= va_arg(args, int);		break;
								case Float:
								case Double:	s_dbl	= va_arg(args, double);		break;
							}
							if ( p != 1 || q != 1 || r != 1 )	{ errorflag = 1; }
						} __except( EXCEPTION_EXECUTE_HANDLER ) { errorflag = 2; }

						/* Copy scalar data to the mxArray */
						if ( errorflag == 0 ) {
							__try {
								switch ( type ) {
									case Long:		memcpy( (void*)mem_ptr, (void*) &(s_long), sizeof(s_long) );	break;
									case Short:
									case Char:
									case Int:		memcpy( (void*)mem_ptr, (void*) &(s_int), sizeof(s_int) );		break;
									case Float:
									case Double:	memcpy( (void*)mem_ptr, (void*) &(s_dbl), sizeof(s_dbl) );		break;
								}
							} __except( EXCEPTION_EXECUTE_HANDLER ) { errorflag = 3; }
						}
						break;

			case 1:		__try {
							switch ( type ) {
								case Long:		v_long_ptr	= va_arg(args, long*);		s_long = v_long_ptr[0];		break;
								case Short:
								case Char:
								case Int:		v_int_ptr	= va_arg(args, int*);		s_int = v_int_ptr[0];		break;
								case Float:
								case Double:	v_dbl_ptr	= va_arg(args, double*);	s_dbl = v_dbl_ptr[0];		break;
							}
							if ( p != 1 || q != 1 )				{ errorflag = 1; }
						} __except( EXCEPTION_EXECUTE_HANDLER ) { errorflag = 2; }

						/* Copy vector data to the mxArray */
						if ( errorflag == 0 ) {
							__try {
								switch ( type ) {
									case Long:		memcpy( (void*)mem_ptr, (void*)v_long_ptr,	r*sizeof(v_long_ptr[0]) );		break;
									case Short:
									case Char:
									case Int:		memcpy( (void*)mem_ptr, (void*)v_int_ptr,	r*sizeof(v_int_ptr[0]) );		break;
									case Float:
									case Double:	memcpy( (void*)mem_ptr, (void*)v_dbl_ptr,	r*sizeof(v_dbl_ptr[0]) );		break;
								}
							} __except( EXCEPTION_EXECUTE_HANDLER ) { errorflag = 3; }
						}
						break;

			case 2:		__try {
							switch ( type ) {
								case Long:		M_long_ptr	= va_arg(args, long**);		v_long_ptr = M_long_ptr[0];		s_long = v_long_ptr[0];		break;
								case Short:
								case Char:
								case Int:		M_int_ptr	= va_arg(args, int**);		v_int_ptr = M_int_ptr[0];		s_int = v_int_ptr[0];		break;
								case Float:
								case Double:	M_dbl_ptr	= va_arg(args, double**);	v_dbl_ptr = M_dbl_ptr[0];		s_dbl = v_dbl_ptr[0];		break;
							}
							if ( p != 1 )						{ errorflag = 1; }
						} __except( EXCEPTION_EXECUTE_HANDLER ) { errorflag = 2; }

						/* Copy matrix data to the mxArray.  Note MATLAB copies memory by column-by-column, while C uses row-by-row, hence memcpy cannot be used here.
							(an alternative would be to memcpy (M^T)_ptr[j] to mem_ptr[j*r], j = 1..q.  The transpose must occur before sending to MATLAB unless q = r.) */
						if ( errorflag == 0 ) {
							__try {
								for (int j = 0; j < q; j++) {
									for (int k = 0; k < r; k++) {
										switch ( type ) {
											case Long:		mem_ptr[j + k*q] = M_long_ptr[j][k];	break;
											case Short:
											case Char:
											case Int:		mem_ptr[j + k*q] = M_int_ptr[j][k];		break;
											case Float:
											case Double:	mem_ptr[j + k*q] = M_dbl_ptr[j][k];		break;
										}
									}
								}
							} __except( EXCEPTION_EXECUTE_HANDLER ) { errorflag = 3; }
						}
						break;

			case 3:		__try {
							switch ( type ) {
								case Long:		T_long_ptr	= va_arg(args, long***);	M_long_ptr	= T_long_ptr[0];
												v_long_ptr	= M_long_ptr[0];			s_long		= v_long_ptr[0];	break;
								case Short:
								case Char:
								case Int:		T_int_ptr	= va_arg(args, int***);		M_int_ptr	= T_int_ptr[0];
												v_int_ptr	= M_int_ptr[0];				s_int		= v_int_ptr[0];		break;
								case Float:
								case Double:	T_dbl_ptr	= va_arg(args, double***);	M_dbl_ptr	= T_dbl_ptr[0];
												v_dbl_ptr	= M_dbl_ptr[0];				s_dbl		= v_dbl_ptr[0];		break;
							}
						} __except( EXCEPTION_EXECUTE_HANDLER ) { errorflag = 2; }

						/* Copy tensor data to mxArray */
						if ( errorflag == 0 ) {
							__try {
								for (int z = 0; z < p; z++) {
									for (int j = 0; j < q; j++) {
										for (int k = 0; k < r; k++) {
											switch ( type ) {
												case Long:		mem_ptr[(z*q*r + j + k*q)] = T_long_ptr[z][j][k];		break;
												case Short:
												case Char:
												case Int:		mem_ptr[(z*q*r + j + k*q)] = T_int_ptr[z][j][k];		break;
												case Float:
												case Double:	mem_ptr[(z*q*r + j + k*q)] = T_dbl_ptr[z][j][k];		break;
											}
										}
									}
								}
							} __except( EXCEPTION_EXECUTE_HANDLER ) { errorflag = 3; }
						}
						break;
			default:
				fprintf(stderr, "ERROR: (Line %d) Dimension of variable #%i must be between 0 (scalar) and 3 (3-D tensor). Exiting...\n", line, i/6 + 1 );
		}

		if (errorflag == 1) {
			fprintf(stderr, "ERROR: (Line %d) Dimensions not consistent with interpretation of variable #%i. Exiting...\n", line, i/6 + 1 );
			va_end(args);		va_end(args_copy);		return EXIT_FAILURE;
		}
		else if (errorflag == 2) {
			fprintf(stderr, "ERROR: (Line %d) Could not read variable #%i safely for d = %i. Exiting...\n", line, i/6 + 1, d );
			va_end(args);		va_end(args_copy);		return EXIT_FAILURE;
		}
		else if (errorflag == 3) {
			fprintf(stderr, "ERROR: (Line %d) Could not copy variable #%i safely (try checking its dimensions). Exiting...\n", line, i/6 + 1 );
			va_end(args);		va_end(args_copy);		return EXIT_FAILURE;
		}

		///* Assuming a successful read, all relevant pointers corresponding to the type (tensor, matrix, vector, etc.) have been defined for
		//any combination of appropriately-valued dimensions.  Given the dimensions, attempt to copy to mxArray (otherwise report failure). */
		//if ( p == 1 ) {
		//	if ( q == 1 ) {
		//		if ( r == 1 ) {
		//			/* Copy scalar data to the mxArray */
		//			__try {
		//				switch ( type ) {
		//					case Long:		memcpy( (void*)mem_ptr, (void*) &(s_long), sizeof(s_long) );	break;
		//					case Short:
		//					case Char:
		//					case Int:		memcpy( (void*)mem_ptr, (void*) &(s_int), sizeof(s_int) );		break;
		//					case Float:
		//					case Double:	memcpy( (void*)mem_ptr, (void*) &(s_dbl), sizeof(s_dbl) );		break;
		//				}
		//			} __except( EXCEPTION_EXECUTE_HANDLER ) {
		//				fprintf(stderr, "ERROR: (Line %d) Could not copy variable #%i safely (try checking its dimensions). Exiting...\n", line, i/6 + 1 );
		//				va_end(args);		va_end(args_copy);		return EXIT_FAILURE;
		//			}
		//		} else {
		//			/* (r > 1) --> Copy vector data to the mxArray */
		//			__try {
		//				switch ( type ) {
		//					case Long:		memcpy( (void*)mem_ptr, (void*)v_long_ptr,	r*sizeof(v_long_ptr[0]) );		break;
		//					case Short:
		//					case Char:
		//					case Int:		memcpy( (void*)mem_ptr, (void*)v_int_ptr,	r*sizeof(v_int_ptr[0]) );		break;
		//					case Float:
		//					case Double:	memcpy( (void*)mem_ptr, (void*)v_dbl_ptr,	r*sizeof(v_dbl_ptr[0]) );		break;
		//				}
		//			} __except( EXCEPTION_EXECUTE_HANDLER ) {
		//				fprintf(stderr, "ERROR: (Line %d) Could not copy variable #%i safely (try checking its dimensions). Exiting...\n", line, i/6 + 1 );
		//				va_end(args);		va_end(args_copy);		return EXIT_FAILURE;
		//			}
		//		}
		//	} else {
		//		/* (q > 1) --> Copy matrix data to the mxArray.  Note MATLAB copies memory by column-by-column, while C uses row-by-row, hence memcpy cannot be used here.
		//		(an alternative would be to memcpy (M^T)_ptr[j] to mem_ptr[j*r], j = 1..q.  The transpose must occur before sending to MATLAB unless q = r.) */
		//		__try {
		//			for (int j = 0; j < q; j++) {
		//				for (int k = 0; k < r; k++) {
		//					switch ( type ) {
		//						case Long:		mem_ptr[j + k*q] = M_long_ptr[j][k];	break;
		//						case Short:
		//						case Char:
		//						case Int:		mem_ptr[j + k*q] = M_int_ptr[j][k];		break;
		//						case Float:
		//						case Double:	mem_ptr[j + k*q] = M_dbl_ptr[j][k];		break;
		//					}
		//				}
		//			}
		//		} __except( EXCEPTION_EXECUTE_HANDLER ) {
		//			fprintf(stderr, "ERROR: (Line %d) Could not copy variable #%i safely (try checking its dimensions). Exiting...\n", line, i/6 + 1 );
		//			va_end(args);		va_end(args_copy);		return EXIT_FAILURE;
		//		}
		//	}
		//} else {
		//	/* (p > 1) --> Copy tensor data to mxArray */
		//	__try {
		//		for (int z = 0; z < p; z++) {
		//			for (int j = 0; j < q; j++) {
		//				for (int k = 0; k < r; k++) {
		//					switch ( type ) {
		//						case Long:		mem_ptr[(z*q*r + j + k*q)] = T_long_ptr[z][j][k];		break;
		//						case Short:
		//						case Char:
		//						case Int:		mem_ptr[(z*q*r + j + k*q)] = T_int_ptr[z][j][k];		break;
		//						case Float:
		//						case Double:	mem_ptr[(z*q*r + j + k*q)] = T_dbl_ptr[z][j][k];		break;
		//					}
		//				}
		//			}
		//		}
		//	} __except( EXCEPTION_EXECUTE_HANDLER ) {
		//		fprintf(stderr, "ERROR: (Line %d) Could not copy variable #%i safely (try checking its dimensions). Exiting...\n", line, i/6 + 1 );
		//		va_end(args);		va_end(args_copy);		return EXIT_FAILURE;
		//	}
		//}

		/* Send the mxArray to MATLAB */
		engPutVariable( matlab, name, mxArrayPtr );
		mxDestroyArray( mxArrayPtr );
	}

	/* If all variables were sent successfully, return true */
	va_end( args );		va_end( args_copy );
	return EXIT_SUCCESS;
}

/*! Adds an illustration of sample node *q* and the selected node *q_near* in tree *T* to the RRT construction figure with handle RRTfig. <BR>
	(Nothing is done if the MATLAB Engine pointer *matlab* is NULL.) \see BuildRRTs, Nearest */
void PlotNearestInMATLAB( struct tree* T, int n, double* q, double* q_near, Engine* matlab ) {

	if (matlab != NULL) {
		assert( SendArraysToMATLAB( __LINE__, matlab, 2,	"q", Double,1, 1,1,n, q,		"q_near", Double,1, 1,1,n, q_near	) == EXIT_SUCCESS );
		engEvalString(matlab, "if(exist('samplehandle','var') & ishandle(samplehandle)) delete( samplehandle ); end;");		// Clear the previous sample plot in case InsertNode was not called to clear it
		if (n >= 3) {
			engEvalString(matlab, "x = linspace( q(1), q_near(1), 2 ); y = linspace( q(2), q_near(2), 2 ); z = linspace( q(3), q_near(3), 2 );");
			engEvalString(matlab, "figure(RRTfig(I)); samplehandle = plot3(x, y, z, '--k', q(1), q(2), q(3), sample_format, q_near(1), q_near(2), q_near(3), nearest_format, \
								  'MarkerSize', nearest_size, 'LineWidth', nearest_linewidth );");
		} else if (n == 2) {
			engEvalString(matlab, "x = linspace( q(1), q_near(1), 2 ); y = linspace( q(2), q_near(2), 2 );");
			engEvalString(matlab, "figure(RRTfig(I)); samplehandle = plot(x, y, '--k', q(1), q(2), sample_format, q_near(1), q_near(2), nearest_format, \
								  'MarkerSize', nearest_size, 'LineWidth', nearest_linewidth );");
		} else {
			engEvalString(matlab, "figure(RRTfig(I)); samplehandle = plot(q, 0, sample_format, q_near, 0, nearest_format, \
								  'MarkerSize', nearest_size, 'LineWidth', nearest_linewidth );");
		}
	}
}

/*! Adds a straight line plot between a node (index *node_index*) and its parent to the RRT construction figure with handle RRTfig. <BR>
	(Use C convention, i.e.\ starting from 0, for *node_index*.  Nothing is done if the MATLAB Engine pointer *matlab* is NULL.) \see BuildRRTs, InsertNode */
void PlotEdgeInMATLAB( struct tree* T, int n, int node_index, Engine* matlab ) {

	if (matlab != NULL) {
		assert( SendArraysToMATLAB( __LINE__, matlab, 4, "node_index", Int,0, 1,1,1, node_index+1,		"q_new", Double,1, 1,1,n, T->nodes[ node_index ],
			"q_parent", Double,1, 1,1,n, T->nodes[ T->parents[node_index] ],		"q0", Double,1, 1,1,n, T->nodes[0] )  == EXIT_SUCCESS );
		engEvalString(matlab, "if (q0 == q_init) tree = 1; else tree = 2; end; if(exist('samplehandle','var') & ishandle(samplehandle)) delete( samplehandle ); end;");
		if (n >= 3) {
			engEvalString(matlab, "x = linspace( q_new(1), q_parent(1), 2 ); y = linspace( q_new(2), q_parent(2), 2 ); z = linspace( q_new(3), q_parent(3), 2 );");
			engEvalString(matlab, "figure(RRTfig(I)); plothandles{tree, node_index} = plot3(x, y, z, edge_format{tree}, q_new(1), q_new(2), q_new(3), node_format{tree}, \
								  'MarkerFaceColor', node_color{tree}, 'MarkerSize', node_size(tree) );");
		} else if (n == 2) {
			engEvalString(matlab, "x = linspace( q_new(1), q_parent(1), 2 ); y = linspace( q_new(2), q_parent(2), 2 );");
			engEvalString(matlab, "figure(RRTfig(I)); plothandles{tree, node_index} = plot(x, y, edge_format{tree}, q_new(1), q_new(2), node_format{tree}, \
								  'MarkerFaceColor', node_color{tree}, 'MarkerSize', node_size(tree) );");
		} else {
			engEvalString(matlab, "figure(RRTfig(I)); plothandles{tree, node_index} = plot(q_new, 0, node_format{tree}, \
								  'MarkerFaceColor', node_color{tree}, 'MarkerSize', node_size(tree) );");
		}
	}
}

/*! Adds a straight line plot between a neighbor node (index *neighbor_index*) and its new parent to the RRT construction figure with handle RRTfig,
	deleting the old edge.	Assumes \f$q_{rewire}\f$ has already been sent to MATLAB from `ReWire` and that "tree" is unchanged from its former definition
	from `InsertNode` and `PlotEdgeInMATLAB`. (Nothing is done if the MATLAB Engine pointer *matlab* is NULL.) \see BuildRRTs, InsertNode, PlotEdgeInMATLAB */
void PlotRewiringInMATLAB( struct tree* T, int n, int neighbor_index, Engine* matlab ) {

	if (matlab != NULL) {
		assert( SendArraysToMATLAB( __LINE__, matlab, 2, "neighbor", Int,0, 1,1,1, neighbor_index+1,	"q_neighbor", Double,1, 1,1,n, T->nodes[neighbor_index] )  == EXIT_SUCCESS );
		engEvalString(matlab, "delete( plothandles{tree, neighbor} );");
		if (n >= 3) {
			engEvalString(matlab, "x = linspace( q_neighbor(1), q_rewire(1), 2 ); y = linspace( q_neighbor(2), q_rewire(2), 2 ); z = linspace( q_neighbor(3), q_rewire(3), 2 );		\
				figure(RRTfig(I)); plothandles{tree, neighbor} = plot3(x, y, z, edge_format{tree}, q_neighbor(1), q_neighbor(2), q_neighbor(3), node_format{tree}, \
				'MarkerFaceColor', node_color{tree}, 'MarkerSize', node_size(tree) );");
		} else if (n == 2) {
			engEvalString(matlab, "x = linspace( q_neighbor(1), q_rewire(1), 2 ); y = linspace( q_neighbor(2), q_rewire(2), 2 );	\
				figure(RRTfig(I)); plothandles{tree, neighbor} = plot(x, y, edge_format{tree}, q_neighbor(1), q_neighbor(2), node_format{tree}, \
				'MarkerFaceColor', node_color{tree}, 'MarkerSize', node_size(tree) );");
		} else {
			engEvalString(matlab, "figure(RRTfig(I)); plothandles{tree, neighbor} = plot(q_neighbor, 0, node_format{tree}, \
				'MarkerFaceColor', node_color{tree}, 'MarkerSize', node_size(tree) );");
		}
	}
}

/*! Plot a new path segment in MATLAB to figure handle PLANfig.  Sends *path_new* and *pathlen_new* to MATLAB, merging path[1:1:(pathlen - pathlen_old + current_path_index)]
with the new path, and displays the result in figure with handle PLANfig (defined in `main`).  "path_old" is the previously planned trajectory for indices
beyond "current_path_index", which is instead replaced by path_new. (Use C convention for indices.  Nothing is done if the MATLAB Engine pointer *matlab* is NULL.) \see main
	\param[in]	plan_index			Index of the current motion plan. Enter as -1 to finalize the plot after motion is over.
	\param[in]	current_path_index	Entering 0 erases the old plan, while entering "pathlen_oldplan" appends the new one.
	\param[in]	pathlen_new			Length of new path to be added to the plot
	\param[in]	path_new			Nodes along the new path */
void PlotPathInMATLAB( int plan_index, int current_path_index, int pathlen_new, int n, double** path_new, Engine* matlab ) {

	if (matlab != NULL) {
		assert( SendArraysToMATLAB( __LINE__, matlab, 4, "plan_index", Double,0, 1,1,1, (double) plan_index+1,		"pathlen_new", Double,0, 1,1,1, (double) pathlen_new,
			"path_old_index", Double,0, 1,1,1, (double) current_path_index+1,		"path_new", Double,2, 1,pathlen_new,n, path_new )  == EXIT_SUCCESS );

		/* Update the total path "path" as the already-traversed path + the new path.  Save "path_old" as the remainder of the current path that was unused.  */
		engEvalString(matlab, "path_index = pathlen - pathlen_old + path_old_index;		path_old = path( max((path_index-1),1):pathlen,: );		\
			path = [path(1:(path_index-1),:); path_new];		pathlen = size(path,1);");

		/* Plot the traversed path (gray) and the new path (colored).  If part of the previously-planned old path was not used, plot it as a dotted black path.  */
		engEvalString(matlab, "if(exist('path_new_handle', 'var')) delete([path_new_handle; waypt_handle; path_trav_handle; path_old_handle]); end;													\
			figure(PLANfig);		 hold on;					path_new_indices = max((path_index-1),1):pathlen;			path_old_indices = [max((path_index-1),1) + (0:size(path_old,1)-1)];	\
			path_new_handle		= plot( path_new_indices,		path(path_new_indices,:),		path_format,	'Linewidth', path_linewidth, 'MarkerSize', waypt_size );							\
			path_trav_handle	= plot( 1:(path_index-1),		path(1:(path_index-1),:) );																											\
			path_old_handle		= plot( path_old_indices,		path_old,						':ok',			'MarkerSize', waypt_size );															\
			if (plan_index > 0);																																									\
				waypt_handle	= plot( pathlen, q_waypoints(plan_index+1,:), waypt_format, 'Linewidth', path_linewidth+1, 'MarkerSize', waypt_size+1 );											\
				waypt_commands(plan_index+1) = pathlen;		legend([path_new_handle; waypt_handle(1); path_old_handle], [q_legend; 'Target Waypt'; 'Former Plan'], 'Location', 'EastOutside');		\
			else title('Complete Joint Angle Motion Plan');		waypt_handle = plot( [1,waypt_commands(2:end)], q_waypoints, waypt_format, 'Linewidth', path_linewidth+1, 'MarkerSize', waypt_size+1 );		\
					legend([path_new_handle; waypt_handle(1)], [q_legend; 'Waypoints'], 'Location', 'EastOutside');		delete([path_new_handle; path_old_handle]);												\
			end;																																													\
			xmax = ceil(1.05*(max([pathlen,path_old_indices])));		set(gca,'xtick',(1:1:xmax));		xlim([0,xmax]);		V = axis;															\
			plot( linspace(V(1),V(2),2), min(q_min).*ones(1,2), '-k', linspace(V(1),V(2),2), max(q_max).*ones(1,2), '-k', 'Linewidth', 2 );"	);

		/* Update the old path to the new path and switch hold to off so PLANfig is refreshed on the next call to PlotPathInMATLAB. */
		engEvalString(matlab, "pathlen_old = pathlen_new;	clear xmax V;");
	}
}

/*! Plots the current manipulator configuration to figure handle TRAJfig.  Plots the manipulator configuration corresponding to the coordinate structure C
returned by `WorldCoords`, adding a visualization of the arm OBB's and point representation to TRAJfig (Requires previous definition of "coord_format", "coord_color",
"link_colors", "link_alpha", and "N_coords". Nothing is done if the MATLAB Engine pointer *matlab* is NULL.) \see main, PlotEndEffectorPathInMATLAB
	\param[in]	C			Coordinates in the world-frame of the manipulator arm
	\param[in]	n_points	Total number of points in C
	\param[in]	opacity		Face opacity. Enter 0 for transparent or 1 for full. */
void PlotRobotConfigInMATLAB( struct coords* C, int n_points, double opacity, Engine* matlab ) {

	if ( matlab != NULL ) {
		assert( SendArraysToMATLAB( __LINE__, matlab, 4,		"x", Double,1, 1,1,n_points, C->x,		"y", Double,1, 1,1,n_points, C->y,
			"z", Double,1, 1,1,n_points, C->z,		"fade", Double,0, 1,1,1, opacity	)  == EXIT_SUCCESS );
		engEvalString( matlab, "figure(TRAJfig);	Cdata = plot3( x, y, z, coord_format, 'Color', (1-fade)*coord_color );							\
			traj_index = size(linkdata,2)+1;																										\
			for j = 1:n;																															\
			f1_indices = sum(N_coords(1:(j-1)))+[1,2,4,3];			f4_indices = sum(N_coords(1:(j-1)))+[3,4,6,5];									\
				f2_indices = sum(N_coords(1:(j-1)))+[7,8,6,5];		f5_indices = sum(N_coords(1:(j-1)))+[1,3,5,7];									\
				f3_indices = sum(N_coords(1:(j-1)))+[1,2,8,7];		f6_indices = sum(N_coords(1:(j-1)))+[2,4,6,8];									\
				linkdata(6*j-5,traj_index)	= patch( x(f1_indices), y(f1_indices), z(f1_indices), link_color, 'FaceAlpha', fade*link_alpha );		\
				linkdata(6*j-4,traj_index)	= patch( x(f2_indices), y(f2_indices), z(f2_indices), link_color, 'FaceAlpha', fade*link_alpha );		\
				linkdata(6*j-3,traj_index)	= patch( x(f3_indices), y(f3_indices), z(f3_indices), link_color, 'FaceAlpha', fade*link_alpha );		\
				linkdata(6*j-2,traj_index)	= patch( x(f4_indices), y(f4_indices), z(f4_indices), link_color, 'FaceAlpha', fade*link_alpha );		\
				linkdata(6*j-1,traj_index)	= patch( x(f5_indices), y(f5_indices), z(f5_indices), link_color, 'FaceAlpha', fade*link_alpha );		\
				linkdata(6*j-0,traj_index)	= patch( x(f6_indices), y(f6_indices), z(f6_indices), link_color, 'FaceAlpha', fade*link_alpha );		\
			end;	clear f1_indices f2_indices f3_indices f4_indices f5_indices f6_indices fade V;"	);
	}
}

/*! Plots the end effector trajectory corresponding to a given path in figure handle TRAJfig.  Generates the points traced out by the end-effector position along a given
joint-angle path using `Steer`, adding a curve of the traced-path plus the planned and former planned paths to TRAJfig.  Also overlays a plot of planar obstacles (updated
with each plot in order to properly span the axis dimensions), the current temperature obstacles, and removes the grasped object (last cuboidal obstacle) if "plan_index"
exceeds the grasp maneuver index.  Must be called immediately after `PlotPathInMATLAB` (relies in the MATLAB Engine environment on many of the same variables).
\see main, PlotRobotConfigInMATLAB, PlotPathInMATLAB, Steer
	\param[in]	n_cuboids_total		Total number of cuboids, including the grasped object.  Used to test whether the grasped object has already been eliminated from plots.
	\param[in]	pathlen_new			Length of new path to be added to the plot
	\param[in]	path_new			Nodes along the new path */
void PlotEndEffectorPathInMATLAB( int n, double epsilon, double* w, int n_cuboids_total, struct obstacles* obs, struct geom *G, struct DHparams *DH,
	int pathlen_new, double** path_new, Engine* matlab ) {

	if (matlab == NULL) {
		return;
	}

	int plot_intermediate_configs = 0,		plot_coords = 0;
	double epsilon_sq		= pow( epsilon, 2.0 ),		*x_end_eff = NULL,		*y_end_eff = NULL,		*z_end_eff = NULL;
	double* q				= (double*)	malloc( n*sizeof(double) );
	int* end_eff_indices	= (int*) malloc( pathlen_new*sizeof(int) );

	/* Create variables for storage of link geometry coordinates */
	int n_points = SumInts(G->N_coords, n);
	struct coords C;
	C.x = (double*) malloc(n_points*sizeof(double));
	C.y = (double*) malloc(n_points*sizeof(double));
	C.z = (double*) malloc(n_points*sizeof(double));

	/* Delete old plots (if they exist). */
	engEvalString( matlab, "if(exist('plane_handles','var') & ishandle(plane_handles) );		delete(plane_handles);		end;	\
							if(exist('tempzone_handles','var') & ishandle(tempzone_handles) );	delete(tempzone_handles);	end;	\
							if(exist('Cdata','var') && ishandle(Cdata) );						delete(Cdata);				end;	\
							if(exist('linkdata','var') & ishandle(linkdata) );					delete(linkdata);	 		end;	\
							if(exist('end_eff_traj', 'var') && ishandle(end_eff_traj) )			delete([end_eff_traj; end_eff_path_trav; end_eff_path_old_handle]); end;"	);
	if ( obs->n_cuboids < n_cuboids_total ) {
		engEvalString( matlab, "if(exist('object_handle', 'var')); delete(object_handle); clear(object_handle); end;" );
	}

	/* Plot temperature zones (part of dynamic obstacle environment - may have changed since last call to function) */
	if ( obs->n_temp_zones > 0 ) {
		assert( SendArraysToMATLAB( __LINE__, matlab, 5,	"n_temp_zones", Int,0, 1,1,1, obs->n_temp_zones,
			"beta", Double,1, 1,1,obs->n_temp_zones, obs->temp_zones->beta,		"h1", Double,1, 1,1,obs->n_temp_zones, obs->temp_zones->h1,
			"h2", Double,1, 1,1,obs->n_temp_zones, obs->temp_zones->h2,			"Tinv", Double,3, obs->n_temp_zones,4,4, obs->temp_zones->Tinv	)  == EXIT_SUCCESS );
		engEvalString( matlab, "tempzone_handles = zeros(3*n_temp_zones,1); figure(TRAJfig);														\
			for j = 1:n_temp_zones;																												\
				[x,y,z] = cylinder( [h1(j)*tand(beta(j)), h2(j)*tand(beta(j))], 30 );					z = (h2(j)-h1(j)).*z + h1(j);			\
				T(1:3,1:3,j) = Tinv(1:3,1:3,j)';		T(1:3,4,j) = -T(1:3,1:3,j)*Tinv(1:3,4,j);	T(4,:,j) = Tinv(4,:,j);						\
				x = x(:)'; y = y(:)'; z = z(:)';		M = T(:,:,j)*[x; y; z; ones(1,length(x))];												\
				x = reshape(M(1,:),2,numel(x)/2);		y = reshape(M(2,:),2,numel(y)/2);				z = reshape(M(3,:),2,numel(z)/2);		\
				tempzone_handles(3*j-2) = surf(x,y,z,ones(size(z)), 'FaceAlpha', obs_alpha); colormap(obs_color);								\
				tempzone_handles(3*j-1) = patch(x(1,:)', y(1,:)', z(1,:)', obs_color, 'FaceAlpha', obs_alpha);									\
				tempzone_handles(3*j)	= patch(x(2,:)', y(2,:)', z(2,:)', obs_color, 'FaceAlpha', obs_alpha);									\
			end; clear x y z M T Tinv beta h1 h2;"	);
	}

	/* Initialize a linkdata handles matrix to keep track of robot config plots.  Plot the initial manipulator configuration. */
	assert( SendArraysToMATLAB( __LINE__, matlab, 1,	"pathlen_new", Double,0, 1,1,1, (double) pathlen_new ) == EXIT_SUCCESS );
	engEvalString( matlab, "linkdata = double.empty([6*n,0]);" );
	WorldCoords( G, DH, path_new[0], n, &C );
	PlotRobotConfigInMATLAB( &C, n_points, 0, matlab );
	engEvalString( matlab, "delete(Cdata);");

	/* Set q initially to the start of the path.  For each waypoint i of path, steer towards waypoint i+1 and save
	the end effector positions.  Once reached, update the plot.   */
	int index = 0;
	end_eff_indices[0] = index;
	for (int i = 0; i < n; i++) {
		q[i]	= path_new[0][i];
	}
	WorldCoords( G, DH, q, n, &C );
	x_end_eff = (double*) realloc( x_end_eff, (index+1)*sizeof(double) );		x_end_eff[index] = C.x[ n_points-1 ];
	y_end_eff = (double*) realloc( y_end_eff, (index+1)*sizeof(double) );		y_end_eff[index] = C.y[ n_points-1 ];
	z_end_eff = (double*) realloc( z_end_eff, (index+1)*sizeof(double) );		z_end_eff[index] = C.z[ n_points-1 ];

	for (int i = 1; i < pathlen_new; i++) {
		while ( DistSq(q,path_new[i],n,w) > epsilon_sq ) {
			Steer( path_new[i], q, n, epsilon, q, w );			// From q starting at path_new[i-1], steer towards path_new[i] and redefine each new point as q again

			/* Find the world coordinates of the robotic arm OBB's and end effector (last point listed for link n) */
			index += 1;
			WorldCoords( G, DH, q, n, &C );
			x_end_eff = (double*) realloc( x_end_eff, (index+1)*sizeof(double) );		x_end_eff[index] = C.x[ n_points-1 ];
			y_end_eff = (double*) realloc( y_end_eff, (index+1)*sizeof(double) );		y_end_eff[index] = C.y[ n_points-1 ];
			z_end_eff = (double*) realloc( z_end_eff, (index+1)*sizeof(double) );		z_end_eff[index] = C.z[ n_points-1 ];
		}

		/* Plot intermediate configurations unless specified otherwise.  The plot for the final manipulator configuration is always shown. */
		end_eff_indices[i] = index;
		if ( i == pathlen_new-1 || plot_intermediate_configs == 1 ) {
			PlotRobotConfigInMATLAB( &C, n_points,	pow(((double) i)/((double) pathlen_new-1),3),	matlab );

			/* Remove the coordinates except on the final iteration (plotting them in the 1st place is required so that the axes bounds are set large enough
				to encompass the link patches (or else an error is returned and nothing is plotted)) */
			if ( i != pathlen_new-1 || plot_coords == 0 ) {
				engEvalString( matlab, "delete(Cdata);" );
			}
		}
	}

	/* Update the end effector path (built in-conjuction with "path" from PlotPathInMATLAB, using the same indices) */
	assert( SendArraysToMATLAB( __LINE__, matlab, 4,	"x", Double,1, 1,1,index+1, x_end_eff,	"y", Double,1, 1,1,index+1, y_end_eff,
		"z", Double,1, 1,1,index+1, z_end_eff,	"end_eff_indices", Int,1, 1,1,pathlen_new, end_eff_indices ) == EXIT_SUCCESS );
	engEvalString( matlab, "																												\
		end_eff_path_index = end_eff_path_indices(max(path_index-1,1));																		\
		end_eff_path_new		= [x', y', z'];																								\
		end_eff_path_old		= end_eff_path( max(end_eff_path_index,1):end_eff_pathlen,: );												\
		if (pathlen_new == 1 && end_eff_path_index == 0); end_eff_path_new = [end_eff_path_old(1,:); end_eff_path_new]; end;				\
		end_eff_path			= [end_eff_path(1:end_eff_path_index,:); end_eff_path_new];		end_eff_pathlen = size(end_eff_path,1);		\
		end_eff_indices			= size(end_eff_path(1:end_eff_path_index,:),1) + 1 + end_eff_indices';										\
		end_eff_path_indices	= [end_eff_path_indices(1:(path_index-1));  end_eff_indices];");

	/* Overlay the end effector trajectory */
	engEvalString( matlab, " figure(TRAJfig);																															\
			end_eff_traj			= plot3( end_eff_path_new(:,1),						end_eff_path_new(:,2),					end_eff_path_new(:,3),					\
											end_eff_format, 'Color', end_eff_traj_color, 'Linewidth', end_eff_linewidth );												\
			end_eff_path_trav		= plot3( end_eff_path(1:end_eff_path_index+1,1),	end_eff_path(1:end_eff_path_index+1,2),	end_eff_path(1:end_eff_path_index+1,3),	\
											'-', 'Color', end_eff_trav_color, 'Linewidth', 1.5 );																		\
			if ( plan_index > 0 );																																		\
				end_eff_path_old_handle	= plot3( end_eff_path_old(:,1),				end_eff_path_old(:,2),				end_eff_path_old(:,3),							\
											'--k', 'Linewidth', 1.5 );																									\
				legend([end_eff_traj; end_eff_path_old_handle], {'New Plan'; 'Former Plan'}, 'Location', 'NorthEastOutside');											\
			else; legend off;	title('Complete End Effector Trajectory');																								\
			end;			view(az_el);		axis equal;		clear x y z;" );

	/* Plot boundary planes (plot last so that they fully span the current axes). */
	if ( obs->n_planes > 0 ) {
		assert( SendArraysToMATLAB( __LINE__, matlab, 4, "planes_a", Double,1, 1,1,obs->n_planes, obs->planes->a,	"planes_b", Double,1, 1,1,obs->n_planes, obs->planes->b,
			"planes_c", Double,1, 1,1,obs->n_planes, obs->planes->c,		"planes_d", Double,1, 1,1,obs->n_planes, obs->planes->d	)  == EXIT_SUCCESS );
		engEvalString( matlab, "figure(TRAJfig); view(az_el); axis equal; V = axis; plane_handles = zeros(n_planes,1); 						\
			for j = 1:n_planes;																												\
				a = planes_a(j);			b = planes_b(j);				c = planes_c(j);				d = planes_d(j);				\
				if	abs(c) > 0.001;			x = [V(1) V(1) V(2) V(2)];		y = [V(3) V(4) V(4) V(3)];		z = -(a.*x + b.*y + d)./c;		\
				elseif	abs(a) > 0.001;		y = [V(3) V(4) V(4) V(3)];		z = [V(5) V(5) V(6) V(6)];		x = -(b.*y + c.*z + d)./a;		\
				else;						x = [V(1) V(1) V(2) V(2)];		z = [V(5) V(6) V(6) V(5)];		y = -(a.*x + c.*z + d)./b;		\
				end;																														\
				plane_handles(j) = patch(x',y',z', obs_color, 'FaceAlpha', obs_alpha);														\
			end;																															\
			view(az_el); axis equal; clear planes_a planes_b planes_c planes_d x y z V;" );
	}

	free(q);
	free(x_end_eff); free(y_end_eff); free(z_end_eff);
	free(C.x); free(C.y); free(C.z);
}

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
			cost = Sqrt( DistSq(q, T->nodes[i], n, w ) );

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

/*! Tests a manipulator configuration *q_new* for violation of constraints.  Determines whether a node *q_new* is safe to add to a tree by testing the manipulator configuration
for violation of obstacle constraints.  First generates all world-frame coordinates corresponding the new manipulator configuration using `WorldCoords`.  Then, depending on the value
specified by *obs_indicator*, examines whether any point intersects one of the obstacles stored within *obs*.  Conducts tests in order of increasing computational complexity, starting
with planes and ending with truncated cones. \see ::obs, ::geom, ::DHparams, BuildRRTs, WorldCoords
	\param[in]	q_new			Query node, of length *n*
	\param[in]	obs_indicator	Indicator of obstacle test type (Enter 0 to test static constraints, 1 to test dynamic constraints, or 2 to test both)
	\returns	A boolean indicating violation (1) or not (0) */
int ConstraintViolation( double* q_new, int n, struct obstacles* obs, struct geom *G, struct DHparams *DH, int obs_indicator ) {

	/* Create variables for OBB coordinate storage (8 points per OBB, for a total of "n" links) */
	int n_points = SumInts(G->N_coords, n);
	struct coords C;
	C.x = (double*) malloc(n_points*sizeof(double));
	C.y = (double*) malloc(n_points*sizeof(double));
	C.z = (double*) malloc(n_points*sizeof(double));

	/* Find the world coordinates of the robot Oriented Bounding Boxes */
	WorldCoords( G, DH, q_new, n, &C );

	//FILE *datafile = fopen( "C:/Users/Joe/Desktop/testcoords.dat", "w+");
	//fprintf(datafile, "%s\t%s\t%s\n", "x", "y", "z");
	//for (int k = 0; k < SumInts(G->N_coords, n); k++) {
	//	fprintf(datafile, "%5.3Lf\t%5.3Lf\t%5.3Lf\n", C.x[k], C.y[k], C.z[k]);
	//}
	//fclose(datafile);

	int boolean = 0;
	double f, f1, f2, f3, f4, f5, f6;
	double** v		= Make2DDoubleArray(4,1);
	double** v_new	= Make2DDoubleArray(4,1);

	/* Test the coordinates for feasibility */
	if ((obs_indicator == 0) || (obs_indicator == 2)) {
		for (int i = 0; i < obs->n_planes; i++) {
			for (int j = 0; j < n_points; j++) {
				f = obs->planes->a[i]*C.x[j] + obs->planes->b[i]*C.y[j] + obs->planes->c[i]*C.z[j] + obs->planes->d[i];
				if (f < 0) {
					boolean = 1;
					goto END_OF_FUNCTION;
				}
			}
		}

		for (int i = 0; i < obs->n_cuboids; i++) {
			for (int j = 0; j < n_points; j++) {
				f1 = obs->cuboids->a[6*i+0]*C.x[j] + obs->cuboids->b[6*i+0]*C.y[j] + obs->cuboids->c[6*i+0]*C.z[j] + obs->cuboids->d[6*i+0];
				f2 = obs->cuboids->a[6*i+1]*C.x[j] + obs->cuboids->b[6*i+1]*C.y[j] + obs->cuboids->c[6*i+1]*C.z[j] + obs->cuboids->d[6*i+1];
				f3 = obs->cuboids->a[6*i+2]*C.x[j] + obs->cuboids->b[6*i+2]*C.y[j] + obs->cuboids->c[6*i+2]*C.z[j] + obs->cuboids->d[6*i+2];
				f4 = obs->cuboids->a[6*i+3]*C.x[j] + obs->cuboids->b[6*i+3]*C.y[j] + obs->cuboids->c[6*i+3]*C.z[j] + obs->cuboids->d[6*i+3];
				f5 = obs->cuboids->a[6*i+4]*C.x[j] + obs->cuboids->b[6*i+4]*C.y[j] + obs->cuboids->c[6*i+4]*C.z[j] + obs->cuboids->d[6*i+4];
				f6 = obs->cuboids->a[6*i+5]*C.x[j] + obs->cuboids->b[6*i+5]*C.y[j] + obs->cuboids->c[6*i+5]*C.z[j] + obs->cuboids->d[6*i+5];
				if ((f1 < 0) && (f2 < 0) && (f3 < 0) && (f4 < 0) && (f5 < 0) && (f6 < 0)) {
					boolean = 1;
					goto END_OF_FUNCTION;
				}
			}
		}

		for (int i = 0; i < obs->n_cylinders; i++) {
			for (int j = 0; j < n_points; j++) {
				v[0][0] = C.x[j];
				v[1][0] = C.y[j];
				v[2][0] = C.z[j];
				v[3][0] = 1;
				MatrixMultiply( obs->cylinders->Tinv[i], v, 4, 4, 1, v_new );
				f = pow( v_new[0][0], 2.0 ) + pow( v_new[1][0], 2.0 ) - pow( obs->cylinders->r[i], 2.0 );
				if ( (f < 0) && (v_new[2][0] < (obs->cylinders->H[i]/2.0)) && (v_new[2][0] > -(obs->cylinders->H[i]/2.0))) {
					boolean = 1;
					goto END_OF_FUNCTION;
				}
			}
		}

		for (int i = 0; i < obs->n_cones; i++) {
			for (int j = 0; j < n_points; j++) {
				v[0][0] = C.x[j];
				v[1][0] = C.y[j];
				v[2][0] = C.z[j];
				v[3][0] = 1;
				MatrixMultiply( obs->cones->Tinv[i], v, 4, 4, 1, v_new );
				f = pow( v_new[0][0], 2.0 ) + pow( v_new[1][0], 2.0 ) - pow( v_new[2][0]*tan(obs->cones->beta[i]*(PI/180)), 2.0 );
				if ( (f < 0) && (v_new[2][0] > obs->cones->h1[i]) && (v_new[2][0] < obs->cones->h2[i]) ) {
					boolean = 1;
					goto END_OF_FUNCTION;
				}
			}
		}
	}
	if ((obs_indicator == 1) || (obs_indicator == 2)) {
		for (int i = 0; i < obs->n_temp_zones; i++) {
			for (int j = 0; j < n_points; j++) {
				v[0][0] = C.x[j];
				v[1][0] = C.y[j];
				v[2][0] = C.z[j];
				v[3][0] = 1;
				MatrixMultiply( obs->temp_zones->Tinv[i], v, 4, 4, 1, v_new );
				f = pow( v_new[0][0], 2.0 ) + pow( v_new[1][0], 2.0 ) - pow( v_new[2][0]*tan(obs->temp_zones->beta[i]*(PI/180)), 2.0 );

				if ( (f < 0) && (v_new[2][0] > obs->temp_zones->h1[i]) && (v_new[2][0] < obs->temp_zones->h2[i]) ) {
					boolean = 1;
					goto END_OF_FUNCTION;
				}
			}
		}
	}

	END_OF_FUNCTION:
	for (int i = 3; i >= 0; i--) {
		free(v[i]); free(v_new[i]);
	}
	free(v); free(v_new);
	free(C.x); free(C.y); free(C.z);

	return boolean;
}

/*! Identify and mark as unsafe any temperature obstacle violators.  Modifies the "safety" property of tree nodes that are found to reside in or result in inevitable collision with
temperature obstacles pointed to by *obs*.  Scans and marks the nodes of trees T[0] (forward tree) and T[1] (reverse tree) one-by-one.  If the tree is a reverse tree, all decendents
leading to an unsafe node are also unsafe, as well as any leaves in the other tree directly connected to it.  This is where the benefits of the augmented data "connections" and
"leaf_lists" come into play.  If the goal node, i.e.\ root of T[1], is found to be unsafe, then abort the program, as nothing can be done to recover the arm and find a new safe path
(all paths lead to collision). \see ::tree, ::obs, WorldCoords, ConstructTempObstacle */
void TempObsViolation(struct tree** T, int* num_nodes, int n, struct obstacles* obs, struct geom *G, struct DHparams *DH ){

	/* Create variables for OBB coordinate storage (8 points per OBB, for a total of "n" links) */
	int n_points = SumInts(G->N_coords, n);
	struct coords C;
	C.x = (double*) malloc(n_points*sizeof(double));
	C.y = (double*) malloc(n_points*sizeof(double));
	C.z = (double*) malloc(n_points*sizeof(double));

	int i, j, k, t, a;
	double f;
	double** v = Make2DDoubleArray(4,1);
	double** v_new = Make2DDoubleArray(4,1);
	struct list_node *ptr;
	int tree_type[] = {1, -1};					/* tree_type = -1 (reverse tree) or +1 (forward tree) */

	for (t = 1; t >= 0; t--) {
		for (k = 0; k < num_nodes[t]; k++) {

			/* Find the world coordinates of the robot Oriented Bounding Boxes */
			if (T[t]->safety[k] == 1) {
				WorldCoords( G, DH, T[t]->nodes[k], n, &C );
			}

			i = 0;
			while ((T[t]->safety[k] == 1) && (i < obs->n_temp_zones)) {
				for (j = 0; j < n_points; j++) {
					v[0][0] = C.x[j];
					v[1][0] = C.y[j];
					v[2][0] = C.z[j];
					v[3][0] = 1;
					MatrixMultiply( obs->temp_zones->Tinv[i], v, 4, 4, 1, v_new );
					f = pow( v_new[0][0], 2.0 ) + pow( v_new[1][0], 2.0 ) - pow( v_new[2][0]*tan(obs->temp_zones->beta[i]*(PI/180)), 2.0 );

					if ( (f < 0) && (v_new[2][0] > obs->temp_zones->h1[i]) && (v_new[2][0] < obs->temp_zones->h2[i]) ) {
						T[t]->safety[k] = 0;

						/* If the tree is a forward tree, any ancestors with ONLY a single line through node k are also unsafe (currently cannot implement) */
						/* If the tree is a reverse tree, all decendents leading to node k are also unsafe, as well as the leaves on the other tree */
						if ( (tree_type[t] == -1) && (k != 0) ) {
							ptr = T[t]->leaf_lists[k];
							while ( ptr != NULL ) {
								a = ptr->data;
								T[(t+1)%2]->safety[ T[t]->connections[a] ] = 0;
								while ( a != k && T[t]->safety[a] == 1 && a != 0 ) {
									T[t]->safety[a] = 0;
									a = T[t]->parents[a];
								}
								ptr = ptr->next;
							}
						}

						break;
					}
				}
				i = i + 1;
			}
		}


		if (tree_type[t] == -1) {
			/* If a reverse-tree, ensure that the root (goal state) is not unsafe or else abort */
		//	assert( T[t]->safety[0] == 1 );
			T[t]->safety[0] = 1;
		}
	}

	for (int i = 3; i >= 0; i--) {
		free(v[i]); free(v_new[i]);
	}
	free(v); free(v_new);
	free(C.x); free(C.y); free(C.z);
}

/*! Implements one iteration of `Steer` and tests for constraint violation, attempting to extend from node *q_near* towards *q*. \see BuildRRTs, Connect, Steer, DistSq, ConstraintViolation
	\param[in]		q				Target joint-angle vector, of length *n*
	\param[in]		q_near			Nearest joint-angle vector to *q* in the current tree, of length *n*
	\param[in,out]	q_new			Pre-allocated vector of length *n*, used to store new joint-angle configuration
	\param[in,out]	cost_to_go		Returned as pointer to cost-to-go from *q_near* to *q_new*
	\param[in]		indicator		Indicator of obstacle test type (see `ConstraintViolation`)
	\returns		The status of `Extend` (either *Trapped* (0), *Advanced* (1), or *Reached* (2)) */
int Extend(double* q, double* q_near, double epsilon, int n, double* w, double* q_new, struct obstacles* obs, struct geom *G, struct DHparams *DH, double* cost_to_go, int indicator) {

	/* Steer from q_near to q, returning new state q_new (which lies within a distance-squared of epsilon from q_near) */
	*cost_to_go += Steer( q, q_near, n, epsilon, q_new, w );

	/* Test whether the node q_new violates the constraints.  It is implicitly assumed by this algorithm that, provided epsilon is small enough,
		we can take the incremental segment between q_near and q_new as safe given that their endpoints are safe. */
	if ( ConstraintViolation( q_new, n, obs, G, DH, indicator ) == false ) {
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

/*! Repeatedly calls `Extend` to incrementally test path safety and attempt to grow a branch from node *q_near* to *q*. \see BuildRRTs, Extend, DistSq, ConstraintViolation
	\param[in]		q				Target joint-angle vector, of length *n*
	\param[in]		q_near			Nearest joint-angle vector to *q* in the current tree, of length *n*
	\param[in,out]	q_new			Pre-allocated vector of length *n*, used to store new joint-angle configuration
	\param[in,out]	cost_to_go		Returned as pointer to cost-to-go from *q_near* to *q_new*
	\param[in]		indicator		Indicator of obstacle test type (see `ConstraintViolation`)
	\returns		The status of `Connect` (must be either *Trapped* (0), or *Reached* (2)) */
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

/*! Adds a new node to an RRT structure.  Re-allocates memory and inserts new node *q_new* into the tree pointed to by *T*, increasing the number of nodes by 1 and
setting the nodes properties.  Depending on the NearestNeighbor algorithm indicated by *NN_alg*, also adds the new node to the corresponding KD-tree.  If the MATLAB
Engine pointer *matlab* is not NULL, adds the node and the edge from its parent to RRTfig. \see ::tree, BuildRRTs, PlotPathInMATLAB
	\param[in,out]	T			Pointer to the tree in which the new node should be inserted.  Returned with updated tree.
	\param[in,out]	num_nodes	Pointer to the number of nodes in the tree. Returned as pointer to updated node count value.
	\param[in]	q_new		New node to insert into the tree
	\param[in]	parent_index, cost_to_go, connection, leaf_list, safety		Properties of the new node (typically initially assumed to be unconnected (-1), with a NULL leaf list, and safe (1)) */
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

/*! Adds *leaf_index* to all ancestors' leaf lists up to and including the root node */
void AddLeafToLists(struct tree *T, int leaf_index) {
	int a	= T->parents[leaf_index];
	while (a != -1) {
		T->leaf_lists[a] = AddListElement(T->leaf_lists[a], leaf_index);
		a			= T->parents[a];
	}
}

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

	/* Identify if the trees to-be-built are new based on the current value of "iter".  Determines whether currently in feedback or not. */
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

			/* Increment "iter".  If in feedback mode, exit the function after a single iteration. */
			*iter = *iter + 1;
			if (feedback_mode == 1) {
				num_nodes[0]	= num_nodesA;
				num_nodes[1]	= num_nodesB;
				goto END_OF_FUNCTION;
			}

			/* Report every 10% of progress to the console window */
			if ( ((10*(*iter)) % max_iter) == 0 ) {
				printf("\t%02d%s\n", (100*(*iter))/max_iter, "% complete...");
			}

			/* Swap tree pointers to alternate between "extending" or "connecting" for each tree */
			temp_tree_ptr = Ta;			temp_int	= num_nodesA;
			Ta		= Tb;				num_nodesA	= num_nodesB;
			Tb		= temp_tree_ptr;	num_nodesB	= temp_int;

			/* Break the loop if a feasible solution has been found and only one is desired */
			if ( (feasible == 1 && strncmp(soln, "feasible", 8) == 0) ) {
				break;
			}
		}

		/* If "iter" is even, an odd-number of loops was conducted (and therefore an odd number of swaps), and vice versa. If this is so,
		Ta and Tb are now mixed-up from their original order.  Swap them back into place (to correspond with n_nodes and treefiles again). */
		if ((*iter % 2) == 0) {
			temp_tree_ptr = Ta;			temp_int	= num_nodesA;		//temp_int2 = node_star_index[0];
			Ta		= Tb;				num_nodesA	= num_nodesB;		//node_star_index[0] = node_star_index[1];
			Tb		= temp_tree_ptr;	num_nodesB	= temp_int;			//node_star_index[1] = temp_int2;
		}

		/* For all parent nodes of connected leaves, add to their "leaf_lists" the indices of the leaves they are connected to.
		Thus, all nodes directly connected to the other tree have in their "leaf_list" the index of the node in the other
		tree to which they are connected.  All nodes not directly connected to the other tree have in their "leaf_list" a list of
		all their descendent nodes that do directly connect to the other tree. */
		printf("\t%s\n", "Saving connected leaf nodes to leaf lists...");
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
		printf("\t%s\n", "Writing tree data to files...");
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
		printf("\t%s\n", "Loading tree data from files...");
		treefile = fopen(strcat(filename, "trees.dat"), "r");		/* Open tree file for reading */
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

/*! Traces the indices along a connected node path.  Traces the path from a node *node1* in the tree *T* up to its ancestor *node2* (or root, if it comes first) and
returns a pointer to the index path array.  Note this path is traced in the backwards direction from what is intended for a forward tree. \see StorePath
	\param[in]		node1		Origin node at which to begin the traversal
	\param[in]		node2		Ancestor node at which to stop
	\param[in,out]	pathlen		Pointer to the number of nodes along the tree path */
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

/*! Stores the total sequence of nodes along a connected pair of forward-tree and reverse-tree paths.  Traverses from the initial point in *pathA* to the final point
in *pathB*, in correct order, saving the nodes encountered along the way. Saves the resulting sequence of nodes to <*filename*>output.dat.  \see TracePath
	\param[in]		Ta, Tb				Pointers to forward and reverse trees, respectively
	\param[in]		pathA, pathB		Index path arrays from Ta and Tb to-be-merged (see `TracePath`)
	\param[in]		pathlenA, pathlenB	Lengths of each *pathA* and *pathB*
	\param[in]		I					Index of current motion plan (used to label saved motion plans. Enter -1 to suppress file output).
	\param[in,out]	path				Pre-allocated `double` array of size \f$(\f$(*pathlenA*+*pathlenB*)\f$\times n)\f$, returned with node path */
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

/*! Split a path edge at node *q* and insert it into the appropriate tree. \see TracePath, StorePath, InsertNode, MergeLeafListBwithA */
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

/*! Replan according to the shortest-distance paths that do not currently violate obstacle constraints.  *Cost-Priority Search*: Generates a sorted list of potential re-plan paths
according to the shortest-paths among all possible paths through the set of *max_replan_neighbors* number of nearest-neighbors in each tree.  Note this does
not conduct a full search, nor does it compute the globally-shortest path (but merely the globally shortest path among the set of locally closest safe nodes).
\see SaveBestPlans, FindSafePath, ExhaustiveRePlan
	\param[in]		max_replans				Maximum number of cost-priority re-plan paths to search for
	\param[in]		max_replan_neighbors	Maximum number of replan neighbors to use during search, for each tree (forward and reverse)
	\param[in,out]	replan_indices			Pre-allocated list of size *max_replans*\f$\times3\f$, used to store indices of re-plan paths (see `SaveBestPlans`).
	\returns The number of re-plan paths found and stored in *replan_indices* */
int RePlan(struct tree **T_ptrs, int* n_nodes, double* q, int n, double* w, int max_replans, int max_replan_neighbors, double eta_RRT, double gamma_RRT, int** replan_indices, char* NN_alg) {

	double cost_to_go;
	int num_replans = 0, n_neighbors = 0, cost_type = 2;
	struct list_node *node_ptr;
	double *replan_costs	= (double*) malloc(max_replans*sizeof(double));
	int *neighbors			= (int*) malloc(max_replan_neighbors*sizeof(int));
	double *costs			= (double*) malloc(max_replan_neighbors*sizeof(double));
	int *I					= (int*) malloc(max_replans*sizeof(int));

	// Cost-Priority Search
	/* Find minimum-cost paths back to the forward tree.  Search the nearest neighbors in the forward tree from q, obtaining the number of candidates
	n_neighbors, the cost to connect to each one, costs, and the location of each, neighbors.  Search according to NN_alg and the cost_type above. */
	NearestNeighbors( T_ptrs[0], n_nodes[0], q, n, w, max_replan_neighbors, cost_type, eta_RRT, gamma_RRT, neighbors, costs, &n_neighbors, NN_alg );
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
	NearestNeighbors( T_ptrs[1], n_nodes[1], q, n, w, max_replan_neighbors, cost_type, eta_RRT, gamma_RRT, neighbors, costs, &n_neighbors, NN_alg );
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

/*! Replan by searching over all safe nearest neighbors until a feasible solution is discovered (if one exists). *Feasibility-Priority Search*: In the case that the
cost-priority search from `RePlan` was unsuccessful, conduct an exhaustive search with focus not on costs but on feasibility.  First search for feasible nodes
over each tree (listed in order of proximity), then accept the first feasible solution found. Calls `FindSafePath` one path at a time until successful. \see SaveBestPlans, RePlan, FindSafePath */
int ExhaustiveRePlan(struct tree **T_ptrs, int* n_nodes, double eta_RRT, double gamma_RRT, char* NN_alg, int* pathlenA, int* pathlenB, int** pathA, int** pathB,
	double*** path, double* q, double epsilon, int n, double* w, struct obstacles* obs, struct geom *G, struct DHparams *DH, char* filename) {

	int n_neighbors = 0, cost_type = 2, k = 0, unsafe = 1;
	int *neighbors				= (int*) malloc( (max(n_nodes[0], n_nodes[1]))*sizeof(int) );
	double *costs				= (double*) malloc( (max(n_nodes[0], n_nodes[1]))*sizeof(double) );
	int **replan_indices		= Make2DIntArray(1,3);
	struct list_node* node_ptr	= NULL;

	// Feasibility-Priority Search
	/* In the case that the cost-priority search from RePlan was unsuccessful, conduct an exhaustive search with focus not on costs but on
	feasibility.  First search for feasible nodes over each tree, then accept the first feasible solution found.  */
	for (int t = 1; t >= 0; t--) {
		NearestNeighbors( T_ptrs[t], n_nodes[t], q, n, w, n_nodes[t], cost_type, eta_RRT, gamma_RRT, neighbors, costs, &n_neighbors, NN_alg );

		while (unsafe == 1 && k < n_neighbors) {
			if ( T_ptrs[t]->safety[ neighbors[k] ] == 1 ) {

				node_ptr = T_ptrs[t]->leaf_lists[ neighbors[k] ];
				while ( node_ptr != NULL && unsafe == 1 ) {
					if ( T_ptrs[t]->safety[ node_ptr->data ] == 1 ) {
						replan_indices[0][0] = t;
						replan_indices[0][1] = neighbors[k];
						replan_indices[0][2] = node_ptr->data;
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

/*! Reset the simulation for a new run.  Resets node saftey properties to 1, removes any temperature obstacles from previous run,
re-defines the new "best" plan by finding the new shortest paths that result from any node and edge additions made during the previous simulation,
and closes old plots so that they may be regenerated during the next simulation. */
void ResetSimulation( struct tree* trees, int n_trees, int n_plans, int* num_nodes, int* feasible, int** node_star_index, struct obstacles* obs, Engine* matlab ) {
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


/*+===========================================+*/
/*| Main function of the Manipulator RRT Code |*/
/*+===========================================+*/

/*! Main function for running the manipulator RRT code.  Code for controlling the Stanford SACL manipulator.  Enter all static input parameters here, including
general user settings, goal waypoint profile, manipulator geometry, obstacle parameters, manipulator hardware parameters, and closed-loop simulation parameters. */
int main() {
	setbuf(stdout, (char*) 0);										/* Completely flush output buffer on early termination so that the location of a program failure is not misidentified */
	printf("\nMANIPULATOR RRT Motion Planner\n");
	printf("------------------------------\n");

	/*+=======================+*/
	/*| Enter User Input here |*/
	/*+=======================+*/

	/* (1) Define general user settings */
	char load_previous			= 'y';								/* Load a previous run? (y/n) */
	char disp_matlab_plots		= 'y';								/* Display MATLAB plots? (y/n/a) [a = "all" (also plots RRT figures, which is very time-consuming)] */
	char filename[FILENAME_MAX]	= "C:/Users/user/Desktop/SACL/RRT1";	/* Full filepath and filename root for reading/writing data, no extension (e.g. "C:/.../Fileroot") */
	char soln[11]				= "suboptimal";						/* Type of solution to seek: "feasible" (exit at 1st solution found), "suboptimal" (attempt to find "max_neighbors" # of solutions) */
	char sampling[13]			= "halton";							/* Sampling sequence: "pseudorandom", "halton" */
	char NN_alg[12]				= "brute_force";					/* Nearest neighbor algorithm: "brute_force", "kd_tree" (INOPERATIVE - BUG IN KDTREE LIBRARY) */
	int max_iter				= 2500;								/* Maximum number of iterations to use for pre-computation */
	int	max_neighbors			= 150;								/* Maximum number of re-wiring neighbors (1 for none, max_iter for full tree) */
	double epsilon				= 2.0;								/* Angular resolution of trees, and tolerance for path execution [deg] */

	/* (2) Specify manipulator waypoints, and define manipulator geometry */
	int n					= 4;									/* Dimension of manipulator C-space */
	int n_waypoints			= 4;									/* Number of waypoints to traverse ( must be >= 2, initial and goal ) */
	double q_waypoints[]	= { 45.063-20.0,	97.429-20.0,	159.777-20.0,	185.92-20.0,		/* Joint configuration waypoints [deg], */
								45.063-20.0,	89.35-20.0,		140.69-20.0,	174.89-20.0,		/*		specified in "n_waypoints" groups of "n" doubles */
								174.88-20.0,	155.73-20.0,	166.777-20.0,	132.598-20.0,
								45.063-20.0,	97.429-20.0,	159.777-20.0,	185.92-20.0 };
	double w[]				= { 1.0, 6.68, 10.5, 2.2 };				/* Weighting parameters (scale factors) to use for the weighted Euclidean distance metric */
	double grip_angles[3]	= { 20.0-20.0, 125.3-20.0, 55.2-20.0 };	/* "Default", "Grasp mode", and "Drop mode" actuator angles for the gripper [deg] */
	int	   grip_actions[2]	= { 1, 2 };								/* Waypoint indices after which "Grasp mode" and "Drop mode" should be implemented (0,1,2,...) */
	double grip_pos[3]		= { 0.0, -3.5, 0.0 };					/* Body-fixed position of the end-effector (w.r.t. link "n") [in] (used mainly for traj. visualization) */
	double q_min[]			= { 0.0, 0.0, 0.0, 0.0 };				/* Minimum bounds on joint angles [deg] */
	double q_max[]			= { 180.0, 180.0, 180.0, 180.0 };		/* Maximum bounds on joint angles [deg] */
	double L[]				= { 3.25, 6.68, 10.5, 2.2 };			/* Lengths	(x along each body-fixed 0 deg line)	of each link [in] */
	double W[]				= { 2.4, 1.0, 1.75, 4.0 };				/* Widths	(y completes the RHR)					of each link [in] */
	double H[]				= { 1.0, 3.3, 2.25, 2.75 };				/* Heights	(z along each body-fixed rotation axis) of each link [in] */
	double rho_x[]			= { -2.375, -0.4, -2.0, -1.575 };		/* Body-fixed x-position of back-left, bottom corner of cuboid OBB's (w.r.t. their body-fixed origin) [in] */
	double rho_y[]			= { -1.2, -0.5, -1.15, -3.5 };			/* Body-fixed y-position of back-left, bottom corner of cuboid OBB's (w.r.t. their body-fixed origin) [in] */
	double rho_z[]			= { -0.5, -1.65, -1.125, -1.375 };		/* Body-fixed z-position of back-left, bottom corner of cuboid OBB's (w.r.t. their body-fixed origin) [in] */
	double d[]				= { 0, 0, 0, 0 };						/* Denavit-Hartenberg (DH) parameter: translation along the z_i (rotation) axes [in] (p.103 LaValle) */
	double a[]				= { 0, 0, 6.75, 8.25 };					/* Denavit-Hartenberg (DH) parameter: translation along the x_i-1 (perp) axes [in] (p.103 LaValle) */
	double alpha[]			= { 0, -90, 180, 180 };					/* Denavit-Hartenberg (DH) parameter: rotation angles about x_i-1 (perp) axes [deg] (p.103 LaValle) */
	int	   n_facepts[]		= { 0, 0, 0, 5, 0, 10,					/* Number of additional points (besides corner points) to use for cuboid OBB's, specified in */
								20, 0, 20, 30, 0, 0 };				/*		groups of 3 for each link (for faces || to the body-fixed xy, yz, and xz respectively) */

	/* (3) Define static obstacle regions (plane: f = dot(nhat,[x,y,z]) + d < 0, cylinder: f = r - R < 0, cuboid: union of six planes)*/
	int n_planes			= 1;									/* Number of planar obstacles */
	double nhat_planes[]	= { 0.0, 0.0, -1.0 };					/* Plane unit-normal vectors (resolved in world frame) */
	double xyz_planes[]		= { 0.0, 0.0, 2.5 };					/* Coordinates (resolved in world frame) of any point in the plane(s) [in] */
	int n_cylinders			= 2;									/* Number of cylindrical obstacles */
	double r_cylinders[]	= { 2.5, 1.5 };							/* Cylinder radii [in] */
	double H_cylinders[]	= { 2.0, 4.2 };							/* Cylinder heights [in] */
	double YPR_cylinders[]	= { 0.0, 0.0, 0.0,						/* Yaw, pitch, and roll (rel. to world frame) of cylinder axes [deg] */
								0.0, 0.0, 0.0 };
	double xyz_cylinders[]	= { 0.0, 0.0, 2.7 - 2.0/2,				/* Coordinates (resolved in world frame) of cylinder centers [in] */
								-5.5, 1.75, 2.7 - 4.2/2 };
	int n_cuboids			= 1;									/* Number of cuboidal obstacles */
	double YPR_cuboids[]	= { -30.0, 0.0, 0.0 };					/* Yaw, pitch, and roll (rel. to world frame) of cuboids [deg] */
	double LWH_cuboids[]	= { 1.0, 0.6, 2.0 };					/* Length, width, and height of each cuboid [in] */
	double xyz_cuboids[]	= { 10.0, 10.0*sin(PI/6), 2.5 - 1.0 };	/* Coordinates (resolved in world frame) of cuboid centers [in] */
	int i_grip_obs			= 0;									/* Index of the cuboidal obstacle corresponding to the grasped object (0,1,2...) */

	/* (4) Set manipulator hardware parameters (used during motion plan execution) */
	int sensor_link				= 3;								/* Link on which the sensing skin resides (1,2,3,...) */
	int channels[]				= { 0, 1, 2, 4 };					/* Servo channels corresponding to each joint angle, q_i */
	int grip_channel			= 5;								/* Servo channel corresponding to the end effector */
	int sensor_channels[]		= { 0, 1, 2, 3, 4, 5 };				/* Temp sensor channels */
	int n_tempsensors			= 6;								/* Number of temperature sensors on the sensing skin */
	int rate_tempsensors		= 500;								/* Data rate of the temperature sensor board */
	int max_replans				= 300;								/* Maximum number of new plans to consider in the case that an original plan fails */
	int max_replan_neighbors	= 100;								/* Maximum number of nearest-neighbors to use for replanning from a current state to each tree */
	double max_temp				= 200;								/* Maximum temperature allowed [deg C] */
	double max_t_horizon		= 15.0;								/* Maximum horizon time [s] */
	double max_t_rebuild		= 30.0;								/* Maximum time allowed to re-build trees in a last-ditch attempt to find a safe path (must be >= max_t_horizon) */
	double t_tempwait			= 6.0;								/* Time to wait for temperature sensor estimation algorithm [s] */
	double AccelThrottle[]		= { 0.3, 0.3, 0.3, 0.3 };			/* Set the accelerations of each link actuator as a fraction from AccelMin to AccelMax */
	double VelLimThrottle[]		= { 0.002, 0.002, 0.002, 0.002 };	/* Set the velocity limits of each link actuator as a fraction from VelMin to VelMax */
	double grip_AccelThrottle	= 0.01;								/* Set the acceleration the end effector actuator as a fraction from AccelMin to AccelMax */
	double grip_VelLimThrottle	= 1.0;								/* Set the velocity limit of the end effector actuator as a fraction from VelMin to VelMax */

	// char	tempsensor_file[]	= "";											/* Enter full path to sensor estimation algorithm output file (or enter "" to use potentiometers as simulated sensors for debugging) */
	char	tempsensor_file[]	= "C:/Users/user/Desktop/SACL/interfacing.txt";	/* Enter full path to sensor estimation algorithm output file (or enter "" to use potentiometers as simulated sensors for debugging) */
	double pos_x_tempsensors[]  = { 2.0, 2.0, 2.0, 5.0, 5.0, 5.0 };				/* Potentiometer simulation: "sensor_link" body-fixed x-position of each simulated temperature sensor */
	double pos_y_tempsensors[]  = { 0.0, -1.15, 0.0, 0.0, -1.15, 0.0 };			/* Potentiometer simulation: "sensor_link" body-fixed y-position of each simulated temperature sensor */
	double pos_z_tempsensors[]  = { 1.125, 0.0, -1.125, 1.125, 0.0, -1.125 };	/* Potentiometer simulation: "sensor_link" body-fixed z-position of each simulated temperature sensor */
	double n_x_tempsensors[]	= { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };				/* Potentiometer simulation: "sensor_link" body-fixed x-component of the outward-pointing normal vectors at each simulated temperature sensor */
	double n_y_tempsensors[]	= { 0.0, -1.0, 0.0, 0.0, -1.0, 0.0 };			/* Potentiometer simulation: "sensor_link" body-fixed y-component of the outward-pointing normal vectors at each simulated temperature sensor */
	double n_z_tempsensors[]	= { 1.0, 0.0, -1.0, 1.0, 0.0, -1.0 };			/* Potentiometer simulation: "sensor_link" body-fixed z-component of the outward-pointing normal vectors at each simulated temperature sensor */

	/* (5) Define temperature obstacle parameters */
	double r_tempobs			= 0.5;			/* Radius at sensor position of truncated-cone temperature obstacle(s) [in] */
	double offset_tempobs		= 1.5;			/* Normal offset from the sensor at which to begin the cone */
	double H_tempobs			= 5;			/* Height of truncated-cone temperature obstacle(s) located above the sensor [in] */
	double beta_tempobs			= 15;			/* Half-angle of truncated-cone temperature obstacle(s) [deg] */
	double	t_obs_intro[]		= { 10.0 };		/* Time at which any artificial temperature obstacles are introduced (must be in ascending order) [s] */
	int		obs_sensor[]		= { 2 };		/* Sensors that "detect" fake temperature obstacles at the times above (1,2,3... - enter NULLs for no obstacles) */
	int		n_fakeobs			= 1;			/* Total number of artificial temperature sensor violations */
	int		n_tempobs_max		= 1;			/* Maximum number of temperature obstacles allowed */

	// Demos: 8.0 s, 6		10.0 s, 2		11.0 s, 5		8.0,13.0 6,5

	/* Verify that user data has been input correctly */
	printf("%s\n", "Verifying input...");
	assert( strncmp(&load_previous, "n", 1) == 0		|| strncmp(&load_previous, "y", 1) == 0 );
	assert( strncmp(&disp_matlab_plots, "n", 1) == 0	|| strncmp(&disp_matlab_plots, "y", 1) == 0			|| strncmp(&disp_matlab_plots, "a", 1) == 0 );
	assert( strncmp(NN_alg, "brute_force", 11) == 0		|| strncmp(NN_alg, "kd_tree", 7) == 0 );
	assert((strncmp(sampling, "halton", 6) == 0			|| strncmp(sampling, "pseudorandom", 12) == 0)		&& strchr( filename, '.' ) == NULL );
	assert( grip_AccelThrottle >= 0		&& grip_AccelThrottle <= 1	&& grip_VelLimThrottle >= 0				&& grip_VelLimThrottle <= 1 );
	assert( Numel(grip_actions) == 2	&& grip_actions[0] >= 0		&& grip_actions[1] > grip_actions[0]	&& grip_actions[1] <= n_waypoints-1 );
	assert( Numel(grip_angles) == 3		&& n_waypoints >= 2			&& n >= 1		&& epsilon > 0			&& Numel(n_facepts) == 3*n );
	assert( max_iter > 0		&& max_neighbors <= max_iter		&& max_replan_neighbors <= max_iter		&& sensor_link >= 1			&& Numel(w) == n );
	assert( i_grip_obs >= 0		&& i_grip_obs <= n_cuboids-1		&& max_t_rebuild > max_t_horizon );
	assert( Numel(L) == n		&& Numel(W) == n		&& Numel(H) == n		&& Numel(rho_x) == n		&& Numel(rho_y) == n		&& Numel(rho_z) == n );
	assert( Numel(d) == n		&& Numel(a) == n		&& Numel(alpha) == n	&& max_t_horizon > 0		&& max_temp > 0				&& Numel(grip_pos) == 3 );
	assert( Numel(q_min) == n	&& Numel(q_max) == n	&& Numel(q_waypoints) == n_waypoints*n );
	assert( Numel(nhat_planes) == 3*n_planes			&& Numel(xyz_planes) == 3*n_planes );
	assert( Numel(r_cylinders) == n_cylinders			&& Numel(H_cylinders) == n_cylinders );
	assert( Numel(YPR_cylinders) == 3*n_cylinders		&& Numel(xyz_cylinders) == 3*n_cylinders );
	assert( Numel(YPR_cuboids) == 3*n_cuboids			&& Numel(xyz_cuboids) == 3*n_cuboids				&& Numel(LWH_cuboids) == 3*n_cuboids );
	assert( r_tempobs > 0	&& Numel(pos_x_tempsensors) == n_tempsensors	&& Numel(pos_y_tempsensors) == n_tempsensors	&& Numel(pos_z_tempsensors) == n_tempsensors );
	assert( H_tempobs > 0	&& Numel(n_x_tempsensors) == n_tempsensors		&& Numel(n_y_tempsensors) == n_tempsensors		&& Numel(n_z_tempsensors) == n_tempsensors );

	/*+================================+*/
	/*| Begin preliminary calculations |*/
	/*+================================+*/

	/* Print input values to file (to be used by the visualization code in MATLAB) and/or load previous input values */
	GenerateInput( filename, soln, sampling, &max_iter, &max_neighbors, &epsilon, &n, &n_waypoints, q_waypoints, q_min, q_max,
		grip_actions, grip_angles, n_facepts, L, W, H, rho_x, rho_y, rho_z, d, a, alpha, &n_planes, nhat_planes, xyz_planes,
		&n_cylinders, YPR_cylinders, xyz_cylinders, r_cylinders, H_cylinders, &n_cuboids, YPR_cuboids, LWH_cuboids, xyz_cuboids, load_previous );

	/* Create a structure for storage of link geometry */
	printf("Creating structures for input variable storage...\n");
	struct geom *G	= (struct geom*) malloc( sizeof(struct geom) );
	G->L			= L;			G->W		= W;			G->H			= H;
	G->rho_x		= rho_x;		G->rho_y = rho_y;			G->rho_z		= rho_z;
	G->Body_coords	= (double***) malloc( n*sizeof(double**) );
	G->N_coords		= (int*) malloc( n*sizeof(int) );
	BodyFixedOBBcoords( G, n_facepts, grip_pos, n );

	/* Create a DH parameters structure */
	struct DHparams *DH = (struct DHparams*) malloc( sizeof(struct DHparams) );
	DH->a	= a;			DH->d	= d;			DH->alpha	= alpha;

	/* Generate obstacle primitives */
	struct obstacles *obs	= (struct obstacles*) malloc( sizeof(struct obstacles) );
	GenerateObstacles( obs, n_planes, nhat_planes, xyz_planes,
		n_cylinders, r_cylinders, H_cylinders, xyz_cylinders, YPR_cylinders,
		n_cuboids, xyz_cuboids, LWH_cuboids, YPR_cuboids, i_grip_obs );

	/* Check feasibility of all waypoints */
	double* q_test = (double*) malloc( n*sizeof(double) );
	for (int i = 0; i < n_waypoints; i++) {
		printf("%s%02d%s\n", "Checking feasibility of waypoint #", i+1,"...");
		for (int j = 0; j < n; j++) {
			q_test[j] = q_waypoints[i*n + j];
		}
		if ( ConstraintViolation( q_test, n, obs, G, DH, 0 ) != false) {
			fprintf(stderr, "\tWaypoint infeasible.  Please check that waypoint #%i\n\t and/or geometry values have been entered correctly...", i+1);
			exit(EXIT_FAILURE);
		}
	}

	/* Generate set of sample points */
	printf("Generating samples of the manipulator C-space...\n");
	double** Q = Make2DDoubleArray(max_iter, n);
	GenerateSamples( Q, sampling, n, max_iter, q_max, q_min, filename );

	/*+=========================================================+*/
	/*| Prepare to run the bi-directional RRT-Connect algorithm |*/
	/*+=========================================================+*/

	/* Initialize variables and tree data structures */
	int n_plans				= (n_waypoints-1);										// Number of motion plans to formulate
	int n_trees				= 2*(n_waypoints-1);									// Number of RRT tree structures to allocate
	struct tree *trees		= (struct tree*) malloc( n_trees*sizeof(struct tree) );	// Pointer to the array of trees
	struct tree **tree_ptrs = (struct tree**) malloc( n_trees*sizeof(struct tree*) );	// Pointers to the array of tree pointers (used during feedback)
	int n_cuboids_total		= obs->n_cuboids;										// Total number of static cuboid obstacles + the cuboidal object
	int obs_indicator		= 0;													// Indicates the types of obstacles to check during RRT construction (see BuildRRTs)
	int* feasible			= (int*) malloc( n_plans*sizeof(int) );					// Array of booleans used to track whether the planner determined a feasible solution
	int* iter				= (int*) malloc( n_plans*sizeof(int) );					// Number of iterations invested in plan #i; used to resume planning from where it was left off
	int* num_nodes			= (int*) malloc( n_trees*sizeof(int) );					// Array of tree cardinalities, i.e. the number of nodes currently stored in each tree
	int** node_star_index	= Make2DIntArray( n_plans, 2 );							// Indices for each plan pointing to the connecting nodes [0] in the forward tree and [1] in the reverse tree through which the current most optimal path passes
	fpos_t fpos;																	// File position pointer used to continue printing tree data where it was left off (for the pre-computed trees only)
	clock_t t_start, t_end;															// Timers for measuring the computation time and motion plan run times

	for (int j = 0; j < n_trees; j++) {
		trees[j].nodes			= Make2DDoubleArray(1, n);
		trees[j].costs			= (double*)				malloc((1)*sizeof(double));
		trees[j].parents		= (int*)				malloc((1)*sizeof(int));
		trees[j].connections	= (int*)				malloc((1)*sizeof(int));
		trees[j].leaf_lists		= (struct list_node**)	malloc((1)*sizeof(struct list_node*));
		trees[j].safety			= (int*)				malloc((1)*sizeof(int));
		trees[j].indices		= (int*)				malloc((1)*sizeof(int));
		trees[j].kd_tree		= kd_create(n);
		tree_ptrs[j]			= &(trees[j]);
	}

	/* Compute eta_RRT and gamma_RRT, used for kd_tree Nearest-Neighbor search */
	double* Cdims		= (double*) malloc( n*sizeof(double) );			// C-space dimensions
	VectorDiff( q_max, q_min, Cdims, n);
	double eta_RRT		= Norm( Cdims, n, 2.0 );						// Maximum radius possible between any two samples produced by Steer() and stored in the trees
	double V_Cfree		= 1;											// Overestimate of the Lebesgue measure of C_free (Lebesgue measure of C)
	for (int i = 0; i < n; i++) {
		V_Cfree = V_Cfree * fabs(Cdims[i]);
	}
	double V_Bn			= pow(1.0/n, n/2.0);							// Underestimate of the Lebesgue measure of a normed ball in R^n (Lebesgue measure of inscribed hypercube)
	double gamma_RRT	= 2*pow( (1 + 1.0/n)*(V_Cfree/V_Bn), 1.0/n);	// Must choose gamma_RRT > 2*[ (1 + 1/n)*(mu(Cfree)/mu(V_Bn)) ]^(1/n), where mu is the Lebesgue measure

	/* Open MATLAB Engine and generate plots. */
	Engine* matlab = NULL;
	if ( (strncmp( &disp_matlab_plots, "y", 1) == 0) || strncmp( &disp_matlab_plots, "a", 1) == 0 ) {
		printf("Opening MATLAB Engine for generating plots...\n");
		matlab = engOpen("\0");
		if (matlab == NULL) {
			fprintf(stderr, "\tCan't start MATLAB engine...\n");
			goto GENERATE_RRTs;
		}
		engEvalString(matlab, "clear all; close all;	scrsz = get(0,'ScreenSize');	figdims = [0.5, 0.75];		fontsize = 12;		titlesize = 14;");
		assert( SendArraysToMATLAB( __LINE__, matlab, 7, "n", Int,0, 1,1,1, n,		"max_iter", Int,0, 1,1,1, max_iter,		"eta_RRT", Double,0, 1,1,1, eta_RRT,
			"gamma_RRT", Double,0, 1,1,1, gamma_RRT,	"q_min", Double,1, 1,1,n, q_min,	"q_max", Double,1, 1,1,n, q_max,	"Q", Double,2, 1,max_iter,n, Q ) == EXIT_SUCCESS );

		/* KD-Tree Search Radius plot */
		if (strncmp( NN_alg, "kd_tree", 7 ) == 0 ) {
			engEvalString(matlab, "k = [(1:1:min(max_iter,100)), (min(max_iter,100):10:ceil(1.5*max_iter))];		radius = gamma_RRT.*((log(k)./k).^(1/n));						\
				KDfig = figure('OuterPosition', [(0.75-figdims(1)/2)*scrsz(3), (1-figdims(2))*scrsz(4), figdims(1)*scrsz(3), figdims(2)*scrsz(4)]); 								\
				plot(k, radius, '--r', k, eta_RRT.*ones(size(k)), '--b', k, min(radius, eta_RRT), '-k', 'Linewidth', 1.5);		set(gca,'FontSize',fontsize);						\
				title('Nearest-Neighbor Search Radius as a Function of Tree Size', 'FontSize', titlesize); xlabel('Number of Nodes'); ylabel('Radius');								\
				xlim([floor(-0.05*max_iter), max_iter]);	legend('gamma_R_R_T [decays as dispersion]', 'eta_R_R_T [Max. set by Steer()]', 'Radius Used', 'Location', 'Best');"	);
		}

		/* Sample set plot */
		engEvalString(matlab, "iterations = (1:1:max_iter)';																														\
			SAMPLEfig = figure('OuterPosition', [(0.75-figdims(1)/2)*scrsz(3), (1-figdims(2))*scrsz(4), figdims(1)*scrsz(3), figdims(2)*scrsz(4)]);	set(gca,'FontSize',fontsize);	\
			plot( iterations, Q, '.' );			hold on;		ylim([min(q_min) - 10, max(q_max) + 10]);		V = axis;															\
			plot( linspace( V(1),V(2),2 ), min(q_min).*ones(1,2), '-k', linspace( V(1),V(2),2 ), max(q_max).*ones(1,2) , '-k', 'Linewidth', 2 );									\
			title( 'Joint Angle Sample Set, Q', 'FontSize', titlesize ); xlabel('Iteration'); ylabel('Joint Angle (deg)');															\
			q_legend = cellstr([repmat('q_', n, 1), num2str((1:1:n)')]); legend(q_legend, 'Location', 'EastOutside'); clear V Q eta_RRT gamma_RRT iterations;"	);
	}

	/*+==========================================================================+*/
	/*| Run the RRT Algorithm.  Pre-compute RRT's between each pair of waypoints |*/
	/*+==========================================================================+*/
	GENERATE_RRTs:

	Engine* matlabRRT = NULL;
	if ( strncmp( &disp_matlab_plots, "a", 1) == 0 ) {
		matlabRRT = matlab;
	}

	for (int i = 0; i < n_plans; i++) {
		printf("Constructing motion plan #%02d...\n", i+1);

		/* Set the first and last waypoints of the current motion plan as the roots of the forward tree and reverse tree, respectively. */
		for (int j = 0; j < n; j++) {
			trees[2*i+0].nodes[0][j]	= q_waypoints[n*(i)	  + j];
			trees[2*i+1].nodes[0][j]	= q_waypoints[n*(i+1) + j];
		}
		num_nodes[2*i+0]			= 1;			node_star_index[i][0]	= -1;			iter[i] = 1;
		num_nodes[2*i+1]			= 1;			node_star_index[i][1]	= -1;
		trees[2*i+0].costs[0]		= 0;			trees[2*i+0].parents[0] = -1;			trees[2*i+0].leaf_lists[0]	= NULL;
		trees[2*i+1].costs[0]		= 0;			trees[2*i+1].parents[0] = -1;			trees[2*i+1].leaf_lists[0]	= NULL;
		trees[2*i+0].connections[0] = -1;			trees[2*i+0].safety[0]	= 1;			trees[2*i+0].indices[0]		= 0;
		trees[2*i+1].connections[0] = -1;			trees[2*i+1].safety[0]	= 1;			trees[2*i+1].indices[0]		= 0;
		assert( kd_insert(trees[2*i+0].kd_tree, trees[2*i+0].nodes[0], const_cast<int*>(&(trees[2*i+0].indices[0])) ) == NULL );		// Returns NULL when no errors are encountered
		assert( kd_insert(trees[2*i+1].kd_tree, trees[2*i+1].nodes[0], const_cast<int*>(&(trees[2*i+1].indices[0])) ) == NULL );

		/* If considering the motion plan after the grasping maneuver, remove the battery from consideration as a cuboidal obstacle */
		if ( i == grip_actions[0] ) {
			obs->n_cuboids = obs->n_cuboids - 1;
		}

		/* Build the forward and backward RRT's */
		t_start	= clock();
		feasible[i] = BuildRRTs( &(trees[2*i+0]), &(trees[2*i+1]), &(num_nodes[2*i+0]), &(num_nodes[2*i+1]), n, w, &(iter[i]), max_iter, soln, sampling, Q, q_max, q_min,
			 NN_alg, max_neighbors, eta_RRT, gamma_RRT, epsilon, obs_indicator, obs, G, DH, node_star_index[i], load_previous, filename, i, &fpos, matlabRRT );
		t_end	= clock();

		printf("\tMotion plan generated in %5.5Lf s...\n", ElapsedTime( t_start, t_end )/1000 );
		if (feasible[i] != 1) {
			printf("\tNo open-loop path found for plan #%02d.  Terminating calculations...\n", i+1);
			break;
		}
	}

	/*+=======================================================================+*/
	/*| Prepare for feedback motion control (or else output results and exit) |*/
	/*+=======================================================================+*/
	BEGIN_MOTION_CONTROL:

	if ( grip_actions[0] == 0 || feasible[ grip_actions[0]-1 ] == 1 ) {
		obs->n_cuboids		= obs->n_cuboids + 1;		// Reset all cuboids, in case replanning is necessary
	}

	/* Generate MATLAB plots */
	if (matlab != NULL) {
		/* Open figure for plotting the current motion plan */
		assert( SendArraysToMATLAB( __LINE__, matlab, 5, "n", Int,0, 1,1,1, n,		"q_min", Double,1, 1,1,n, q_min,	"q_max", Double,1, 1,1,n, q_max,
			"n_waypoints", Int,0, 1,1,1, n_waypoints,	"q_waypoints", Double,1, 1,1,n*n_waypoints, q_waypoints )  == EXIT_SUCCESS );
		engEvalString(matlab,	"q_legend = cellstr([repmat('q_', n, 1), num2str((1:1:n)')]); 		fontsize = 12;		titlesize = 14;												\
			scrsz	= get(0,'ScreenSize');		q_waypoints = reshape(q_waypoints, n_waypoints, n)';			figdims = [0.4, 0.5];												\
			path	= double.empty(0,1);		pathlen = 0;		pathlen_old = 0;		path_index = 0;					set(gcf, 'Visible', 'off');								\
			waypt_format = 'ok';		waypt_size = 5;				path_format = '-o';		path_linewidth = 1.5;																	\
			PLANfig = figure('OuterPosition', [(0.2-figdims(1)/2)*scrsz(3), (0.55-figdims(2))*scrsz(4), figdims(1)*scrsz(3), figdims(2)*scrsz(4)]);	set(gca,'FontSize',fontsize);	\
			title('Current Joint Angle Motion Plan', 'FontSize', titlesize); xlabel('Command'); ylabel('Joint Angle (deg)'); ylim([min(q_min) - 10, max(q_max) + 10]); box on;"	);

		/* Open a figure in MATLAB for the end effector path plot */
		assert( SendArraysToMATLAB( __LINE__, matlab, 5,	"n", Int,0, 1,1,1, n,		"n_cylinders", Int,0, 1,1,1, obs->n_cylinders,
			"n_cuboids", Int,0, 1,1,1, obs->n_cuboids,		"n_planes", Int,0, 1,1,1, obs->n_planes,		 "N_coords", Int,1, 1,1,n, G->N_coords ) == EXIT_SUCCESS );
		engEvalString( matlab,
			"end_eff_path = double.empty(0,3);	end_eff_pathlen = 0;			end_eff_path_indices	= 0;	figdims = [0.5, 0.75];												\
			obs_color = [192 192 192]./256;		obs_alpha = 0.6;				az_el = [24,32];				fontsize = 12;		titlesize = 14;									\
			link_alpha = 0.4;					link_color = [0 176 0]./256;	end_eff_traj_color = [0 0 255]./256;	end_eff_trav_color = [0 139 188]./256;						\
			coord_format = '.';					coord_color = [0 0 1];			end_eff_format = '-';					end_eff_linewidth = 3;										\
			TRAJfig = figure('OuterPosition', [(0.75-figdims(1)/2)*scrsz(3), (1-figdims(2))*scrsz(4), figdims(1)*scrsz(3), figdims(2)*scrsz(4)]); set(gca,'FontSize',fontsize);		\
			title('End Effector Trajectory for Current Path', 'FontSize', titlesize); xlabel('x'); ylabel('y'); zlabel('z', 'Rotation', 0);											\
			set(gca,'ydir','reverse','zdir','reverse');				box on;			grid on;		axis equal;			view(az_el);		hold on;"			);

		/* Plot boundary cylinders (part of static obstacle environment) */
		if ( obs->n_cylinders > 0 ) {
			assert( SendArraysToMATLAB( __LINE__, matlab, 3,	"r_cylinders", Double,1, 1,1,obs->n_cylinders, obs->cylinders->r,
				"H_cylinders", Double,1, 1,1,obs->n_cylinders, obs->cylinders->H,		"Tinv", Double,3, obs->n_cylinders,4,4, obs->cylinders->Tinv	)  == EXIT_SUCCESS );
			engEvalString( matlab, "figure(TRAJfig);																								\
				for j = 1:n_cylinders;																												\
					[x,y,z] = cylinder( [r_cylinders(j),r_cylinders(j)], 30 );								z = H_cylinders(j).*(z - 0.5);			\
					T(1:3,1:3,j) = Tinv(1:3,1:3,j)';		T(1:3,4,j) = -T(1:3,1:3,j)*Tinv(1:3,4,j);		T(4,:,j) = Tinv(4,:,j);					\
					x = x(:)'; y = y(:)'; z = z(:)';		M = T(:,:,j)*[x; y; z; ones(1,length(x))];												\
					x = reshape(M(1,:),2,numel(x)/2);		y = reshape(M(2,:),2,numel(y)/2);				z = reshape(M(3,:),2,numel(z)/2);		\
					surf(x,y,z,ones(size(z)), 'FaceAlpha', obs_alpha); colormap(obs_color);															\
					patch(x(1,:)', y(1,:)', z(1,:)', obs_color, 'FaceAlpha', obs_alpha);															\
					patch(x(2,:)', y(2,:)', z(2,:)', obs_color, 'FaceAlpha', obs_alpha);															\
				end; clear x y z M T Tinv r_cylinders H_cylinders;");
		}

		/* Plot boundary cuboids (including the object, which is deleted later when obs->n_cuboids is decremented) */
		if ( obs->n_cuboids > 0 ) {
			assert( SendArraysToMATLAB( __LINE__, matlab, 2, "LWH_cuboids", Double,1, 1,1,3*obs->n_cuboids, LWH_cuboids,
				"T", Double,3, obs->n_cuboids,4,4, obs->cuboids->T	) == EXIT_SUCCESS );
			engEvalString( matlab, "figure(TRAJfig);	object_handle = zeros(6,1);																	\
				for j = 1:n_cuboids;																												\
					L = LWH_cuboids(3*j-2);		W = LWH_cuboids(3*j-1);		H = LWH_cuboids(3*j);													\
					x = (L/2).*[ -1 -1 -1 -1,  1  1  1  1, -1  1  1 -1, -1  1  1 -1, -1 -1  1  1, -1 -1  1  1 ];									\
					y = (W/2).*[ -1 -1  1  1, -1 -1  1  1,  1  1  1  1, -1 -1 -1 -1, -1  1  1 -1, -1  1  1 -1 ];									\
					z = (H/2).*[ -1  1  1 -1, -1  1  1 -1, -1 -1  1  1, -1 -1  1  1,  1  1  1  1, -1 -1 -1 -1 ];									\
					M = T(:,:,j)*[x; y; z; ones(1,length(x))];																						\
					for s = 1:6;																													\
						object_handle(s) = patch( M(1,(4*s-3):4*s)', M(2,(4*s-3):4*s)', M(3,(4*s-3):4*s)', obs_color, 'FaceAlpha', obs_alpha);		\
					end;																															\
				end; clear x y z M T LWH_cuboids L W H;" );
		}
	}

	/* Initialize the manipulator arm and temperature sensors.  If no servo is attached, write the computed optimal paths to file and exit the main function. */
	int *pathA, *pathB, pathlenA, pathlenB, index = 0;
	char user_input[]	= "n";
	double **path;

	printf("Initializing the manipulator arm and temperature sensors...\n");
	(CPhidgetAdvancedServoHandle) servo = InitializeServos(n, channels, grip_channel, AccelThrottle, VelLimThrottle, grip_AccelThrottle, grip_VelLimThrottle);
	ifKit	= NULL;
	// (CPhidgetInterfaceKitHandle) ifKit	= InitializeTempSensors(n_tempsensors, sensor_channels, rate_tempsensors);
	if (servo == NULL) {
		printf("Printing pre-computed best paths to file and terminating code...\n");
		for (int i = 0; i < n_plans; i++) {
			if (feasible[i] != 1) break;
			if (i == grip_actions[0]) {
				obs->n_cuboids = obs->n_cuboids - 1;
			}

			pathA	= TracePath( &trees[2*i+0], node_star_index[i][0], 0, &pathlenA );
			pathB	= TracePath( &trees[2*i+1], node_star_index[i][1], 0, &pathlenB );
			path	= Make2DDoubleArray( pathlenA + pathlenB, n );
			StorePath( &trees[2*i+0], &trees[2*i+1], n, pathA, pathB, pathlenA, pathlenB, filename, i, path );
			PlotPathInMATLAB( i, index, pathlenA + pathlenB, n, path, matlab );
			PlotEndEffectorPathInMATLAB( n, epsilon, w, n_cuboids_total, obs, G, DH, pathlenA + pathlenB, path, matlab );
			Sleep(1500);

			index	= pathlenA + pathlenB;
		}
		PlotPathInMATLAB( -1, index, 1, n, &(path[index-1]), matlab );
		PlotEndEffectorPathInMATLAB( n, epsilon, w, n_cuboids_total, obs, G, DH, 1, &(path[index-1]), matlab );

		if (matlab != NULL) {
			printf("Close figures and quit MATLAB? (y/n + ENTER): ");	fflush(stdout);		scanf("%s", user_input);
			if ( strncmp( user_input, "y", 1 ) == 0 ) engEvalString(matlab, "close all; quit;");
		}
		printf("END\n");	Sleep(1500);		exit(1);
	}

	// Normalize temperature sensor normal vectors
	double tempvec[3], norm;
	for (int i = 0; i < n_tempsensors; i++) {
		tempvec[0] = n_x_tempsensors[i];					tempvec[1] = n_y_tempsensors[i];					tempvec[2] = n_z_tempsensors[i];
		norm = Norm(tempvec, 3, 2.0);
		n_x_tempsensors[i] = n_x_tempsensors[i]/norm;		n_y_tempsensors[i] = n_y_tempsensors[i]/norm;		n_z_tempsensors[i] = n_z_tempsensors[i]/norm;
	}

	/*+===============================================================================+*/
	/*| Initialize the manipulator arm.  Define feedback loop motion plan parameters. |*/
	/*+===============================================================================+*/

	/* Define variables used during motion plan execution. */
	printf("Beginning motion...\n");
	int unsafe = 0, new_temp = 0, num_replans, fakeobs_index = 0, violation, ready_flag;
	double pos_maxheat[6] = { 0.0 }, n_hat_tempobs[3] = { 0.0 }, q_grip, epsilon_sq = pow(epsilon,2.0);
	double* q				= (double*) malloc(n*sizeof(double));
	double* q_current		= (double*) malloc(n*sizeof(double));
	int **replan_indices	= Make2DIntArray(max_replans,3);
	int* temp				= (int*) malloc(n_tempsensors*sizeof(int));
	obs_indicator			= 2;						// For any new nodes during feedback, check both static and dynamic obstacles for constraint violations.
	max_t_horizon			= max_t_horizon*1000;		// Convert the horizon time to milliseconds
	max_t_rebuild			= max_t_rebuild*1000;		// Convert the maximum rebuild time into milliseconds
	FILE* file_ptr;										// File pointer for reading in temperature sensor normal vectors
	clock_t t_beginmotion, t_current;					// Timers for updating the trajectory after each fixed horizon time

	/* Assign the manipulator initial position */
	printf("Assigning initial configuration...\n");
	for (int k = 0; k < n; k++) CPhidgetAdvancedServo_setPosition(servo, channels[k], Deg2Command(q_waypoints[k]) );
	CPhidgetAdvancedServo_setPosition(servo, grip_channel, Deg2Command(grip_angles[0]));

	/* Engage arm actuators.  Wait for the arm to reach its initial configuration. */
	printf("Engaging arm actuators...\n");
	for (int i = 0; i < n; i++) CPhidgetAdvancedServo_setEngaged(servo, channels[i], 1);
	CPhidgetAdvancedServo_setEngaged(servo, grip_channel, 1);

	do {
		for (int k = 0; k < n; k++) {
			CPhidgetAdvancedServo_getPosition(servo, channels[k], &(q[k]));
			q[k] = Command2Deg(q[k]);
		}
		CPhidgetAdvancedServo_getPosition(servo, grip_channel, &q_grip );
	} while ( (DistSq(q, &(q_waypoints[0]), n, w) > epsilon_sq) || fabs(Command2Deg(q_grip) - grip_angles[0]) > epsilon );

	/* Open the temperature sensor file for reading (or, if using simulated sensors, tempsensor_file must equal "") */
	file_ptr = fopen( tempsensor_file, "r" );
	if (file_ptr == NULL && strlen(tempsensor_file) > 0) {
		printf("\tCould not read tempsensor file.  Exiting...");
		exit(1);
	}

	/*+========================================+*/
	/*| Execute the motion plan with feedback. |*/
	/*+========================================+*/

	/* While safe, the robot will traverse the shortest paths pre-computed from the RRT's.  Iterate through each pair of trees
	(one pair for each motion plan between waypoints).  Build RRT's while committed to initial segments and replan if necessary. */
	t_beginmotion = clock();

	for (int i = 0; i < n_plans; i++) {

		if ( matlabRRT != NULL ) {
			assert( SendArraysToMATLAB( __LINE__, matlabRRT, 1,		"I", Int,0, 1,1,1, i+1 ) == EXIT_SUCCESS );
			engEvalString( matlabRRT, "figure(RRTfig(I));" );
		}

		/* Command the gripper open-loop according to pre-determined manual settings */
		if (i == grip_actions[0]) {
			obs->n_cuboids = obs->n_cuboids - 1;		// Remove the grasped object from consideration in obstacle collisions

			printf("\tGrasping target object...\n");
			CPhidgetAdvancedServo_setPosition(servo, grip_channel, Deg2Command(grip_angles[1]));
			do {
				CPhidgetAdvancedServo_getPosition(servo, grip_channel, &q_grip );
			} while ( fabs( Command2Deg(q_grip) - grip_angles[1]) > epsilon );
		}
		else if (i == grip_actions[1]) {
			printf("\tDropping target object...\n");
			CPhidgetAdvancedServo_setPosition(servo, grip_channel, Deg2Command(grip_angles[2]));
			do {
				CPhidgetAdvancedServo_getPosition(servo, grip_channel, &q_grip );
			} while ( fabs( Command2Deg(q_grip) - grip_angles[2]) > epsilon );
		}

		/* For plan i, reconstruct the best path for the manipulator arm from the pre-computed trees.  If no path exists, exit the loop and shutdown. */
		printf("Building motion plan #%02d...\n", i+1);
		if (feasible[i] == 1) {
			pathA	= TracePath( &(trees[2*i]), node_star_index[i][0], 0, &pathlenA );
			pathB	= TracePath( &(trees[2*i+1]), node_star_index[i][1], 0, &pathlenB );
			path	= Make2DDoubleArray( pathlenA + pathlenB, n );
			StorePath( &(trees[2*i]), &(trees[2*i+1]), n, pathA, pathB, pathlenA, pathlenB, filename, i, path );
			PlotPathInMATLAB( i, index, pathlenA+pathlenB, n, path, matlab );
			PlotEndEffectorPathInMATLAB( n, epsilon, w, n_cuboids_total, obs, G, DH, pathlenA + pathlenB, path, matlab );
		}
		else {
			printf("%s%02d%s", "\tNo open-loop path found for plan #", i+1, ".  Terminating motion...\n");
			goto END_MOTION_PLAN;
		}

		/* Execute plan i, if it was found to be feasible.  Begin the timer. */
		printf("Executing motion plan #%02d...\n", i+1);
		index	= 0;
		t_start = clock();
		do {
			for (int k = 0; k < n; k++) {
				CPhidgetAdvancedServo_setPosition(servo, channels[k], Deg2Command(path[index][k]) );
			}

			/* While the current position is not equal (within epsilon) of the desired position, repeatedly examine
			the path and assign a new plan mid-course if a temperature obstacle is encountered */
			do {
				/* Check temperature sensors */
				violation	= 0;
				ready_flag	= 0;
				if ( obs->n_temp_zones < n_tempobs_max ) {
					if (file_ptr != NULL) {
						rewind(file_ptr);
						fscanf( file_ptr, "%i", &violation );
						printf("Checked for violation from tempsensor file.");
					} else {
						//t_current = clock();
						//if (ifKit != NULL || ( (fakeobs_index < n_fakeobs) && (ElapsedTime( t_beginmotion, t_current ) > 1000*t_obs_intro[fakeobs_index]) ) ) {
						//	for (int k = 0; k < n_tempsensors; k++) {
						//		if (ifKit == NULL) {
						//			/* The time must have exceeded the obstacle introduction time.  Introduce a fake disturbance. */
						//			if (k == (obs_sensor[fakeobs_index]-1)) {
						//				violation		= 1;
						//				pos_maxheat[0]	= pos_x_tempsensors[k];			n_hat_tempobs[0]	= n_x_tempsensors[k];
						//				pos_maxheat[1]	= pos_y_tempsensors[k];			n_hat_tempobs[1]	= n_y_tempsensors[k];
						//				pos_maxheat[2]	= pos_z_tempsensors[k];			n_hat_tempobs[2]	= n_z_tempsensors[k];
						//				fakeobs_index	+= 1;
						//				break;
						//			} else continue;
						//		} else {
						//			/* The Interface Kit is actually connected.  Measure the temperature reading from each sensor. */
						//			CPhidgetInterfaceKit_getSensorValue(ifKit, sensor_channels[k], &new_temp );
						//			if ((new_temp > max_temp) && (temp[k] < max_temp)) {
						//				violation		= 1;
						//				pos_maxheat[0]	= pos_x_tempsensors[k];			n_hat_tempobs[0]	= n_x_tempsensors[k];
						//				pos_maxheat[1]	= pos_y_tempsensors[k];			n_hat_tempobs[1]	= n_y_tempsensors[k];
						//				pos_maxheat[2]	= pos_z_tempsensors[k];			n_hat_tempobs[2]	= n_z_tempsensors[k];
						//				temp[k] = new_temp;
						//				break;
						//			}
						//			temp[k] = new_temp;
						//		}
							//}
					//	}
					}
				}

				if (violation == 1) {
					/* Stop moving and determine current configuration */
					printf("\tTemperature constraints violated.  Halting motion and constructing temperature obstacle...\n");
					for (int k = 0; k < n; k++) {
						CPhidgetAdvancedServo_getPosition(servo, channels[k], &(q_current[k]));
						q_current[k] = Command2Deg(q_current[k]);
						CPhidgetAdvancedServo_setPosition( servo, channels[k], Deg2Command(path[index][k]) );
					}
					do {
						for (int k = 0; k < n; k++) {
							CPhidgetAdvancedServo_getPosition(servo, channels[k], &(q[k]));
							q[k] = Command2Deg(q[k]);
						}
					} while (DistSq(q, path[index], n, w) > epsilon_sq);

					/* Allow time to run estimation algorithm to find location of maximum temperature and its normal vector */
					do {
						Sleep(1000*((DWORD) t_tempwait));
						if (file_ptr != NULL) {
							rewind(file_ptr);	fscanf( file_ptr, "%i%i", &violation, &ready_flag );
						} else ready_flag = 1;
					} while ( ready_flag == 0 );

					if (file_ptr != NULL) {
						fscanf( file_ptr, "%Lf%Lf%Lf%Lf%Lf%Lf", &(pos_maxheat[0]), &(pos_maxheat[1]), &(pos_maxheat[2]), &(pos_maxheat[3]), &(pos_maxheat[4]), &(pos_maxheat[5]) );
						for (int j = 0; j < 3; j++) n_hat_tempobs[j] = pos_maxheat[j+3] - pos_maxheat[j];
						norm = Norm( n_hat_tempobs, 3, 2.0 );
						for (int j = 0; j < 3; j++) n_hat_tempobs[j] = n_hat_tempobs[j]/norm;
					}

					/* Develop a representative temperature obstacle */
					unsafe	= 1;
					ConstructTempObstacle( sensor_link, pos_maxheat, r_tempobs, H_tempobs, offset_tempobs, n_hat_tempobs, beta_tempobs, DH, q_current, obs );

					/* Check temperature obstacles, and keep lists of violating nodes in each tree (set their "safety" properties to 1 or 0, depending) */
					TempObsViolation( &(tree_ptrs[2*i]), &(num_nodes[2*i]), n, obs, G, DH );
				}

				/* Unless unsafe = 1 and the arm is already stopped, commit motion to path[index][k]. Attempt to improve the current plan while executing
				motion by adding new branches to the tree (note new edges will be conflict-free for both the static and dynamic environment since obs_indicator = 2) */
				BuildRRTs( &(trees[2*i + (iter[i] % 2)]), &(trees[2*i + ((iter[i]+1) % 2)]), &(num_nodes[2*i + (iter[i] % 2)]), &(num_nodes[2*i + ((iter[i]+1) % 2)]),
					n, w, &(iter[i]), max_iter, soln, sampling, Q, q_max, q_min, NN_alg, max_neighbors, eta_RRT, gamma_RRT, epsilon, obs_indicator, obs, G, DH,
					node_star_index[i], 'n', filename, i, &fpos, matlabRRT );

				t_end = clock();
				if ( (ElapsedTime( t_start, t_end ) >= max_t_horizon) || (unsafe == 1) ) {
					/* If unsafe or the time has exceeded its allowable limit, stop moving and determine the current configuration */
					printf("\tUnsafe and/or exceeded allowed horizon time.  Formulating new plan...\n");
					for (int k = 0; k < n; k++) {
						CPhidgetAdvancedServo_getPosition(servo, channels[k], &(q[k]));
						q[k] = Command2Deg(q[k]);
					}

					/* Add q to the appropriate tree, as indicated by the transition index (pathlenA) at which pathA of "path" switches to pathB */
					SplitPathAtNode( path, q, w, pathA, pathB, pathlenA, index, &(tree_ptrs[2*i]), &(num_nodes[2*i]), n, NN_alg, matlabRRT );

					/* Find new candidate plans that avoid the use of unsafe nodes from q to q_waypoints[n*(i+1)] using trees[2*i] and trees[2*i+1]
							(should return same path if nothing new was found due to addition of q to appropriate tree (0 length edge) -> same as MPC )
							(num_replans <= max_replans -> some feasible paths may be neglected depending on size of max_replan_neighbors) */
					num_replans = RePlan( &(tree_ptrs[2*i]), &(num_nodes[2*i]), q, n, w, max_replans, max_replan_neighbors, eta_RRT, gamma_RRT, replan_indices, NN_alg );

					/* Find a safe path among the candidate plans, listed in (locally!) optimal shortest-path order in "replan_indices" */
					unsafe = FindSafePath( &(tree_ptrs[2*i]), replan_indices, num_replans, &pathlenA, &pathlenB, &pathA, &pathB, &path, q, epsilon, n, w, obs, G, DH, filename);

					/* If still unsafe, stop moving (if not already stopped) and do an exhaustive search to find a possible solution.  Return the first one found. */
					if (unsafe == 1) {
						for (int k = 0; k < n; k++) {
							CPhidgetAdvancedServo_setPosition(servo, channels[k], q[k]);
						}
						printf("\tNo cost-efficient plan found.  Conducting an exhaustive search for any feasible path...\n");
						unsafe = ExhaustiveRePlan( &(tree_ptrs[2*i]), &(num_nodes[2*i]), eta_RRT, gamma_RRT, NN_alg, &pathlenA, &pathlenB, &pathA, &pathB, &path, q, epsilon, n, w, obs, G, DH, filename);
					}

					/* Continue to build new paths if still unsafe.  Otherwise set the manipulator to track the first point on the new path (fall-through on switch is intentional). */
					switch (unsafe) {
						case 1:
							printf("\tNo safe path found.  Attempting to construct new paths...\n");
							t_start = clock();	t_end = clock();
							while ((unsafe == 1) && (ElapsedTime( t_start, t_end ) >= max_t_rebuild)) {
								BuildRRTs( &(trees[2*i + (iter[i] % 2)]), &(trees[2*i + ((iter[i]+1) % 2)]), &(num_nodes[2*i + (iter[i] % 2)]), &(num_nodes[2*i + ((iter[i]+1) % 2)]),
									n, w, &(iter[i]), max_iter, soln, sampling, Q, q_max, q_min, NN_alg, max_neighbors, eta_RRT, gamma_RRT, epsilon, obs_indicator, obs, G, DH,
									node_star_index[i], 'n', filename, i, &fpos, matlabRRT );

								if (ElapsedTime( t_end, clock() ) >= max_t_horizon) {
									SplitPathAtNode( path, q, w, pathA, pathB, pathlenA, index, &(tree_ptrs[2*i]), &(num_nodes[2*i]), n, NN_alg, matlabRRT );
									unsafe = ExhaustiveRePlan( &(tree_ptrs[2*i]), &(num_nodes[2*i]), eta_RRT, gamma_RRT, NN_alg, &pathlenA, &pathlenB, &pathA, &pathB, &path, q, epsilon, n, w, obs, G, DH, filename);
								}
								t_end = clock();
							}
							if (unsafe == 1) {
								printf("\tCould not find safe path.  Aborting motion plan...\n");
								goto END_MOTION_PLAN;
							}

						case 0:
							PlotPathInMATLAB( i, index, pathlenA + pathlenB, n, path, matlab );
							PlotEndEffectorPathInMATLAB( n, epsilon, w, n_cuboids_total, obs, G, DH, pathlenA + pathlenB, path, matlab );
							t_start = clock();			// Reset the timer.
							index = 0;					// Reset the path index (path, pathlenA, and pathlenB have all been reset)
							for (int k = 0; k < n; k++) CPhidgetAdvancedServo_setPosition(servo, channels[k], Deg2Command(path[0][k]));
					};
				}

				/* Find the current configuration and its deviation from the planned position */
				for (int k = 0; k < n; k++) {
					CPhidgetAdvancedServo_getPosition(servo, channels[k], &(q[k]));
					q[k] = Command2Deg(q[k]);
				}
			} while ( DistSq(q, path[index], n, w) > epsilon_sq );

			/* Increment index to execute move to next node in the path */
			index = index + 1;
		} while ( index < (pathlenA + pathlenB) );
		Sleep(1000);

		/* If temperature obstacles were added and the current motion plan is complete,
		update the trees of the next motion plan to account for the temperature obstacles */
		if ( (obs->n_temp_zones > 0) && (i < n_plans - 1) ) {
			TempObsViolation( &(tree_ptrs[2*(i+1)]), &(num_nodes[2*(i+1)]), n, obs, G, DH );
			unsafe		= 1;		// Short-circuits next iteration to a replan cycle
		}

		/* Free memory associated with "path" before implementing next plan i+1 */
		memcpy( q, path[index-1], n*sizeof(double) );
		for (int k = pathlenA + pathlenB - 1; k >= 0; k--) free(path[k]);
		free(path); free(pathA); free(pathB);
	}

	/*+============================================================+*/
	/*| End the motion plan.  Prompt for re-run, or else shutdown. |*/
	/*+============================================================+*/
	END_MOTION_PLAN:

	if ( unsafe == 0 ) {
		/* Reset the gripper position at the end of a successful motion plan */
		printf("Resetting end effector...\n");
		CPhidgetAdvancedServo_setPosition(servo, grip_channel, Deg2Command(grip_angles[0]));
		Sleep(3000);

		/* Complete the joint angle plan and end effector path plots */
		if ( matlab != NULL ) {
			PlotPathInMATLAB( -1, index, 1, n, &(q), matlab );
			PlotEndEffectorPathInMATLAB( n, epsilon, w, n_cuboids_total, obs, G, DH, 1, &(q), matlab );
		}
	}

	/* Prompt to run again, if desired.  If "y", reset the simulation by resetting trees, replotting, and clearing variables. */
	printf("Run the simulation again? (y/n): "); fflush(stdout);
	scanf("%s", user_input);
	if (strncmp(user_input, "y", 1) == 0) {

		/* Reset simulation for a new run.  Return everything to their original state before motion plan execution except for the
		trees (which have grown to include more nodes). */
		ResetSimulation( trees, n_trees, n_plans, num_nodes, feasible, node_star_index, obs, matlab );

		goto BEGIN_MOTION_CONTROL;
	}

	// Disengage the actuators and shutdown
	printf("Disengaging actuators and shutting down...\n");
	for (int i = 0; i < n; i++) CPhidgetAdvancedServo_setEngaged(servo, channels[i], 0);
	CPhidgetAdvancedServo_setEngaged(servo, grip_channel, 0);
	Sleep(1000);

	CPhidget_close((CPhidgetHandle) servo);
	CPhidget_delete((CPhidgetHandle) servo);
	CPhidget_close((CPhidgetHandle) ifKit);
	CPhidget_delete((CPhidgetHandle) ifKit);

	if (matlab != NULL) {
		printf("Close figures and quit MATLAB? (y/n + ENTER): "); fflush(stdout);
		scanf("%s", user_input);
		if ( strncmp( user_input, "y", 1 ) == 0 ) engEvalString(matlab, "close all; quit;");
		engClose(matlab);
	}

	/*+============================================+*/
	/*| Clean-up memory and terminate the program. |*/
	/*+============================================+*/

	/* Free memory from heaps */
	printf("Freeing memory...\n");
	for (int i = n-1; i >= 0; i--) {
		for (int j = 2; j >= 0; j--) free(G->Body_coords[i][j]);
	}
	free(G->Body_coords); free(G->N_coords); free(G); free(DH);

	free(obs->planes->a); free(obs->planes->b); free(obs->planes->c); free(obs->planes->d);
	for (int i = obs->n_cylinders-1; i >= 0; i--) {
		for (int j = 3; j >= 0; j--) free(obs->cylinders->Tinv[i][j]);
		free(obs->cylinders->Tinv[i]);
	}
	free(obs->cylinders->r); free(obs->cylinders->H); free(obs->cylinders->Tinv);
	free(obs->cuboids->a); free(obs->cuboids->b); free(obs->cuboids->c); free(obs->cuboids->d);
	if (obs->n_temp_zones >= 1) {
		for (int i = obs->n_temp_zones - 1; i >= 0; i--) {
			for (int j = 3; j >= 0; j--) free(obs->temp_zones->Tinv[i][j]);
			free(obs->temp_zones->Tinv[i]);
		}
		free(obs->temp_zones->beta); free(obs->temp_zones->h1); free(obs->temp_zones->h2); free(obs->temp_zones->Tinv);
	}
	free(obs);

	for (int i = max_iter-1; i >= 0; i--) free(Q[i]);
	free(Q);

	for (int j = n_trees-1; j >= 0; j--) {
		for (int i = num_nodes[j]-1; i >= 0; i--) {
			free(trees[j].nodes[i]);
		}
		free(trees[j].costs);
		free(trees[j].parents);
		free(trees[j].connections);
		for (int k = num_nodes[j]-1; k >= 0; k--) {
			FreeList(trees[j].leaf_lists[k]);
		}
		kd_free(trees[j].kd_tree);
	}
	free(trees);
	free(feasible); free(num_nodes); free(q_test);

	if ( file_ptr != NULL ) {
		fclose( file_ptr );
	}
	free(q); free(q_current); free(temp); free(Cdims);
	for (int j = max_replans-1; j >= 0; j--) free(replan_indices[j]);
	free(replan_indices);

	for (int j = n_plans-1; j >= 0; j--) free(node_star_index[j]);
	free(node_star_index);

	printf("END\n");
	return 0;
}

