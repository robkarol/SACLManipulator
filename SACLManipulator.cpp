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

#include "utils.h"
#include "planning.h"
#include "manipulator_geometry.h"
#include "obstacles.h"
#include "collision_detection.h"
#include "kdtree.h"
#include "matlab_interface.h"
#include "phidget_interface.h"

#pragma warning (disable : 4996)
/*! Suppress compiler warnings about deprecated functions, e.g.\ `strcpy` (warning C4996) */
#define _CRT_SECURE_NO_DEPRECATE
/* If any of the following declarations are not already defined, define them for use here */
/*! Numerical approximation of \f$\pi\f$ */

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
	//char filename[FILENAME_MAX]	= "C:/Users/user/Desktop/SACL/RRT1";	/* Full filepath and filename root for reading/writing data, no extension (e.g. "C:/.../Fileroot") */
	char filename[FILENAME_MAX] = "C:/Users/tk001/Desktop/Stanford/Coding/Robotic Arm/data";
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
	char	tempsensor_file[]	= "C:/Users/tk001/Desktop/Stanford/Coding/Robotic Arm/interfacing.txt";	/* Enter full path to sensor estimation algorithm output file (or enter "" to use potentiometers as simulated sensors for debugging) */
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
		if ( (obs->n_temp_zones > 0) && (i <= n_plans - 1) ) {
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