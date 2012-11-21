#include "matlab_interface.h"
#include "tree_data_structure.h"
#include "manipulator_geometry.h"
#include "obstacles.h"
#include "planning.h"

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

void PlotRobotConfigInMATLAB( struct coords* C, int n_points, double opacity, Engine* matlab ) {
	
	if ( matlab != NULL ) {
		assert( SendArraysToMATLAB( __LINE__, matlab, 4,		"x", Double,1, 1,1,n_points, C->x,		"y", Double,1, 1,1,n_points, C->y,		
			"z", Double,1, 1,1,n_points, C->z,		"fade", Double,0, 1,1,1, opacity	)  == EXIT_SUCCESS );
		engEvalString( matlab, "figure(TRAJfig);	Cdata = plot3( x, y, z, coord_format, 'Color', (1-fade)*coord_color );							\
			traj_index = size(linkdata,2)+1;																										\
			for j = 1:(n+2);																														\
				f1_indices = sum(N_coords(1:(j-1)))+[1,2,4,3];		f4_indices = sum(N_coords(1:(j-1)))+[3,4,6,5];									\
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

void PlotTempObstaclesInMATLAB( struct obstacles* obs, Engine* matlab ) {

	if ( matlab != NULL ) {
		if ( obs->n_temp_zones > 0 ) {
			engEvalString( matlab, "if(exist('tempzone_handles','var') & ishandle(tempzone_handles) );	delete(tempzone_handles);	end;" );
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
	}
}

void PlotEndEffectorPathInMATLAB( int n, double epsilon, double* w, int n_cuboids_total, struct obstacles* obs, struct geom *G, struct DHparams *DH, 
	int pathlen_new, double** path_new, Engine* matlab ) {
	
	if (matlab == NULL) {
		return;
	}

	int plot_intermediate_configs = 0,		plot_coords = 1;
	double epsilon_sq		= pow( epsilon, 2.0 ),		*x_end_eff = NULL,		*y_end_eff = NULL,		*z_end_eff = NULL;
	double* q				= (double*)	malloc( n*sizeof(double) );
	int* end_eff_indices	= (int*) malloc( pathlen_new*sizeof(int) );

	/* Create variables for storage of link geometry coordinates (the last point is the representative end effector position, "grip_pos") */
	int n_points = SumInts(G->N_coords, n+2);
	struct coords C;
	C.x = (double*) malloc(n_points*sizeof(double));
	C.y = (double*) malloc(n_points*sizeof(double));
	C.z = (double*) malloc(n_points*sizeof(double));

	/* Delete old plots (if they exist). */
	engEvalString( matlab, "if(exist('plane_handles','var') & ishandle(plane_handles) );		delete(plane_handles);		end;	\
							if(exist('Cdata','var') && ishandle(Cdata) );						delete(Cdata);				end;	\
							if(exist('linkdata','var') & ishandle(linkdata) );					delete(linkdata);	 		end;	\
							if(exist('end_eff_traj', 'var') && ishandle(end_eff_traj) )			delete([end_eff_traj; end_eff_path_trav; end_eff_path_old_handle]); end;"	);
	if ( obs->n_cuboids < n_cuboids_total ) {
		engEvalString( matlab, "if(exist('object_handle', 'var')); delete(object_handle); clear(object_handle); end;" );
	}

	/* Plot temperature zones (part of dynamic obstacle environment - may have changed since last call to function) */
	PlotTempObstaclesInMATLAB( obs, matlab );

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
