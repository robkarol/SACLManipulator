#include "collision_detection.h"
#include "utils.h"
#include "manipulator_geometry.h"
#include "obstacles.h"
#include "tree_data_structure.h"
#include "linked_list.h"
#include <iostream>

using namespace std;

int ConstraintViolation( double* q_new, int n, struct obstacles* obs, struct geom *G, struct DHparams *DH, int obs_indicator ) {
	
	/* Create variables for OBB coordinate storage (8 points per OBB, for a total of "n" links) */
	int n_points = SumInts(G->N_coords, n+2);
	struct coords C;
	C.x = (double*) malloc(n_points*sizeof(double));
	C.y = (double*) malloc(n_points*sizeof(double));
	C.z = (double*) malloc(n_points*sizeof(double));

	/* Find the world coordinates of the robot Oriented Bounding Boxes */
	WorldCoords( G, DH, q_new, n, &C );

	/* Decrement n_points so that the last point (representative point of end effector position, "grip_pos") is not included in collision detection */
	n_points -= 1;

	//FILE *datafile = fopen( "C:/Users/Joe/Desktop/testcoords.dat", "w+");
	//fprintf(datafile, "%s\t%s\t%s\n", "x", "y", "z");
	//for (int k = 0; k < n_points; k++) {
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
					cout << "Collision with cylinder " << i << endl;
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

void TempObsViolation(struct tree** T, int* num_nodes, int n, struct obstacles* obs, struct geom *G, struct DHparams *DH ){
	
	/* Create variables for OBB coordinate storage (8 points per OBB, for a total of "n" links) */
	int n_points = SumInts(G->N_coords, n+2);
	struct coords C;
	C.x = (double*) malloc(n_points*sizeof(double));
	C.y = (double*) malloc(n_points*sizeof(double));
	C.z = (double*) malloc(n_points*sizeof(double));
	
	/* Decrement n_points so that the last point (representative point of end effector position, "grip_pos") is not included in collision detection */
	n_points -= 1;

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
			if (T[t]->safety[0] == 1) {
				cout << "Goal node is unsafe!!!" << endl;
			}
			T[t]->safety[0] = 1;
		}
	}

	for (int i = 3; i >= 0; i--) {
		free(v[i]); free(v_new[i]);
	}
	free(v); free(v_new);
	free(C.x); free(C.y); free(C.z);
}
