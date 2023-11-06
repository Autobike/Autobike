#include "../submodules/qpSWIFT/include/qpSWIFT.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
    double A[4];            // discretized, row-major order
    double B[2];            // discretized, row-major order
    uint32_t N;             // prediction horizon
    double Q[4];            // state penalty, row-major order
    double Pf[4];           // final prediction penalty, row-major order
    double R;               // control action penalty
    double state_bounds[2]; // |e1| <= state_bounds[0], |e2| <= state_bounds[1]
    double input_bound;     // |u| <= input_bound
    settings qp_options;    // qp solver options
    bool debug;             // print matrices etc
} MpcParams;

void y_Ax(double *A, double *x, double *y, unsigned int A_rows, unsigned int A_cols);
void kron_eye(double *M, size_t M_rows, size_t M_cols, size_t n, double *res);
void horzcat(double *M1, size_t M1_cols, double *M2, size_t M2_cols, size_t rows, double *res);
void blkdiag(double *M1, size_t M1_rows, size_t M1_cols, double *M2, size_t M2_rows, size_t M2_cols, double *res);
void messy_matrix_function(double *res, size_t res_rows, size_t res_cols, double *M, size_t M_rows, size_t M_cols);
void print_matrix(double *M, size_t rows, size_t cols);

extern int trajectory_mpc(double e1, double e2, MpcParams mpc_params, double *u);
