#include "mpc_trajectory.h"
#include "trajectory_selector.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define g 9.81
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

int sign(double value);          // sign value for double
double min(double a, double b);  // Min function for double
double mod(double a, double b);  // Modulo function (remainder of a division)
double wrap_angle(double angle); // Wrap angle between pi and -pi

/**
 * @name Pre Trajectory MPC
 *
 * @brief Copied from trajectory_controller.c and modified to call mpc_trajectory.c
 *
 * @param[in] traj                  Trajectory reference [X_ref Y_ref Psi_ref]
 * @param[in] X_est                 X position estimated state from kalman filter
 * @param[in] Y_est                 Y position estimated state from kalman filter
 * @param[in] Psi_est               Heading estimated state from kalman filter
 * @param[in] Ns                    Number of points in local trajectory
 * @param[in] mpc_params            MPC parameters
 * @param[in] reset                 ?
 * @param[in] abort                 ?
 *
 * Output:
 * @param[out] qp_exitcode           [1]
 * @param[out] roll_ref              Roll reference fot the balance control
 * @param[out] closestpoint_idx_out  ?
 * @param[out] e1_out                ?
 * @param[out] e2_out                ?
 *
 * [1]:
 * 0: optimal solution found
 * 1: failure in solving LDL' factorization
 * 2: maximum number of iterations exceeded
 * 3: unknown problem in solver
 */
extern void pre_mpc_trajectory(double *traj, int32_t *traj_size, double X_est, double Y_est, double Psi_est, int Ns,
                               MpcParams *mpc_params, long *qp_exitcode, double *roll_ref,
                               int32_t *closestpoint_idx_out, double *e1_out, double *e2_out, double reset,
                               int32_t abort)
{
    double X_loc[Ns];
    double Y_loc[Ns];
    double Psi_loc[Ns];

    // Second point in traj is current selected closest point
    static int closestpoint_idx;
    if (reset == 0)
    {
        closestpoint_idx = 1;
        *closestpoint_idx_out = 0;
    }

    trajectory_selector(traj, traj_size, closestpoint_idx, Ns, reset, X_loc, Y_loc, Psi_loc, X_est, Y_est, Psi_est,
                        abort);
    closestpoint_idx = 0;

    // Search for closest point (find the closest point going forward, stop when distance increases)
    while (pow(X_loc[closestpoint_idx] - X_est, 2.0) + pow(Y_loc[closestpoint_idx] - Y_est, 2.0) >=
               pow(X_loc[closestpoint_idx + 1] - X_est, 2.0) + pow(Y_loc[closestpoint_idx + 1] - Y_est, 2.0) &&
           closestpoint_idx <= (*traj_size) - 3)
    {
        closestpoint_idx += 1;
    }

    // select same closest point for heading and position error
    int closestpoint_heading_idx = closestpoint_idx;

    // Compute X and Y distance from current location to selected closest point
    double dx, dy;
    dx = X_est - X_loc[closestpoint_idx];
    dy = Y_est - Y_loc[closestpoint_idx];

    // Interpolation algorithm (IMPROVEMENT)
    // Compute distance between point before closespoint and current location
    double dis_true_previous =
        sqrt(pow(X_loc[closestpoint_idx - 1] - X_est, 2.0) + pow(Y_loc[closestpoint_idx - 1] - Y_est, 2.0));
    // Angle of line between previous closestpoint and current location
    double alpha_star = atan((Y_loc[closestpoint_idx - 1] - Y_est) / (X_loc[closestpoint_idx - 1] - X_est));
    // heading of previous closestpoint - alpha_star
    double alpha_projected_dist = alpha_star - Psi_loc[closestpoint_idx];
    // Distance from previous closestpoint to the projection of the bike in the trajectory
    double projected_dist = fabs(dis_true_previous * cos(alpha_projected_dist));
    double compared_dist = sqrt(pow(X_loc[closestpoint_idx] - X_loc[closestpoint_idx - 1], 2.0) +
                                pow(Y_loc[closestpoint_idx] - Y_loc[closestpoint_idx - 1], 2.0));

    // When bike passses the closestpoint, heading from next point is taken
    if (projected_dist >= compared_dist)
    {
        closestpoint_heading_idx = closestpoint_idx + 1;
    }

    // Compute e1 and e2
    double e1 = dy * cos(Psi_loc[closestpoint_heading_idx]) - dx * sin(Psi_loc[closestpoint_heading_idx]);
    double e2 = Psi_est - Psi_loc[closestpoint_heading_idx];
    // Keep heading error between -pi and pi
    e2 = wrap_angle(e2);

    *e1_out = e1;
    *e2_out = e2;

    // All the above copied from trajectory_controller.C

    *qp_exitcode = trajectory_mpc(e1, e2, *mpc_params, roll_ref);
}

// sign value for double
int sign(double value)
{
    if (value > 0.0)
    {
        return 1;
    }
    else if (value < 0.0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

// Min function for double
double min(double a, double b)
{
    if (a < b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

// Modulo function (remainder of a division)
double mod(double a, double b)
{
    double quotient = a / b;
    double quotient_floor = floor(quotient);
    double remainder = a - quotient_floor * b;
    return remainder;
}

// Wrap angle between pi and -pi
double wrap_angle(double angle)
{
    double wrapped_angle = fmod(angle + M_PI, 2 * M_PI);
    if (wrapped_angle < 0)
    {
        wrapped_angle += 2 * M_PI;
    }
    return wrapped_angle - M_PI;
}