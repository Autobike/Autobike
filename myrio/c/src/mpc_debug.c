#include "pre_mpc_trajectory.c"

#define TRAJ_SIZE 10

/**
 * @brief Debug pre_mpc_trajectory.c
 *
 * @todo Use proper values from an old test log or simulation
 *
 * @author Lucas Haglund
 */
int main(void)
{
    int traj_size = TRAJ_SIZE;
    double traj[TRAJ_SIZE] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double X_est = 0.0;
    double Y_est = 0.0;
    double Psi_est = 0.0;
    int Ns = 4;
    MpcParams mpc_params = {{1.0, 0.024, 0.0, 1.0},
                            {0.0111, 0.0219},
                            3,
                            {0.00001, 0.0, 0.0, 0.00001},
                            {1.0, 0.0, 0.0, 100.0},
                            1.0,
                            {100.0, 100.0},
                            0.3491,
                            {.verbose = 2, .maxit = 4},
                            true};
    long qp_exitcode = 0;
    double roll_ref[TRAJ_SIZE] = {0.0};
    int closestpoint_idx_out = 0;
    double e1_out = 0.0;
    double e2_out = 0.0;
    double reset = 0.0;
    int abort = 0;

    for (int i = 0; i < TRAJ_SIZE; i++)
    {

        pre_mpc_trajectory(traj, &traj_size, X_est, Y_est, Psi_est, Ns, &mpc_params, &qp_exitcode, &roll_ref[i],
                           &closestpoint_idx_out, &e1_out, &e2_out, reset, abort);
    }

    printf("DEBUG: Trajectory MPC:\nu = [");
    for (int i = 0; i < TRAJ_SIZE; i++)
    {
        printf("%.2f%c ", roll_ref[i], (i == TRAJ_SIZE - 1) ? ']' : ',');
    }
    printf("\n");
}