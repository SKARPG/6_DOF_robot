#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_matrix.h"

// dimensions in mm
#define DELTA0  100.0f
#define DELTA1  80.0f
#define DELTA2  80.0f
#define DELTA3  63.0f
#define DELTA4  63.0f
#define DELTA5  26.0f
#define L0      220.0f
#define L1      210.0f

// zero position in mm
#define X_ZERO          0.0f
#define Y_ZERO          37.0f
#define Z_ZERO          593.0f

// zero orientation in radians
#define PHI_ZERO        90.0f
#define PSI_ZERO        0.0f
#define THETA_ZERO      0.0f

// acceptable position errors in mm
#define ACC_X_ERR       1.0f
#define ACC_Y_ERR       1.0f
#define ACC_Z_ERR       1.0f

// acceptable orientation errors in radians
#define ACC_PHI_ERR     deg2rad(1.0f)
#define ACC_PSI_ERR     deg2rad(1.0f)
#define ACC_THETA_ERR   deg2rad(1.0f)

// max number of iterations
#define MAX_ITER        1000


void calc_inv_kin(double* desired_pos, double* joint_pos);