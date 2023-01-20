#include <stdio.h>
#include <math.h>
#define NUM_DOF 3  // number of degrees of freedom

typedef struct {
  double theta[NUM_DOF];  // joint angles
  double d[NUM_DOF];      // link lengths
  double a[NUM_DOF];      // link offsets
  double alpha[NUM_DOF];  // link orientation
} Manipulator;


// computing a joint specific homogeneous transformation matrix
void joint_transform(double theta, double d, double a, double alpha, double* T) {

    // computing transformation matrix elements
    double c_theta = cos(theta);
    double s_theta = sin(theta);
    double c_alpha = cos(alpha);
    double s_alpha = sin(alpha);

    // filling in the transformation matrix
    T[0] = c_theta;
    T[1] = -s_theta*c_alpha;
    T[2] = s_theta*s_alpha;
    T[3] = a*c_theta;
    T[4] = c_alpha*s_theta;
    T[5] = -s_theta*s_alpha;
    T[6] = -a*s_theta;
    T[7] = s_alpha;
    T[8] = c_alpha*c_theta;
    T[9] = d*c_theta;
    T[10] = c_alpha*s_theta;
    T[11] = -s_theta*s_alpha;
    T[12] = -d*s_theta;
    T[13] = s_alpha;
    T[14] = c_alpha*c_theta;
    T[15] = d;
}

int main() {
    // defining a robot with Manipulator struct parameters
    Manipulator robot;

    // defining the joint parameters for the 3DOF manipulator
    double alpha[NUM_DOF] = {0.0, M_PI / 2.0, 0.0};
    double a[NUM_DOF] = {0.5, 0.0, 0.0};
    double d[NUM_DOF] = {1.0, 2.0, 1.5};
    double theta[NUM_DOF] = {0.0, M_PI / 2.0, -M_PI / 3.0};

    // computation of the transformation matrices for each joint
    double T1[16], T2[16], T3[16];
    joint_transform(theta[0], d[0], a[0], alpha[0], T1);
    joint_transform(theta[1], d[1], a[1], alpha[1], T2);
    joint_transform(theta[2], d[2], a[2], alpha[2], T3);

    // manipulator's origin to end effector homogenous transformation matrix is given as the product of three individual joint matrices T1*T2*T3

}
