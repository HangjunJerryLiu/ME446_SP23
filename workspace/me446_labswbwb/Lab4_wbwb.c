#include "math.h"
#include "F28335Serial.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.432144;     //-0.37;
float offset_Enc3_rad = 0.208741;     //0.27;


// Your global varialbes.

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;
#pragma DATA_SECTION(var2, ".my_vars")
float var2 = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

long arrayindex = 0;
int UARTprint = 0;

// measured motor angles, CRS robot arm has 3 motors for 3 joints
float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

// D-H angles for each joint (radians)
float theta1DH = 0;
float theta2DH = 0;
float theta3DH = 0;

// D-H angles for each joint (degrees)
float theta1inD = 0;
float theta2inD = 0;
float theta3inD = 0;          // angles in degrees

// D-H angles calculated using inverse kinematics
float t1DH_IK = 0.0;
float t2DH_IK = 0.0;
float t3DH_IK = 0.0;

// motor angles calculated using IK
float t1m_IK = 0.0;
float t2m_IK = 0.0;
float t3m_IK = 0.0;

// variables to find IK
float Lp = 0.0;
float alpha = 0.0;
float beta = 0.0;
float gamma = 0.0;

// task space position of end-effector
float x = 0.0;
float y = 0.0;
float z = 0.0;

float dist = 0;

// initial positions for line following code
float x_a = 0.31;
float y_a = 0.10;
float z_a = 0.18;

// final position for line following code
float x_b = 0;
float y_b = 0;
float z_b = 0;

// distance between initial positions (x_a, y_a, z_a) and final positions (x_b, y_b, z_b)
float dx = 0.07;
float dy = 0.1;
float dz = 0.0; //part 7

// variables used for desired speed calculations for straight line following
float t_start = 0.0;
float t_total = 0.61; //v = 0.2m/s
float speed = 0.1;   // desired speed of straight line following

// previous values of angular velocities used for the velocity filter
float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;

float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;

// desired motor angles
float theta1_ref = 0.0;
float theta2_ref = 0.0;
float theta3_ref = 0.0;

// error of theta_dot trajectory
float edot1 = 0.0;
float edot2 = 0.0;
float edot3 = 0.0;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

// kp and kd gains in xyz direction used in lab4
float kpx = 500; //60;
float kpy = 500;
float kpz = 500;    //80,   kpx gains start from 500
float kdx = 20;   //2;
float kdy = 20;   //2
float kdz = 20;   //2       kdx gains start from 500

// kp and kd gains in xyz direction in the new frame used in lab4
float kpxn = 2000; //60;   2000,      = 100 to make it weak in one direction
float kpyn= 2000;    // 2000,         = 15 to make it weak in one direction
float kpzn = 2000;    //2000,   kpx gains start from 500
float kdxn = 35;   //35;
float kdyn = 35;   //35;
float kdzn = 35;   //35;       kdx gains start from 500

// KI gains
float kI1 = 500;
float kI2 = 500;
float kI3 = 500;

// error terms for the joint angles
float e1 = 0;
float e2 = 0;
float e3 = 0;
float e1_old = 0;
float e2_old = 0;
float e3_old = 0;

int state = 1;    // state variable

// variables for the friction compensation
// these coefficients will be used to calculate control effort for friction compensation
float minimum_vel1 = 0.1;
float minimum_vel23 = 0.05;
float slope2 = 3.0;
float slope_between = 3.0;    // slope 3
// viscous friction coefficient for joint 1
float Viscous_p1 = 0.1;   //0.2513;
float Viscous_n1 = 0.1;   //0.2477;
// coulombic friction coefficient for joint 1
float Coulomb_p1 = 0.3637;
float Coulomb_n1 = -0.2948;

// viscous friction coefficient for joint 2
float Viscous_p2 = 0.25;
float Viscous_n2 = 0.287;
// coulombic friction coefficient for joint 2
float Coulomb_p2 = 0.4;  //0.4759;
float Coulomb_n2 = -0.5031;

// viscous friction coefficient for joint 3
float Viscous_p3 = 0.12;   //0.1922;
float Viscous_n3 = 0.2;   //0.2132;
// coulombic friction coefficient for joint 2
float Coulomb_p3 = 0.25;   //0.5339;
float Coulomb_n3 = -0.3;   // -0.519;    // for friction compensation
// control effort for friction compensation
float u_fric1 = 0.0;
float u_fric2 = 0.0;
float u_fric3 = 0.0;

// variables for step desired coordinates
float mystep = 0.0;
float step2 = 0.0;
float step3 = 0.0;   // steps for Part3.2

// desired joint angles and their time derivatives
float qd1 = 0.0;
float qd2 = 0.0;
float qd3 = 0.0;       // theta desired for part 3.2
float qd1_d = 0.0;
float qd2_d = 0.0;
float qd3_d = 0.0;     // theta desired dot
float qd1_dd = 0.0;
float qd2_dd = 0.0;
float qd3_dd = 0.0;     // theta desired double dot

// initialize values for angle calculations for Jacobian transpose. They will take the motor angle from each joint. 
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;

// initialize values for Jacobian transpose entries (3x3), this will be used for task space/impedence control calculations
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;   

// initialize values for the rotation matrix
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;

// initialize the rotation values(radians) for x, y, and z axis
float thetaz = 0.85;   // 45 degrees, 0.85 radians
float thetax = 0;
float thetay = 0;   // angles in new frames after rotation

// initialize rotation matrix entries Rwn (3X3), from world frame to new frame
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;   

// initialize rotation matrix transpose entries Rnw (3X3), from new frame to world frame 
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;  

// initialize entries for matrix JT*Rwn (3X3)
float JTR11 = 0;
float JTR12 = 0;
float JTR13 = 0;
float JTR21 = 0;
float JTR22 = 0;
float JTR23 = 0;
float JTR31 = 0;
float JTR32 = 0;
float JTR33 = 0;

// the terms for matrix Kp*RT (3X3)
float KPRT11 = 0;
float KPRT12 = 0;
float KPRT13 = 0;
float KPRT21 = 0;
float KPRT22 = 0;
float KPRT23 = 0;
float KPRT31 = 0;
float KPRT32 = 0;
float KPRT33 = 0;
// initialize entries for matrix Kd*RT (3X3)
float KDRT11 = 0;
float KDRT12 = 0;
float KDRT13 = 0;
float KDRT21 = 0;
float KDRT22 = 0;
float KDRT23 = 0;
float KDRT31 = 0;
float KDRT32 = 0;
float KDRT33 = 0;

// desired xyz coordinates and velocities
float xd = 0;
float yd = 0;
float zd = 0;
float xd_dot = 0;
float yd_dot = 0;
float zd_dot = 0;

// actual xyz velocities, calculated from IK
float x_dot = 0;
float y_dot = 0;
float z_dot = 0;
// past values of xyz coordinates. This will be used for the 3-point average filter for velocity measurements.
float x_old = 0;
float y_old = 0;
float z_old = 0;

// past values of xyz velocities. This will be used for the 3-point average filter for velocity measurements.
float xdot_old1 = 0;
float xdot_old2 = 0;
float ydot_old1 = 0;
float ydot_old2 = 0;
float zdot_old1 = 0;
float zdot_old2 = 0;

float ff = 0.6;     // 0.9 // factor multiplied to the friction compensation
float fzcmd = 0;    // force value in z direction
float kt = 6.0;     // torque constant

// initialize forces in N frame. This will be used for the task space/impedence controller
float Fxn = 0;
float Fyn = 0;
float Fzn = 0;

int change_control = 0;   // int variable to change control law

// discrete filter from matlab
typedef struct steptraj_s {
    long double b[4];
    long double a[4];
    long double xk[4];
    long double yk[4];
    float qd_old;
    float qddot_old;
    int size;
} steptraj_t;

steptraj_t trajectory = {2.3633609011755938e-06L,7.0900827035267814e-06L,7.0900827035267814e-06L,2.3633609011755938e-06L,
                         1.0000000000000000e+00L,-2.9200789343857920e+00L,2.8422869943478877e+00L,-9.2218915307488614e-01L,
                         0,0,0,0,
                         0,0,0,0,
                         0,
                         0,
                         4};
steptraj_t trajectory2 = {2.3633609011755938e-06L,7.0900827035267814e-06L,7.0900827035267814e-06L,2.3633609011755938e-06L,
                          1.0000000000000000e+00L,-2.9200789343857920e+00L,2.8422869943478877e+00L,-9.2218915307488614e-01L,
                          0,0,0,0,
                          0,0,0,0,
                          0,
                          0,
                          4};
steptraj_t trajectory3 = {2.3633609011755938e-06L,7.0900827035267814e-06L,7.0900827035267814e-06L,2.3633609011755938e-06L,
                          1.0000000000000000e+00L,-2.9200789343857920e+00L,2.8422869943478877e+00L,-9.2218915307488614e-01L,
                          0,0,0,0,
                          0,0,0,0,
                          0,
                          0,
                          4};


void implement_discrete_tf(steptraj_t *traj, float step, float *qd, float *qd_dot, float *qd_ddot) {
    int i = 0;

    traj->xk[0] = step;
    traj->yk[0] = traj->b[0]*traj->xk[0];
    for (i = 1;i<traj->size;i++) {
        traj->yk[0] = traj->yk[0] + traj->b[i]*traj->xk[i] - traj->a[i]*traj->yk[i];
    }

    for (i = (traj->size-1);i>0;i--) {
        traj->xk[i] = traj->xk[i-1];
        traj->yk[i] = traj->yk[i-1];
    }

    *qd = traj->yk[0];
    *qd_dot = (*qd - traj->qd_old)*1000;  //0.001 sample period
    *qd_ddot = (*qd_dot - traj->qddot_old)*1000;

    traj->qd_old = *qd;
    traj->qddot_old = *qd_dot;
}

void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

    if(mycount%8000<4000)
    {
        mystep = 0.85;
    }else
    {
        mystep = 0.25;
    }                      // steps for joints 1, part 3.2

    if(mycount%8000<4000)
    {
        step2 = 0.85;
    }else
    {
        step2 = 0.25;
    }                      // steps for joints 2, part 3.2
    if(mycount%8000<4000)
    {
        step3 = -0.3;
    }else
    {
        step3 = 0.3;
    }                      // steps for joints 3, part 3.2

    implement_discrete_tf(&trajectory, mystep, &qd1, &qd1_d, &qd1_dd);
    implement_discrete_tf(&trajectory2, step2, &qd2, &qd2_d, &qd2_dd);
    implement_discrete_tf(&trajectory3, step3, &qd3, &qd3_d, &qd3_dd);     // implement steps part3.2


    // section below will calculate the required matrices for task space PD control and impedence control
    // Jacobian Transpose for lab4 part1
    // angles for Jacobian Transpose
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    // Calculate each entries for the Jacobian Transpose (3x3)
    JT_11 = -0.254*sinq1*(cosq3 + sinq2);
    JT_12 = 0.254*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 0.254*cosq1*(cosq2 - sinq3);
    JT_22 = 0.254*sinq1*(cosq2 - sinq3);
    JT_23 = -0.254*(cosq3 + sinq2);
    JT_31 = -0.254*cosq1*sinq3;
    JT_32 = -0.254*sinq1*sinq3;
    JT_33 = -0.254*cosq3;


    // Rotation angles for the rotation matrix
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    
    // Entries of rotation matirx zxy and its Transpose
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;

    // calculate each entry for matrix JT*Rwn
    JTR11 = JT_11*R11 + JT_12*R21 + JT_13*R31;
    JTR12 = JT_11*R12 + JT_12*R22 + JT_13*R32;
    JTR13 = JT_11*R13 + JT_12*R23 + JT_13*R33;
    JTR21 = JT_21*R11 + JT_22*R21 + JT_23*R31;
    JTR22 = JT_21*R12 + JT_22*R22 + JT_23*R32;
    JTR23 = JT_21*R13 + JT_22*R23 + JT_23*R33;
    JTR31 = JT_31*R11 + JT_32*R21 + JT_33*R31;
    JTR32 = JT_31*R12 + JT_32*R22 + JT_33*R32;
    JTR33 = JT_31*R13 + JT_32*R23 + JT_33*R33;

    // calculate each entry for matrix multiplication Kp*Rnw
    KPRT11 = kpxn*RT11;
    KPRT12 = kpxn*RT12;
    KPRT13 = kpxn*RT13;
    KPRT21 = kpyn*RT21;
    KPRT22 = kpyn*RT22;
    KPRT23 = kpyn*RT23;
    KPRT31 = kpzn*RT31;
    KPRT32 = kpzn*RT32;
    KPRT33 = kpzn*RT33;
    
    // calculate each entry for matrix multiplication Kp*Rwn
    KDRT11 = kdxn*RT11;
    KDRT12 = kdxn*RT12;
    KDRT13 = kdxn*RT13;
    KDRT21 = kdyn*RT21;
    KDRT22 = kdyn*RT22;
    KDRT23 = kdyn*RT23;
    KDRT31 = kdzn*RT31;
    KDRT32 = kdzn*RT32;
    KDRT33 = kdzn*RT33;

    // joint motor angles degrees
    theta1inD=theta1motor*180/PI;
    theta2inD=theta2motor*180/PI;
    theta3inD=theta3motor*180/PI;
    
    // motor angles in DH convention
    theta1DH = theta1motor;
    theta2DH = theta2motor - PI/2;
    theta3DH = -theta2motor + theta3motor + PI/2;

    // Omega (angular velocity) used for calculating friction compensation
    Omega1 = (theta1motor - Theta1_old)/0.001;          // calculate every 1ms
    Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;        // 3 point average
    Theta1_old = theta1motor;
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;       // update old values

    Omega2 = (theta2motor - Theta2_old)/0.001;          // calculate every 1ms
    Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;        // 3 point average
    Theta2_old = theta2motor;
    Omega2_old2 = Omega2_old1;
    Omega2_old1 = Omega2;       // update old values

    Omega3 = (theta3motor - Theta3_old)/0.001;          // calculate every 1ms
    Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;        // 3 point average
    Theta3_old = theta3motor;
    Omega3_old2 = Omega3_old1;
    Omega3_old1 = Omega3;       // update old values


    // find actual xyz coordinates of end effector using FK
    x = (127.0*cos(theta1DH)*(cos(theta2DH + theta3DH) + cos(theta2DH)))/500.0;
    y = (127.0*sin(theta1DH)*(cos(theta2DH + theta3DH) + cos(theta2DH)))/500.0;
    z = 127.0/500.0 - (127.0*sin(theta2DH))/500.0 - (127.0*sin(theta2DH + theta3DH))/500.0;
    //    xd = 0.254;
    //    yd = 0.254;
    //    zd = 0.254;    // assign desired xyz coordinates in meters

    // 3 point average filter for xyz velocities
    x_dot = (x - x_old)/0.001;
    x_dot = (x_dot + xdot_old1 + xdot_old2)/3.0;
    y_dot = (y - y_old)/0.001;
    y_dot = (y_dot + ydot_old1 + ydot_old2)/3.0;
    z_dot = (z - z_old)/0.001;
    z_dot = (z_dot + zdot_old1 + zdot_old2)/3.0;

    // update old values of the xyz coordinates and velocities
    x_old = x;
    y_old = y;
    z_old = z;
    xdot_old2 = xdot_old1;
    xdot_old1 = x_dot;
    ydot_old2 = ydot_old1;
    ydot_old1 = y_dot;
    zdot_old2 = zdot_old1;
    zdot_old1 = z_dot;

    // calculate friction compensation for each joint
    if (Omega1 > minimum_vel1) {
        u_fric1 = Viscous_p1*Omega1 + Coulomb_p1 ;
    } else if (Omega1 < -minimum_vel1) {
        u_fric1 = Viscous_n1*Omega1 + Coulomb_n1;
    } else {
        u_fric1 = slope_between*Omega1;
    }

    if (Omega2 > minimum_vel23) {
        u_fric2 = Viscous_p2*Omega2 + Coulomb_p2 ;
    } else if (Omega2 < -minimum_vel23) {
        u_fric2 = Viscous_n2*Omega2 + Coulomb_n2;
    } else {
        u_fric2 = slope2*Omega2;
    }

    if (Omega3 > minimum_vel23) {
        u_fric3 = Viscous_p3*Omega3 + Coulomb_p3 ;
    } else if (Omega3 < -minimum_vel23) {
        u_fric3 = Viscous_n3*Omega3 + Coulomb_n3;
    } else {
        u_fric3 = slope_between*Omega3;
    }


    // These are the task space/impedence controller for lab 4
    //    *tau1 = JT_11*(kpx*(xd - x) + kdx*(xd_dot - x_dot)) + JT_12*(kpy*(yd - y) + kdy*(yd_dot - y_dot)) + JT_13*(kpz*(zd - z) + kdz*(zd_dot - z_dot)) + ff*u_fric1;  // JT_13 = 0
    //    *tau2 = JT_21*(kpx*(xd - x) + kdx*(xd_dot - x_dot)) + JT_22*(kpy*(yd - y) + kdy*(yd_dot - y_dot)) + JT_23*(kpz*(zd - z) + kdz*(zd_dot - z_dot)) + ff*u_fric2 + JT_23*(fzcmd/kt);
    //    *tau3 = JT_31*(kpx*(xd - x) + kdx*(xd_dot - x_dot)) + JT_32*(kpy*(yd - y) + kdy*(yd_dot - y_dot)) + JT_33*(kpz*(zd - z) + kdz*(zd_dot - z_dot)) + ff*u_fric3 + JT_33*(fzcmd/kt);
    // calculate the control torques for each joint,     // calculation include friction compensation, force value in z direction, and torque constant,     // control laws used for parts 1 and 2 of lab4

    //    *tau1 = JTR11*(kpx*(xd - x) + kdx*(xd_dot - x_dot)) + JTR12*(kpy*(yd - y) + kdy*(yd_dot - y_dot)) + JTR13*(kpz*(zd - z) + kdz*(zd_dot - z_dot)) + u_fric1;  // JT_13 = 0
    //    *tau2 = JTR21*(kpx*(xd - x) + kdx*(xd_dot - x_dot)) + JTR22*(kpy*(yd - y) + kdy*(yd_dot - y_dot)) + JTR23*(kpz*(zd - z) + kdz*(zd_dot - z_dot)) + u_fric2;
    //    *tau3 = JTR31*(kpx*(xd - x) + kdx*(xd_dot - x_dot)) + JTR32*(kpy*(yd - y) + kdy*(yd_dot - y_dot)) + JTR33*(kpz*(zd - z) + kdz*(zd_dot - z_dot)) + u_fric3;
    // calculate the control torques for each joint,     // calculation include friction compensation, force value in z direction, and torque constant,     // control laws used for parts 1 and 2 of lab4

    // calculate force for part 3 (both kp and kd term), N frame with the given formular
    Fxn = KPRT11*(xd - x) + KPRT12*(yd - y) + KPRT13*(zd - z) + KDRT11*(xd_dot - x_dot) + KDRT12*(yd_dot - y_dot) + KDRT13*(zd_dot - z_dot);
    Fyn = KPRT21*(xd - x) + KPRT22*(yd - y) + KPRT23*(zd - z) + KDRT21*(xd_dot - x_dot) + KDRT22*(yd_dot - y_dot) + KDRT23*(zd_dot - z_dot);
    Fzn = KPRT31*(xd - x) + KPRT32*(yd - y) + KPRT33*(zd - z) + KDRT31*(xd_dot - x_dot) + KDRT32*(yd_dot - y_dot) + KDRT33*(zd_dot - z_dot);

    // state machine for straight line following
    float t = mycount*0.001;
    switch(state){

    /*
     State machine for calculating the desired velocity for straight line following:
     first calculate the desired xyz coordinates by calculating the time 't' in secs,
     then if required, swap the initial points (xa,ya,za)
     to (xb.yb.zb) and proceed with computing the component-wise
     differences dx, dy, dz.
     Based on the user-defined total speed(0.1 for this lab) tb_total, calculate
     the x,y,z velocities and then use the following formula to
     compute the next desired position:
        xd = dx*((t-t_start)/t_total)+x_a;
        yd = dy*((t-t_start)/t_total)+y_a;
        zd = dz*((t-t_start)/t_total)+z_a;
     If one of the ends of the straight line is reached,
     the point xa,ya,za and xb,yb,zb must be swapped.
     To that end, we track the value of an integer variable
     'state'. If state=1, we want to go from 'a' to 'b',
     otherwise if state=2, we want to go from 'b' to 'a'.
     This switching depends on the difference ((mycount*0.001)-t_start) > t_total)
     */

    case 1:
        xd = dx*((t-t_start)/t_total)+x_a;
        yd = dy*((t-t_start)/t_total)+y_a;
        zd = dz*((t-t_start)/t_total)+z_a;

        if(((mycount*0.001)-t_start) > t_total){
            x_b = 0.15;
            y_b = 0.30;
            z_b = 0.18;
            x_a = 0.38;
            y_a = 0.2;
            z_a = 0.18;

            dx = x_b - x_a;
            dy = y_b - y_a;
            dz = z_b - z_a;
            dist = sqrt(dx*dx+dy*dy+dz*dz);
            t_total = dist/speed;
            t_start = mycount*0.001;

            state = 2;
        }
        break;

    case 2:
        xd = dx*((t-t_start)/t_total)+x_a;
        yd = dy*((t-t_start)/t_total)+y_a;
        zd = dz*((t-t_start)/t_total)+z_a;

        if(((mycount*0.001)-t_start) > t_total){
            x_a = 0.15;
            y_a = 0.30;
            z_a = 0.18;
            x_b = 0.38;
            y_b = 0.2;
            z_b = 0.18;  //3 to 2


            dx = x_b - x_a;
            dy = y_b - y_a;
            dz = z_b - z_a;

            dist = sqrt(dx*dx+dy*dy+dz*dz);
            t_total = dist/speed;
            t_start = mycount*0.001;

            state = 1;
        }
        break;
    }

    // calculate the torque in task space using impedance control, used Fxn, Fyn, and Fzn values from the previous section
    *tau1 = JTR11*Fxn + JTR12*Fyn + JTR13 * Fzn;// + u_fric1;
    *tau2 = JTR21*Fxn + JTR22*Fyn + JTR23 * Fzn;// + u_fric2;
    *tau3 = JTR31*Fxn + JTR32*Fyn + JTR33 * Fzn;// + u_fric3;
    // tau1 is always feed forward, tau2 and tau3 changes based on the "change_control" variable

    // torque magnitude limit 5
    if (*tau1 > 5) {
        *tau1 = 5;
    } else if (*tau1 < -5) {
        *tau1 = -5;
    }
    if (*tau2 > 5) {
        *tau2 = 5;
    } else if (*tau2 < -5) {
        *tau2 = -5;
    }
    if (*tau3 > 5) {
        *tau3 = 5;
    } else if (*tau3 < -5) {
        *tau3 = -5;
    }


    if ((mycount%50)==0) {

        theta1array[arrayindex] = theta1motor;
        theta2array[arrayindex] = theta2motor;

        if (arrayindex >= 100) {
            arrayindex = 0;
        } else {
            arrayindex++;
        }

    }

    if ((mycount%500)==0) {
        UARTprint = 1;
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
        GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
    }

    printtheta1motor = theta1motor;
    printtheta2motor = theta2motor;
    printtheta3motor = theta3motor;

    Simulink_PlotVar1 = theta2motor;        //theta1motor;
    Simulink_PlotVar2 = theta2_ref;    //theta2motor;
    Simulink_PlotVar3 = theta3motor;     //theta3motor;
    Simulink_PlotVar4 = theta3_ref;


    mycount++;
}



void printing(void){
    //serial_printf(&SerialA, "%.2f %.2f, %.2f \n\r",printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI);
    serial_printf(&SerialA, "measured: %.2f %.2f, %.2f, x,y,z: %.2f %.2f, %.2f, DH from IK: %.2f %.2f, %.2f,  motor from IK: %.2f %.2f, %.2f \n\r",
                  printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI, x, y, z, t1DH_IK*180/PI,t2DH_IK*180/PI,t3DH_IK*180/PI, t1m_IK*180/PI,t2m_IK*180/PI,t3m_IK*180/PI);

}

