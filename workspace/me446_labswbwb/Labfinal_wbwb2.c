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
float dx = 0.14;
float dy = 0.29;
float dz = 0.44; //part 7

// variables used for desired speed calculations for straight line following
float t_start = 0.0;
float t_total = 0;
float speed = 0.6;   // desired speed of straight line following

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
// the variable to check which control method the robot is using,
// 1 for force control when pushing the egg, 2 for impedance control when robot arm is moving freely
float check = 0;

int state = 1;    // state variable

// variables for the friction compensation
float minimum_vel1 = 0.1;
float minimum_vel23 = 0.05;
float slope2 = 3.0;
float slope_between = 3.0;    // slope 3
float Viscous_p1 = 0.1;   //0.2513;
float Viscous_n1 = 0.1;   //0.2477;
float Coulomb_p1 = 0.3637;
float Coulomb_n1 = -0.2948;

float Viscous_p2 = 0.25;
float Viscous_n2 = 0.287;
float Coulomb_p2 = 0.4;  //0.4759;
float Coulomb_n2 = -0.5031;

float Viscous_p3 = 0.12;   //0.1922;
float Viscous_n3 = 0.2;   //0.2132;
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

// variables for lab4
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;    // Jacobian transpose entries
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0.85;   // 45 degrees, 0.85 radians
float thetax = 0;
float thetay = 0;   // angles in new frames after rotation
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;   // rotation matrix entries Rwn, from world frame to new frame
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;  // rotation matrix transpose entries Rnw, from new frame to world frame

// the terms for matrix JT*Rwn
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
// the terms for matrix Kd*RT (3X3)
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
// desired xyz velocities
float x_dot = 0;
float y_dot = 0;
float z_dot = 0;
// past values of xyz coordinates
float x_old = 0;
float y_old = 0;
float z_old = 0;
// past values of xyz velocities
float xdot_old1 = 0;
float xdot_old2 = 0;
float ydot_old1 = 0;
float ydot_old2 = 0;
float zdot_old1 = 0;
float zdot_old2 = 0;

float ff = 0.6;    // 0.9 // factor multiplied to the friction compensation
float fzcmd = -8.5;    // force value in z direction
float kt = 6.0;     // torque constant

// forces in N frame
float Fxn = 0;
float Fyn = 0;
float Fzn = 0;

float t3 = 0;

int change_control = 0;   // int variable to change control law

#define NUMPOINTS 4
typedef struct point_tag {
    float x;
    float y;
    float z;
    float theta_z;
    int mode;

}point;

#define XYZSTIFF 1
#define ZSTIFF   2
#define XZSTIFF  3
#define HOLE     4
#define XYSTIFF  5
#define EGG      6
#define EGGHOLE  7

//waypoints for the robot arm
point myarrayofpoints [] = {
                            {0.25,  0.0,   0.51,  0,   XYZSTIFF},              // start point
                            {0.19,  0.29,  0.37,  0,   XYZSTIFF},
                            {0.0309, 0.356, 0.22,  0,  ZSTIFF},         // above peg
                            {0.0309, 0.356, 0.130, 0,  HOLE},        // insert into peg xyz coordinates
                            {0.0309, 0.356, 0.22,  0,  XYZSTIFF},         // pull out from peg
                            {0.19,  0.29,  0.37,  0,   XYZSTIFF},                                           // index = 5

                            {0.192,  0.094,  0.325, 0,  XYZSTIFF},     // middle points to the start of slot
                            {0.3889,  0.0982, 0.210, 0, XYZSTIFF},     // start of way points for zig-zag
                            {0.3945, 0.0698, 0.210, 0,  XZSTIFF},
                            {0.4132, 0.0487, 0.210, 0,  XZSTIFF},
                            {0.4187, 0.0351, 0.210, 0,  XZSTIFF},                                    // index = 10

                            {0.4099, 0.0244,  0.210, 0,  XZSTIFF},       // end of first zig
                            {0.3429, 0.0361,  0.210, 0,  XZSTIFF},       // end of zag
                            {0.3333, 0.0262,  0.210, 0,  XZSTIFF},
                            {0.3346, 0.0142,  0.210, 0,  XZSTIFF},
                            {0.3886,-0.0558,  0.210, 0,  XYZSTIFF},       // way-points for the zig-zag, this point is at the exit point of the zig-zag       index = 15
                            {0.2489,-0.05897, 0.3414, 0, XYZSTIFF},
                            {0.2448, 0.19076, 0.39,   0, XZSTIFF},
                            {0.2448, 0.19076, 0.303,  0, EGG},          // point right above the egg
                            {0.2448, 0.19076, 0.2930,  0, XYSTIFF},
                            {0.2448, 0.19076, 0.2930,  0, XYSTIFF},    // INDEX 20     // keep pushing the egg for 2 seconds
                            {0.2448, 0.19076, 0.2930,  0, XYZSTIFF},
                            {0.2448, 0.19076, 0.2930,  0, XYZSTIFF}
};


//index
int startpoint = 0;






void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {

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


    // commands for straight line following
    float t = mycount*0.001;

    xd = dx*((t-t_start)/t_total)+x_a;
    yd = dy*((t-t_start)/t_total)+y_a;
    zd = dz*((t-t_start)/t_total)+z_a;

    // commands to go to way-points or return to starting point
    if(((mycount*0.001)-t_start) > t_total) {
        // commands to move arm back to the start point after all tasks are completed,
        // change index as the number of way-points increase
        if (startpoint == 22){
            startpoint = 22;
            x_b = 0.25;
            y_b = 0;
            z_b = 0.5;
            x_a = myarrayofpoints[startpoint].x;
            y_a = myarrayofpoints[startpoint].y;
            z_a = myarrayofpoints[startpoint].z;

            if ( sqrt((x_b - x)*(x_b - x) + (y_b - y)*(y_b - y) + (z_b - z)*(z_b - z)) < 0.1) {
                x_a = x_b;
                y_a = y_b;
                z_a = z_b;
            }

            dx = x_b - x_a;
            dy = y_b - y_a;
            dz = z_b - z_a;
            dist = sqrt(dx*dx+dy*dy+dz*dz);
            t_total = dist/speed;
            t_start = mycount*0.001;
        }

        // commands to let the peg stay in the hole for 1.5 seconds
        else if (myarrayofpoints[startpoint].mode == HOLE || myarrayofpoints[startpoint].mode == XYSTIFF) {
            t_total = 1.0;
            if(((mycount*0.001)-t_start) < t_total) {
                x_b = myarrayofpoints[startpoint].x;
                y_b = myarrayofpoints[startpoint].y;
                z_b = myarrayofpoints[startpoint].z;
                x_a = myarrayofpoints[startpoint].x;
                y_a = myarrayofpoints[startpoint].y;
                z_a = myarrayofpoints[startpoint].z;

                dx = x_b - x_a;
                dy = y_b - y_a;
                dz = z_b - z_a;
                t_start = mycount*0.001;
                thetaz = myarrayofpoints[startpoint+1].theta_z;
            }

            // after the peg has been in the hole for 2 seconds, pull the peg out
            else {
                x_b = myarrayofpoints[startpoint+1].x;
                y_b = myarrayofpoints[startpoint+1].y;
                z_b = myarrayofpoints[startpoint+1].z;
                x_a = myarrayofpoints[startpoint].x;
                y_a = myarrayofpoints[startpoint].y;
                z_a = myarrayofpoints[startpoint].z;
                startpoint ++;

                dx = x_b - x_a;
                dy = y_b - y_a;
                dz = z_b - z_a;
                dist = sqrt(dx*dx+dy*dy+dz*dz);
                t_total = dist/speed;
                t_start = mycount*0.001;
                thetaz = myarrayofpoints[startpoint+1].theta_z;
            }
        }

        // EGGHOLE is the mode for tuning control gains when peg is pressing the egg, user has 20 seconds.
        else if (myarrayofpoints[startpoint].mode == EGGHOLE) {
            t_total = 20.0;
            if(((mycount*0.001)-t_start) < t_total) {
                x_b = myarrayofpoints[startpoint].x;
                y_b = myarrayofpoints[startpoint].y;
                z_b = myarrayofpoints[startpoint].z;
                x_a = myarrayofpoints[startpoint].x;
                y_a = myarrayofpoints[startpoint].y;
                z_a = myarrayofpoints[startpoint].z;

                dx = x_b - x_a;
                dy = y_b - y_a;
                dz = z_b - z_a;
                t_start = mycount*0.001;
                thetaz = myarrayofpoints[startpoint+1].theta_z;
            }
            // move to next location after 20 seconds
            else {
                x_b = myarrayofpoints[startpoint+1].x;
                y_b = myarrayofpoints[startpoint+1].y;
                z_b = myarrayofpoints[startpoint+1].z;
                x_a = myarrayofpoints[startpoint].x;
                y_a = myarrayofpoints[startpoint].y;
                z_a = myarrayofpoints[startpoint].z;
                startpoint ++;

                dx = x_b - x_a;
                dy = y_b - y_a;
                dz = z_b - z_a;
                dist = sqrt(dx*dx+dy*dy+dz*dz);
                t_total = dist/speed;
                t_start = mycount*0.001;
                thetaz = myarrayofpoints[startpoint+1].theta_z;
            }
        }


        // commands to move to way-points,
        else {
            x_b = myarrayofpoints[startpoint+1].x;
            y_b = myarrayofpoints[startpoint+1].y;
            z_b = myarrayofpoints[startpoint+1].z;
            x_a = myarrayofpoints[startpoint].x;
            y_a = myarrayofpoints[startpoint].y;
            z_a = myarrayofpoints[startpoint].z;

            dx = x_b - x_a;
            dy = y_b - y_a;
            dz = z_b - z_a;
            dist = sqrt(dx*dx+dy*dy+dz*dz);

            // slow down when the peg is moving through the zig-zag
            if (myarrayofpoints[startpoint].mode == XZSTIFF) {
                speed = 0.4;
            }
            // slow down and reduce gains in the z direction when pressing the egg
            else if (myarrayofpoints[startpoint].mode == EGG) {
                speed = 0.1;
                kpzn = 400;
                kdzn = 15;
            }
            // return to normal speed and normal gains when moving the arm freely
            else {
                speed = 0.6;
                kpzn = 2000;
                kdzn = 35;
            }
            t_total = dist/speed;
            t_start = mycount*0.001;
            thetaz = myarrayofpoints[startpoint+1].theta_z;
            startpoint ++;
        }

    }            // bracket for the general if statement


    // Jacobian Transpose for lab4 part1
    cosq1 = cos(theta1motor);
    sinq1 = sin(theta1motor);
    cosq2 = cos(theta2motor);
    sinq2 = sin(theta2motor);
    cosq3 = cos(theta3motor);
    sinq3 = sin(theta3motor);
    JT_11 = -0.254*sinq1*(cosq3 + sinq2);
    JT_12 = 0.254*cosq1*(cosq3 + sinq2);
    JT_13 = 0;
    JT_21 = 0.254*cosq1*(cosq2 - sinq3);
    JT_22 = 0.254*sinq1*(cosq2 - sinq3);
    JT_23 = -0.254*(cosq3 + sinq2);
    JT_31 = -0.254*cosq1*sinq3;
    JT_32 = -0.254*sinq1*sinq3;
    JT_33 = -0.254*cosq3;

    // Entries of rotation matirx zxy and its Transpose
    cosz = cos(thetaz);
    sinz = sin(thetaz);
    cosx = cos(thetax);
    sinx = sin(thetax);
    cosy = cos(thetay);
    siny = sin(thetay);
    RT11 = R11 = cosz*cosy-sinz*sinx*siny;
    RT21 = R12 = -sinz*cosx;
    RT31 = R13 = cosz*siny+sinz*sinx*cosy;
    RT12 = R21 = sinz*cosy+cosz*sinx*siny;
    RT22 = R22 = cosz*cosx;
    RT32 = R23 = sinz*siny-cosz*sinx*cosy;
    RT13 = R31 = -cosx*siny;
    RT23 = R32 = sinx;
    RT33 = R33 = cosx*cosy;

    // calculate the terms for matrix JT*Rwn
    JTR11 = JT_11*R11 + JT_12*R21 + JT_13*R31;
    JTR12 = JT_11*R12 + JT_12*R22 + JT_13*R32;
    JTR13 = JT_11*R13 + JT_12*R23 + JT_13*R33;
    JTR21 = JT_21*R11 + JT_22*R21 + JT_23*R31;
    JTR22 = JT_21*R12 + JT_22*R22 + JT_23*R32;
    JTR23 = JT_21*R13 + JT_22*R23 + JT_23*R33;
    JTR31 = JT_31*R11 + JT_32*R21 + JT_33*R31;
    JTR32 = JT_31*R12 + JT_32*R22 + JT_33*R32;
    JTR33 = JT_31*R13 + JT_32*R23 + JT_33*R33;

    // terms for matrix multiplication Kp*Rnw
    KPRT11 = kpxn*RT11;
    KPRT12 = kpxn*RT12;
    KPRT13 = kpxn*RT13;
    KPRT21 = kpyn*RT21;
    KPRT22 = kpyn*RT22;
    KPRT23 = kpyn*RT23;
    KPRT31 = kpzn*RT31;
    KPRT32 = kpzn*RT32;
    KPRT33 = kpzn*RT33;
    // terms for matrix multiplication Kp*Rwn
    KDRT11 = kdxn*RT11;
    KDRT12 = kdxn*RT12;
    KDRT13 = kdxn*RT13;
    KDRT21 = kdyn*RT21;
    KDRT22 = kdyn*RT22;
    KDRT23 = kdyn*RT23;
    KDRT31 = kdzn*RT31;
    KDRT32 = kdzn*RT32;
    KDRT33 = kdzn*RT33;

    // calculate force for part 3 (both kp and kd term), N frame with the given formula
    Fxn = KPRT11*(xd - x) + KPRT12*(yd - y) + KPRT13*(zd - z) + KDRT11*(xd_dot - x_dot) + KDRT12*(yd_dot - y_dot) + KDRT13*(zd_dot - z_dot);
    Fyn = KPRT21*(xd - x) + KPRT22*(yd - y) + KPRT23*(zd - z) + KDRT21*(xd_dot - x_dot) + KDRT22*(yd_dot - y_dot) + KDRT23*(zd_dot - z_dot);
    Fzn = KPRT31*(xd - x) + KPRT32*(yd - y) + KPRT33*(zd - z) + KDRT31*(xd_dot - x_dot) + KDRT32*(yd_dot - y_dot) + KDRT33*(zd_dot - z_dot);

    // calculate the torque in task space using impedance control
    // use force control when pressing on the egg
    if (myarrayofpoints[startpoint].mode == XYSTIFF) {
        check = 1;
        *tau1 = JT_11*(kpx*(xd - x) + kdx*(xd_dot - x_dot)) + JT_12*(kpy*(yd - y) + kdy*(yd_dot - y_dot)) + JT_13*(kpz*(zd - z) + kdz*(zd_dot - z_dot)) + ff*u_fric1;  // JT_13 = 0
        *tau2 = JT_21*(kpx*(xd - x) + kdx*(xd_dot - x_dot)) + JT_22*(kpy*(yd - y) + kdy*(yd_dot - y_dot)) + JT_23*(kpz*(zd - z) + kdz*(zd_dot - z_dot)) + ff*u_fric2 + JT_23*(fzcmd/kt);
        *tau3 = JT_31*(kpx*(xd - x) + kdx*(xd_dot - x_dot)) + JT_32*(kpy*(yd - y) + kdy*(yd_dot - y_dot)) + JT_33*(kpz*(zd - z) + kdz*(zd_dot - z_dot)) + ff*u_fric3 + JT_33*(fzcmd/kt);
    }
    else {
        check = 2;
        *tau1 = JTR11*Fxn + JTR12*Fyn + JTR13 * Fzn + u_fric1;
        *tau2 = JTR21*Fxn + JTR22*Fyn + JTR23 * Fzn + u_fric2;
        *tau3 = JTR31*Fxn + JTR32*Fyn + JTR33 * Fzn + u_fric3;
    }




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
    serial_printf(&SerialA,"x:%.4f, y:%.4f, z:%.4f \n\r",x, y, z);
    //    serial_printf(&SerialA, "measured: %.2f %.2f, %.2f, x,y,z: %.2f %.2f, %.2f, DH from IK: %.2f %.2f, %.2f,  motor from IK: %.2f %.2f, %.2f \n\r",
    //                  printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI, x, y, z, t1DH_IK*180/PI,t2DH_IK*180/PI,t3DH_IK*180/PI, t1m_IK*180/PI,t2m_IK*180/PI,t3m_IK*180/PI);

}

