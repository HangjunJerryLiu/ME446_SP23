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

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float theta1DH = 0;
float theta2DH = 0;
float theta3DH = 0;

float theta1inD = 0;
float theta2inD = 0;
float theta3inD = 0;          // angles in degrees

float t1DH_IK = 0.0;
float t2DH_IK = 0.0;
float t3DH_IK = 0.0;         // DH angles calculated using inverse kinematics

float t1m_IK = 0.0;
float t2m_IK = 0.0;
float t3m_IK = 0.0;          // motor angles calculated using IK

float Lp = 0.0;
float alpha = 0.0;
float beta = 0.0;
float gamma = 0.0;         // variables to find IK, derived from law of cosine, (shown in lab 1 report, figure 2.4)

float x = 0.0;
float y = 0.0;
float z = 0.0;      // position of end-effector

float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;

float theta1_ref = 0.0;
float theta2_ref = 0.0;
float theta3_ref = 0.0;       // desired motor angles

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

float kp1 = 60;
float kp2 = 80;
float kd1 = 2;
float kd2 = 2;
float kI1 = 500;
float kI2 = 500;
float kI3 = 500;

float e1 = 0;
float e2 = 0;
float e3 = 0;    // error terms for the thetas

// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


    *tau1 = 0;
    *tau2 = 0;
    *tau3 = 0;

    //Motor torque limitation(Max: 5 Min: -5)

    // save past states
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

    Simulink_PlotVar1 = theta1motor;
    Simulink_PlotVar2 = theta2motor;
    Simulink_PlotVar3 = theta3motor;
    Simulink_PlotVar4 = 0;

    theta1inD=theta1motor*180/PI;
    theta2inD=theta2motor*180/PI;
    theta3inD=theta3motor*180/PI;

    theta1DH = theta1motor;
    theta2DH = theta2motor - PI/2;
    theta3DH = -theta2motor + theta3motor + PI/2;        // motor angles in DH convention

//    x = (127.0*cos(theta1inD)*(cos(theta2inD + theta3inD) + cos(theta2inD)))/500.0;
//    y = (127.0*sin(theta1inD)*(cos(theta2inD + theta3inD) + cos(theta2inD)))/500.0;
//    z = 127.0/500.0 - (127.0*sin(theta2inD))/500.0 - (127.0*sin(theta2inD + theta3inD))/500.0;
//
    x = (127.0*cos(theta1DH)*(cos(theta2DH + theta3DH) + cos(theta2DH)))/500.0;
    y = (127.0*sin(theta1DH)*(cos(theta2DH + theta3DH) + cos(theta2DH)))/500.0;
    z = 127.0/500.0 - (127.0*sin(theta2DH))/500.0 - (127.0*sin(theta2DH + theta3DH))/500.0;     // find xyz coordinates using FK

    Lp = sqrt(x*x + y*y + (z-0.254)*(z-0.254));
    gamma = asin((z-0.254)/Lp);
    alpha = acos(Lp/(2*0.254));
    beta = acos((0.254*0.254*2-Lp*Lp)/(2*0.254*0.254)); // these equations are derived from law of cosine, shown in figure 2.4 in the lab 1 report

    t1DH_IK = atan2(y,x);
    t2DH_IK = -gamma - alpha;
    t3DH_IK = PI - beta;             // calculate theta_DH using IK

    t1m_IK = t1DH_IK;
    t2m_IK = t2DH_IK + PI/2;
    t3m_IK = t3DH_IK + t2DH_IK;     // calculate theta_motor using IK

    mycount++;

}

void printing(void){
    //serial_printf(&SerialA, "%.2f %.2f, %.2f \n\r",printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI);
    serial_printf(&SerialA, "measured: %.2f %.2f, %.2f, x,y,z: %.2f %.2f, %.2f, DH from IK: %.2f %.2f, %.2f,  motor from IK: %.2f %.2f, %.2f \n\r",
                  printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI, x, y, z, t1DH_IK*180/PI,t2DH_IK*180/PI,t3DH_IK*180/PI, t1m_IK*180/PI,t2m_IK*180/PI,t3m_IK*180/PI);

}

