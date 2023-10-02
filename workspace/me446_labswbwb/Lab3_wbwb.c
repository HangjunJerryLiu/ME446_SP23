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
float gamma = 0.0;         // variables to find IK

float x = 0.0;
float y = 0.0;
float z = 0.0;      // position of end-effector

float dist = 0;

float x_a = 0.31;
float y_a = 0.10;
float z_a = 0.12;

float x_b = 0;
float y_b = 0;
float z_b = 0;

float dx = 0.07;
float dy = 0.1;
float dz = 0.0; //part 7

float t_start = 0.0;
float t_total = 0.61; //v = 0.2m/s


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

float theta1_ref = 0.0;
float theta2_ref = 0.0;
float theta3_ref = 0.0;       // desired motor angles

// used in the cubic trajectory
float theta_cu = 0.0;
float theta_cu_dot = 0.0;
float theta_cu_dd = 0.0;

float edot1 = 0.0;
float edot2 = 0.0;
float edot3 = 0.0;       // error of theta_dot trajectory

float J1 = 0.0167;
float J2 = 0.03;
float J3 = 0.0128;

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

float kp1 = 14; //60;
float kp2 = 2000;
float kp3 = 2667;    //80
float kd1 = 2.5;   //2;
float kd2 = 67;  //2
float kd3 = 67;  //2
float kI1 = 500;
float kI2 = 500;
float kI3 = 500;

float Ik1 = 0.0;
float Ik2 = 0.0;
float Ik3 = 0.0;
float Ik1_old = 0.0;
float Ik2_old = 0.0;
float Ik3_old = 0.0;

float e1 = 0;
float e2 = 0;
float e3 = 0;       // error terms for the thetas
float e1_old = 0;
float e2_old = 0;
float e3_old = 0;

float thresh1 = 0.001;
float thresh2 = 0.001;
float thresh3 = 0.001;    // threshold to switch to PID

int state = 1;    // state variable

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
float u_fric1 = 0.0;
float u_fric2 = 0.0;
float u_fric3 = 0.0;

float a_theta2 = 0.0;
float a_theta3 = 0.0;

float mystep = 0.0;
float step2 = 0.0;
float step3 = 0.0;   // steps for Part3.2

float qd1 = 0.0;
float qd2 = 0.0;
float qd3 = 0.0;       // theta desired for part 3.2
float qd1_d = 0.0;
float qd2_d = 0.0;
float qd3_d = 0.0;     // theta desired dot
float qd1_dd = 0.0;
float qd2_dd = 0.0;
float qd3_dd = 0.0;     // theta desired double dot

int change_control = 0;   // int variable to change control law

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

void cubic(int timecount) {             // calculate cubic trajectory
    float time = timecount*0.001;

    if (time < 1) {
        theta_cu = 1.5*time*time - time*time*time;
        theta_cu_dot = 3*time - 3*time*time;
        theta_cu_dd = 3 - 6*time;
    }
    if ((time >= 1)&&(time <= 2)) {
        theta_cu = -2+6*time-4.5*time*time+time*time*time;
        theta_cu_dot = 6-9*time+3*time*time;
        theta_cu_dd = -9+6*time;
    }
    if (time > 2) {
        theta_cu = 0;
        theta_cu_dot = 0;
        theta_cu_dd = 0;
    }

}


void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


//    if ((mycount%1000)==0) {
//        if (theta1_ref > 0.1) {
//            theta1_ref = 0;
//        } else {
//            theta1_ref = PI/6.0;
//        }
//        if (theta2_ref > 0.1) {
//            theta2_ref = 0;
//        } else {
//            theta2_ref = PI/6.0;
//        }
//        if (theta3_ref > 0.1) {
//            theta3_ref = 0;
//        } else {
//            theta3_ref = PI/6.0;
//        }
//    }              // step every 1 second

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

//    cubic(mycount%2000);

    implement_discrete_tf(&trajectory, mystep, &qd1, &qd1_d, &qd1_dd);
    implement_discrete_tf(&trajectory2, step2, &qd2, &qd2_d, &qd2_dd);
    implement_discrete_tf(&trajectory3, step3, &qd3, &qd3_d, &qd3_dd);     // implement steps part3.2


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
    beta = acos((0.254*0.254*2-Lp*Lp)/(2*0.254*0.254));



    t1DH_IK = atan2(y,x);
    t2DH_IK = -gamma - alpha;
    t3DH_IK = PI - beta;             // calculate theta_DH using IK

    t1m_IK = t1DH_IK;
    t2m_IK = t2DH_IK + PI/2;
    t3m_IK = t3DH_IK + t2DH_IK;     // calculate theta_motor using IK

    theta1_ref = qd1;
    theta2_ref = qd2;
    theta3_ref = qd3;     // desired angles

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

    e1 = theta1_ref - theta1motor;
    e2 = theta2_ref - theta2motor;
    e3 = theta3_ref - theta3motor;      // the error terms

    //    e1 = theta_cu - theta1motor;
    //    e2 = theta_cu - theta2motor;
    //    e3 = theta_cu - theta3motor;         // error terms for cubic trajectory

    edot1 = qd1_d - Omega1;
    edot2 = qd2_d - Omega2;
    edot3 = qd3_d - Omega3;       // error of angular velocity trajectory

    a_theta2 = qd2_dd + kp2*(e2) + kd2*(edot2);
    a_theta3 = qd3_dd + kp3*(e3) + kd3*(edot3);       // PD & feed forward

    Ik1_old = Ik1;
    Ik2_old = Ik2;
    Ik3_old = Ik3;
    e1_old = e1;
    e2_old = e2;
    e3_old = e3;      // update old values

    //    *tau1 = J1*theta_cu_dd + kp1*e1 + kd1*edot1;
    //    *tau2 = J2*theta_cu_dd + kp2*e2 + kd2*edot2;
    //    *tau3 = J3*theta_cu_dd + kp3*e3 + kd3*edot3;    // feed forward

    //    *tau1 = kp1*e1 - kd1*Omega1;
    //    *tau2 = kp2*e2 - kd2*Omega2;
    //    *tau3 = kp3*e3 - kd3*Omega3;

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

    //Part3
    *tau1 = u_fric1 + J1*qd1_dd + kp1*e1 + kd1*edot1;   // feed forward

    if (change_control == 0) {
        kp2 = 130;
        kd2 = 3.5;
        kp3 = 130;
        kd3 = 2.8;
        *tau2 = u_fric2 + J2*qd2_dd + kp2*e2 + kd2*edot2;
        *tau3 = u_fric3 + J3*qd3_dd + kp3*e3 + kd3*edot3;   // feed forward
    } if (change_control == 1) {
        kp2 = 2500;
        kd2 = 90;
        kp3 = 3500;
        kd3 = 85.5;
        *tau2 = u_fric2 + 0.03*a_theta2 - 0.0076*sin(theta3motor - theta2motor)*a_theta3 - 0.0076*cos(theta3motor - theta2motor)*Omega3*Omega3 - 0.73869*sin(theta2motor);
        *tau3 = u_fric3 - 0.0076*sin(theta3motor - theta2motor)*a_theta2 + 0.0128*a_theta3 + 0.0076*cos(theta3motor - theta2motor)*Omega2*Omega2 - 0.292338*cos(theta3motor);   //tau2 and tau3 are inverse dynamics
    }

    // tau1 is always feed forward, tau2 and tau3 changes based on the "change_control" variable

    //    if (fabs(e1) < thresh1) {
    //        Ik1 = Ik1_old + (e1 + e1_old)*0.001/2;
    //        *tau1 = kp1*e1 - kd1*Omega1 - kI1*Ik1;
    //    }
    //
    //    if (fabs(e2) < thresh2) {
    //        Ik2 = Ik2_old + (e2 + e2_old)*0.001/2;
    //        *tau2 = kp2*e2 - kd2*Omega2 - kI2*Ik2;
    //    }
    //
    //    if (fabs(e3) < thresh3) {
    //        Ik3 = Ik3_old + (e3 + e3_old)*0.001/2;     // integral terms
    //        *tau3 = kp3*e3 - kd3*Omega3 - kI3*Ik3;
    //    }
    //
    //    if (fabs(*tau1) > 5) {
    //        Ik1 = Ik1_old;
    //    }
    //    if (fabs(*tau2) > 5) {
    //        Ik2 = Ik2_old;
    //    }
    //    if (fabs(*tau3) > 5) {
    //        Ik3 = Ik3_old;
    //    }                                  // integral wind-up

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
    }                         // torque limit


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

        Simulink_PlotVar1 = e2;        //theta1motor;
        Simulink_PlotVar2 = e3;    //theta2motor;
        Simulink_PlotVar3 = theta3motor;     //theta3motor;
        Simulink_PlotVar4 = theta3_ref;


    mycount++;
}



void printing(void){
    //serial_printf(&SerialA, "%.2f %.2f, %.2f \n\r",printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI);
    serial_printf(&SerialA, "measured: %.2f %.2f, %.2f, x,y,z: %.2f %.2f, %.2f, DH from IK: %.2f %.2f, %.2f,  motor from IK: %.2f %.2f, %.2f \n\r",
                  printtheta1motor*180/PI, printtheta2motor*180/PI, printtheta3motor*180/PI, x, y, z, t1DH_IK*180/PI,t2DH_IK*180/PI,t3DH_IK*180/PI, t1m_IK*180/PI,t2m_IK*180/PI,t3m_IK*180/PI);

}

