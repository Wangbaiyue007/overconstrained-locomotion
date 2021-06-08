#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <webots/keyboard.h>

#include "easyMat.h"
#include "m_controller.h"

robotTypeDef robot;
double Ts;

void FootTranslation(matTypeDef *A01, matTypeDef *A02, double theta1, double theta2);
void ZTranslation(matTypeDef *z0, matTypeDef *z1, double theta1);
void StateUpdate();
void VelocityControl();
void LegUpdate();
void JacobianUpdate();

void robot_init()
{
    robot.k_s = 1200;
    robot.k_v = 5;
    robot.k_beta = 15;
    robot.k_beta_v = 0.2;
    robot.k_spring = 200;
    robot.k_spring_v = 1;
    robot.spring_relax_length = 0.65;
    robot.spring_current_length = 1;
    robot.spring_diff_v = 0;
    robot.v_leg = 0;
    robot.spring_force = 0;
    robot.omega = 20;
    robot.beta = 0;
    robot.beta_dot = 0;

    easyMat_create(&robot.A_0_1, 4, 4);
    easyMat_eye(&robot.A_0_1);
    easyMat_create(&robot.A_0_2, 4, 4);
    easyMat_eye(&robot.A_0_2);
    easyMat_create(&robot.A_2_0, 4, 4);
    easyMat_eye(&robot.A_2_0);
    easyMat_create(&robot.z0, 3, 1);
    easyMat_clear(&robot.z0);
    easyMat_create(&robot.z1, 3, 1);
    easyMat_clear(&robot.z1);
    easyMat_create(&robot.Jacobian, 6, 2);
    easyMat_clear(&robot.Jacobian);
    robot.z0.data[0][0] = 0;
    robot.z0.data[1][0] = 0;
    robot.z0.data[2][0] = 1;

    robot.q.motor_angle1 = 0;
    robot.q_dot.motor_angle1 = 0;
    robot.q.motor_angle2 = 0;
    robot.q_dot.motor_angle2 = 0;

    easyMat_create(&robot.o0, 3, 1);
    easyMat_clear(&robot.o0);
    easyMat_create(&robot.o1, 3, 1);
    easyMat_clear(&robot.o1);
    easyMat_create(&robot.EndPoint, 3, 1);
    easyMat_clear(&robot.EndPoint);
    easyMat_create(&robot.EndPoint_dot, 3, 1);
    easyMat_clear(&robot.EndPoint_dot);
    easyMat_create(&robot.EndPoint_F, 6, 1);
    easyMat_clear(&robot.EndPoint_F);

    // temporary variables
    easyMat_create(&robot.tau, 2, 1);
    easyMat_clear(&robot.tau);
    easyMat_create(&robot.JT, 2, 6);
    easyMat_clear(&robot.JT);
    easyMat_create(&robot.Tx, 4, 4);
    easyMat_clear(&robot.Tx);
    easyMat_create(&robot.Rx, 4, 4);
    easyMat_clear(&robot.Rx);
    easyMat_create(&robot.Rz, 4, 4);
    easyMat_clear(&robot.Rz);
    easyMat_create(&robot.rx, 3, 3);
    easyMat_clear(&robot.rx);
    easyMat_create(&robot.rz, 3, 3);
    easyMat_clear(&robot.rz);
    easyMat_create(&robot.diff1, 3, 1);
    easyMat_clear(&robot.diff1);
    easyMat_create(&robot.diff2, 3, 1);
    easyMat_clear(&robot.diff2);
    easyMat_create(&robot.J11, 3, 1);
    easyMat_clear(&robot.J11);
    easyMat_create(&robot.J12, 3, 1);
    easyMat_clear(&robot.J12);

    Ts = 0;
    robot.Ts = 0;
    robot.system_ms = 0;
}

void robot_free()
{
    easyMat_free(&robot.A_0_1);
    easyMat_free(&robot.A_0_2);
    easyMat_free(&robot.A_2_0);
    easyMat_free(&robot.EndPoint);
    easyMat_free(&robot.EndPoint_dot);
    easyMat_free(&robot.EndPoint_F);
    easyMat_free(&robot.Jacobian);
    easyMat_free(&robot.JT);
    easyMat_free(&robot.o1);
    easyMat_free(&robot.tau);
    easyMat_free(&robot.z0);
    easyMat_free(&robot.z1);
    easyMat_free(&robot.Tx);
    easyMat_free(&robot.Rx);
    easyMat_free(&robot.Rz);
    easyMat_free(&robot.rx);
    easyMat_free(&robot.rz);
    easyMat_free(&robot.diff1);
    easyMat_free(&robot.diff2);
    easyMat_free(&robot.J11);
    easyMat_free(&robot.J12);
}

void updateRobotState()
{
    /*Clock update*/
    robot.system_ms += TIME_STEP;

    /*Motor angles*/
    double current_motor_angle1 = get_motor_angle1();
    double current_motor_angle2 = get_motor_angle2(); //printf("motor angle 2: %f \n", current_motor_angle2);
    double current_motor_angle_dot1 = (current_motor_angle1 - robot.q.motor_angle1) / (0.001 * TIME_STEP);
    double current_motor_angle_dot2 = (current_motor_angle2 - robot.q.motor_angle2) / (0.001 * TIME_STEP);
    robot.q.motor_angle1 = 0.5 * current_motor_angle1 + 0.5 * robot.q.motor_angle1;
    robot.q.motor_angle2 = 0.5 * current_motor_angle2 + 0.5 * robot.q.motor_angle2;
    robot.q_dot.motor_angle1 = 0.5 * current_motor_angle_dot1 + 0.5 * robot.q_dot.motor_angle1;
    robot.q_dot.motor_angle2 = 0.5 * current_motor_angle_dot2 + 0.5 * robot.q_dot.motor_angle2;

    FootTranslation(&robot.A_0_1, &robot.A_0_2, current_motor_angle1, current_motor_angle2); //printf("%f \n", robot.A_0_2.data[0][3]);
    ZTranslation(&robot.z0, &robot.z1, current_motor_angle1);                                //printf("z1: %f\n", robot.z1.data[0][0]);
    StateUpdate();
    VelocityControl();
    LegUpdate();
    JacobianUpdate();
}

void FootTranslation(matTypeDef *A01, matTypeDef *A02, double theta1, double theta2)
{
    matTypeDef temp1, temp2;
    easyMat_create(&temp1, 4, 4);
    easyMat_create(&temp2, 4, 4);
    rotX(&robot.Rx, 1.0f);
    rotZ(&robot.Rz, theta1);
    tslX(&robot.Tx, 0.5f);
    easyMat_mult(&temp1, &robot.Rz, &robot.Tx);
    easyMat_mult(A01, &temp1, &robot.Rx);
    easyMat_mult(&temp2, &temp1, &robot.Rx);
    //printf("theta1: %f \n", theta1);
    rotX(&robot.Rx, -1.0f);
    rotZ(&robot.Rz, theta2);
    tslX(&robot.Tx, 0.5f);
    easyMat_mult(&temp1, &temp2, &robot.Rz);
    //printf("%f \n", temp.data[0][3]);
    easyMat_mult(&temp2, &temp1, &robot.Tx);
    easyMat_mult(A02, &temp2, &robot.Rx);
    easyMat_free(&temp1);
    easyMat_free(&temp2);
}

void ZTranslation(matTypeDef *z0, matTypeDef *z1, double theta1)
{
    matTypeDef temp;
    easyMat_create(&temp, 3, 3);
    easyMat_rotZ(&robot.rz, theta1);
    easyMat_rotX(&robot.rx, 1.0f);
    easyMat_mult(&temp, &robot.rz, &robot.rx); //printf("temp: %f\n", temp.data[0][0]);
    easyMat_mult(z1, &temp, z0);
    easyMat_free(&temp);
}

void StateUpdate()
{
    // Update stace time
    if (robot.spring_current_length < robot.spring_relax_length && is_foot_touching())
    {
        Ts += 0.001 * TIME_STEP;
        robot.state = STANCE;
    }
    else
    {
        if(robot.state == STANCE)
            robot.Ts = (robot.Ts + Ts) / 2;
        Ts = 0;
        robot.state = FLIGHT;
    }//printf("Stance time: %f \n", robot.Ts);
}

void VelocityControl()
{
    double v_tangential = - robot.beta_dot * robot.spring_current_length;
    double v_radial = - robot.spring_diff_v;
    double projection_angle = 0;
    if(robot.state == STANCE)
    {
        robot.v_forward = v_radial * sin(robot.beta) + v_tangential * cos(robot.beta);
        projection_angle = acos(sqrt(pow(robot.EndPoint.data[1][0], 2) + pow(robot.EndPoint.data[2][0], 2)) / robot.spring_current_length) - 1;
        robot.v_forward = robot.v_forward * cos(projection_angle);
        //printf("v: %f\n", robot.v_forward);
    }

    double beta_previous = robot.beta;
    robot.beta = atan2(robot.EndPoint.data[1][0], robot.EndPoint.data[2][0]); 
    robot.beta_dot = (robot.beta - beta_previous) / (0.001 * TIME_STEP);
    //printf("beta: %f ", robot.beta);
    robot.beta_desire = (robot.v_forward * robot.Ts) / (2 * robot.spring_relax_length * cos(projection_angle));
    //printf("beta_desire: %f \n", robot.beta_desire);
}

void LegUpdate()
{
    for (uint16_t i = 0; i < 3; i++)
    {
        robot.EndPoint_dot.data[i][0] = (robot.A_0_2.data[i][3] - robot.EndPoint.data[i][0]) / (0.001 * TIME_STEP);
        robot.EndPoint.data[i][0] = robot.A_0_2.data[i][3];
        //printf("A: %f \n", robot.EndPoint.data[i][0]);
        robot.o1.data[i][0] = robot.A_0_1.data[i][3];
    }
    double spring_current_length = sqrt(pow(robot.EndPoint.data[0][0], 2) + pow(robot.EndPoint.data[1][0], 2) + pow(robot.EndPoint.data[2][0], 2));
    robot.spring_diff_v = (spring_current_length - robot.spring_current_length) / (0.001 * TIME_STEP);
    robot.spring_current_length = spring_current_length;
    robot.v_leg = sqrt(pow(robot.EndPoint_dot.data[0][0], 2) + pow(robot.EndPoint_dot.data[1][0], 2) + pow(robot.EndPoint_dot.data[2][0], 2));
    double spring_diff = robot.spring_current_length - robot.spring_relax_length;

    // Calculate leg force
    if (robot.state == FLIGHT)
    {
        robot.spring_force = -robot.k_spring * (spring_diff - 0.00) - robot.k_spring_v * robot.spring_diff_v;
    }
    else if (robot.state == STANCE)
    {
        robot.spring_force = (robot.k_s * (-spring_diff) + robot.k_v * (-robot.spring_diff_v/robot.omega)) / (sqrt(pow(spring_diff, 2) + pow( (robot.spring_diff_v/robot.omega), 2)) + 0.05);
    }

    // update endpoint force
    if (robot.state == FLIGHT)
    {
        robot.EndPoint_F.data[0][0] = robot.spring_force * (robot.EndPoint.data[0][0] / robot.spring_current_length);
        robot.EndPoint_F.data[1][0] = robot.spring_force * (robot.EndPoint.data[1][0] / robot.spring_current_length);
        robot.EndPoint_F.data[2][0] = robot.spring_force * (robot.EndPoint.data[2][0] / robot.spring_current_length);
        robot.EndPoint_F.data[5][0] = -robot.k_beta * (robot.beta - robot.beta_desire) - robot.k_beta_v * robot.beta_dot; // touch down angle adjustment
        for (uint16_t i = 0; i < 2; i++)
        {
            robot.EndPoint_F.data[i + 3][0] = 0; //printf("Ff %d: %f \n", i+1, robot.EndPoint_F.data[i][0]);
        }
    }
    else if (robot.state == STANCE)
    {
        for (uint16_t i = 0; i < 3; i++)
        {
            robot.EndPoint_F.data[i][0] = robot.spring_force * (robot.EndPoint.data[i][0] / robot.spring_current_length);
            robot.EndPoint_F.data[i + 3][0] = 0; //printf("Fs %d: %f \n", i+1, robot.EndPoint_F.data[i][0]);
        }
    }
}

void JacobianUpdate()
{
    matTypeDef S;
    easyMat_create(&S, 3, 3);
    easyMat_sub(&robot.diff1, &robot.EndPoint, &robot.o0);
    easyMat_sub(&robot.diff2, &robot.EndPoint, &robot.o1); //printf("diff: %f \n", robot.diff2.data[0][0]);
    skew(&S, &robot.z0);                                   //printf("S: %f\n", S.data[1][0]);
    easyMat_mult(&robot.J11, &S, &robot.diff1);
    skew(&S, &robot.z1); //printf("S: %f\n", S.data[1][2]);
    easyMat_mult(&robot.J12, &S, &robot.diff2);
    for (uint16_t i = 0; i < 3; i++)
    {
        robot.Jacobian.data[i][0] = robot.J11.data[i][0]; //printf("j11: %f\n", robot.Jacobian.data[i][0]);
        robot.Jacobian.data[i][1] = robot.J12.data[i][0]; //printf("j12: %f\n", robot.Jacobian.data[i][1]);
        robot.Jacobian.data[i + 3][0] = robot.z0.data[i][0];
        robot.Jacobian.data[i + 3][1] = robot.z1.data[i][0];
    }
    easyMat_free(&S);
    easyMat_trans(&robot.JT, &robot.Jacobian);
}

void robot_control()
{
    matTypeDef endpoint_v, q_dot;
    easyMat_create(&endpoint_v, 6, 1);
    easyMat_create(&q_dot, 3, 1);
    q_dot.data[0][0] = (fabs(robot.q_dot.motor_angle1) + 0.1)*(robot.q.motor_angle1 / fabs(robot.q_dot.motor_angle1));
    q_dot.data[1][0] = (fabs(robot.q_dot.motor_angle2) + 0.1)*(robot.q.motor_angle2 / fabs(robot.q_dot.motor_angle2));
    easyMat_mult(&endpoint_v, &robot.Jacobian, &q_dot);
    easyMat_mult(&robot.tau, &robot.JT, &robot.EndPoint_F);
    double product = endpoint_v.data[0][0] * robot.EndPoint.data[0][0] + endpoint_v.data[1][0] * robot.EndPoint.data[1][0] + endpoint_v.data[2][0] * robot.EndPoint.data[2][0];
    
    if (robot.q.motor_angle2 < 0.01) // avoid initial leg configuration
    {
        set_torque1(-20);
        set_torque2(10);
    }
    else
    {
        if (fabs(product) < 0.01) // avoid singularities
        {
            set_torque1(0.01);
            set_torque2(0.01);
        }
        else
        {
            set_torque1(robot.tau.data[0][0]); // printf("tau: %f \n", robot.tau.data[0][0]);
            set_torque2(robot.tau.data[1][0]);
        }
    }
}