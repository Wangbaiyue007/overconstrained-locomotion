#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <webots/keyboard.h>

#include "easyMat.h"
#include "b_controller.h"

robotTypeDef robot;
double Ts;

void FootTranslation_l(matTypeDef *A01, matTypeDef *A02, double theta1, double theta2);
void FootTranslation_r(matTypeDef *A01, matTypeDef *A02, double theta1, double theta2);
void ZTranslation_l(matTypeDef *z0, matTypeDef *z1, double theta1);
void ZTranslation_r(matTypeDef *z0, matTypeDef *z1, double theta1);
void StateUpdate();
void VelocityControl();
void LegUpdate();
void JacobianUpdate();

void robot_init()
{
    robot.k_s = 800;
    robot.k_v = 1;
    robot.k_beta = 3;
    robot.k_beta_v = 0.025;
    robot.k_spring = 50;
    robot.k_spring_v = 0.22;
    robot.k_pitch = 0.5;
    robot.k_pitch_v = 0.3;
    robot.spring_relax_length = 0.7;
    robot.spring_current_length_l = 1;
    robot.spring_current_length_r = 1;
    robot.spring_diff_v_l = 0;
    robot.spring_diff_v_r = 0;
    robot.v_leg_l = 0;
    robot.v_leg_r = 0;
    robot.spring_force_l = 0;
    robot.spring_force_r = 0;
    robot.omega = 20;
    robot.beta_l = 0;
    robot.beta_r = 0;
    robot.beta_dot_l = 0;
    robot.beta_dot_r = 0;

    easyMat_create(&robot.A_0_1_l, 4, 4);
    easyMat_eye(&robot.A_0_1_l);
    easyMat_create(&robot.A_0_2_l, 4, 4);
    easyMat_eye(&robot.A_0_2_l);
    easyMat_create(&robot.A_2_0_l, 4, 4);
    easyMat_eye(&robot.A_2_0_l);
    easyMat_create(&robot.z0_l, 3, 1);
    easyMat_clear(&robot.z0_l);
    easyMat_create(&robot.z1_l, 3, 1);
    easyMat_clear(&robot.z1_l);
    easyMat_create(&robot.Jacobian_l, 6, 2);
    easyMat_clear(&robot.Jacobian_l);
    robot.z0_l.data[0][0] = 0;
    robot.z0_l.data[1][0] = 0;
    robot.z0_l.data[2][0] = 1;

    easyMat_create(&robot.A_0_1_r, 4, 4);
    easyMat_eye(&robot.A_0_1_r);
    easyMat_create(&robot.A_0_2_r, 4, 4);
    easyMat_eye(&robot.A_0_2_r);
    easyMat_create(&robot.A_2_0_r, 4, 4);
    easyMat_eye(&robot.A_2_0_r);
    easyMat_create(&robot.z0_r, 3, 1);
    easyMat_clear(&robot.z0_r);
    easyMat_create(&robot.z1_r, 3, 1);
    easyMat_clear(&robot.z1_r);
    easyMat_create(&robot.Jacobian_r, 6, 2);
    easyMat_clear(&robot.Jacobian_r);
    robot.z0_r.data[0][0] = 0;
    robot.z0_r.data[1][0] = 0;
    robot.z0_r.data[2][0] = 1;

    robot.q_l.motor_angle1 = 0;
    robot.q_dot_l.motor_angle1 = 0;
    robot.q_l.motor_angle2 = 0;
    robot.q_dot_l.motor_angle2 = 0;

    robot.q_r.motor_angle1 = 0;
    robot.q_dot_r.motor_angle1 = 0;
    robot.q_r.motor_angle2 = 0;
    robot.q_dot_r.motor_angle2 = 0;

    easyMat_create(&robot.o0_l, 3, 1);
    easyMat_clear(&robot.o0_l);
    easyMat_create(&robot.o1_l, 3, 1);
    easyMat_clear(&robot.o1_l);
    easyMat_create(&robot.EndPoint_l, 3, 1);
    easyMat_clear(&robot.EndPoint_l);
    easyMat_create(&robot.EndPoint_dot_l, 3, 1);
    easyMat_clear(&robot.EndPoint_dot_l);
    easyMat_create(&robot.EndPoint_F_l, 6, 1);
    easyMat_clear(&robot.EndPoint_F_l);

    easyMat_create(&robot.o0_r, 3, 1);
    easyMat_clear(&robot.o0_r);
    easyMat_create(&robot.o1_r, 3, 1);
    easyMat_clear(&robot.o1_r);
    easyMat_create(&robot.EndPoint_r, 3, 1);
    easyMat_clear(&robot.EndPoint_r);
    easyMat_create(&robot.EndPoint_dot_r, 3, 1);
    easyMat_clear(&robot.EndPoint_dot_r);
    easyMat_create(&robot.EndPoint_F_r, 6, 1);
    easyMat_clear(&robot.EndPoint_F_r);

    // temporary variables
    easyMat_create(&robot.tau_l, 2, 1);
    easyMat_clear(&robot.tau_l);
    easyMat_create(&robot.tau_r, 2, 1);
    easyMat_clear(&robot.tau_r);
    easyMat_create(&robot.JT_l, 2, 6);
    easyMat_clear(&robot.JT_l);
    easyMat_create(&robot.JT_r, 2, 6);
    easyMat_clear(&robot.JT_r);
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

    robot.body_angle.pitch = 0;
    robot.body_angle.roll = 0;
    robot.body_angle.yaw = 0;
    robot.body_angle_w.pitch = 0;
    robot.body_angle_w.roll = 0;
    robot.body_angle_w.yaw = 0;
    Ts = 0;
    robot.Ts = 0;
    robot.system_ms = 0;
}

void robot_free()
{
    easyMat_free(&robot.A_0_1_l);
    easyMat_free(&robot.A_0_2_l);
    easyMat_free(&robot.A_2_0_l);
    easyMat_free(&robot.EndPoint_l);
    easyMat_free(&robot.EndPoint_dot_l);
    easyMat_free(&robot.EndPoint_F_l);
    easyMat_free(&robot.Jacobian_l);
    easyMat_free(&robot.o1_l);
    easyMat_free(&robot.z0_l);
    easyMat_free(&robot.z1_l);
    easyMat_free(&robot.A_0_1_r);
    easyMat_free(&robot.A_0_2_r);
    easyMat_free(&robot.A_2_0_r);
    easyMat_free(&robot.EndPoint_r);
    easyMat_free(&robot.EndPoint_dot_r);
    easyMat_free(&robot.EndPoint_F_r);
    easyMat_free(&robot.Jacobian_r);
    easyMat_free(&robot.o1_r);
    easyMat_free(&robot.z0_r);
    easyMat_free(&robot.z1_r);
    easyMat_free(&robot.JT_l);
    easyMat_free(&robot.JT_r);
    easyMat_free(&robot.tau_l);
    easyMat_free(&robot.tau_r);
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

    /*Body angles*/
    double previous_roll = robot.body_angle.roll;
    double previous_pitch = robot.body_angle.pitch;
    double previous_yaw = robot.body_angle.yaw;
    robot.body_angle = get_IMU_Angle(); //printf("pitch angle: %f \n", robot.body_angle.pitch);
    robot.body_angle_w.roll = (robot.body_angle.roll - previous_roll) / (0.001 * TIME_STEP);
    robot.body_angle_w.pitch = (robot.body_angle.pitch - previous_pitch) / (0.001 * TIME_STEP);
    robot.body_angle_w.yaw = (robot.body_angle.yaw - previous_yaw) / (0.001 * TIME_STEP);

    /*Motor angles*/
    double current_motor_angle1_l = get_motor_angle1_l();
    double current_motor_angle2_l = get_motor_angle2_l(); //printf("motor angle 2: %f \n", current_motor_angle2);
    double current_motor_angle1_r = get_motor_angle1_r(); 
    double current_motor_angle2_r = get_motor_angle2_r(); //printf("motor angle 2: %f \n", current_motor_angle2_r);
    double current_motor_angle_dot1_l = (current_motor_angle1_l - robot.q_l.motor_angle1) / (0.001 * TIME_STEP);
    double current_motor_angle_dot2_l = (current_motor_angle2_l - robot.q_l.motor_angle2) / (0.001 * TIME_STEP);
    double current_motor_angle_dot1_r = (current_motor_angle1_r - robot.q_r.motor_angle1) / (0.001 * TIME_STEP);
    double current_motor_angle_dot2_r = (current_motor_angle2_r - robot.q_r.motor_angle2) / (0.001 * TIME_STEP);
    robot.q_l.motor_angle1 = 0.5 * current_motor_angle1_l + 0.5 * robot.q_l.motor_angle1;
    robot.q_l.motor_angle2 = 0.5 * current_motor_angle2_l + 0.5 * robot.q_l.motor_angle2;
    robot.q_r.motor_angle1 = 0.5 * current_motor_angle1_r + 0.5 * robot.q_r.motor_angle1;
    robot.q_r.motor_angle2 = 0.5 * current_motor_angle2_r + 0.5 * robot.q_r.motor_angle2;
    robot.q_dot_l.motor_angle1 = 0.5 * current_motor_angle_dot1_l + 0.5 * robot.q_dot_l.motor_angle1;
    robot.q_dot_l.motor_angle2 = 0.5 * current_motor_angle_dot2_l + 0.5 * robot.q_dot_l.motor_angle2;
    robot.q_dot_r.motor_angle1 = 0.5 * current_motor_angle_dot1_r + 0.5 * robot.q_dot_r.motor_angle1;
    robot.q_dot_r.motor_angle2 = 0.5 * current_motor_angle_dot2_r + 0.5 * robot.q_dot_r.motor_angle2;

    /*Leg forward kinematics*/
    FootTranslation_l(&robot.A_0_1_l, &robot.A_0_2_l, current_motor_angle1_l, current_motor_angle2_l); //printf("%f \n", robot.A_0_2.data[0][3]);
    FootTranslation_r(&robot.A_0_1_r, &robot.A_0_2_r, current_motor_angle1_r, current_motor_angle2_r);
    ZTranslation_l(&robot.z0_l, &robot.z1_l, current_motor_angle1_l); //printf("z1: %f\n", robot.z1.data[0][0]);
    ZTranslation_r(&robot.z0_r, &robot.z1_r, current_motor_angle1_r);

    /*Controllers*/
    JacobianUpdate();
    StateUpdate();
    VelocityControl();
    LegUpdate();
}

void FootTranslation_l(matTypeDef *A01, matTypeDef *A02, double theta1, double theta2)
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
    //printf("%f \n", temp.data[0][3]);
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
void FootTranslation_r(matTypeDef *A01, matTypeDef *A02, double theta1, double theta2)
{
    matTypeDef temp1, temp2;
    easyMat_create(&temp1, 4, 4);
    easyMat_create(&temp2, 4, 4);
    rotX(&robot.Rx, -1.0f);
    rotZ(&robot.Rz, theta1);
    tslX(&robot.Tx, 0.5f);
    easyMat_mult(&temp1, &robot.Rz, &robot.Tx);
    easyMat_mult(A01, &temp1, &robot.Rx);
    easyMat_mult(&temp2, &temp1, &robot.Rx);
    //printf("%f \n", temp.data[0][3]);
    rotX(&robot.Rx, 1.0f);
    rotZ(&robot.Rz, theta2);
    tslX(&robot.Tx, 0.5f);
    easyMat_mult(&temp1, &temp2, &robot.Rz);
    //printf("%f \n", temp.data[0][3]);
    easyMat_mult(&temp2, &temp1, &robot.Tx);
    easyMat_mult(A02, &temp2, &robot.Rx);
    easyMat_free(&temp1);
    easyMat_free(&temp2);
}

void ZTranslation_l(matTypeDef *z0, matTypeDef *z1, double theta1)
{
    matTypeDef temp;
    easyMat_create(&temp, 3, 3);
    easyMat_rotZ(&robot.rz, theta1);
    easyMat_rotX(&robot.rx, 1.0f);
    easyMat_mult(&temp, &robot.rz, &robot.rx); //printf("temp: %f\n", temp.data[0][0]);
    easyMat_mult(z1, &temp, z0);
    easyMat_free(&temp);
}
void ZTranslation_r(matTypeDef *z0, matTypeDef *z1, double theta1)
{
    matTypeDef temp;
    easyMat_create(&temp, 3, 3);
    easyMat_rotZ(&robot.rz, theta1);
    easyMat_rotX(&robot.rx, -1.0f);
    easyMat_mult(&temp, &robot.rz, &robot.rx); //printf("temp: %f\n", temp.data[0][0]);
    easyMat_mult(z1, &temp, z0);
    easyMat_free(&temp);
}

void StateUpdate()
{
    // Update stace time
    if (robot.spring_current_length_l < robot.spring_relax_length && robot.spring_current_length_r < robot.spring_relax_length && is_foot_touching_l() && is_foot_touching_r())
    {
        Ts += 0.001 * TIME_STEP;
        robot.state = STANCE;
    }
    else
    {
        if (robot.state == STANCE)
            robot.Ts = (robot.Ts + Ts) / 2;
        Ts = 0;
        robot.state = FLIGHT;
    } //printf("Stance time: %f \n", robot.Ts);
}

void VelocityControl()
{
    double v_tangential_l = -robot.beta_dot_l * robot.spring_current_length_l;
    double v_radial_l = -robot.spring_diff_v_l;
    double v_tangential_r = -robot.beta_dot_r * robot.spring_current_length_r;
    double v_radial_r = -robot.spring_diff_v_r;
    double projection_angle_l = 0;
    double projection_angle_r = 0;
    if (robot.state == STANCE)
    {
        double v_forward_l = v_radial_l * sin(robot.beta_l) + v_tangential_l * cos(robot.beta_l); //printf("v l: %f\n", v_forward_l);
        double v_forward_r = - v_radial_r * sin(robot.beta_r) - v_tangential_r * cos(robot.beta_r); //printf("v r: %f\n", v_forward_r);
        projection_angle_l = acos(sqrt(pow(robot.EndPoint_l.data[1][0], 2) + pow(robot.EndPoint_l.data[2][0], 2)) / robot.spring_current_length_l) - 1;
        projection_angle_r = acos(sqrt(pow(robot.EndPoint_r.data[1][0], 2) + pow(robot.EndPoint_r.data[2][0], 2)) / robot.spring_current_length_r) - 1;
        robot.v_forward = (v_forward_l * cos(projection_angle_l) + v_forward_r * cos(projection_angle_r)) / 2;
        printf("v: %f\n", robot.v_forward);
    }

    double beta_previous_l = robot.beta_l;
    double beta_previous_r = robot.beta_r;
    robot.beta_l = atan2(robot.EndPoint_l.data[1][0], robot.EndPoint_l.data[2][0]); //printf("beta l: %f\n", robot.beta_l);
    robot.beta_dot_l = (robot.beta_l - beta_previous_l) / (0.001 * TIME_STEP);
    robot.beta_desire_l = (robot.v_forward * robot.Ts) / (2 * robot.spring_relax_length * cos(projection_angle_l)); //printf("beta desire l: %f\n", robot.beta_desire_l);
    robot.beta_r = atan2(robot.EndPoint_r.data[1][0], robot.EndPoint_r.data[2][0]); //printf("beta r: %f\n", robot.beta_r);
    robot.beta_dot_r = (robot.beta_r - beta_previous_r) / (0.001 * TIME_STEP);
    robot.beta_desire_r = -(robot.v_forward * robot.Ts) / (2 * robot.spring_relax_length * cos(projection_angle_r));
}

void LegUpdate()
{
    for (uint16_t i = 0; i < 3; i++)
    {
        robot.EndPoint_dot_l.data[i][0] = (robot.A_0_2_l.data[i][3] - robot.EndPoint_l.data[i][0]) / (0.001 * TIME_STEP);
        robot.EndPoint_l.data[i][0] = robot.A_0_2_l.data[i][3];
        robot.o1_l.data[i][0] = robot.A_0_1_l.data[i][3];
        robot.EndPoint_dot_r.data[i][0] = (robot.A_0_2_r.data[i][3] - robot.EndPoint_r.data[i][0]) / (0.001 * TIME_STEP);
        robot.EndPoint_r.data[i][0] = robot.A_0_2_r.data[i][3];
        robot.o1_r.data[i][0] = robot.A_0_1_r.data[i][3];
    }
    double spring_current_length_l = sqrt(pow(robot.EndPoint_l.data[0][0], 2) + pow(robot.EndPoint_l.data[1][0], 2) + pow(robot.EndPoint_l.data[2][0], 2));
    robot.spring_diff_v_l = (spring_current_length_l - robot.spring_current_length_l) / (0.001 * TIME_STEP);
    robot.spring_current_length_l = spring_current_length_l;
    robot.v_leg_l = sqrt(pow(robot.EndPoint_dot_l.data[0][0], 2) + pow(robot.EndPoint_dot_l.data[1][0], 2) + pow(robot.EndPoint_dot_l.data[2][0], 2));
    double spring_diff_l = robot.spring_current_length_l - robot.spring_relax_length;

    double spring_current_length_r = sqrt(pow(robot.EndPoint_r.data[0][0], 2) + pow(robot.EndPoint_r.data[1][0], 2) + pow(robot.EndPoint_r.data[2][0], 2));
    robot.spring_diff_v_r = (spring_current_length_r - robot.spring_current_length_r) / (0.001 * TIME_STEP);
    robot.spring_current_length_r = spring_current_length_r;
    robot.v_leg_r = sqrt(pow(robot.EndPoint_dot_r.data[0][0], 2) + pow(robot.EndPoint_dot_r.data[1][0], 2) + pow(robot.EndPoint_dot_r.data[2][0], 2));
    double spring_diff_r = robot.spring_current_length_r - robot.spring_relax_length;

    // Calculate virtual force
    if (robot.state == FLIGHT)
    {
        robot.spring_force_l = -robot.k_spring * (spring_diff_l - 0.00) - robot.k_spring_v * robot.spring_diff_v_l; //printf("Fs_l: %f \n", robot.spring_force_l);
        robot.spring_force_r = -robot.k_spring * (spring_diff_r - 0.00) - robot.k_spring_v * robot.spring_diff_v_r; //printf("Fs_r: %f \n", robot.spring_force_r);
    }
    else if (robot.state == STANCE)
    {
        robot.spring_force_l = (robot.k_s * (-spring_diff_l) + robot.k_v * (-robot.spring_diff_v_l / robot.omega)) / (sqrt(pow(spring_diff_l, 2) + pow((robot.spring_diff_v_l / robot.omega), 2)) + 0.2);
        robot.spring_force_r = (robot.k_s * (-spring_diff_r) + robot.k_v * (-robot.spring_diff_v_r / robot.omega)) / (sqrt(pow(spring_diff_r, 2) + pow((robot.spring_diff_v_r / robot.omega), 2)) + 0.2);
    }

    // update endpoint force
    if (robot.state == FLIGHT)
    {
        robot.EndPoint_F_l.data[0][0] = robot.spring_force_l * (robot.EndPoint_l.data[0][0] / robot.spring_current_length_l);
        robot.EndPoint_F_l.data[1][0] = robot.spring_force_l * (robot.EndPoint_l.data[1][0] / robot.spring_current_length_l);
        robot.EndPoint_F_l.data[2][0] = robot.spring_force_l * (robot.EndPoint_l.data[2][0] / robot.spring_current_length_l);
        robot.EndPoint_F_l.data[5][0] = -robot.k_beta * (robot.beta_l - robot.beta_desire_l) - robot.k_beta_v * robot.beta_dot_l; // touch down angle adjustment
        robot.EndPoint_F_r.data[0][0] = robot.spring_force_r * (robot.EndPoint_r.data[0][0] / robot.spring_current_length_r);
        robot.EndPoint_F_r.data[1][0] = robot.spring_force_r * (robot.EndPoint_r.data[1][0] / robot.spring_current_length_r);
        robot.EndPoint_F_r.data[2][0] = robot.spring_force_r * (robot.EndPoint_r.data[2][0] / robot.spring_current_length_r);
        robot.EndPoint_F_r.data[5][0] = -robot.k_beta * (robot.beta_r - robot.beta_desire_r) - robot.k_beta_v * robot.beta_dot_r; // touch down angle adjustment
        for (uint16_t i = 0; i < 2; i++)
        {
            robot.EndPoint_F_l.data[i + 3][0] = 0; //printf("Fl %d: %f \n", i+1, robot.EndPoint_F_l.data[i][0]);
            robot.EndPoint_F_r.data[i + 3][0] = 0; //printf("Fr %d: %f \n", i+1, robot.EndPoint_F_r.data[i][0]);
        }
    }
    else if (robot.state == STANCE)
    {
        robot.EndPoint_F_l.data[5][0] = -robot.k_pitch * (robot.body_angle.pitch) - robot.k_pitch_v * robot.body_angle_w.pitch;
        robot.EndPoint_F_r.data[5][0] = robot.k_pitch * (robot.body_angle.pitch) + robot.k_pitch_v * robot.body_angle_w.pitch;
        for (uint16_t i = 0; i < 3; i++)
        {
            robot.EndPoint_F_l.data[i][0] = robot.spring_force_l * (robot.EndPoint_l.data[i][0] / robot.spring_current_length_l);
            robot.EndPoint_F_r.data[i][0] = robot.spring_force_r * (robot.EndPoint_r.data[i][0] / robot.spring_current_length_r);
        }
        for (uint16_t i = 0; i < 2; i++)
        {
            robot.EndPoint_F_l.data[i + 3][0] = 0; //printf("Fs %d: %f \n", i+1, robot.EndPoint_F_l.data[i][0]);
            robot.EndPoint_F_r.data[i + 3][0] = 0;
        }
    }
}

void JacobianUpdate()
{
    matTypeDef S;
    easyMat_create(&S, 3, 3);
    easyMat_sub(&robot.diff1, &robot.EndPoint_l, &robot.o0_l);
    easyMat_sub(&robot.diff2, &robot.EndPoint_l, &robot.o1_l); //printf("diff: %f \n", robot.diff2.data[0][0]);
    skew(&S, &robot.z0_l);                                     //printf("S: %f\n", S.data[1][0]);
    easyMat_mult(&robot.J11, &S, &robot.diff1);
    skew(&S, &robot.z1_l); //printf("S: %f\n", S.data[1][2]);
    easyMat_mult(&robot.J12, &S, &robot.diff2);
    for (uint16_t i = 0; i < 3; i++)
    {
        robot.Jacobian_l.data[i][0] = robot.J11.data[i][0]; //printf("j11: %f\n", robot.Jacobian.data[i][0]);
        robot.Jacobian_l.data[i][1] = robot.J12.data[i][0]; //printf("j12: %f\n", robot.Jacobian.data[i][1]);
        robot.Jacobian_l.data[i + 3][0] = robot.z0_l.data[i][0];
        robot.Jacobian_l.data[i + 3][1] = robot.z1_l.data[i][0];
    }

    easyMat_sub(&robot.diff1, &robot.EndPoint_r, &robot.o0_r);
    easyMat_sub(&robot.diff2, &robot.EndPoint_r, &robot.o1_r); //printf("diff: %f \n", robot.diff2.data[0][0]);
    skew(&S, &robot.z0_r);                                     //printf("S: %f\n", S.data[1][0]);
    easyMat_mult(&robot.J11, &S, &robot.diff1);
    skew(&S, &robot.z1_r); //printf("S: %f\n", S.data[1][2]);
    easyMat_mult(&robot.J12, &S, &robot.diff2);
    for (uint16_t i = 0; i < 3; i++)
    {
        robot.Jacobian_r.data[i][0] = robot.J11.data[i][0]; //printf("j11: %f\n", robot.Jacobian.data[i][0]);
        robot.Jacobian_r.data[i][1] = robot.J12.data[i][0]; //printf("j12: %f\n", robot.Jacobian.data[i][1]);
        robot.Jacobian_r.data[i + 3][0] = robot.z0_r.data[i][0];
        robot.Jacobian_r.data[i + 3][1] = robot.z1_r.data[i][0];
    }

    easyMat_free(&S);
    easyMat_trans(&robot.JT_l, &robot.Jacobian_l);
    easyMat_trans(&robot.JT_r, &robot.Jacobian_r);
}

void robot_control()
{
    matTypeDef endpoint_v, q_dot;
    easyMat_create(&endpoint_v, 6, 1);
    easyMat_create(&q_dot, 3, 1);
    q_dot.data[0][0] = (fabs(robot.q_dot_l.motor_angle1) + 0.01) * (robot.q_l.motor_angle1 / fabs(robot.q_dot_l.motor_angle1));
    q_dot.data[1][0] = (fabs(robot.q_dot_l.motor_angle2) + 0.01) * (robot.q_l.motor_angle2 / fabs(robot.q_dot_l.motor_angle2));
    easyMat_mult(&endpoint_v, &robot.Jacobian_l, &q_dot);
    easyMat_mult(&robot.tau_l, &robot.JT_l, &robot.EndPoint_F_l);
    double product_l = endpoint_v.data[0][0] * robot.EndPoint_l.data[0][0] + endpoint_v.data[1][0] * robot.EndPoint_l.data[1][0] + endpoint_v.data[2][0] * robot.EndPoint_l.data[2][0];

    q_dot.data[0][0] = (fabs(robot.q_dot_r.motor_angle1) + 0.01) * (robot.q_r.motor_angle1 / fabs(robot.q_dot_r.motor_angle1));
    q_dot.data[1][0] = (fabs(robot.q_dot_r.motor_angle2) + 0.01) * (robot.q_r.motor_angle2 / fabs(robot.q_dot_r.motor_angle2));
    easyMat_mult(&endpoint_v, &robot.Jacobian_r, &q_dot);
    easyMat_mult(&robot.tau_r, &robot.JT_r, &robot.EndPoint_F_r);
    double product_r = endpoint_v.data[0][0] * robot.EndPoint_r.data[0][0] + endpoint_v.data[1][0] * robot.EndPoint_r.data[1][0] + endpoint_v.data[2][0] * robot.EndPoint_r.data[2][0];

    /**/
    if (robot.q_l.motor_angle2 < 0.01 ) // avoid initial leg configuration
    {
        set_torque1_l(-4);
        set_torque2_l(2);
    }
    else
    {
        if (fabs(product_l) < 0.1) // avoid singularities
        {
            printf("left Singularities!!");
            set_torque1_l(0.01);
            set_torque2_l(0.01);
        }
        else
        {
            set_torque1_l(robot.tau_l.data[0][0]); //printf("tau: %f \n", robot.tau.data[0][0]);
            set_torque2_l(robot.tau_l.data[1][0]);
        }
    }

    /**/
    if (robot.q_r.motor_angle2 > -0.01) // avoid initial leg configuration
    {
        set_torque1_r(4);
        set_torque2_r(-2);
    }
    else
    {
        if (fabs(product_r) < 0.1) // avoid singularities
        {
            printf("right Singularities!!");
            set_torque1_r(-0.01);
            set_torque2_r(-0.01);
        }
        else
        {
            set_torque1_r(robot.tau_r.data[0][0]);  //printf("tau: %f \n", robot.tau_r.data[0][0]);
            set_torque2_r(robot.tau_r.data[1][0]);
        }
    }
}