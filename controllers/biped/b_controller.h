#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdbool.h>

#include "easyMat.h"
#include "b_webotsInterface.h"

/*
State Machine
*/
typedef enum
{
    STANCE = 0x00,
    FLIGHT = 0x01,
} stateMachineTypeDef;

/*
Joint Space
*/
typedef struct
{
    double motor_angle1; // joint 1 angle
    double motor_angle2; // joint 2 angle
} jointSpaceTypeDef;

/*
this robot
*/
typedef struct
{
    // robot parameters
    double k_s;                     // ks during stance
    double k_v;                     // kv during stance
    double k_beta;                  // k to adjust touch down angle
    double k_beta_v;                // kv to adjust touch down angle
    double k_spring;                // virtual spring constant
    double k_spring_v;              // damping constant
    double k_pitch;                 // k to adjust pitch during stance
    double k_pitch_v;               // k to adjust pitch velocity during stance
    double spring_relax_length;     // virtual spring normal length
    double spring_current_length_l; // virtual spring length of left leg
    double spring_current_length_r; // virtual spring length of right leg
    double spring_diff_v_l;         // virtual spring deflection velocity
    double spring_diff_v_r;         // virtual spring deflection velocity
    double spring_force_l;          // virtual spring force of left leg
    double spring_force_r;          // virtual spring force of right leg
    double v_leg_l;                 // velocity of the leg's end point of left leg
    double v_leg_r;                 // velocity of the leg's end point of right leg
    double v_forward;               // forward velocity
    double omega;                   // natrual frequency
    double beta_l;                  // touch down angle
    double beta_r;                  // touch down angle
    double beta_dot_l;              // touch down angle velocity
    double beta_dot_r;              // touch down angle velocity
    double beta_desire_l;           // desired touch down angle
    double beta_desire_r;           // desired touch down angle

    // robot state, left leg
    matTypeDef A_0_1_l;        // translational matrix from hip to first joint
    matTypeDef A_0_2_l;        // translational matrix from hip to end point
    matTypeDef A_2_0_l;        // inverse translational matrix
    matTypeDef z0_l;           // first joint vector
    matTypeDef z1_l;           // second joint vector
    matTypeDef Jacobian_l;     // a 6 * 2 jacobian matrix
    jointSpaceTypeDef q_l;     // position of joints
    jointSpaceTypeDef q_dot_l; // velocity of joints
    matTypeDef o0_l;           //position of first joint
    matTypeDef o1_l;           // position of middle joint
    matTypeDef EndPoint_l;     // position of endpoint
    matTypeDef EndPoint_dot_l; // velocity of endpoint
    matTypeDef EndPoint_F_l;   // force and torque at the endpoint
    // robot state, ribht leg
    matTypeDef A_0_1_r;        // translational matrix from hip to first joint
    matTypeDef A_0_2_r;        // translational matrix from hip to end point
    matTypeDef A_2_0_r;        // inverse translational matrix
    matTypeDef z0_r;           // first joint vector
    matTypeDef z1_r;           // second joint vector
    matTypeDef Jacobian_r;     // a 6 * 2 jacobian matrix
    jointSpaceTypeDef q_r;     // position of joints
    jointSpaceTypeDef q_dot_r; // velocity of joints
    matTypeDef o0_r;           //position of first joint
    matTypeDef o1_r;           // position of middle joint
    matTypeDef EndPoint_r;     // position of endpoint
    matTypeDef EndPoint_dot_r; // velocity of endpoint
    matTypeDef EndPoint_F_r;   // force and torque at the endpoint

    stateMachineTypeDef state;      // state machine
    eulerAngleTypeDef body_angle;   // robot body angle
    eulerAngleTypeDef body_angle_w; // robot body angle velocity

    // middle variables
    matTypeDef tau_l, tau_r, JT_l, JT_r; // torque on the two joints, transpose of jacobian
    matTypeDef Tx, Rz, Rx, rz, rx;
    matTypeDef J11, J12, diff1, diff2;

    int system_ms; // timer
    double Ts;     // duriation of time during stance
} robotTypeDef;

extern robotTypeDef robot;
extern void robot_init();
extern void robot_free();
extern void robot_control();
extern void updateRobotState();

#endif