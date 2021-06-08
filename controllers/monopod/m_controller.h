#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdbool.h>

#include "easyMat.h"
#include "m_webotsInterface.h"

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
    double m;                     // mass
    double k_s;                   // ks during stance
    double k_v;                   // kv during stance
    double k_beta;                // k to adjust touch down angle
    double k_beta_v;              // kv to adjust touch down angle
    double k_spring;              // virtual spring constant
    double k_spring_v;            // damping constant
    double spring_relax_length;   // virtual spring normal length
    double spring_current_length; // virtual spring length
    double spring_diff_v;         // virtual spring deflection velocity
    double spring_force;          // virtual spring force
    double v_leg;                 // velocity of the leg's end point
    double v_forward;             // forward velocity
    double omega;                 // natrual frequency
    double beta;                  // touch down angle
    double beta_dot;              // touch down angle velocity
    double beta_desire;           // desired touch down angle

    // robot state
    matTypeDef A_0_1;          // translational matrix from hip to first joint
    matTypeDef A_0_2;          // translational matrix from hip to end point
    matTypeDef A_2_0;          // inverse translational matrix
    matTypeDef z0;             // first joint vector
    matTypeDef z1;             // second joint vector
    matTypeDef Jacobian;       // a 6 * 2 jacobian matrix
    jointSpaceTypeDef q;       // position of joints
    jointSpaceTypeDef q_dot;   // velocity of joints
    matTypeDef o0;             //position of first joint
    matTypeDef o1;             // position of middle joint
    matTypeDef EndPoint;       // position of endpoint
    matTypeDef EndPoint_dot;   // velocity of endpoint
    matTypeDef EndPoint_F;     // force and torque at the endpoint
    stateMachineTypeDef state; // state machine

    // middle variables
    matTypeDef tau, JT; // torque on the two joints, transpose of jacobian
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