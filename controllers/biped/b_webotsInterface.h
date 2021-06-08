#ifndef _WEBOTSINTERFACE_H_
#define _WEBOTSINTERFACE_H_

#define PI (3.141892654)
#define TIME_STEP (2)


typedef struct
{
  double roll;  //x
  double pitch; //z
  double yaw;   //y
} eulerAngleTypeDef;

extern void webots_device_init();
extern void set_torque1_l(double torque);
extern void set_torque2_l(double torque);
extern void set_torque1_r(double torque);
extern void set_torque2_r(double torque);
extern double get_motor_angle1_l();
extern double get_motor_angle2_l();
extern double get_motor_angle1_r();
extern double get_motor_angle2_r();
extern eulerAngleTypeDef get_IMU_Angle();
extern bool is_foot_touching_l();
extern bool is_foot_touching_r();

#endif