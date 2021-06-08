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
extern void set_torque1(double torque);
extern void set_torque2(double torque);
extern double get_motor_angle1();
extern double get_motor_angle2();
extern eulerAngleTypeDef get_IMU_Angle();
extern bool is_foot_touching();
extern double* robot_coordinates();

#endif