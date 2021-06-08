#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/inertial_unit.h>

#include "b_webotsInterface.h"

WbDeviceTag IMU;
WbDeviceTag motor1_l;
WbDeviceTag motor2_l;
WbDeviceTag motor1_r;
WbDeviceTag motor2_r;
WbDeviceTag motor_pos_sensor1_l;
WbDeviceTag motor_pos_sensor2_l;
WbDeviceTag motor_pos_sensor1_r;
WbDeviceTag motor_pos_sensor2_r;
WbDeviceTag touch_sensor_l;
WbDeviceTag touch_sensor_r;

void webots_device_init()
{
    IMU = wb_robot_get_device("inertial unit");
    motor1_l = wb_robot_get_device("motor1");
    motor2_l = wb_robot_get_device("motor2");
    motor1_r = wb_robot_get_device("motor1_r");
    motor2_r = wb_robot_get_device("motor2_r");
    motor_pos_sensor1_l = wb_robot_get_device("sensor1");
    motor_pos_sensor2_l = wb_robot_get_device("sensor2");
    motor_pos_sensor1_r = wb_robot_get_device("sensor1_r");
    motor_pos_sensor2_r = wb_robot_get_device("sensor2_r");
    touch_sensor_l = wb_robot_get_device("touch sensor");
    touch_sensor_r = wb_robot_get_device("touch sensor r");
    wb_inertial_unit_enable(IMU, TIME_STEP);
    wb_position_sensor_enable(motor_pos_sensor1_l, TIME_STEP);
    wb_position_sensor_enable(motor_pos_sensor2_l, TIME_STEP);
    wb_position_sensor_enable(motor_pos_sensor1_r, TIME_STEP);
    wb_position_sensor_enable(motor_pos_sensor2_r, TIME_STEP);
    wb_touch_sensor_enable(touch_sensor_l, TIME_STEP);
    wb_touch_sensor_enable(touch_sensor_r, TIME_STEP);
}

void set_torque1_l(double torque)
{
    wb_motor_set_torque(motor1_l, torque);
}
void set_torque2_l(double torque)
{
    wb_motor_set_torque(motor2_l, torque);
}
void set_torque1_r(double torque)
{
    wb_motor_set_torque(motor1_r, torque);
}
void set_torque2_r(double torque)
{
    wb_motor_set_torque(motor2_r, torque);
}

double get_motor_angle1_l()
{
    double angle = wb_position_sensor_get_value(motor_pos_sensor1_l);
    return angle;
}
double get_motor_angle2_l()
{
    double angle = wb_position_sensor_get_value(motor_pos_sensor2_l);
    return angle;
}
double get_motor_angle1_r()
{
    double angle = wb_position_sensor_get_value(motor_pos_sensor1_r);
    return angle;
}
double get_motor_angle2_r()
{
    double angle = wb_position_sensor_get_value(motor_pos_sensor2_r);
    return angle;
}

bool is_foot_touching_l()
{
    return wb_touch_sensor_get_value(touch_sensor_l);
}
bool is_foot_touching_r()
{
    return wb_touch_sensor_get_value(touch_sensor_r);
}

/*
Read IMU data
*/
eulerAngleTypeDef get_IMU_Angle()
{
  const double* data = wb_inertial_unit_get_roll_pitch_yaw(IMU);
  
  eulerAngleTypeDef eulerAngle;
  eulerAngle.roll  = data[0];
  eulerAngle.pitch = data[1];
  eulerAngle.yaw   = data[2];
  
  return eulerAngle;
}

//-----------------------------------------------------------keyboard
/*
Read Keyboard data
*/
int get_keyboard()
{
  return wb_keyboard_get_key();
}