#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>
// #include <webots/keyboard.h>
#include <webots/inertial_unit.h>
#include <webots/supervisor.h>

#include "m_webotsInterface.h"

WbDeviceTag IMU;
WbDeviceTag motor1;
WbDeviceTag motor2;
WbDeviceTag motor_pos_sensor1;
WbDeviceTag motor_pos_sensor2;
WbDeviceTag touch_sensor;
WbNodeRef this_robot;

void webots_device_init()
{
    IMU = wb_robot_get_device("inertial unit");
    motor1 = wb_robot_get_device("motor1");
    motor2 = wb_robot_get_device("motor2");
    motor_pos_sensor1 = wb_robot_get_device("sensor1");
    motor_pos_sensor2 = wb_robot_get_device("sensor2");
    touch_sensor = wb_robot_get_device("touch sensor");
    wb_position_sensor_enable(motor_pos_sensor1, TIME_STEP);
    wb_position_sensor_enable(motor_pos_sensor2, TIME_STEP);
    wb_touch_sensor_enable(touch_sensor, TIME_STEP);
    this_robot = wb_supervisor_node_get_from_def("monopod.restriction");
}

void set_torque1(double torque)
{
    wb_motor_set_torque(motor1, torque);
}
void set_torque2(double torque)
{
    wb_motor_set_torque(motor2, torque);
}

double get_motor_angle1()
{
    double angle = wb_position_sensor_get_value(motor_pos_sensor1);
    return angle;
}
double get_motor_angle2()
{
    double angle = wb_position_sensor_get_value(motor_pos_sensor2);
    return angle;
}

bool is_foot_touching()
{
    return wb_touch_sensor_get_value(touch_sensor);
}

double* robot_coordinates()
{
    double *x = wb_supervisor_node_get_position(this_robot);
    return x;
}