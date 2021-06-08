#include <stdio.h>
#include <stdlib.h>

#include <webots/keyboard.h>
#include <webots/robot.h>

#include "b_controller.h"
#include "b_webotsInterface.h"

int main(int argc, char **argv)
{
  wb_robot_init();
  webots_device_init(); //webots initialization
  robot_init();         //robot initialization
  while (wb_robot_step(TIME_STEP) != -1)
  {
    updateRobotState(); //update the current state of the robot
    robot_control();    //controlling the robot
  };
  robot_free(); //free the memory
  wb_robot_cleanup();
  return 0;
}