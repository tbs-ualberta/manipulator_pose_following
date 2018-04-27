#include <ros/ros.h>

#include "manipulator_teleop/DeltaPoseRPY.h"

#include <curses.h>
#include <iostream>
#include <stdio.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle n;

  ros::Publisher delta_pub = n.advertise<manipulator_teleop::DeltaPoseRPY>(
      "teleop/delta_pose_rpy", 1);
  manipulator_teleop::DeltaPoseRPY deltas;
  ROS_INFO("Keyboard teleop online.");

  // system("stty raw");

  bool stop = false;
  initscr();
  while (ros::ok && !stop) {
    // Clear the deltas.
    deltas.delta_pose_rpy.clear();
    deltas.delta_pose_rpy.resize(6, 0);
    initscr();
    // nodelay(stdscr, TRUE);
    noecho();
    cbreak();

    char command = getch();
    // ros::Duration(0.1).sleep();
    // std::cout << command << std::endl;
    switch (command) {
    case '1':
      deltas.delta_pose_rpy.at(0) = 1.0;
      break;
    case '2':
      deltas.delta_pose_rpy.at(1) = 1.0;
      break;
    case '3':
      deltas.delta_pose_rpy.at(2) = 1.0;
      break;
    case '4':
      deltas.delta_pose_rpy.at(3) = 1.0;
      break;
    case '5':
      deltas.delta_pose_rpy.at(4) = 1.0;
      break;
    case '6':
      deltas.delta_pose_rpy.at(5) = 1.0;
      break;
    case 'q':
      deltas.delta_pose_rpy.at(0) = -1.0;
      break;
    case 'w':
      deltas.delta_pose_rpy.at(1) = -1.0;
      break;
    case 'e':
      deltas.delta_pose_rpy.at(2) = -1.0;
      break;
    case 'r':
      deltas.delta_pose_rpy.at(3) = -1.0;
      break;
    case 't':
      deltas.delta_pose_rpy.at(4) = -1.0;
      break;
    case 'y':
      deltas.delta_pose_rpy.at(5) = -1.0;
      break;
    case 'x':
      stop = true;
    case ERR:
      break;
    default:
      break;
    }
    delta_pub.publish(deltas);
    ros::spinOnce();
    // system("stty cooked");
  }

  endwin();
  return (0);
}
