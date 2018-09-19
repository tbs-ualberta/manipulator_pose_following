#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "manipulator_pose_following/DeltaPoseRPY.h"
#include "manipulator_pose_following/SetVelocity.h"

#include <Eigen/Dense>

// --- Globals
double g_max_translat_acc, g_translat_acc;
double g_max_rot_acc, g_rot_acc;
ros::Time g_t_start, g_t_last;
manipulator_pose_following::DeltaPoseRPY g_dof;

void cb_cmd_vel_fct(const geometry_msgs::Twist::ConstPtr &msg) {
  // Clear the dof.
  g_dof.data.clear();
  g_dof.data.resize(6, 0);
  g_dof.data.at(0) = msg->linear.x;
  g_dof.data.at(1) = msg->linear.y;
  g_dof.data.at(2) = msg->linear.z;
  g_dof.data.at(3) = msg->angular.x;
  g_dof.data.at(4) = msg->angular.y;
  g_dof.data.at(5) = msg->angular.z;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "smooth_jogging");
  ros::NodeHandle n;

  ros::Publisher cmd_vel_pub =
      n.advertise<geometry_msgs::Twist>("pose_following/cmd_vel", 1);
  ros::Subscriber cmd_vel_sub =
      n.subscribe("smooth_jogging/cmd_vel", 1, cb_cmd_vel_fct);

  int rate_hz = 40;
  n.getParam("smooth_jogging/rate", rate_hz);
  ros::Rate loop_rate_hz(rate_hz);

  double dt = 0.1;

  g_max_translat_acc = .015;
  g_translat_acc = g_max_translat_acc;

  g_max_rot_acc = 0.05;
  g_rot_acc = g_max_rot_acc;
  double k_rotation = 0.1;
  double b_rotation = 0.95;

  Eigen::VectorXd cart_acc(6), cart_vel(6);
  Eigen::VectorXd vel_err(6), err_dot(6);
  Eigen::Vector3d ref_point(0.0, 0.0, 0.0);

  // initialize Cartesian delta vector;
  cart_acc.setZero();
  cart_vel.setZero();

  g_t_start = ros::Time::now();
  g_t_last = ros::Time::now();
  g_dof.data.resize(6, 0);

  while (ros::ok()) {

    // Time since last point:
    dt = (ros::Time::now() - g_t_last).toSec();
    g_t_last = ros::Time::now();

    // Compute command velocity
    Eigen::VectorXd vel_command(6);
    double sum_squares = 0;
    for (unsigned int i = 0; i < 3; i++) {
      sum_squares = sum_squares + g_dof.data.at(i) * g_dof.data.at(i);
    }
    double delta_mag = pow(sum_squares, 0.5);
    for (unsigned int i = 0; i < 3; i++) {
      if (delta_mag > 0.0)
        vel_command[i] = g_dof.data.at(i);
      else
        vel_command[i] = 0.0;
    }

    sum_squares = 0;
    for (unsigned int i = 3; i < 6; i++) {
      sum_squares = sum_squares + g_dof.data.at(i) * g_dof.data.at(i);
    }
    delta_mag = pow(sum_squares, 0.5);
    for (unsigned int i = 3; i < 6; i++) {
      if (delta_mag > 0.0)
        vel_command[i] = g_dof.data.at(i);
      else
        vel_command[i] = 0.0;
    }
    // Compute velocity error.
    err_dot = 1 / dt * ((vel_command - cart_vel) - vel_err);
    vel_err = vel_command - cart_vel;

    double err_mag = vel_err.norm();
    ROS_DEBUG_STREAM_NAMED("stream_dbg", "err_mag: " << err_mag);

    // Compute acceleration
    // This ensures smooth start-up and stop motion
    if (err_mag <= 0.015) {
      cart_vel = vel_command;
      cart_acc.setZero();
    } else {
      for (unsigned int i = 0; i < 3; i++) {
        cart_acc[i] = vel_err.normalized()[i] * g_translat_acc / dt;
      }
      for (unsigned int i = 3; i < 6; i++) {
        cart_acc[i] = vel_err.normalized()[i] * g_rot_acc / dt;
      }
      cart_vel = (cart_vel + dt * cart_acc);
    }

    if (cart_vel.norm() == 0)
      cart_vel.setZero();

    manipulator_pose_following::DeltaPoseRPY delta_pose;
    delta_pose.data.resize(6, 0);
    for (int i = 0; i < 6; i++) {
      delta_pose.data[i] = cart_vel[i];
    }

    geometry_msgs::Twist msg_twist;
    msg_twist.linear.x = delta_pose.data[0];
    msg_twist.linear.y = delta_pose.data[1];
    msg_twist.linear.z = delta_pose.data[2];
    msg_twist.angular.x = delta_pose.data[3];
    msg_twist.angular.y = delta_pose.data[4];
    msg_twist.angular.z = delta_pose.data[5];

    cmd_vel_pub.publish(msg_twist);
    loop_rate_hz.sleep();
    ros::spinOnce();
  }

  return (0);
}
