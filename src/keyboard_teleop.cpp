#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "manipulator_teleop/DeltaPoseRPY.h"
#include "manipulator_teleop/SetVelocity.h"

#include <curses.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <Eigen/Dense>

// --- Globals
double g_jogging_velocity;
double g_max_translat_acc, g_translat_acc;
double g_max_rot_acc, g_rot_acc;
ros::Time g_t_start, g_t_last;

// --- Function declarations
char my_getch(); // Non-blocking getch()

bool cb_set_vel(manipulator_teleop::SetVelocity::Request &req,
                manipulator_teleop::SetVelocity::Response &res) {

  if (req.velocity < 0 || req.velocity > 1.0) {
    ROS_WARN(
        "Invalid jogging velocity requested. Value must be on interval [0,1].");
    res.result = false;
  } else {
    g_jogging_velocity = req.velocity;
    g_translat_acc = req.velocity * g_max_translat_acc;
    g_rot_acc = req.velocity * g_max_rot_acc;
    res.result = true;
  }
  return res.result;
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle n;
  ros::NodeHandle nh_set_vel;
  ros::CallbackQueue queue_set_vel;
  nh_set_vel.setCallbackQueue(&queue_set_vel);
  ros::AsyncSpinner spin_set_vel(1, &queue_set_vel);
  spin_set_vel.start();

  ros::Publisher delta_pub =
      n.advertise<manipulator_teleop::DeltaPoseRPY>("teleop/delta_pose_rpy", 1);
  manipulator_teleop::DeltaPoseRPY dof;

  ros::ServiceServer srv_set_velocity =
      nh_set_vel.advertiseService("/keyboard_teleop/set_velocity", cb_set_vel);
  ROS_INFO("Keyboard teleop online.");

  // TODO Many of these parameters should be set on the parameter server, not
  //      hard coded. Especially the loop sampling time should be enforced using
  //      ros::rate.sleep().
  double dt = 0.1;
  g_jogging_velocity = 0.4;

  double cart_translation_vel = .75;
  g_max_translat_acc = .015;
  g_translat_acc = g_max_translat_acc;

  double cart_rotation_vel = 1.5;
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
  // streaming_pub.publish(dummy_traj);

  g_t_start = ros::Time::now();
  g_t_last = ros::Time::now();
  dof.data.resize(6, 0);

  ros::Rate loop_rate_hz(100);

  bool stop = false;
  // use system call to make terminal send all keystrokes directly to stdin
  //system("/bin/stty raw");

  initscr();
  while (ros::ok && !stop) {
    //nodelay(stdscr, TRUE);
    noecho();
    cbreak();

    // Clear the dof.
    dof.data.clear();
    dof.data.resize(6, 0);

    // TODO Put this in a separate thread. Blocking is not good here as it
    //      affects the calculation of a smooth start-and stop velocity
    char command = getch();
    loop_rate_hz.sleep();
    // ros::Duration(0.1).sleep();
    switch (command) {
    case '1':
      dof.data.at(0) = 1.0;
      break;
    case '2':
      dof.data.at(1) = 1.0;
      break;
    case '3':
      dof.data.at(2) = 1.0;
      break;
    case '4':
      dof.data.at(3) = 1.0;
      break;
    case '5':
      dof.data.at(4) = 1.0;
      break;
    case '6':
      dof.data.at(5) = 1.0;
      break;
    case 'q':
      dof.data.at(0) = -1.0;
      break;
    case 'w':
      dof.data.at(1) = -1.0;
      break;
    case 'e':
      dof.data.at(2) = -1.0;
      break;
    case 'r':
      dof.data.at(3) = -1.0;
      break;
    case 't':
      dof.data.at(4) = -1.0;
      break;
    case 'y':
      dof.data.at(5) = -1.0;
      break;
    case 'x':
      stop = true;
    case ERR:
      break;
    default:
      break;
    }

    // Time since last point:
    dt = (ros::Time::now() - g_t_last).toSec();
    g_t_last = ros::Time::now();

    // Compute command velocity
    Eigen::VectorXd vel_command(6);
    double sum_squares = 0;
    for (unsigned int i = 0; i < 3; i++) {
      sum_squares = sum_squares + dof.data.at(i) * dof.data.at(i);
    }
    double delta_mag = pow(sum_squares, 0.5);
    for (unsigned int i = 0; i < 3; i++) {
      if (delta_mag > 0.0)
        vel_command[i] = dof.data.at(i) * g_jogging_velocity *
                         cart_translation_vel;
      else
        vel_command[i] = 0.0;
    }

    sum_squares = 0;
    for (unsigned int i = 3; i < 6; i++) {
      sum_squares = sum_squares + dof.data.at(i) * dof.data.at(i);
    }
    delta_mag = pow(sum_squares, 0.5);
    for (unsigned int i = 3; i < 6; i++) {
      if (delta_mag > 0.0)
        vel_command[i] =
            dof.data.at(i) * g_jogging_velocity * cart_rotation_vel;
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

    manipulator_teleop::DeltaPoseRPY delta_pose;
    delta_pose.data.resize(6, 0);
    for (int i = 0; i < 6; i++) {
      delta_pose.data[i] = cart_vel[i];
    }

    delta_pub.publish(delta_pose);
  }
  // use system call to set terminal behaviour to more normal behaviour
  //system("/bin/stty cooked");
  endwin();

  return (0);
}

char my_getch() {
  // https://answers.ros.org/question/63491/keyboard-key-pressed/
  fd_set set;
  struct timeval timeout;
  int rv;
  char buff = 0;
  int len = 1;
  int filedesc = 0;
  FD_ZERO(&set);
  FD_SET(filedesc, &set);

  timeout.tv_sec = 0;
  timeout.tv_usec = 1000;

  rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

  struct termios old = {0};
  if (tcgetattr(filedesc, &old) < 0)
    printf("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag |= ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(filedesc, TCSANOW, &old) < 0)
    printf("tcsetattr ICANON");

  if (rv == -1)
    printf("select");
  else if (rv == 0)
    buff = -1;
  else
    read(filedesc, &buff, len);

  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
    printf("tcsetattr ~ICANON");
  return (buff);
}
