#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "manipulator_teleop/DeltaPoseRPY.h"
#include "manipulator_teleop/SetVelocity.h"

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
char getch_async(); // Non-blocking getch_async()

int main(int argc, char **argv) {

  ros::init(argc, argv, "kb_jogging");
  ros::NodeHandle n;

  ros::Publisher delta_pub =
      n.advertise<manipulator_teleop::DeltaPoseRPY>("teleop/delta_pose_rpy", 1);
  manipulator_teleop::DeltaPoseRPY dof;

  ROS_INFO("Keyboard teleop online.");
  ROS_INFO_STREAM(
      "\n"
      << "\n"
      << "Key assignments:\n"
      << "\n"
      << "Translation: +x: \"1\" | +y: \"2\" | +z: \"3\"\n"
      << "             -x: \"q\" | -y: \"w\" | -z: \"e\"\n"
      << "\n"
      << "Rotation:    +x: \"4\" | +y: \"5\" | +z: \"6\"\n"
      << "             -x: \"r\" | -y: \"t\" | -z: \"y\"\n"
      << "\n"
      << "Velocity:     +: \"0\"\n"
      << "              -: \"p\"\n"
      << "\n"
      << "Stop:            \"x\""
  );

  // --- Get params from parameter server
  g_jogging_velocity = 0.1;
  n.getParam("kb_jogging/jogging_vel", g_jogging_velocity);

  int rate_hz = 40;
  n.getParam("kb_jogging/rate", rate_hz);
  ros::Rate loop_rate_hz(rate_hz);

  const double max_cart_translation_vel = .5;
  double cart_translation_vel = max_cart_translation_vel;
  n.getParam("kb_jogging/max_trans_vel", cart_translation_vel);
  if(cart_translation_vel > max_cart_translation_vel){
    cart_translation_vel = max_cart_translation_vel;
  }

  const double max_cart_rotation_vel = 1;
  double cart_rotation_vel = max_cart_rotation_vel;
  n.getParam("kb_jogging/max_rot_vel", cart_rotation_vel);
  if(cart_rotation_vel > max_cart_rotation_vel){
    cart_rotation_vel = max_cart_rotation_vel;
  }

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
  // streaming_pub.publish(dummy_traj);

  g_t_start = ros::Time::now();
  g_t_last = ros::Time::now();
  dof.data.resize(6, 0);

  bool stop = false;

  system("/bin/stty raw");   // Raw mode (send all keystrokes directly to stdin)
  system("/bin/stty -echo"); // Turn off echo

  while (ros::ok && !stop) {

    // Clear the dof.
    dof.data.clear();
    dof.data.resize(6, 0);

    // TODO Add buttons for jogging speed adjustment
    char command = getch_async();
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
    case '0':
      g_jogging_velocity += 0.1;
      if (g_jogging_velocity > 1){
        g_jogging_velocity = 1;
      }
      ROS_INFO("Jogging velocity factor: %1.1f\r", g_jogging_velocity);
      break;
    case 'p':
      g_jogging_velocity -= 0.1;
      if (g_jogging_velocity < 0){
        g_jogging_velocity = 0;
      }
      ROS_INFO("Jogging velocity factor: %1.1f\r", g_jogging_velocity);
      break;
    case 'x':
      stop = true;
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
    loop_rate_hz.sleep();
  }
  // use system call to set terminal behaviour to more normal behaviour
  system("/bin/stty cooked");
  system("/bin/stty echo");

  return (0);
}

char getch_async() {
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

  struct termios oldattr = {0};
  struct termios newattr = {0};
  if (tcgetattr(filedesc, &oldattr) < 0)
    printf("tcsetattr()");

  newattr = oldattr;
  newattr.c_lflag &= ~ICANON;
  newattr.c_lflag &= ~ECHO;
  newattr.c_lflag |= ECHONL;
  newattr.c_lflag |= (OCRNL | ONLCR);
  newattr.c_cc[VMIN] = 1;
  newattr.c_cc[VTIME] = 0;
  if (tcsetattr(filedesc, TCSANOW, &newattr) < 0)
    printf("tcsetattr ICANON");

  if (rv == -1)
    printf("select");
  else if (rv == 0)
    buff = -1;
  else
    read(filedesc, &buff, len);

  if (tcsetattr(filedesc, TCSADRAIN, &oldattr) < 0)
    printf("tcsetattr ~ICANON");
  return (buff);
}
