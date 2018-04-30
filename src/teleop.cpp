
// TODO Implement a service that starts teleoperation (start_teleop).
// TODO Implement a service that stops teleoperation (stop_teleop).

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include "manipulator_teleop/DeltaPoseRPY.h"
#include "manipulator_teleop/StartStopTeleop.h"

// --- Globals
// TODO Remove global variables if not necessary
bool g_do_teleop = false;
sensor_msgs::JointState current_joints;
std::map<std::string, int> joint_name_map;
moveit::core::RobotStatePtr kinematic_state;
trajectory_msgs::JointTrajectory dummy_traj;
trajectory_msgs::JointTrajectoryPoint point;
const moveit::core::JointModelGroup *joint_model_group;
ros::Time start_time, time_last, last_callback;
// TODO Make below message a stamped one
manipulator_teleop::DeltaPoseRPY g_delta_pose;
double jogging_velocity;
double max_translation_acc, translation_acc;
double max_rotation_acc, rotation_acc;
const int STATE_IDLE = 0;
const int STATE_TELEOP = 1;
int g_state = STATE_IDLE;

Eigen::MatrixXd invert(Eigen::MatrixXd J) {
  // TODO Test jacobian condition and conditionally add "damping"
  Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
  return (J.transpose() * (J * J.transpose() + .0000001 * I).inverse());
}

void cb_joint_state(sensor_msgs::JointState msg) { current_joints = msg; }

bool cb_start_teleop(manipulator_teleop::StartStopTeleop::Request &req,
                     manipulator_teleop::StartStopTeleop::Response &res) {

  ROS_INFO("/teleop/start signal received.");
  g_do_teleop = true;
  g_state = STATE_TELEOP;
  res.reply = 0;
  return 0;
}

bool cb_stop_teleop(manipulator_teleop::StartStopTeleop::Request &req,
                    manipulator_teleop::StartStopTeleop::Response &res) {

  ROS_INFO("/teleop/stop signal received.");
  g_do_teleop = false;
  g_state = STATE_IDLE;
  res.reply = 0;
  return 0;
}

void cb_delta_pose_rpy(const manipulator_teleop::DeltaPoseRPY::ConstPtr &msg) {

  last_callback = ros::Time::now();
  // Check valid deltas:
  if (msg->delta_pose_rpy.size() != 6) {
    ROS_ERROR("Delta pose must be of size 6 (position, orientation (RPY)). "
              "Ignoring message.");
    return;
  }
  // TODO make 'DeltaPoseRPY' a stamped message (header) to merge the timestamp
  //      last_callback into it.
  g_delta_pose = *msg;
  ROS_DEBUG_NAMED(
      "stream_dpose", "new_deltas = [%1.1f %1.1f %1.1f %1.1f %1.1f %1.1f]",
      g_delta_pose.delta_pose_rpy.at(0), g_delta_pose.delta_pose_rpy.at(1),
      g_delta_pose.delta_pose_rpy.at(2), g_delta_pose.delta_pose_rpy.at(3),
      g_delta_pose.delta_pose_rpy.at(4), g_delta_pose.delta_pose_rpy.at(5));
}

int main(int argc, char **argv) {

  // --- Initializations
  ros::init(argc, argv, "teleop");

  // --- Setup node handles for
  //     - teleoperation callbacks
  ros::NodeHandle nh_teleop;
  ros::NodeHandle nh_startstop;

  // --- Setup custom callback queues for
  //     - start_teleop and stop_teleop callbacks
  ros::CallbackQueue queue_startstop;
  nh_startstop.setCallbackQueue(&queue_startstop);

  // --- Start an AsyncSpinner with one thread for calls to services
  //     'start_teleop' and 'stop_teleop'.
  // TODO does this thread need to be stopped when main() is left?
  ros::AsyncSpinner spin_startstop(1, &queue_startstop);
  spin_startstop.start();

  // --- Setup publishers
  ros::Publisher streaming_pub =
      nh_teleop.advertise<trajectory_msgs::JointTrajectory>("joint_command",
                                                            10);

  // --- Setup topic subscritions
  ros::Subscriber sub_pose =
      nh_teleop.subscribe("teleop/delta_pose_rpy", 1, cb_delta_pose_rpy);
  ros::Subscriber sub_joint =
      nh_teleop.subscribe("joint_states", 1, cb_joint_state);

  // --- Advertise services
  //     - start_teleop
  //     - stop_teleop
  ros::ServiceServer srv_start =
      nh_teleop.advertiseService("/teleop/start", cb_start_teleop);
  ros::ServiceServer srv_stop =
      nh_teleop.advertiseService("/teleop/stop", cb_stop_teleop);

  // --- Obtain parameters
  int rate_hz = 10;
  nh_teleop.getParam("teleop/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);

  std::string group_st = "manipulator";
  nh_teleop.getParam("teleop/group", group_st);

  // --- Setup MoveIt interface
  ROS_DEBUG("Setting up MoveGroupInterface...");
  moveit::planning_interface::MoveGroupInterface arm(group_st);
  ROS_DEBUG("Done!");

  // ---------------------------------------------------------------------------

  ROS_DEBUG("Setting up teleoperation...");
  double dt = 0.1;
  double move_timeout = 2.0;
  jogging_velocity = 0.4;

  double cart_translation_vel = .75;
  max_translation_acc = .015;
  translation_acc = max_translation_acc;
  double k_translation = 0.1;
  double b_translation = 0.95;

  double cart_rotation_vel = 1.5;
  max_rotation_acc = 0.05;
  rotation_acc = max_rotation_acc;
  double k_rotation = 0.1;
  double b_rotation = 0.95;

  double jacobian_condition = 1.0;

  Eigen::VectorXd d_X(6), d_theta(7), cart_acc(6), cart_vel(6), d_theta_tmp(7);
  Eigen::VectorXd vel_err(6), err_dot(6);
  Eigen::MatrixXd J(6, 7);
  Eigen::Vector3d ref_point(0.0, 0.0, 0.0);

  // initialize Cartesian delta vector;
  cart_acc.setZero();
  cart_vel.setZero();
  d_X.setZero();

  // Get set up for kinematics:
  robot_model_loader::RobotModelLoader model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = model_loader.getModel();
  ROS_INFO("Model Frame: %s", kinematic_model->getModelFrame().c_str());

  // kinematic_state = new moveit::core::RobotState(kinematic_model);
  // kinematic_state = std::shared_ptr<moveit::core::RobotState>(
  //    new moveit::core::RobotState(kinematic_model));
  moveit::core::RobotState kinematic_state(kinematic_model);
  // kinematic_state = boost::shared_ptr<moveit::core::RobotState>(new
  // moveit::core::RobotState(kinematic_model));
  kinematic_state.setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup(group_st);

  const std::vector<std::string> &joint_names =
      joint_model_group->getJointModelNames();
  std::vector<double> joint_values;
  kinematic_state.copyJointGroupPositions(joint_model_group, joint_values);
  ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
  ROS_DEBUG("joint_names: [%s %s %s %s %s %s %s]", joint_names[0].c_str(),
            joint_names[1].c_str(), joint_names[2].c_str(),
            joint_names[3].c_str(), joint_names[4].c_str(),
            joint_names[5].c_str(), joint_names[6].c_str());
  ROS_DEBUG("joint_values: [%1.2f %1.2f %1.2f %1.2f %1.2f %1.2f %1.2f]",
            joint_values[0], joint_values[1], joint_values[2], joint_values[3],
            joint_values[4], joint_values[5], joint_values[6]);
  Eigen::Affine3d frame_tf = kinematic_state.getFrameTransform("link_t");
  std::ostream stream(nullptr);
  std::stringbuf str;
  stream.rdbuf(&str);
  kinematic_state.printTransform(frame_tf, stream);
  // std::cout << "transform: \n" << str.str() << endl;
  ROS_DEBUG_STREAM("transform: " << str.str());
  std::ostream stream2(nullptr);
  std::stringbuf str2;
  stream2.rdbuf(&str2);
  joint_model_group->printGroupInfo(stream2);
  ROS_DEBUG_STREAM("joint_model_group_info: " << str2.str());

  // Initialize jogging start.
  dummy_traj.joint_names = current_joints.name;
  for (unsigned int joint = 0; joint < 7; joint++) {
    point.positions.push_back(current_joints.position.at(joint));
    point.velocities.push_back(0);
  }
  point.time_from_start = ros::Duration(0.0);
  dummy_traj.points.push_back(point);
  streaming_pub.publish(dummy_traj);

  start_time = ros::Time::now();
  time_last = ros::Time::now();
  g_delta_pose.delta_pose_rpy.resize(6, 0);

  ROS_DEBUG("Done!");

  // ---------------------------------------------------------------------------

  while (ros::ok()) {
    switch (g_state) {
    case STATE_IDLE:
      break;
    case STATE_TELEOP:
      //------------------------------------------------------------------------

      if ((ros::Time::now() - last_callback).toSec() > .1) {
        for (unsigned int i = 0; i < 6; i++) {
          g_delta_pose.delta_pose_rpy.at(i) = 0.0;
        }
      }

      Eigen::Affine3d frame_tf = kinematic_state.getFrameTransform("link_t");
      std::ostream stream(nullptr);
      std::stringbuf str;
      stream.rdbuf(&str);
      kinematic_state.printTransform(frame_tf, stream);
      // std::cout << "transform: \n" << str.str() << endl;
      ROS_DEBUG_STREAM_NAMED("stream_tf", "transform: " << str.str());

      // Time since last point:
      // ros::topic::waitForMessage<sensor_msgs::JointState>("joint_states");
      dt = (ros::Time::now() - time_last).toSec();
      time_last = ros::Time::now();
      kinematic_state.setVariableValues(current_joints);

      // Compute command velocity
      Eigen::VectorXd vel_command(6);
      double sum_squares = 0;
      for (unsigned int i = 0; i < 3; i++) {
        sum_squares = sum_squares +
                      g_delta_pose.delta_pose_rpy.at(i) *
                          g_delta_pose.delta_pose_rpy.at(i);
      }
      double delta_mag = pow(sum_squares, 0.5);
      for (unsigned int i = 0; i < 3; i++) {
        if (delta_mag > 0.0)
          vel_command[i] = g_delta_pose.delta_pose_rpy.at(i) / delta_mag *
                           jogging_velocity * cart_translation_vel;
        else
          vel_command[i] = 0.0;
      }

      sum_squares = 0;
      for (unsigned int i = 3; i < 6; i++) {
        sum_squares = sum_squares +
                      g_delta_pose.delta_pose_rpy.at(i) *
                          g_delta_pose.delta_pose_rpy.at(i);
      }
      delta_mag = pow(sum_squares, 0.5);
      for (unsigned int i = 3; i < 6; i++) {
        if (delta_mag > 0.0)
          vel_command[i] = g_delta_pose.delta_pose_rpy.at(i) / delta_mag *
                           jogging_velocity * cart_rotation_vel;
        else
          vel_command[i] = 0.0;
      }
      // std::cout << "vel_command norm: " << vel_command.norm() << std::endl;
      // Compute velocity error.
      err_dot = 1 / dt * ((vel_command - cart_vel) - vel_err);
      vel_err = vel_command - cart_vel;

      double err_mag = vel_err.norm();
      // std::cout << "vel_error norm: " << vel_err.squaredNorm() << std::endl;

      // Compute acceleration
      if (err_mag <= 0.015) {
        cart_vel = vel_command;
        cart_acc.setZero();
      } else {
        for (unsigned int i = 0; i < 3; i++) {
          cart_acc[i] = vel_err.normalized()[i] * translation_acc / dt;
        }
        for (unsigned int i = 3; i < 6; i++) {
          cart_acc[i] = vel_err.normalized()[i] * rotation_acc / dt;
        }
        cart_vel = (cart_vel + dt * cart_acc);
      }

      if (cart_vel.norm() == 0)
        cart_vel.setZero();

      // d_X = cart_vel*dt;
      d_X = cart_vel;
      // std::cout << "accel:\n" << cart_acc << std::endl;
      // std::cout << "err_dot:\n" << err_dot << std::endl;
      // std::cout << "vel:\n" << cart_vel << std::endl;
      // std::cout <<" velocity norm: " << cart_vel.norm() <<std::endl;
      // std::cout << "velocity:\n" << cart_vel << std::endl;
      // std::cout << "d_X:\n" << d_X << std::endl;
      // std::cout << "vel_err.norm: " << vel_err.norm() << std::endl;
      ROS_DEBUG_NAMED("stream_cart", "accel: %1.2f", cart_acc[0]);
      ROS_DEBUG_NAMED("stream_cart", "vel: %1.2f", cart_vel[0]);
      ROS_DEBUG_NAMED("stream_cart", "err_dot: %1.2f", err_dot[0]);

      // Get the Jacobian
      kinematic_state.getJacobian(
          joint_model_group, kinematic_state.getLinkModel(
                                 joint_model_group->getLinkModelNames().back()),
          ref_point, J);
      d_theta = invert(J) * d_X;
      // d_theta_tmp.setZero();
      // d_theta_tmp(0) = d_X(0);
      // d_theta = d_theta_tmp;
      ROS_DEBUG_STREAM_ONCE("J: \n" << J);
      ROS_DEBUG_STREAM_NAMED("stream_d_theta", "d_theta: \n" << d_theta);
      ROS_DEBUG_STREAM_NAMED("stream_d_X", "d_X: \n" << d_X);

      // check condition number:
      jacobian_condition = J.norm() * invert(J).norm();
      ROS_DEBUG_NAMED("stream_J_cnd", "Jacobian condition: %e",
                      jacobian_condition);
      // std::cout << "Jacobian condition: " << jacobian_condition << std::endl;
      /*if(jacobian_condition > 39)
      {
        ROS_ERROR("Cannot do Cartesian Jogging due to ill-conditioned
      Jacobian");
        ROS_DEBUG_STREAM_NAMED("foobar", "J: \n" << J);
        ROS_DEBUG_NAMED("foobar", "Jacobian condition: %e", jacobian_condition);
        //delete &kinematic_state;
        //delete &kinematic_model;
        return 0;
      }*/

      // std::cout << "d_theta:\n" << d_theta << std::endl;
      for (unsigned int j = 0; j < 7; j++) {
        // point.positions.at(j) = current_joints.position.at(j) + d_theta[j];
        point.positions.at(j) = current_joints.position.at(j) + d_theta[j] * dt;
        // point.velocities.at(j) = d_theta[j]/dt;
        point.velocities.at(j) = d_theta[j];
      }

      point.time_from_start = ros::Time::now() - start_time;
      dummy_traj.points.at(0) = point;
      streaming_pub.publish(dummy_traj);
      ros::Duration(0.01).sleep();

      //--------------------------------------------------------------------------
    } // end switch(g_state)

    // FIXME This is a blocking call that should be avoided in the future
    ros::spinOnce();
  } // end while(ros::ok())
  return 1;
}
