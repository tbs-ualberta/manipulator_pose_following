
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <tf/tf.h>

#include "manipulator_teleop/MoveHome.h"
#include "manipulator_teleop/MoveToQuat.h"
#include "manipulator_teleop/MoveToRPY.h"
#include "manipulator_teleop/StartStopTeleop.h"

#include <vector>

static const std::string PLANNING_GROUP = "manipulator";
geometry_msgs::Pose g_pose_ref;

bool start_teleop() {
  // --- Stop teleop node (STATE_TELEOP --> STATE_IDLE)
  std::string srv_name = "/teleop/start";
  bool success = ros::service::exists(srv_name, true);
  if (success) {
    manipulator_teleop::StartStopTeleop::Request req;
    manipulator_teleop::StartStopTeleop::Response res;
    ros::service::call(srv_name, req, res);
    ros::service::waitForService(srv_name);
  }
  return success;
}

bool stop_teleop() {
  // --- Stop teleop node (STATE_TELEOP --> STATE_IDLE)
  std::string srv_name = "/teleop/stop";
  if (ros::service::exists(srv_name, true)) {
    manipulator_teleop::StartStopTeleop::Request req;
    manipulator_teleop::StartStopTeleop::Response res;
    ros::service::call(srv_name, req, res);
    ros::service::waitForService(srv_name);
  }
}

bool callbackMoveHome(manipulator_teleop::MoveHome::Request &req,
                      manipulator_teleop::MoveHome::Response &res) {

  // Stop the teleop-node (STATE_TELEOP --> STATE_IDLE)
  stop_teleop();

  // FIXME Velocity and acceleration limit does not seem to be working
  //       (changing it does not seem to have any effect on the robot's speed)
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
  move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);
  ROS_INFO("Max velocity scaling factor: %1.1f", req.max_vel_fact);
  ROS_INFO("Max acceleration scaling factor: %1.1f", req.max_acc_fact);

  std::vector<double> joint_angles = {0, 0, 0, 0, 0, 0, 0};
  move_group.setJointValueTarget(joint_angles);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success_plan = (move_group.plan(plan) ==
                       moveit::planning_interface::MoveItErrorCode::SUCCESS);

  bool success_exec = false;
  if (success_plan) {
    success_exec = (move_group.execute(plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

  // Start the teleop-node (STATE_STOP --> STATE_TELEOP)
  start_teleop();

  if (success_exec) {
    res.reply = 0;
    return true;
  } else {
    res.reply = -1;
    return false;
  }
}

bool callbackMoveToRPY(manipulator_teleop::MoveToRPY::Request &req,
                       manipulator_teleop::MoveToRPY::Response &res) {

  // Stop the teleop-node (STATE_TELEOP --> STATE_IDLE)
  stop_teleop();

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  ROS_INFO("Max velocity scaling factor: %1.1f", req.max_vel_fact);
  ROS_INFO("Max acceleration scaling factor: %1.1f", req.max_acc_fact);
  tf::Quaternion tf_q;
  tf_q.setEuler(req.pose_rpy[5], req.pose_rpy[4], req.pose_rpy[3]);
  geometry_msgs::Pose pose;
  pose.position.x = req.pose_rpy[0];
  pose.position.y = req.pose_rpy[1];
  pose.position.z = req.pose_rpy[2];
  pose.orientation.x = tf_q.getX();
  pose.orientation.y = tf_q.getY();
  pose.orientation.z = tf_q.getZ();
  pose.orientation.w = tf_q.getW();

  move_group.setPoseTarget(pose, "link_t");

  // NOTE There seems to be some issue with the setRPYTarget()-function of
  //      MoveGroupInterface. Supplying the same pose multiple times results in
  //      the robot changing its pose while it should not move at all.
  /*
  move_group.setPositionTarget(req.pose_rpy[0], req.pose_rpy[1],
                               req.pose_rpy[2], "");
  move_group.setRPYTarget(req.pose_rpy[3], req.pose_rpy[4], req.pose_rpy[5],
                          "");*/
  ROS_INFO(
      "Planning and executing pose goal [%1.2f %1.2f %1.2f %1.2f %1.2f %1.2f]",
      req.pose_rpy[0], req.pose_rpy[1], req.pose_rpy[2], req.pose_rpy[3],
      req.pose_rpy[4], req.pose_rpy[5]);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success_plan = (move_group.plan(plan) ==
                       moveit::planning_interface::MoveItErrorCode::SUCCESS);

  bool success_exec = false;
  if (success_plan) {
    success_exec = (move_group.execute(plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

  // Start the teleop-node (STATE_STOP --> STATE_IDLE)
  start_teleop();

  if (success_exec) {
    res.reply = 0;
    return true;
  } else {
    res.reply = -1;
    return false;
  }
}

bool callbackMoveToQuat(manipulator_teleop::MoveToQuat::Request &req,
                        manipulator_teleop::MoveToQuat::Response &res) {

  // Stop the teleop-node (STATE_TELEOP --> STATE_IDLE)
  stop_teleop();

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  ROS_INFO("Max velocity scaling factor: %1.1f", req.max_vel_fact);
  ROS_INFO("Max acceleration scaling factor: %1.1f", req.max_acc_fact);

  geometry_msgs::Pose pose;
  pose.position.x = req.pose_quat[0];
  pose.position.y = req.pose_quat[1];
  pose.position.z = req.pose_quat[2];
  pose.orientation.x = req.pose_quat[3];
  pose.orientation.y = req.pose_quat[4];
  pose.orientation.z = req.pose_quat[5];
  pose.orientation.w = req.pose_quat[6];
  move_group.setPoseTarget(pose, "link_t");
  ROS_INFO("Planning and executing pose goal [%1.2f %1.2f %1.2f %1.2f %1.2f "
           "%1.2f  %1.2f]",
           req.pose_quat[0], req.pose_quat[1], req.pose_quat[2],
           req.pose_quat[3], req.pose_quat[4], req.pose_quat[5],
           req.pose_quat[6]);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success_plan = (move_group.plan(plan) ==
                       moveit::planning_interface::MoveItErrorCode::SUCCESS);

  bool success_exec = false;
  if (success_plan) {
    success_exec = (move_group.execute(plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }

  // Start the teleop-node (STATE_STOP --> STATE_IDLE)
  start_teleop();

  if (success_exec) {
    res.reply = 0;
    return true;
  } else {
    res.reply = -1;
    return false;
  }
}

int main(int argc, char **argv) {

  // --- Initializations
  ros::init(argc, argv, "moveit_interface");

  // --- Setup node handles for
  //     - move_home and move_to service callbacks
  ros::NodeHandle nh_move;

  // --- Setup custom callback queues for
  //     - move_home and move_to service callbacks
  ros::CallbackQueue queue_move;
  nh_move.setCallbackQueue(&queue_move);

  // --- Start an AsyncSpinner with two threads for
  //     - calls to service 'move_home' and
  //     - trajectory planning and execution (moveit_group needs its own async
  //       spinner)
  ros::AsyncSpinner spin_move(1, &queue_move);
  spin_move.start();
  ros::AsyncSpinner spin_plan(1);
  spin_plan.start();

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // --- Advertise services
  //     - move_home
  //     - move_to
  ros::ServiceServer srv_move_home =
      nh_move.advertiseService("/moveit_interface/move_home", callbackMoveHome);
  ros::ServiceServer srv_move_to_rpy = nh_move.advertiseService(
      "/moveit_interface/move_to_rpy", callbackMoveToRPY);
  ros::ServiceServer srv_move_to_quat = nh_move.advertiseService(
      "/moveit_interface/move_to_quat", callbackMoveToQuat);

  while (ros::ok()) {
    ros::Duration(0.1).sleep();
  }
}
