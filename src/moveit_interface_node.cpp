#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <tf/tf.h>

#include "manipulator_pose_following/MoveHome.h"
#include "manipulator_pose_following/MoveToQuat.h"
#include "manipulator_pose_following/MoveToRPY.h"
#include "manipulator_pose_following/ReplyInt.h"

#include <vector>

static const std::string PLANNING_GROUP = "manipulator";
geometry_msgs::Pose g_pose_ref;
bool g_success_plan = false;
moveit::planning_interface::MoveGroupInterface::Plan g_plan;

bool start_pose_following() {
  // --- Stop pose_following node (STATE_POSE_FOLLOW --> STATE_IDLE)
  std::string srv_name = "/pose_following/start";
  bool success = ros::service::exists(srv_name, true);
  if (success) {
    manipulator_pose_following::ReplyInt::Request req;
    manipulator_pose_following::ReplyInt::Response res;
    ros::service::call(srv_name, req, res);
    ros::service::waitForService(srv_name);
  }
  return success;
}

bool stop_pose_following() {
  // --- Stop pose_following node (STATE_POSE_FOLLOW --> STATE_IDLE)
  std::string srv_name = "/pose_following/stop";
  if (ros::service::exists(srv_name, true)) {
    manipulator_pose_following::ReplyInt::Request req;
    manipulator_pose_following::ReplyInt::Response res;
    ros::service::call(srv_name, req, res);
    ros::service::waitForService(srv_name);
  }
}

bool callbackMoveHome(manipulator_pose_following::MoveHome::Request &req,
                      manipulator_pose_following::MoveHome::Response &res) {

  // Stop the pose_following-node (STATE_POSE_FOLLOW --> STATE_IDLE)
  stop_pose_following();

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

  // Start the pose_following-node (STATE_STOP --> STATE_POSE_FOLLOW)
  start_pose_following();

  if (success_exec) {
    res.reply = 0;
    return true;
  } else {
    res.reply = -1;
    return false;
  }
}

bool callbackPlanToRPY(manipulator_pose_following::MoveToRPY::Request &req,
                       manipulator_pose_following::MoveToRPY::Response &res) {

  tf::Quaternion tf_q;
  // NOTE The below assignment for RPY is strange:
  //      yaw -> Y, pitch -> X, roll -> Z
  tf_q.setEuler(req.pose_rpy[4], req.pose_rpy[3], req.pose_rpy[5]);
  geometry_msgs::Pose pose;
  pose.position.x = req.pose_rpy[0];
  pose.position.y = req.pose_rpy[1];
  pose.position.z = req.pose_rpy[2];
  pose.orientation.x = tf_q.getX();
  pose.orientation.y = tf_q.getY();
  pose.orientation.z = tf_q.getZ();
  pose.orientation.w = tf_q.getW();

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setPoseTarget(pose, "link_t");
  move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
  move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);

  g_success_plan = (move_group.plan(g_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (g_success_plan) {
    res.reply = 0;
  } else {
    res.reply = -1;
  }
  return g_success_plan;
}

bool callbackPlanToQuat(manipulator_pose_following::MoveToQuat::Request &req,
                        manipulator_pose_following::MoveToQuat::Response &res) {

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
  move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
  move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);

  g_success_plan = (move_group.plan(g_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (g_success_plan) {
    res.reply = 0;
  } else {
    res.reply = -1;
  }
  return g_success_plan;
}

bool callbackExecutePlan(manipulator_pose_following::ReplyInt::Request &req,
                         manipulator_pose_following::ReplyInt::Response &res) {

  // Stop the pose_following-node (STATE_POSE_FOLLOW --> STATE_IDLE)
  stop_pose_following();

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  bool success_exec = false;
  ROS_DEBUG("Planned successfully: %s", (g_success_plan) ? "true" : "false");
  if (g_success_plan) {
    success_exec = (move_group.execute(g_plan) ==
                    moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  ROS_DEBUG("Executed successfully: %s", (success_exec) ? "true" : "false");

  // Start the pose_following-node (STATE_STOP --> STATE_IDLE)
  start_pose_following();

  if (success_exec) {
    res.reply = 0;
    return true;
  } else {
    res.reply = -1;
    return false;
  }
}

bool callbackMoveToRPY(manipulator_pose_following::MoveToRPY::Request &req,
                       manipulator_pose_following::MoveToRPY::Response &res) {

  // Stop the pose_following-node (STATE_POSE_FOLLOW --> STATE_IDLE)
  stop_pose_following();

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  ROS_INFO("Max velocity scaling factor: %1.1f", req.max_vel_fact);
  ROS_INFO("Max acceleration scaling factor: %1.1f", req.max_acc_fact);
  tf::Quaternion tf_q;
  tf_q.setEuler(req.pose_rpy[4], req.pose_rpy[3], req.pose_rpy[5]);
  geometry_msgs::Pose pose;
  pose.position.x = req.pose_rpy[0];
  pose.position.y = req.pose_rpy[1];
  pose.position.z = req.pose_rpy[2];
  pose.orientation.x = tf_q.getX();
  pose.orientation.y = tf_q.getY();
  pose.orientation.z = tf_q.getZ();
  pose.orientation.w = tf_q.getW();

  move_group.setPoseTarget(pose, "link_t");
  move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
  move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);

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

  // Start the pose_following-node (STATE_STOP --> STATE_IDLE)
  start_pose_following();

  if (success_exec) {
    res.reply = 0;
    return true;
  } else {
    res.reply = -1;
    return false;
  }
}

bool callbackMoveToQuat(manipulator_pose_following::MoveToQuat::Request &req,
                        manipulator_pose_following::MoveToQuat::Response &res) {

  // Stop the pose_following-node (STATE_POSE_FOLLOW --> STATE_IDLE)
  stop_pose_following();

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
  move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
  move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);
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

  // Start the pose_following-node (STATE_STOP --> STATE_IDLE)
  start_pose_following();

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

  // --- Advertise services
  //     - move_home
  //     - plan_to_rpy
  //     - plan_to_quat
  //     - execute_plan
  //     - move_to_rpy
  //     - move_to_quat
  ros::ServiceServer srv_move_home =
      nh_move.advertiseService("/moveit_interface/move_home", callbackMoveHome);
  ros::ServiceServer srv_plan_to_rpy = nh_move.advertiseService(
      "/moveit_interface/plan_to_rpy", callbackPlanToRPY);
  ros::ServiceServer srv_plan_to_quat = nh_move.advertiseService(
      "/moveit_interface/plan_to_quat", callbackPlanToQuat);
  ros::ServiceServer srv_exec_plan = nh_move.advertiseService(
      "/moveit_interface/execute_plan", callbackExecutePlan);
  ros::ServiceServer srv_move_to_rpy = nh_move.advertiseService(
      "/moveit_interface/move_to_rpy", callbackMoveToRPY);
  ros::ServiceServer srv_move_to_quat = nh_move.advertiseService(
      "/moveit_interface/move_to_quat", callbackMoveToQuat);

  while (ros::ok()) {
    ros::Duration(0.1).sleep();
  }
}
