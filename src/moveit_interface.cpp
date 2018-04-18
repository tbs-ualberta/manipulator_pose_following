
// TODO Implement a subscriber to a topic "pose" for continuous streaming of
//      poses. This is for instance intended for teleoperation.

// TODO Implement a service for planning and execution of one particular pose
//      (move_to) for instance for moving the end-effector to an initial pose
//      before starting the pose stream for teleoperation

// TODO Implement a service that starts a teleoperation loop (start_teleop).
//      The service is to be called after the service 'move_to'.

#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

#include "motoman_sia5f_teleop/MoveHome.h"
#include <vector>

static const std::string PLANNING_GROUP = "manipulator";
geometry_msgs::Pose g_pose_ref;

void callbackPose(const geometry_msgs::Pose::ConstPtr &msg) {
  // TODO Make sure that the duration for planning is limited to the sampling
  //      rate.
  g_pose_ref = *msg;
}

bool callbackMoveHome(motoman_sia5f_teleop::MoveHome::Request &req,
                      motoman_sia5f_teleop::MoveHome::Response &res) {

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  move_group.setMaxVelocityScalingFactor(req.max_vel_fact);
  move_group.setMaxAccelerationScalingFactor(req.max_acc_fact);

  std::vector<double> joint_angles = {0, 0, 0, 0, 0, 0, 0};
  move_group.setJointValueTarget(joint_angles);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (move_group.plan(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    if (move_group.execute(plan) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
          res.reply = 0;
      return true;
    } else {
      res.reply = -1;
      return false;
    }
  }
}

int main(int argc, char **argv) {

  // --- Initializations ---
  ros::init(argc, argv, "moveit_interface");
  ros::NodeHandle node_handle;

  // --- Start an AsyncSpinner with two threads for
  //     - trajectory planning and execution and
  //     - calls to service 'move_home'.
  ros::AsyncSpinner spinner(2);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // --- Serup topic subscritsions
  ros::Subscriber sub_pose = node_handle.subscribe("pose", 1, callbackPose);

  // --- Advertise services
  //     - move_home
  ros::ServiceServer srv_move_home = node_handle.advertiseService(
      "/moveit_interface/move_home", callbackMoveHome);

  // --- Obtain parameters
  int rate_hz = 10;
  node_handle.getParam("moveit_interface/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);
  move_group.setPlanningTime(1.0 / (double)rate_hz -
                             1.0 / (2 * (double)rate_hz));

  while (ros::ok()) {

    ros::spinOnce();
    /*
        move_group.setPoseTarget(g_pose_ref);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (move_group.plan(plan) ==
                        moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
          move_group.execute(plan);
        }
    */
    // Loop until user aborts (Ctrl+C)
    loop_rate.sleep();
  }
}
