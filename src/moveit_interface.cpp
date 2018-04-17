
// TODO Implement a subscriber to a topic "pose" for continuous streaming of
//      poses. This is for instance intended for teleoperation.

// TODO Implement a service for planning and execution of one particular pose
//      (move_to) for instance for moving the end-effector to an initial pose
//      before starting the pose stream for teleoperation

// TODO Implement a service for planning and execution of moving to the robot's
//      home position (move_home). Calling the service moves the manipulator to
//      its home position before starting the pose stream for teleoperation.

// TODO Implement a service that starts a teleoperation loop (start_teleop).
//      The service is to be called after the service 'move_to'.

#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

static const std::string PLANNING_GROUP = "manipulator";
moveit::planning_interface::MoveGroupInterface g_move_group(PLANNING_GROUP);

void callbackPose(const geometry_msgs::Pose::ConstPtr &msg) {
  // TODO Make sure that the duration for planning is limited to the sampling
  //      rate.
  g_move_group.setPoseTarget(*msg);
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  bool success = (g_move_group.plan(plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success) {
    g_move_group.execute(plan);
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "moveit_interface");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // --- Initilalize the moveit interface ---

  ros::Subscriber sub_pose =
      node_handle.subscribe("pose", 1, callbackPose);

  // --- Obtain parameters ---
  int rate_hz = 10;
  node_handle.getParam("moveit_interface/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);
  g_move_group.setPlanningTime(1.0 / (double)rate_hz -
                               1.0 / (2 * (double)rate_hz));

  // Loop until user aborts (Ctrl+C)
  ros::spin();
}
