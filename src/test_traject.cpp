
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_traject");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0;
  target_pose1.orientation.y = 0.7;
  target_pose1.orientation.z = 0;
  target_pose1.orientation.w = 0;
  target_pose1.position.x = 0.5;
  target_pose1.position.y = 0.1;
  target_pose1.position.z = 0.45;
  move_group.setPoseTarget(target_pose1);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  if (success) {
    move_group.execute(my_plan);
  }
}
