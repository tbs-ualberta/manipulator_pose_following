
#include <cmath>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

#define _USE_MATH_DEFINES

const int TRAJ_1 = 0;
const int TRAJ_2 = 1;

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_traject");
  ros::NodeHandle node_handle;

  ros::Publisher pub_pose = node_handle.advertise<geometry_msgs::PoseStamped>(
      "pose_following/pose", 1);

  // --- Obtain parameters ---
  int rate_hz = 100;
  node_handle.getParam("test_traject/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);

  geometry_msgs::Pose pose_init;
  pose_init.position.x = 0.489;
  pose_init.position.y = 0;
  pose_init.position.z = 0.64;
  pose_init.orientation.x = 0;
  pose_init.orientation.y = 0;
  pose_init.orientation.z = 0;
  pose_init.orientation.w = 1;

  ros::Time t_init = ros::Time::now();
  double a = 0.25;
  double a_ref[2] = {0, 0};
  double v_ref[2] = {0, 0};
  double pos_ref[2] = {0, 0};
  double t_period = 7;
  double amp = 0.05;
  int traj = TRAJ_1;
  double dt = 0;
  double t_last_d = 0;
  while (ros::ok()) {
    double t_now_d = (ros::Time::now() - t_init).toSec();
    dt = t_now_d - t_last_d;
    t_last_d = t_now_d;
    ROS_DEBUG_NAMED("stream_dbg", "dt = %2.2f", dt);

    switch (traj) {
    case TRAJ_1:
      pos_ref[0] = amp * cos(2 * M_PI * t_now_d / t_period) - amp;
      pos_ref[1] = amp * sin(2 * M_PI * t_now_d / t_period);
      break;
    case TRAJ_2:
      if (t_now_d < 1) {
        a_ref[0] = a;
        ROS_DEBUG("Accelerating: a = %1.0f", a_ref[0]);
      } else if (t_now_d >= 1 && t_now_d < 2) {
        ROS_DEBUG("Constant velocity. a = %1.0f", a_ref[0]);
        a_ref[0] = 0;
      } else if (t_now_d >= 2 && t_now_d < 3) {
        a_ref[0] = -a;
        ROS_DEBUG("Decelerating: a = %1.0f", a_ref[0]);
      } else if (t_now_d >= 3) {
        a_ref[0] = 0;
        ROS_DEBUG("Stopped");
      }
      v_ref[0] = v_ref[0] + a_ref[0] * dt;
      pos_ref[0] = pos_ref[0] + v_ref[0] * dt;
      ROS_DEBUG("a_ref[0] = %2.3f", a_ref[0]);
      ROS_DEBUG("v_ref[0] = %2.3f", v_ref[0]);
      ROS_DEBUG("pos_ref[0] = %2.3f", pos_ref[0]);
    default:
      break;
    }

    geometry_msgs::PoseStamped pose_ref;
    pose_ref.header.stamp = ros::Time::now();
    pose_ref.header.frame_id = "/base";
    pose_ref.pose.orientation = pose_init.orientation;
    pose_ref.pose.position.x = pose_init.position.x;
    pose_ref.pose.position.y = pose_init.position.y + pos_ref[0];
    pose_ref.pose.position.z = pose_init.position.z + pos_ref[1];

    pub_pose.publish(pose_ref);

    loop_rate.sleep();
  }
}
