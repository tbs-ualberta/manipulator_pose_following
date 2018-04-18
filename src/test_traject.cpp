
#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#define _USE_MATH_DEFINES

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_traject");
  ros::NodeHandle node_handle;

  ros::Publisher pub_pose =
      node_handle.advertise<geometry_msgs::Pose>("pose", 1);

  // --- Obtain parameters ---
  int rate_hz = 10;
  node_handle.getParam("test_traject/rate", rate_hz);
  ros::Rate loop_rate(rate_hz);

  geometry_msgs::Pose pose_init;
  pose_init.orientation.x = 0;
  pose_init.orientation.y = 0;
  pose_init.orientation.z = 0;
  pose_init.orientation.w = 1;
  pose_init.position.x = 0.489;
  pose_init.position.y = 0;
  pose_init.position.z = 0.64;

  ros::Time t_init_abs = ros::Time::now();
  double t_init_abs_d = (double)t_init_abs.sec + (double)t_init_abs.nsec * 1e-9;
  double pos_ref[3];
  double t_period = 5;
  double amp = 0.05;
  while (ros::ok()) {
    ros::Time t_now_abs = ros::Time::now();
    double t_now_abs_d = (double)t_now_abs.sec + (double)t_now_abs.nsec * 1e-9;
    double t_now_d = t_now_abs_d - t_init_abs_d;

    pos_ref[0] = amp * sin(2 * M_PI * t_now_d / t_period);
    pos_ref[1] = amp * cos(2 * M_PI * t_now_d / t_period) - amp;

    geometry_msgs::Pose pose_ref;
    pose_ref.orientation = pose_init.orientation;
    pose_ref.position.x = pose_init.position.x;
    pose_ref.position.y = pose_init.position.y + pos_ref[0];
    pose_ref.position.z = pose_init.position.z + pos_ref[1];

    pub_pose.publish(pose_ref);

    loop_rate.sleep();
  }
}
