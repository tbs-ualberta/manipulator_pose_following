
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// Globals
double g_jogging_velocity;
int g_rate_hz = 40;

// --- Function declarations
char getch_async(); // Non-blocking getch_async()

int main(int argc, char **argv) {

  ros::init(argc, argv, "kb_jogging");
  ros::NodeHandle n;

  ros::Publisher cmd_vel_fct_pub =
      n.advertise<geometry_msgs::Twist>("pose_following/cmd_vel", 1);

  // --- Get params from parameter server
  g_jogging_velocity = 0.1;
  n.getParam("kb_jogging/jogging_vel", g_jogging_velocity);

  n.getParam("kb_jogging/rate", g_rate_hz);
  ros::Rate loop_rate_hz(g_rate_hz);

  const double max_cart_translation_vel = .5;
  double cart_translation_vel = max_cart_translation_vel;
  n.getParam("kb_jogging/max_trans_vel", cart_translation_vel);
  if (cart_translation_vel > max_cart_translation_vel) {
    cart_translation_vel = max_cart_translation_vel;
  }

  const double max_cart_rotation_vel = 1;
  double cart_rotation_vel = max_cart_rotation_vel;
  n.getParam("kb_jogging/max_rot_vel", cart_rotation_vel);
  if (cart_rotation_vel > max_cart_rotation_vel) {
    cart_rotation_vel = max_cart_rotation_vel;
  }

  ROS_INFO("Keyboard jogging online.");
  ROS_INFO_STREAM("\n"
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
                  << "Stop:            \"x\"");

  bool stop = false;

  system("/bin/stty raw");   // Raw mode (send all keystrokes directly to stdin)
  system("/bin/stty -echo"); // Turn off echo

  while (ros::ok() && !stop) {

    geometry_msgs::Twist msg_twist;

    char command = getch_async();
    switch (command) {
    case '1':
      msg_twist.linear.x = g_jogging_velocity * cart_translation_vel;
      break;
    case '2':
      msg_twist.linear.y = g_jogging_velocity * cart_translation_vel;
      break;
    case '3':
      msg_twist.linear.z = g_jogging_velocity * cart_translation_vel;
      break;
    case '4':
      msg_twist.angular.x = g_jogging_velocity * cart_rotation_vel;
      break;
    case '5':
      msg_twist.angular.y = g_jogging_velocity * cart_rotation_vel;
      break;
    case '6':
      msg_twist.angular.z = g_jogging_velocity * cart_rotation_vel;
      break;
    case 'q':
      msg_twist.linear.x = -g_jogging_velocity * cart_translation_vel;
      break;
    case 'w':
      msg_twist.linear.y = -g_jogging_velocity * cart_translation_vel;
      break;
    case 'e':
      msg_twist.linear.z = -g_jogging_velocity * cart_translation_vel;
      break;
    case 'r':
      msg_twist.angular.x = -g_jogging_velocity * cart_rotation_vel;
      break;
    case 't':
      msg_twist.angular.y = -g_jogging_velocity * cart_rotation_vel;
      break;
    case 'y':
      msg_twist.angular.z = -g_jogging_velocity * cart_rotation_vel;
      break;
    case '0':
      g_jogging_velocity += 0.1;
      if (g_jogging_velocity > 1) {
        g_jogging_velocity = 1;
      }
      ROS_INFO("Jogging velocity factor: %1.1f\r", g_jogging_velocity);
      break;
    case 'p':
      g_jogging_velocity -= 0.1;
      if (g_jogging_velocity < 0) {
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

    cmd_vel_fct_pub.publish(msg_twist);
    loop_rate_hz.sleep();
  }
  // use system call to set terminal behaviour to more normal behaviour
  system("/bin/stty cooked");
  system("/bin/stty echo");
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
  timeout.tv_usec = g_rate_hz*1000;

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
