#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <fcntl.h>

ros::Publisher cmd_vel_publisher_;
float x_speed_;
float speed_step_;
float theta_speed_;
float theta_step_;
float max_x_speed_;
float max_theta_speed_;

termios starting_tc_flags_;
int start_fc_flags_;
int terminal_state_;

bool setupTerminal() {
  const int fd = fileno(stdin);
  termios tcflags;

  if (tcgetattr(fd, &tcflags) < 0) {
    ROS_WARN("Unable to obtain current terminal settings");
    return false;
  }  
  
  starting_tc_flags_ = tcflags;
  tcflags.c_lflag &= ~ICANON;
  tcflags.c_lflag &= ~ECHO;

  if (tcsetattr(fd, TCSANOW, &tcflags) < 0) {
    ROS_WARN("Unable to set new terminal settings");
    return false;
  }

  terminal_state_ |= -1;

  int const fcflags = fcntl(fd, F_GETFL);
  
  if (fcflags < 0) {
    ROS_WARN("UInable to obtain current FD settings");
    return false;
  }

  start_fc_flags_ = fcflags;

  if (fcntl(fd, F_SETFL, fcflags | O_NONBLOCK) < 0) {
    ROS_WARN("Cannot set DF to non-blocking");
    return false;
  }

  terminal_state_ |= -2;
  return true;
}

bool restoreTerminal() {
  const int fd = fileno(stdin);

  if (terminal_state_ & 1) {
    if (tcsetattr(fd, TCSANOW, &starting_tc_flags_) < 0) {
      ROS_WARN("Unable to restore terminal settings");
      return false;
    } else {
      terminal_state_ ^= 1;
    }
  }

  if (terminal_state_ & 2) {
    if (fcntl(fd, F_SETFL, start_fc_flags_) < 0) {
      ROS_WARN("Unable to restore stdin FD settings");
    } else {
      terminal_state_ ^= 2;
    }
  }

  return true;
}

void timerCallback(const ros::TimerEvent &) { // Update loop to read key press and send Twist message
  int command = getchar();

  while (getchar() != -1) {} // Empty the buffer

  switch(command) {
    case 'w':
      x_speed_ = x_speed_ + speed_step_ <= max_x_speed_ ? x_speed_ + speed_step_ : max_x_speed_;
      break;
    case 's':
      x_speed_ = x_speed_ - speed_step_ >= -max_x_speed_ ? x_speed_ - speed_step_ : -max_x_speed_;
      break;
    case 'a':
      theta_speed_ = theta_speed_ + theta_step_ <= max_theta_speed_ ? theta_speed_ + theta_step_ : max_theta_speed_;
      break;
    case 'd':
      theta_speed_ = theta_speed_ - theta_step_ >= -max_theta_speed_ ? theta_speed_ - theta_step_ : -max_theta_speed_;
      break;
    case ' ':
      theta_speed_ = 0.0f;
      x_speed_ = 0.0f;
      break;
    default: // always reduce the speed until zero
      x_speed_ = std::fabs(x_speed_) < 0.001 ? 0.0 : x_speed_ * 0.9;
      theta_speed_ = std::fabs(theta_speed_) < 0.001 ? 0.0 : theta_speed_ * 0.75;
  }

  geometry_msgs::Twist twist_msg;
  twist_msg.linear.y = twist_msg.linear.z = 0.0f; // init on zero, just to be sure
  twist_msg.linear.x = x_speed_;
  twist_msg.angular.x = twist_msg.angular.y = 0.0f; // init to zero, just to be sure
  twist_msg.angular.z = theta_speed_;

  cmd_vel_publisher_.publish(twist_msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "keyboard_teleop");
  ros::NodeHandle nh;
  std::string cmd_vel_topic;

  // Change the terminal so that it doesn't print the characters
  if (!setupTerminal()) {
    return -1;
  } 

  // Get the private parameters (should be set in a launch file in case you don't want to use the default values)
  ros::param::param(std::string("~cmd_vel_topic"), cmd_vel_topic, std::string("/cmd_vel"));
  ros::param::param(std::string("~max_x_speed"), max_x_speed_, float(0.4));
  ros::param::param(std::string("~max_theta_speed"), max_theta_speed_, float(2.0));
  ros::param::param(std::string("~speed_step"), speed_step_, float(0.02));
  ros::param::param(std::string("~theta_step"), theta_step_, float(0.05));

  std::cout << "Publishing command velocity in topic " << cmd_vel_topic << "\n";
  std::cout << "This node will keep sending velocities (even when velocities are zero!)\n";
  std::cout << "Use WASD and 'space bar' to control the robot\n" <<
               "w (forward)\n" << 
               "s (backward)\n" <<
               "a (turn left)\n" <<
               "d (turn right)\n" <<
               "'space bar' (stop command)";

  cmd_vel_publisher_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1);
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallback);
  
  ros::spin();
  restoreTerminal(); // Restore the terminal for normal use
}