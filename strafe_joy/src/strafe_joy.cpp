#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

class TeleopSwerve {
 public:
  TeleopSwerve();

 private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linearx_, lineary_, angular_, enable_, turbo_, start_, select_;
  double l_scale_, a_scale_, l_scale_turbo_, a_scale_turbo_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
};

TeleopSwerve::TeleopSwerve() : linearx_(0), lineary_(0), angular_(0), enable_(0), turbo_(0), start_(0), select_(0) {
  nh_.param("axis_linear_x", linearx_, linearx_);
  nh_.param("axis_linear_y", lineary_, lineary_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh_.param("scale_angular_turbo", a_scale_turbo_, a_scale_turbo_);
  nh_.param("scale_linear_turbo", l_scale_turbo_, l_scale_turbo_);
  nh_.param("enable_button", enable_, enable_);
  nh_.param("enable_turbo_button", turbo_, turbo_);
  nh_.param("start_button", start_, start_);
  nh_.param("select_button", select_, select_);

  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopSwerve::joyCallback, this);
}

void TeleopSwerve::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
  // This is for publishing cmd_vel
  geometry_msgs::Twist twist;
  if (joy->buttons[enable_] == 1 && joy->buttons[turbo_] == 1)  // In case turbo and normal buttons are pressed simultaneously
  {
    twist.angular.z = a_scale_ * joy->axes[angular_];
    twist.linear.x = l_scale_ * joy->axes[linearx_];
    twist.linear.y = l_scale_ * joy->axes[lineary_];
    // Make sure velocities can't be -0.0 (this would cause the wheels to turn 180° even though no negative velocity is given)
    if (-1e-4 < twist.angular.z && twist.angular.z < 1e-4) {
      twist.angular.z = 0.0;
    }
    if (-1e-4 < twist.linear.x && twist.linear.x < 1e-4) {
      twist.linear.x = 0.0;
    }
    if (-1e-4 < twist.linear.y && twist.linear.y < 1e-4) {
      twist.linear.y = 0.0;
    }
  } else if (joy->buttons[enable_] == 1) {
    twist.angular.z = a_scale_ * joy->axes[angular_];
    twist.linear.x = l_scale_ * joy->axes[linearx_];
    twist.linear.y = l_scale_ * joy->axes[lineary_];
    // As above
    if (-1e-4 < twist.angular.z && twist.angular.z < 1e-4) {
      twist.angular.z = 0.0;
    }
    if (-1e-4 < twist.linear.x && twist.linear.x < 1e-4) {
      twist.linear.x = 0.0;
    }
    if (-1e-4 < twist.linear.y && twist.linear.y < 1e-4) {
      twist.linear.y = 0.0;
    }
  } else if (joy->buttons[turbo_] == 1) {
    twist.angular.z = a_scale_turbo_ * joy->axes[angular_];
    twist.linear.x = l_scale_turbo_ * joy->axes[linearx_];
    twist.linear.y = l_scale_turbo_ * joy->axes[lineary_];
    // As above
    if (-1e-4 < twist.angular.z && twist.angular.z < 1e-4) {
      twist.angular.z = 0.0;
    }
    if (-1e-4 < twist.linear.x && twist.linear.x < 1e-4) {
      twist.linear.x = 0.0;
    }
    if (-1e-4 < twist.linear.y && twist.linear.y < 1e-4) {
      twist.linear.y = 0.0;
    }
  }
  vel_pub_.publish(twist);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_swerve");
  TeleopSwerve teleop_swerve;

  ros::spin();
}
