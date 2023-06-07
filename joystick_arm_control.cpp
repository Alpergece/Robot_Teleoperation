#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <math.h>  
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

// vector
#include <iostream>
#include <numeric>
#include <cmath>
#include <complex>

#define ANGULAR_V 0.05

class TeleopPR2Joystick
{
  private:
  geometry_msgs::PoseStamped cmd_right_;
  geometry_msgs::PoseStamped cmd_left_;
  geometry_msgs::Twist cmdvel_;  
  sensor_msgs::JointState gripper_right_open_;
  sensor_msgs::JointState gripper_right_close_;
  sensor_msgs::JointState gripper_left_open_;
  sensor_msgs::JointState gripper_left_close_;
  

  ros::NodeHandle n_;
  ros::Publisher pose_pub_right_;
  ros::Publisher pose_pub_left_;
  ros::Publisher pub_;  
  ros::Publisher gripper_pub_right_;
  ros::Publisher gripper_pub_left_;
  // Added for joystick teleoperation
  ros::Subscriber joy_sub_;


  tf2::Quaternion q_r_;
  tf2::Quaternion q_py_;
  tf2::Quaternion q_right;
  tf2::Quaternion q_left;
  double walk_vel_;  
  double run_vel_;  
  double yaw_rate_;  
  double yaw_rate_run_;

  // This function gets the variables for axes and buttons from the joy message.
  std::vector<float> axes_joy_ = std::vector<float>(8, 0.0);
  std::vector<int> buttons_joy_ = std::vector<int>(11, 0);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);


  double l2_norm(std::vector<double> const& x) {
    double x_sum = 0.;
    for (double x_i : x) {
        x_sum += x_i * x_i;
    }
    return sqrt(x_sum);
  }

  double rescaling(double const& x, double x_min = 0., double x_max = 1.) { 
    return (x-x_min)/(x_max-x_min);
  } 

  double lt_rt(double const& x, double const& threshold) {
    return rescaling(x, threshold);
  } 

  std::vector<double> lsb_rsb(std::vector<double> const& x, double const& threshold) {
    std::vector<double> x_out = std::vector<double>(2, 0.0);
    double scaling_factor = std::max(0., std::min(rescaling(l2_norm(x), threshold)/l2_norm(x), 1.));   
    std::transform(x.begin(), x.end(), x_out.begin(), [&scaling_factor](auto& c){ return scaling_factor * c; });
    return x_out;
  } 
  
  // Add variables for joycallback
  int selected_control_type;
  bool selected_control_flag;
  bool dirty;
  double max_tv;  
  double max_rv;
  double threshold; 
  int speed;  
  int turn;  
  int goal;

  public:
  void init()
  {
    //header - this is impt
    cmd_right_.header.frame_id = "/torso_lift_link";
    q_r_.setRPY(-ANGULAR_V,0.0,0.0); //radian 0.17 is about 10 degree this is angular velocity

    //Clear out our cmd - these values are roundabout initials
    cmd_right_.pose.position.x=0.27;
    cmd_right_.pose.position.y=-0.57;
    cmd_right_.pose.position.z=0.34;
    
    cmd_left_.pose.position.x = 0.27;
    cmd_left_.pose.position.y = 0.57;
    cmd_left_.pose.position.z = 0.34;

    // tf2::Quaternion q_(0.707,0.0,0.707,0.0);
    q_right.setX(0.707);
    q_right.setY(0.0);
    q_right.setZ(0.707);
    q_right.setW(0.0);

    q_left.setX(-0.707);
    q_left.setY(0.0);
    q_left.setZ(0.707);
    q_left.setW(0.0);

    // Normalize the quaternion for right
    q_right.normalize();
    q_left.normalize();

    // This is the orientaion of gripper forward direction
    cmd_right_.pose.orientation.x = q_right.x();
    cmd_right_.pose.orientation.y = q_right.y();
    cmd_right_.pose.orientation.z = q_right.z();
    cmd_right_.pose.orientation.w = q_right.w();

    cmd_left_.pose.orientation.x = q_left.x();
    cmd_left_.pose.orientation.y = q_left.y();
    cmd_left_.pose.orientation.z = q_left.z();
    cmd_left_.pose.orientation.w = q_left.w();

    // initialize the base vel
    cmdvel_.linear.x = 0.0;  
    cmdvel_.angular.z = 0.0;  

    // Gripper joint set msgs initialise
    // Actual joint pos value and publisher inside cases
    gripper_right_open_.header.stamp = ros::Time::now();
    gripper_right_open_.name.resize(2);
    gripper_right_open_.name[0] = "gripper_right_left_finger_joint";
    gripper_right_open_.name[1] = "gripper_right_right_finger_joint";
    gripper_right_open_.position.resize(2);
    gripper_right_open_.position[0]=4.0;
    gripper_right_open_.position[1]=4.0;

    gripper_right_close_.header.stamp = ros::Time::now();
    gripper_right_close_.name.resize(2);
    gripper_right_close_.name[0] = "gripper_right_left_finger_joint";
    gripper_right_close_.name[1] = "gripper_right_right_finger_joint";
    gripper_right_close_.position.resize(2);
    gripper_right_close_.position[0]=0.0;
    gripper_right_close_.position[1]=0.0;

    gripper_left_open_.header.stamp = ros::Time::now();
    gripper_left_open_.name.resize(2);
    gripper_left_open_.name[0] = "gripper_left_left_finger_joint";
    gripper_left_open_.name[1] = "gripper_left_right_finger_joint";
    gripper_left_open_.position.resize(2);
    gripper_left_open_.position[0]=4.0;
    gripper_left_open_.position[1]=4.0;

    gripper_left_close_.header.stamp = ros::Time::now();
    gripper_left_close_.name.resize(2);
    gripper_left_close_.name[0] = "gripper_left_left_finger_joint";
    gripper_left_close_.name[1] = "gripper_left_right_finger_joint";
    gripper_left_close_.position.resize(2);
    gripper_left_close_.position[0]=0.0;
    gripper_left_close_.position[1]=0.0;
    
    pose_pub_right_ = n_.advertise<geometry_msgs::PoseStamped>("/arm_right/free_positioning/gripper_marker_pose", 1);
    pose_pub_left_ = n_.advertise<geometry_msgs::PoseStamped>("/arm_left/free_positioning/gripper_marker_pose", 1);

    pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);  

    gripper_pub_right_ = n_.advertise<sensor_msgs::JointState>("/arm_right/gripper/joint_command", 1);
    gripper_pub_left_ = n_.advertise<sensor_msgs::JointState>("/arm_left/gripper/joint_command", 1);

    ros::NodeHandle n_private("~");
    n_private.param("walk_vel", walk_vel_, 0.5);  
    n_private.param("run_vel", run_vel_, 1.0);  
    n_private.param("yaw_rate", yaw_rate_, 1.0);  
    n_private.param("yaw_rate_run", yaw_rate_run_, 1.5);     

    // Add subscriber for joystick
    joy_sub_ = n_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopPR2Joystick::joyCallback, this);
  }

  ~TeleopPR2Joystick()   { }
  void joystickLoop();
  void stopRobot()
  {  
      cmdvel_.linear.x = 0.0;  
      cmdvel_.angular.z = 0.0;  
      pub_.publish(cmdvel_);  
  }  
};

int kfd = 0;
struct termios cooked, raw;
bool done;  

void TeleopPR2Joystick::joystickLoop()
{
  dirty =false;

  selected_control_type = 0;
  selected_control_flag = false;

  threshold = 0.25;


  max_tv = walk_vel_;  
  max_rv = yaw_rate_;  
  speed = 0;  
  turn = 0;  
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from joystick");
  puts("-----------------------------------");
  puts("--------------for arm--------------");
  puts("Use 'start' to control base, left arm, right arm or none");  
  puts("Use 'LT/RT' to forward/back");
  puts("Use 'LSB(push to the left and right)' to left/right ");
  puts("Use 'LSB(push up and down)' to up/down"); 
  puts("Use 'A/B' to open/close  gripper");
  puts("Use 'LB/RB' to Roll up/down");
  puts("Use 'RSB(push to the left and right)' to Pitch up/down");
  puts("Use 'RSB(push up and down)' to Yaw up/down");
  puts("--------------for base--------------");
  puts("Use 'D-PAD UP/D-PAD DOWN' to forward/back ");
  // puts("Use 'Shift+IK' to move faster");  
  puts("Use 'D-PAD LEFT/D-PAD RIGHT' to  turn left/right");
  puts("Use 'Guide' to  stop");

  // Use Ros spin instead of for loop
  ros::spin();  
  
}

void TeleopPR2Joystick::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  axes_joy_ = joy->axes;
  buttons_joy_ = joy->buttons;

  // Select control type with start button
  if (buttons_joy_[7] == 1 & selected_control_flag == false) {
    selected_control_type += 1;
    selected_control_type %= 4;
    selected_control_flag = true;
    switch (selected_control_type)
    {
    case 0:
      ROS_INFO("Control: None");
      break;
    case 1:
      ROS_INFO("Control: Base");
      break;
    case 2:
      ROS_INFO("Control: Left-arm");
      break;
    case 3:
      ROS_INFO("Control: Right-arm");
      break;
    }
  } 
  else if (buttons_joy_[7] == 0) {
    selected_control_flag = false;
  }

  // LT: Position change along the X axis;
  double _threshold = 0.0; 
  std::vector<double> vec2 = {- axes_joy_[2]};
  double x_2 = lt_rt(- axes_joy_[2], _threshold);
  if (l2_norm(vec2) >= _threshold)   {
    if (selected_control_type == 3) {
      cmd_right_.pose.position.x = cmd_right_.pose.position.x - 0.001 * x_2; 
    }
    else if (selected_control_type == 2) {
    cmd_left_.pose.position.x = cmd_left_.pose.position.x - 0.001 * x_2;  
    }
    dirty = true;
  }

  // RT: Position change along the -X axis; 
  std::vector<double> vec5 = {- axes_joy_[5]};
  double x_5 = lt_rt(- axes_joy_[5], _threshold);
  if (l2_norm(vec5) >= _threshold) {
    if (selected_control_type == 3) {
      cmd_right_.pose.position.x = cmd_right_.pose.position.x + 0.001 * x_5; 
    }
    else if (selected_control_type == 2) {
      cmd_left_.pose.position.x = cmd_left_.pose.position.x + 0.001 * x_5;  
    }
    dirty = true;
  }

  // LSB: Position change along the Y and Z axis;  
  std::vector<double> vec01 = {axes_joy_[0], axes_joy_[1]};
  if (l2_norm(vec01) >= threshold) {
    std::vector<double> vec01_scaled = lsb_rsb(vec01, threshold);
    if (selected_control_type == 3) {
      cmd_right_.pose.position.y = cmd_right_.pose.position.y + 0.001 * vec01_scaled[0];
      cmd_right_.pose.position.z = cmd_right_.pose.position.z + 0.001 * vec01_scaled[1];  
      }
    else if (selected_control_type == 2) {
      cmd_left_.pose.position.y = cmd_left_.pose.position.y + 0.001 * vec01_scaled[0];  
      cmd_left_.pose.position.z = cmd_left_.pose.position.z + 0.001 * vec01_scaled[1];  
    }
      dirty = true;
  }

  //RSB: Pitch/Yaw up and down - Second and third quaternion 
  std::vector<double> angular_v_py = {axes_joy_[3], axes_joy_[4]};
  if (l2_norm(angular_v_py) >= threshold) {
    std::vector<double> angular_v_py_scaled = lsb_rsb(angular_v_py, threshold);
    double angular_v = 0.1 * ANGULAR_V;  
    std::transform(angular_v_py_scaled.begin(), angular_v_py_scaled.end(), angular_v_py_scaled.begin(), [angular_v](auto& c){ return angular_v * c; });
    q_py_.setRPY(0.0, - angular_v_py_scaled[1], angular_v_py_scaled[0]);  
    if (selected_control_type == 3) {
      q_right = q_py_*q_right;
      q_right.normalize();
      cmd_right_.pose.orientation.x = q_right.x();
      cmd_right_.pose.orientation.y = q_right.y();
      cmd_right_.pose.orientation.z = q_right.z();
      cmd_right_.pose.orientation.w = q_right.w();
      }
    else if (selected_control_type == 2) {
      q_left = q_py_*q_left;
      q_left.normalize();
      cmd_left_.pose.orientation.x = q_left.x();
      cmd_left_.pose.orientation.y = q_left.y();
      cmd_left_.pose.orientation.z = q_left.z();
      cmd_left_.pose.orientation.w = q_left.w();
    }
      dirty = true;
  } 

  // open and close the gripper with button A and B
    if (buttons_joy_[0] == 1) {
      if (selected_control_type == 3) {
        gripper_pub_right_.publish(gripper_right_open_); 
      }
      else if (selected_control_type == 2) {
        gripper_pub_left_.publish(gripper_left_open_);  
      }
      dirty = true;
    }
    if (buttons_joy_[1] == 1) {
      if (selected_control_type == 3) {
        gripper_pub_right_.publish(gripper_right_close_); 
      }
      else if (selected_control_type == 2) {
        gripper_pub_left_.publish(gripper_left_close_);
      }
      dirty = true;
    }

    // Roll up movement with LB button
    if (buttons_joy_[4] == 1) {
      if (selected_control_type == 3) {
        q_right= q_r_*q_right;
        q_right.normalize();
        cmd_right_.pose.orientation.x = q_right.x();
        cmd_right_.pose.orientation.y = q_right.y();
        cmd_right_.pose.orientation.z = q_right.z();
        cmd_right_.pose.orientation.w = q_right.w(); 
      }
      else if (selected_control_type == 2) {
        q_left= q_r_*q_left;
        q_left.normalize();
        cmd_left_.pose.orientation.x = q_left.x();
        cmd_left_.pose.orientation.y = q_left.y();
        cmd_left_.pose.orientation.z = q_left.z();
        cmd_left_.pose.orientation.w = q_left.w();  
      }
      dirty = true;
    }

    // Roll down movement with RB button
    if (buttons_joy_[5] == 1) {
      if (selected_control_type == 3) {
        q_right= q_r_.inverse()*q_right;
        q_right.normalize();
        cmd_right_.pose.orientation.x = q_right.x();
        cmd_right_.pose.orientation.y = q_right.y();
        cmd_right_.pose.orientation.z = q_right.z();
        cmd_right_.pose.orientation.w = q_right.w();
      }
      else if (selected_control_type == 2) {
        q_left= q_r_.inverse()*q_left;
        q_left.normalize();
        cmd_left_.pose.orientation.x = q_left.x();
        cmd_left_.pose.orientation.y = q_left.y();
        cmd_left_.pose.orientation.z = q_left.z();
        cmd_left_.pose.orientation.w = q_left.w();  
      }
      dirty = true;
    }

    //Move the base with dpad buttons
    if (axes_joy_[7] == 1) {
      if (selected_control_type == 1) {
        max_tv = walk_vel_;  
        speed = 1;  
        turn = 0;   
      }
      dirty = true;
    }

    if (axes_joy_[7] == -1) {
      if (selected_control_type == 1) {
        max_tv = walk_vel_;  
        speed = -1;  
        turn = 0;   
      }
      dirty = true;
    }

    if (axes_joy_[6] == 1) {
      if (selected_control_type == 1) {
        max_tv = walk_vel_;  
        speed = 0;  
        turn = 1;   
      }
      dirty = true;
    }

    if (axes_joy_[6] == -1) {
      if (selected_control_type == 1) {
        max_tv = walk_vel_;  
        speed = 0;  
        turn = -1;   
      }
      dirty = true;
    }
    
    if (buttons_joy_[8] == 1) {
      if (selected_control_type == 1) {
        max_rv =0.0;
        max_tv =0.0;
        speed = 0;  
        turn = -1;   
      }
      dirty = true;
    }

   if (dirty == true)
    {
      pose_pub_right_.publish(cmd_right_);
      pose_pub_left_.publish(cmd_left_);
      cmdvel_.linear.x = speed * max_tv;  
      cmdvel_.angular.z = turn * max_rv;  
      pub_.publish(cmdvel_);  

      // void stopRobot() 
    }
  }




void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_arms_joystick");

  TeleopPR2Joystick tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.joystickLoop();

  return(0);
}