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
  tf2::Quaternion q_p_;
  tf2::Quaternion q_y_;
  tf2::Quaternion q_r_inv_;
  tf2::Quaternion q_p_inv_;
  tf2::Quaternion q_y_inv_;
  tf2::Quaternion q_;
  double walk_vel_;  
  double run_vel_;  
  double yaw_rate_;  
  double yaw_rate_run_;

  // This function gets the variables for axes and buttons from the joy message.
  std::vector<float> axes_joy_ = std::vector<float>(8, 0.0);
  std::vector<int> buttons_joy_ = std::vector<int>(11, 0);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  // Add variables for joycallback
  int selected_control_type;
  bool selected_control_flag;
  bool dirty;
  double max_tv;  
  double max_rv;
  float threshold;  
  int speed;  
  int turn;  
  int goal;

  public:
  void init()
  {
    //header - this is impt
    cmd_right_.header.frame_id = "/torso_lift_link";
    q_r_.setRPY(ANGULAR_V,0.0,0.0); //radian 0.17 is about 10 degree this is angular velocity
    q_p_.setRPY(0.0,ANGULAR_V,0.0);
    q_y_.setRPY(0.0,0.0,ANGULAR_V);

    //Clear out our cmd - these values are roundabout initials
    cmd_right_.pose.position.x=0.27;
    cmd_right_.pose.position.y=-0.57;
    cmd_right_.pose.position.z=0.34;
    
    // This is the orientaion of gripper forqard direction
    cmd_right_.pose.orientation.x=0.707;
    cmd_right_.pose.orientation.y=0.0;
    cmd_right_.pose.orientation.z=0.707;
    cmd_right_.pose.orientation.w=0.0;

    // tf2::Quaternion q_(0.707,0.0,0.707,0.0);
    q_.setX(0.707);
    q_.setY(0.0);
    q_.setZ(0.707);
    q_.setW(0.0);

    // Normalize the quaternion 
    q_.normalize(); 
    
    cmd_right_.pose.orientation.x = q_.x();
    cmd_right_.pose.orientation.y = q_.y();
    cmd_right_.pose.orientation.z = q_.z();
    cmd_right_.pose.orientation.w = q_.w();


    cmd_left_.pose.position.x = cmd_right_.pose.position.x;
    cmd_left_.pose.position.y =-cmd_right_.pose.position.y;
    cmd_left_.pose.position.z = cmd_right_.pose.position.z;
    cmd_left_.pose.orientation.x=-cmd_right_.pose.orientation.x;
    cmd_left_.pose.orientation.y=cmd_right_.pose.orientation.y;
    cmd_left_.pose.orientation.z=cmd_right_.pose.orientation.z;
    cmd_left_.pose.orientation.w=cmd_right_.pose.orientation.w;

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

    gripper_pub_right_ = n_.advertise<sensor_msgs::JointState>("/joint_command", 1);
    gripper_pub_left_ = n_.advertise<sensor_msgs::JointState>("/joint_command", 1);

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
  puts("Use 'X' to control left arm, use 'Y' to control right arm");  
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

  // here are the buttons and control

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



    if (axes_joy_[2] <= 0.0)   {
      if (selected_control_type == 3) {
        cmd_right_.pose.position.x = cmd_right_.pose.position.x + 0.002 * (- axes_joy_[2]); 
      }
      else if (selected_control_type == 2) {
        cmd_left_.pose.position.x = cmd_left_.pose.position.x + 0.002 * (- axes_joy_[2]);  
      }
      dirty = true;
    }
    if (axes_joy_[5] <= 0.0) {
      if (selected_control_type == 3) {
        cmd_right_.pose.position.x = cmd_right_.pose.position.x - 0.002 * (- axes_joy_[5]); 
      }
      else if (selected_control_type == 2) {
        cmd_left_.pose.position.x = cmd_left_.pose.position.x - 0.002 * (- axes_joy_[5]);  
      }
      dirty = true;
    }

    if (axes_joy_[0] != 0 | axes_joy_[1] != 0) {
      if (axes_joy_[0] >= threshold) {
        if (selected_control_type == 3) {
          cmd_right_.pose.position.y = cmd_right_.pose.position.y + 0.002 * axes_joy_[0]; 
        }
        else if (selected_control_type == 2) {
          cmd_left_.pose.position.y = cmd_left_.pose.position.y + 0.002 * axes_joy_[0];  
        }
        dirty = true;
      } 
      else if (axes_joy_[0] <= -threshold) {
        if (selected_control_type == 3) {
          cmd_right_.pose.position.y = cmd_right_.pose.position.y - 0.002 * (- axes_joy_[0]); 
        }
        else if (selected_control_type == 2) {
          cmd_left_.pose.position.y = cmd_left_.pose.position.y - 0.002 * (- axes_joy_[0]);  
        }
        dirty = true;
      }
      if (axes_joy_[1] >= threshold) {
        if (selected_control_type == 3) {
          cmd_right_.pose.position.z = cmd_right_.pose.position.z + 0.002 * axes_joy_[1]; 
        }
        else if (selected_control_type == 2) {
          cmd_left_.pose.position.z = cmd_left_.pose.position.z + 0.002 * axes_joy_[1];  
        }
        dirty = true;
      }
      else if (axes_joy_[1] <= -threshold) {
        if (selected_control_type == 3) {
          cmd_right_.pose.position.z = cmd_right_.pose.position.z - 0.002 * (- axes_joy_[1]); 
        }
        else if (selected_control_type == 2) {
          cmd_left_.pose.position.z = cmd_left_.pose.position.z - 0.002 * (- axes_joy_[1]);  
        }
        dirty = true;
      }
    }

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

    if (buttons_joy_[4] == 1) {
      if (selected_control_type == 3) {
        q_= q_r_*q_;
        q_.normalize();
        cmd_right_.pose.orientation.x = q_.x();
        cmd_right_.pose.orientation.y = q_.y();
        cmd_right_.pose.orientation.z = q_.z();
        cmd_right_.pose.orientation.w = q_.w(); 
      }
      else if (selected_control_type == 2) {
        q_= q_r_*q_;
        q_.normalize();
        cmd_left_.pose.orientation.x = -q_.x();
        cmd_left_.pose.orientation.y = q_.y();
        cmd_left_.pose.orientation.z = q_.z();
        cmd_left_.pose.orientation.w = q_.w();  
      }
      dirty = true;
    }

    if (buttons_joy_[5] == 1) {
      if (selected_control_type == 3) {
        q_= q_r_.inverse()*q_;
        q_.normalize();
        cmd_right_.pose.orientation.x = q_.x();
        cmd_right_.pose.orientation.y = q_.y();
        cmd_right_.pose.orientation.z = q_.z();
        cmd_right_.pose.orientation.w = q_.w();
      }
      else if (selected_control_type == 2) {
        q_= q_r_.inverse()*q_;
        q_.normalize();
        cmd_left_.pose.orientation.x = -q_.x();
        cmd_left_.pose.orientation.y = q_.y();
        cmd_left_.pose.orientation.z = q_.z();
        cmd_left_.pose.orientation.w = q_.w();  
      }
      dirty = true;
    }

    if (axes_joy_[3] != 0 | axes_joy_[4] != 0) {
      if (axes_joy_[3] >= threshold)
        if (selected_control_type == 3) {
          q_= q_p_*q_;
          q_.normalize();
          cmd_right_.pose.orientation.x = q_.x();
          cmd_right_.pose.orientation.y = q_.y();
          cmd_right_.pose.orientation.z = q_.z();
          cmd_right_.pose.orientation.w = q_.w();
      }
        else if (selected_control_type == 2) {
          q_= q_p_*q_;
          q_.normalize();
          cmd_left_.pose.orientation.x = -q_.x();
          cmd_left_.pose.orientation.y = q_.y();
          cmd_left_.pose.orientation.z = q_.z();
          cmd_left_.pose.orientation.w = q_.w();
      }
      dirty = true;
    }
      else if (axes_joy_[3] <= -threshold) {
        if (selected_control_type == 3) {
          q_= q_p_.inverse()*q_;
          q_.normalize();
          cmd_right_.pose.orientation.x = q_.x();
          cmd_right_.pose.orientation.y = q_.y();
          cmd_right_.pose.orientation.z = q_.z();
          cmd_right_.pose.orientation.w = q_.w();
      }
        else if (selected_control_type == 2) {
          q_= q_p_.inverse()*q_;
          q_.normalize();
          cmd_left_.pose.orientation.x = -q_.x();
          cmd_left_.pose.orientation.y = q_.y();
          cmd_left_.pose.orientation.z = q_.z();
          cmd_left_.pose.orientation.w = q_.w();
      }
      dirty = true;
    }
      if (axes_joy_[4] >= threshold) {
        if (selected_control_type == 3) {
          q_= q_y_*q_;
          q_.normalize();
          cmd_right_.pose.orientation.x = q_.x();
          cmd_right_.pose.orientation.y = q_.y();
          cmd_right_.pose.orientation.z = q_.z();
          cmd_right_.pose.orientation.w = q_.w();
      }
        else if (selected_control_type == 2) {
          q_= q_y_*q_;
          q_.normalize();
          cmd_left_.pose.orientation.x = -q_.x();
          cmd_left_.pose.orientation.y = q_.y();
          cmd_left_.pose.orientation.z = q_.z();
          cmd_left_.pose.orientation.w = q_.w();
      }
      dirty = true;
    }
      else if (axes_joy_[4] <= -threshold) {
        if (selected_control_type == 3) {
          q_= q_y_.inverse()*q_;
          q_.normalize();
          cmd_right_.pose.orientation.x = q_.x();
          cmd_right_.pose.orientation.y = q_.y();
          cmd_right_.pose.orientation.z = q_.z();
          cmd_right_.pose.orientation.w = q_.w();
      }
        else if (selected_control_type == 2) {
          q_= q_y_.inverse()*q_;
          q_.normalize();
          cmd_left_.pose.orientation.x = -q_.x();
          cmd_left_.pose.orientation.y = q_.y();
          cmd_left_.pose.orientation.z = q_.z();
          cmd_left_.pose.orientation.w = q_.w();
      }
    dirty = true;
    }

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