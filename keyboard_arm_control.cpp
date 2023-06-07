#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/JointState.h>

// Key ASCII in 0x (16)
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
// for orientation
#define KEYCODE_Y 0x79
#define KEYCODE_U 0x75
#define KEYCODE_H 0x68
#define KEYCODE_J 0x6A
#define KEYCODE_N 0x6E
#define KEYCODE_M 0x6D

// Gripper control
#define KEYCODE_G 0x67
#define KEYCODE_F 0x66

#define ANGULAR_V 0.1

class TeleopPR2Keyboard
{
  private:
  geometry_msgs::PoseStamped cmd_right_;
  geometry_msgs::PoseStamped cmd_left_;
  sensor_msgs::JointState gripper_right_open_;
  sensor_msgs::JointState gripper_right_close_;

  ros::NodeHandle n_;
  ros::Publisher pose_pub_right_;
  ros::Publisher pose_pub_left_;
  ros::Publisher gripper_pub_right_;

  tf2::Quaternion q_r_;
  tf2::Quaternion q_p_;
  tf2::Quaternion q_y_;
  tf2::Quaternion q_r_inv_;
  tf2::Quaternion q_p_inv_;
  tf2::Quaternion q_y_inv_;
  tf2::Quaternion q_;


  public:
  void init()
  {
    //header - this is impt
    cmd_right_.header.frame_id = "/torso_lift_link";

    q_r_.setRPY(ANGULAR_V,0.0,0.0); //radian 0.17 is about 10 degree this is angular velocity
    q_p_.setRPY(0.0,ANGULAR_V,0.0);
    q_y_.setRPY(0.0,0.0,ANGULAR_V);

    //Clear out our cmd - these values are roundabout initials
    cmd_right_.pose.position.x=0.345216155052;
    cmd_right_.pose.position.y=-0.57;
    cmd_right_.pose.position.z=-0.37;
    // cmd_right_.pose.orientation.x=0.79;
    // cmd_right_.pose.orientation.y=0.02;
    // cmd_right_.pose.orientation.z=0.61;
    // cmd_right_.pose.orientation.w=0.03;
    
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

    //Left arm not implemented yet, now it just stop at init position
    // TODO Left arm control, need another keyset for input like uiojkl, 
    // Integration with gamepad will be fast by modifying this code

    cmd_left_.pose.position.x = cmd_right_.pose.position.x;
    cmd_left_.pose.position.y =-cmd_right_.pose.position.y;
    cmd_left_.pose.position.z = cmd_right_.pose.position.z;
    cmd_left_.pose.orientation.x=-cmd_right_.pose.orientation.x;
    cmd_left_.pose.orientation.y=cmd_right_.pose.orientation.y;
    cmd_left_.pose.orientation.z=cmd_right_.pose.orientation.z;
    cmd_left_.pose.orientation.w=cmd_right_.pose.orientation.w;

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
    

    pose_pub_right_ = n_.advertise<geometry_msgs::PoseStamped>("/arm_right/free_positioning/gripper_marker_pose", 1);
    pose_pub_left_ = n_.advertise<geometry_msgs::PoseStamped>("/arm_left/free_positioning/gripper_marker_pose", 1);
    gripper_pub_right_ = n_.advertise<sensor_msgs::JointState>("/joint_command", 1);



    ros::NodeHandle n_private("~");
  }

  ~TeleopPR2Keyboard()   { }
  void keyboardLoop();
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_arms_keyboard");

  TeleopPR2Keyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void TeleopPR2Keyboard::keyboardLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WS' to forward/back");
  puts("Use 'AD' to left/right");
  puts("Use 'QE' to up/down");
  puts("Use 'YU' to Roll up/down");
  puts("Use 'HJ' to Pitch up/down");
  puts("Use 'NM' to Yaw up/down");
  puts("Use 'GF' to open/close right gripper");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch(c)
    {
      // Armcontrol
      // value 0.01 is a virtual velocity
    case KEYCODE_W:
      cmd_right_.pose.position.x = cmd_right_.pose.position.x+0.01;
      dirty = true;
      break;
    case KEYCODE_S:
      cmd_right_.pose.position.x = cmd_right_.pose.position.x-0.01;
      dirty = true;
      break;
    case KEYCODE_A:
      cmd_right_.pose.position.y = cmd_right_.pose.position.y+0.01;
      dirty = true;
      break;
    case KEYCODE_D:
      cmd_right_.pose.position.y = cmd_right_.pose.position.y-0.01;
      dirty = true;
      break;
    case KEYCODE_Q:
      cmd_right_.pose.position.z = cmd_right_.pose.position.z+0.01;
      dirty = true;
      break;
    case KEYCODE_E:
      cmd_right_.pose.position.z = cmd_right_.pose.position.z-0.01;
      dirty = true;
      break;

    // Orientation Roll(x)Y+U- Pitch(y)H+J- Yaw(z)N+M-
    case KEYCODE_Y:
      q_= q_r_*q_;
      q_.normalize();
      cmd_right_.pose.orientation.x = q_.x();
      cmd_right_.pose.orientation.y = q_.y();
      cmd_right_.pose.orientation.z = q_.z();
      cmd_right_.pose.orientation.w = q_.w();
      dirty = true;
      break;
    case KEYCODE_U:
      q_= q_r_.inverse()*q_;
      q_.normalize();
      cmd_right_.pose.orientation.x = q_.x();
      cmd_right_.pose.orientation.y = q_.y();
      cmd_right_.pose.orientation.z = q_.z();
      cmd_right_.pose.orientation.w = q_.w();
      dirty = true;
      break;
    case KEYCODE_H:
      q_= q_p_*q_;
      q_.normalize();
      cmd_right_.pose.orientation.x = q_.x();
      cmd_right_.pose.orientation.y = q_.y();
      cmd_right_.pose.orientation.z = q_.z();
      cmd_right_.pose.orientation.w = q_.w();
      dirty = true;
      break;
    case KEYCODE_J:
      q_= q_p_.inverse()*q_;
      q_.normalize();
      cmd_right_.pose.orientation.x = q_.x();
      cmd_right_.pose.orientation.y = q_.y();
      cmd_right_.pose.orientation.z = q_.z();
      cmd_right_.pose.orientation.w = q_.w();
      dirty = true;
      break;
    case KEYCODE_N:
      q_= q_y_*q_;
      q_.normalize();
      cmd_right_.pose.orientation.x = q_.x();
      cmd_right_.pose.orientation.y = q_.y();
      cmd_right_.pose.orientation.z = q_.z();
      cmd_right_.pose.orientation.w = q_.w();
      dirty = true;
      break;
    case KEYCODE_M:
      q_= q_y_.inverse()*q_;
      q_.normalize();
      cmd_right_.pose.orientation.x = q_.x();
      cmd_right_.pose.orientation.y = q_.y();
      cmd_right_.pose.orientation.z = q_.z();
      cmd_right_.pose.orientation.w = q_.w();
      dirty = true;
      break;

    case KEYCODE_G:
      gripper_pub_right_.publish(gripper_right_open_);
      break;

    case KEYCODE_F:
      gripper_pub_right_.publish(gripper_right_close_);
      break;
    }


    if (dirty == true)
    {
      pose_pub_right_.publish(cmd_right_);
      pose_pub_left_.publish(cmd_left_);
    }


  }
}