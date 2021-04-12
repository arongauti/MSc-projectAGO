#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf2_eigen/tf2_eigen.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Duration.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_T 0x74
#define KEYCODE_H 0x68

class PandaKeyboard
{
  private:
  geometry_msgs::PoseStamped cmd; // Topic of the type posestamped
  double stepsize;

  ros::NodeHandle n_;
  ros::Publisher pose_pub_;

  public:
  void init()
  {
    stepsize=0.03;
    //header - this is impt
    cmd.header.frame_id = "/panda_link8";

    //Clear out our cmd - these values are roundabout initials
    /*cmd.pose.position.x=0.307;
    cmd.pose.position.y=0.0;
    cmd.pose.position.z=0.59;
    cmd.pose.orientation.x=-0.00244781865415;
    cmd.pose.orientation.y=-0.548220284495;
    cmd.pose.orientation.z=0.00145617884538;
    cmd.pose.orientation.w=0.836329126239;
    */
    Eigen::Isometry3d pose= Eigen::Isometry3d::Identity();

    //double step=2*M_PI/20;
    pose = Eigen::Translation3d(0.307,0.0, 0.59);
    //pose = Eigen::Translation3d(initialpose.position.x+radius*cos(2*M_PI*i/steps),initialpose.position.y-radius*sin(2*M_PI*i/steps), initialpose.position.z);
    pose *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    //pose *= Eigen::AngleAxisd(stepsperrev*i-M_PI/2.0+atan2(obbpose.position.y,obbpose.position.x), Eigen::Vector3d::UnitY());
    //ALIGN DOWN----------------------------------------------
    //pose *= Eigen::AngleAxisd(M_PI/7.0, Eigen::Vector3d::UnitX());
    //---------------------------------------
    pose *= Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d::UnitZ());
    geometry_msgs::Pose converted_pose;
    converted_pose = tf2::toMsg(pose);
    cmd.pose=converted_pose;

    pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/keyboard", 1);

    ros::NodeHandle n_private("~");
  }

  ~PandaKeyboard()   { }
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
  ros::init(argc, argv, "panda_keyboard");

  PandaKeyboard tpk;

  // Goes to home position
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void PandaKeyboard::keyboardLoop()
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
  puts("Use 'T' to test position");
  puts("Use 'H' to home position");

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
      // Walking
    case KEYCODE_W:
      cmd.pose.position.x = cmd.pose.position.x+stepsize;
      dirty = true;
      break;
    case KEYCODE_S:
      cmd.pose.position.x = cmd.pose.position.x-stepsize;
      dirty = true;
      break;
    case KEYCODE_A:
      cmd.pose.position.y = cmd.pose.position.y+stepsize;
      dirty = true;
      break;
    case KEYCODE_D:
      cmd.pose.position.y = cmd.pose.position.y-stepsize;
      dirty = true;
      break;
    case KEYCODE_Q:
      cmd.pose.position.z = cmd.pose.position.z+stepsize;
      dirty = true;
      break;
    case KEYCODE_E:
      cmd.pose.position.z = cmd.pose.position.z-stepsize;
      dirty = true;
      break;
    case KEYCODE_H:
        //Home 0.307,0.0, 0.59)
        cmd.pose.position.x = 0.307;
        cmd.pose.position.y = 0;
        cmd.pose.position.z =0.59;
        break;
    case KEYCODE_T:
      // Test position changed by Aron
      //Home
      cmd.pose.position.x = 0.427;
      cmd.pose.position.y = 0;
      cmd.pose.position.z =0.59;
      pose_pub_.publish(cmd);
      ros::Duration(3).sleep();
      // Fyrir ofan brusa
      cmd.pose.position.x = 0.457;
      cmd.pose.position.y = -0.33;
      cmd.pose.position.z =0.47;
      pose_pub_.publish(cmd);
      ros::Duration(3).sleep();
      // Picka brusa
      cmd.pose.position.x = 0.457;
      cmd.pose.position.y = -0.33;
      cmd.pose.position.z =0.18;
      pose_pub_.publish(cmd);
      ros::Duration(3).sleep();
      // Fyrir ofan brusa
      cmd.pose.position.x = 0.457;
      cmd.pose.position.y = -0.33;
      cmd.pose.position.z =0.40;
      pose_pub_.publish(cmd);
      ros::Duration(3).sleep();
      //Home
      cmd.pose.position.x = 0.427;
      cmd.pose.position.y = 0;
      cmd.pose.position.z =0.59;
      pose_pub_.publish(cmd);
      ros::Duration(3).sleep();
      //Sleppa brúsa
      cmd.pose.position.x = 0.397;
      cmd.pose.position.y = 0.03;
      cmd.pose.position.z =0.23;
      pose_pub_.publish(cmd);
      ros::Duration(3).sleep();
      // Home
      cmd.pose.position.x = 0.427;
      cmd.pose.position.y = 0;
      cmd.pose.position.z =0.59;
      pose_pub_.publish(cmd);
      break;

    }


    if (dirty == true) // Sends to test_scanner
    {
      pose_pub_.publish(cmd);
    }


  }
}
