#include <auto_labeller/pandascanner.h>
#include <geometry_msgs/PoseStamped.h>


int main(int argc, char **argv)
{
  //namespace fs = boost::filesystem;
  ROS_INFO("RUNNING Panda");

  ros::init(argc, argv, "panda_controller_test");
  int nr_scans=9;
  double arc_dist=0.32;
  double align_angle=M_PI/7.0;
  double initial_height;
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  //ros::NodeHandle nh;
  //ObjSeg::ScanServer scan(&nh);
  PandaScanner scanner(&node_handle);
  ros::Rate rate(5);


  //ros::Subscriber keyb_sub = node_handle.subscribe("/keyboard", 1, keyboard_callback);
  ros::waitForShutdown();

  return 0;

}

