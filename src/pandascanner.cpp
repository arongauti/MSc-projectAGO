#include "../include/auto_labeller/pandascanner.h"

/*PandaScanner::PandaScanner(const std::string &robot_description, const std::string &robot_topic, const std::string &marker_topic, const std::string &imarker_topic)
{

}*/
PandaScanner::PandaScanner(ros::NodeHandle* nh):
  move_group_("panda_arm"),
  nh_(*nh)
{
robotSetup();
runtest();
keyboard_callb_=true;
if (keyboard_callb_){
  initialize_callbacks();
}

}

void PandaScanner::initialize_callbacks(){
  keyb_sub_ = nh_.subscribe("/keyboard", 1, &PandaScanner::keyboard_callback,this);
}

void PandaScanner::keyboard_callback(const geometry_msgs::PoseStamped &commandpose){
  //if(commandpose.pose.position.x>0.7)
  move_group_.setPoseTarget(commandpose.pose);
  move_group_.move();
  ROS_INFO_STREAM("moved to :" << commandpose.pose.position);
}

void PandaScanner::robotSetup(){

  visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("panda_link0", "/moveit"));
  //visual_tools->loadRemoteControl();
  //moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_ARM);
  //visual_tools->deleteAllMarkers();

  const std::string planner_id = "RRTConnectkConfigDefault";
  //moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  //const robot_state::JointModelGroup* joint_model_group=
  joint_model_group_= move_group_.getCurrentState()->getJointModelGroup("panda_arm");
  move_group_.setPlannerId(planner_id);

  ROS_INFO_NAMED("MOVEGROUP", "Reference frame: %s", move_group_.getPlanningFrame().c_str());
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("MOVEGROUP", "End effector link: %s", move_group_.getEndEffectorLink().c_str());
  auto robotmodel=move_group_.getRobotModel();
  ROS_INFO_STREAM("ROBOT MODEL!: " << robotmodel->getName());
  //move_group.setEndEffectorLink("kinect_depth_frame");

  move_group_.setGoalOrientationTolerance(0.01);
  move_group_.setGoalPositionTolerance(0.01);
  move_group_.setMaxVelocityScalingFactor(0.1);//FIXME: ONLY SIMULATION!!!!!!

  move_group_.setPlanningTime(5.0);//10sec
  move_group_.allowReplanning(true);

  move_group_.setNumPlanningAttempts(10);

  ROS_INFO_STREAM("PLANNER: " << move_group_.getPlannerId());

  //bool success = (move_group_.plan(my_plan_) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //moveit::planning_interface::PlanningSceneInterface planning_scene;

}
void PandaScanner::runtest(){
  int counter=0;
  move_group_.setNamedTarget("ready");
  move_group_.move();

  move_group_.setStartStateToCurrentState();

}


/*
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

  ros::waitForShutdown();

  return 0;

}
*/
