#ifndef PANDASCANNER_H
#define PANDASCANNER_H

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/planning_interface/planning_interface.h>
#include <sensor_msgs/PointCloud2.h>
#include <picking_perception/scan.h>
#include <franka_msgs/FrankaState.h>
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
#include <vision_msgs/OBB.h>

class PandaScanner
{
public:

  PandaScanner(ros::NodeHandle* nh);
  //~PandaScanner();

  void setMoveGroup(const std::string& groupname);
  const std::string& getMoveGroup() const;

  //void setJointModelGroup();
  void robotSetup();
  void runtest();
  void keyboard_callback(const geometry_msgs::PoseStamped& commandpose);
  void initialize_callbacks();

  moveit_msgs::CollisionObject setmodelcollision( std::string collisionframe, vision_msgs::OBB obbpose, double max_obj_height);
  std::vector<moveit_msgs::CollisionObject> setobstacles( std::string collisionframe, vision_msgs::OBB obbpose);
  /**
   * @brief circularPath
   * Lookat function, aim camera at the specified object while revolving around in a circular motion
   * around the detected object. Aligned with the panda end link(panda_link8)
   * @param initialpose : the center object pose with a specified height and alignment w.r.t the object
   * @param radius radius[m] of the orbit
   * @param steps nr of interpolated steps in the circle
   * @param obj_height detected height of the object
   * @return
   */
  std::vector<Eigen::Isometry3d> circularPath(geometry_msgs::Pose initialpose, double radius, int steps, double obj_height);


private:
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  //ROBOT sETUP
  moveit::planning_interface::MoveGroupInterface move_group_;
  const std::string planner_id_;
  const robot_state::JointModelGroup* joint_model_group_;
  const std::string PLANNING_GROUP_ARM;
  //const robot_model::JointModelGroup* group_;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_;


  //Variables
  ros::NodeHandle nh_;
  bool keyboard_callb_;

  ros::Publisher robot_state_publisher_;
  ros::Publisher world_state_publisher_;
  ros::Subscriber keyb_sub_;



  static const double WORLD_BOX_SIZE_;
  ros::Timer publish_timer_;
  ros::Time init_time_;
  ros::Time last_callback_time_;
  ros::Duration average_callback_duration_;
  static const ros::Duration min_delay_;
  int schedule_request_count_;

};


#endif // PANDASCANNER_H
