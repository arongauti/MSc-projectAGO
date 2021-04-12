//#include <jsoncpp/json/json.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/planning_interface/planning_interface.h>
#include <sensor_msgs/PointCloud2.h>
#include <picking_perception/scan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vision_msgs/OBB.h>

static const std::string PLANNING_GROUP_ARM = "panda_arm";
/*
geometry_msgs::Pose applyrot(double r, double p, double y, geometry_msgs::Pose current){


  tf2::Quaternion q_orig, q_rot, q_new;
  geometry_msgs::Pose appliedpose;
  // Get the original orientation of 'commanded_pose'

  tf2::convert(current.orientation , q_orig);

  //double r=0, p=M_PI/6, y=0;  // Rotate the previous pose by 180* about X
  q_rot.setRPY(r, p, y);

  q_new = q_rot*q_orig;
  q_new.normalize();

  appliedpose.position=current.position;
  appliedpose.orientation.w=q_new.w();
  appliedpose.orientation.x=q_new.x();

  appliedpose.orientation.y=q_new.y();
  appliedpose.orientation.z=q_new.z();

  return appliedpose;

}*/

std::vector<Eigen::Isometry3d> circularposes(geometry_msgs::Pose initialpose, double radius, int steps){
  std::vector<Eigen::Isometry3d>poses;
  double stepsperrev=2.0*M_PI/steps;
  for (unsigned int i = 0; i < steps; ++i)
          {
              Eigen::Isometry3d pose= Eigen::Isometry3d::Identity();
              //Eigen::Translation3d position;
              Eigen::Isometry3d pose_test= Eigen::Isometry3d::Identity();

              geometry_msgs::Point inposition=initialpose.position;
              Eigen::Vector3d curr_p(inposition.x+radius*cos(stepsperrev*i),inposition.y+radius*sin(stepsperrev*i), inposition.z);
              Eigen::Vector3d zax(curr_p[0]-inposition.x,curr_p[1]-inposition.y,curr_p[2]-inposition.z);

              //yaxis=frame_pose.matrix().col(1).head<3>();
              //visual_tools->publishAxis(circtest[i],0.05,0.005,"ax"+std::to_string(i));


              //zax[0]=inposition.x-curr_p[0];
              //inposition.y -=0.018;
              //inposition.z-=0.03;
              //inposition.x += 0.018;
              //inposition.y +=0.041;
              //inposition.z -=0.056;

              //double step=2*M_PI/20;
              pose = Eigen::Translation3d(inposition.x+radius*cos(stepsperrev*i),inposition.y+radius*sin(stepsperrev*i), inposition.z);

              //pose = Eigen::Translation3d(initialpose.position.x+radius*cos(stepsperrev*i),initialpose.position.y+radius*sin(stepsperrev*i), initialpose.position.z);
              pose *= Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitX());
              pose *= Eigen::AngleAxisd(stepsperrev*i-M_PI/2.0+atan2(inposition.y,inposition.x), Eigen::Vector3d::UnitY());
              //pose *= Eigen::AngleAxisd(stepsperrev*i-M_PI/2.0+atan2(initialpose.position.y,initialpose.position.x), Eigen::Vector3d::UnitY());
              //ALIGN DOWN----------------------------------------------
              pose *= Eigen::AngleAxisd(M_PI/7.0, Eigen::Vector3d::UnitX());
              //---------------------------------------
              std::cout<< "ORIG:\n" << pose.rotation()<<std::endl;

              pose *= Eigen::AngleAxisd(M_PI/4.0, Eigen::Vector3d::UnitZ());

              poses.push_back(pose);

              //- Translation: [0.009, -0.034, 0.056]
              //- Rotation: in Quaternion [0.653, -0.271, 0.653, 0.271]
              //            in RPY (radian) [-0.000, -1.571, 2.356]



          }
  return poses;

}


std::vector<Eigen::Isometry3d> dummy(geometry_msgs::Pose initialpose, double radius, int steps, double obj_height){
  std::vector<Eigen::Isometry3d>poses;
  double stepsperrev=2.0*M_PI/steps;
  for (unsigned int i = 0; i < steps; ++i)
          {
              Eigen::Isometry3d pose= Eigen::Isometry3d::Identity();
              //Eigen::Translation3d position;
              Eigen::Isometry3d pose_test= Eigen::Isometry3d::Identity();

              geometry_msgs::Point inposition=initialpose.position;
              //inposition.y+=0.03;
              Eigen::Vector3d zax;
              Eigen::Vector3d curr_p(inposition.x+radius*cos(stepsperrev*i),inposition.y+radius*sin(stepsperrev*i), inposition.z);
              Eigen::Vector3d lookd(inposition.x-curr_p[0],inposition.y- curr_p[1],(obj_height/2.0)-curr_p[2]);
              zax=lookd.normalized();
              Eigen::Vector3d tm(0.0,0.0,1.0);

              //Eigen::Vector3d x_ax()
              //pose *= Eigen::AngleAxisd(stepsperrev*i-M_PI/2.0+atan2(inposition.y,inposition.x), Eigen::Vector3d::UnitY());
              //Eigen::Vector3d tang(cos(stepsperrev*i),sin(stepsperrev*i),atan2(curr_p[1],curr_p[0]));
              //tang=tang.normalized();
              //std::cout<< "tang:: "<< tang<< std::endl;

              Eigen::Vector3d up=zax.cross(tm.normalized());
              Eigen::Vector3d third=up.normalized().cross(zax);
              pose_test.matrix().col(0).head<3>()=third;
              pose_test.matrix().col(1).head<3>()=up;
              pose_test.matrix().col(2).head<3>()=zax;




              pose_test.translation()=curr_p;
              pose_test *= Eigen::AngleAxisd(-M_PI/4.0, Eigen::Vector3d::UnitZ());
              std::cout<< "ROT\n" << pose_test.rotation()<<std::endl;
              poses.push_back(pose_test);
              //yaxis=frame_pose.matrix().col(1).head<3>();
              //visual_tools->publishAxis(circtest[i],0.05,0.005,"ax"+std::to_string(i));

          }
  return poses;

}




std::vector<moveit_msgs::CollisionObject> setobstacles( std::string collisionframe, vision_msgs::OBB obbpose)//geometry_msgs::Pose model_pose)
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = collisionframe;
  collision_object.id = "leftwall";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.0;
  primitive.dimensions[1] = 0.02;
  primitive.dimensions[2] = 1.0;


  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  //box_pose.orientation.x = ;
  //box_pose.orientation.y = ;
  //box_pose.orientation.z = ;

  // MoveIt! planning scene expects the center of the object as position.
  // We add half of its dimension to its position
  box_pose.position.x = 0.0;
  box_pose.position.y = -0.8;
  box_pose.position.z = 0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> objects;
  objects.push_back(collision_object);


  collision_object.header.frame_id = collisionframe;
  collision_object.id = "rightwall";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.0;
  primitive.dimensions[1] = 0.02;
  primitive.dimensions[2] = 1.0;

  box_pose.orientation.w = 1.0;

  box_pose.position.x = 0.0;
  box_pose.position.y = 0.8;
  box_pose.position.z = 0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  objects.push_back(collision_object);


  collision_object.header.frame_id = collisionframe;
  collision_object.id = "behind";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.02;
  primitive.dimensions[1] = 1.0;
  primitive.dimensions[2] = 1.0;

  box_pose.orientation.w = 1.0;

  box_pose.position.x = -0.4;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.5;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  objects.push_back(collision_object);

  //--------------------MODEL COLLISION OBJECT-----TEST ONLY
  /*
  collision_object.header.frame_id = collisionframe;
  collision_object.id = "model";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = obbpose.width-0.06;//0.07;
  primitive.dimensions[1] =obbpose.height-0.06; //0.07;
  primitive.dimensions[2] = 0.24; //0.18;

  box_pose.orientation.w = 1.0;

  box_pose.position.x = obbpose.pose_orient.pose.position.x; //model_pose.position.x;
  box_pose.position.y = obbpose.pose_orient.pose.position.y;//model_pose.position.y;
  box_pose.position.z = obbpose.pose_orient.pose.position.z+0.12;//model_pose.position.z-0.13;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  objects.push_back(collision_object);*/
  //====================UPPER================
  collision_object.id = "upper";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.0;
  primitive.dimensions[1] = 1.0;
  primitive.dimensions[2] = 0.01;

  box_pose.orientation.w = 1.0;

  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  objects.push_back(collision_object);

  //==================UPPER====

  //====================lower================
  collision_object.id = "lower";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 1.0;
  primitive.dimensions[1] = 1.0;
  primitive.dimensions[2] = 0.01;

  box_pose.orientation.w = 1.0;

  box_pose.position.x = 0.0;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.01;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;
  objects.push_back(collision_object);

  //==================lower====

  return objects;
}


moveit_msgs::CollisionObject setmodelcollision( std::string collisionframe, vision_msgs::OBB obbpose, double max_obj_height)
{
  moveit_msgs::CollisionObject collision_object;

  shape_msgs::SolidPrimitive primitive;


  //--------------------MODEL COLLISION OBJECT-----TEST ONLY
  collision_object.header.frame_id = collisionframe;
  collision_object.id = "model";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = obbpose.width-0.06;//0.07;
  primitive.dimensions[1] =obbpose.height-0.06; //0.07;
  primitive.dimensions[2] = max_obj_height;
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;


  box_pose.position.x = obbpose.pose_orient.pose.position.x; //model_pose.position.x;
  box_pose.position.y = obbpose.pose_orient.pose.position.y;//model_pose.position.y;
  box_pose.position.z = max_obj_height/2.0; //obbpose.pose_orient.pose.position.z+0.12;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}


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
  ros::ServiceClient client = node_handle.serviceClient<picking_perception::scan>("scanserver");
  picking_perception::scan srv;

  moveit_visual_tools::MoveItVisualToolsPtr visual_tools;
  visual_tools.reset(new moveit_visual_tools::MoveItVisualTools("panda_link0", "/moveit"));
  visual_tools->loadRemoteControl();
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP_ARM);
  visual_tools->deleteAllMarkers();

  ros::Publisher cloud_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/allclouds", 10);
  ros::WallDuration sleep_t(0.5);

  const std::string planner_id = "RRTConnectkConfigDefault";
  //moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  const robot_state::JointModelGroup* joint_model_group=
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  move_group.setPlannerId(planner_id);

  ROS_INFO_NAMED("MOVEGROUP", "Reference frame: %s", move_group.getPlanningFrame().c_str());
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("MOVEGROUP", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  auto robotmodel=move_group.getRobotModel();
  //move_group.setEndEffectorLink("kinect_depth_frame");

  move_group.setGoalOrientationTolerance(0.01);
  move_group.setGoalPositionTolerance(0.01);
  move_group.setMaxVelocityScalingFactor(0.2);

  move_group.setPlanningTime(10.0);
  move_group.allowReplanning(true);

  move_group.setNumPlanningAttempts(10);

  std::vector<sensor_msgs::PointCloud2> clouds;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  moveit::planning_interface::PlanningSceneInterface planning_scene;

  ros::WallDuration(1.0).sleep();



  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  //move_group.move();
  geometry_msgs::Pose currenteefpose,initialpose;

  ROS_INFO_STREAM("Panda initial position of eef:" << initialpose.position);
  ros::Rate rate(5);
  int counter=0;
  move_group.setNamedTarget("ready");
  move_group.move();

  move_group.setStartStateToCurrentState();

  currenteefpose=move_group.getCurrentPose().pose;
  initialpose=move_group.getCurrentPose().pose;
  srv.request.scan_nr=20;
  srv.request.objectname="can";
  srv.request.initialization=true;
  srv.request.initiatemesh=false;
  //ros::WallDuration(1.0).sleep();
  ros::Duration(1.0).sleep();
  sensor_msgs::PointCloud2 initialcloud;
  vision_msgs::OBB obb_msg;

  vision_msgs::OBBConstPtr recentobb =
      ros::topic::waitForMessage<vision_msgs::OBB>("obb", node_handle);
  obb_msg=*recentobb;
  ROS_INFO_STREAM("OBB received, setup collision model at: " <<obb_msg.pose_orient.pose.position );
  ROS_INFO_STREAM("veggir ");
  std::vector<moveit_msgs::CollisionObject> collision_objects = setobstacles(move_group.getPlanningFrame(),obb_msg);//alignpose);


  //THE CENTER OF THE MODEL SCAN POSE-------------------------------------------
  geometry_msgs::Pose alignpose;
  //alignpose=initialpose;
  //alignpose.position.x+=0.07;
  //alignpose.position.z-=0.30;
  double max_obj_height;
  ROS_INFO_STREAM("wait a bit ");
  ros::Duration(1.0).sleep();
  ROS_INFO_STREAM("waited ");
  if(client.call(srv)){
    ROS_INFO_STREAM("caught cloud: INITIALIZATION" );
    //initialcloud=srv.response.transformed_cloud;
    //cloud_pub.publish(initialcloud);
    //clouds.push_back(initialcloud);
    alignpose=srv.response.centerpose;
    max_obj_height=srv.response.collobj_height;
    ROS_INFO_STREAM("Initialpose(Home): \n" << initialpose  << "\nModified initialscanpose:\n"<<alignpose);


  }
  else {
    ROS_INFO_STREAM("Service not AVAILABLE!!!");
  }
  moveit_msgs::CollisionObject scan_object= setmodelcollision(move_group.getPlanningFrame(),obb_msg,max_obj_height);
  collision_objects.push_back(scan_object);
  planning_scene.addCollisionObjects(collision_objects);
  visual_tools->trigger();
  visual_tools->prompt("INITIALIZE!");

  //MOVE TO THE INITIAL SCAN POSE=-----------------------------

  move_group.setPoseTarget(alignpose);
  move_group.plan(my_plan);
  visual_tools->publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
  visual_tools->trigger();
  visual_tools->prompt("Movetoaligned Apriltag pose");
  move_group.execute(my_plan);
  bool alignsuccess=(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  std::cout<< "is success: " << alignsuccess<<std::endl;
  ros::Duration(1.0).sleep();


  //---------------------------------------------------------------------------------------




  //geometry_msgs::Pose homepose=move_group.getPoseTarget();
  Eigen::Isometry3d frame_pose=Eigen::Isometry3d::Identity();
  Eigen::Vector3d xaxis(1.0,0.0,0.0);
  Eigen::Vector3d yaxis(0.0,1.0,0.0);
  Eigen::Vector3d zaxis(0.0,0.0,1.0);
  Eigen::AngleAxisd ax_ang(0,xaxis);
  //visual_tools->prompt("PUSH NEXT");
  //frame_pose.translation()= //initialpose.position;
  xaxis=frame_pose.matrix().col(0).head<3>();
  yaxis=frame_pose.matrix().col(1).head<3>();
  zaxis=frame_pose.matrix().col(2).head<3>();
  Eigen::Isometry3d frame_pose2=Eigen::Isometry3d::Identity();
  ROS_INFO_STREAM("current RPY: " << move_group.getCurrentRPY()[0]<< ", " <<move_group.getCurrentRPY()[1]<< ", "<< move_group.getCurrentRPY()[2]<<"\n" );
  ROS_INFO_STREAM("current orientation tol: " << move_group.getGoalOrientationTolerance() <<"\n" );

  tf::poseMsgToEigen(initialpose,frame_pose2);

  Eigen::Isometry3d pose= Eigen::Isometry3d::Identity();
  //TODO: SETA AS PARAMETER, Number of Scans and Distance to obj
  //FIXME: SEM FYRST!
  //Visualization only, before planning!----------------------------------
  std::vector<Eigen::Isometry3d> circdummy=dummy(alignpose,arc_dist,nr_scans, max_obj_height);

  std::vector<Eigen::Isometry3d> circtest=circularposes(alignpose,arc_dist,nr_scans);
  for (int i=0;i<circtest.size(); ++i){
    geometry_msgs::Pose circlepose3;
    //tf::poseEigenToMsg(circtest[i],circlepose3);
    //circlepose3.position.x -= 0.009;
    //circlepose3.position.y +=0.034;
    //circlepose3.position.z -=0.056;
    //circlepose3.position.x += 0.056;
    //circlepose3.position.y -=0.017;
    //circlepose3.position.z -=0.030;

    visual_tools->publishAxis(circdummy[i],0.02,0.005,"axtrials"+std::to_string(i));
    visual_tools->publishAxis(circtest[i],0.05,0.005,"ax"+std::to_string(i));  //circ[i]
    //visual_tools->publishAxis(circlepose3,0.03,0.005,"a22x"+std::to_string(i));  //circ[i]
    visual_tools->trigger();
  }
  //------------------------------------------------------------------------


  /*
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link7";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;//move_group.getCurrentPose().pose.orientation.w;
  ocm.absolute_x_axis_tolerance = 1.0;
  ocm.absolute_y_axis_tolerance = 1.0;
  ocm.absolute_z_axis_tolerance = 1.1;
  ocm.weight = 1.0;
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);
  */

  //Actual scanning movements------------------------------------------------------------
  std::vector<Eigen::Isometry3d> circ=circularposes(alignpose,arc_dist,nr_scans);
  srv.request.initialization=false;
  for (int i=0;i<circ.size(); ++i){
    geometry_msgs::Pose circlepose;
    visual_tools->publishAxis(circ[i],0.05,0.005,"ax"+std::to_string(i));  //circ[i]
    //ros::WallDuration(0.4).sleep();
    tf::poseEigenToMsg(circ[i],circlepose);
    std::cout<< "distance Z>" << circlepose.position<< std::endl;

    move_group.setPoseTarget(circlepose);
    move_group.plan(my_plan);
    visual_tools->publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
    visual_tools->trigger();
    bool success2=(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    visual_tools->prompt("NEXT POSE i hring");
    if(success2){
      move_group.execute(my_plan);
    }
    ros::Duration(2.0).sleep();
    //move_group.execute(my_plan);
    //sensor_msgs::PointCloud2 curr_cloud;
    //bool success2=(move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    std::cout<< "is success: " << success2<<std::endl;
    srv.request.scan_nr=i;
    srv.request.objectname="can";
    if(client.call(srv) ){
      ROS_INFO_STREAM("caught cloud: " << srv.response.succeeded << " nr: " << i );
      //curr_cloud=srv.response.transformed_cloud;
      //cloud_pub.publish(curr_cloud);
      //clouds.push_back(curr_cloud);
    }

  }
  int lastscan=srv.request.scan_nr+1;
  std::cout<< "last scan nr: " << lastscan<< std::endl;
  ROS_INFO_STREAM("no of clouds: " << clouds.size());
  move_group.setNamedTarget("ready");
  move_group.plan(my_plan);
  visual_tools->publishTrajectoryLine(my_plan.trajectory_,joint_model_group);
  visual_tools->trigger();
  visual_tools->prompt("HOME");
  move_group.execute(my_plan);
  ros::Duration(1.0).sleep();
  //ros::WallDuration(1.0).sleep();
  srv.request.initiatemesh=true;
  srv.request.objectname="can";
  srv.request.scan_nr=lastscan;
  if(client.call(srv)){
    ROS_INFO_STREAM("caught cloud: " << srv.response.succeeded << " initiate meshing" );
    //curr_cloud=srv.response.transformed_cloud;
    //cloud_pub.publish(curr_cloud);
    //clouds.push_back(curr_cloud);
  }

 
  std::vector<std::string> collobj;
  collobj.push_back("model");
  planning_scene.removeCollisionObjects(collobj);
  visual_tools->trigger();

  //geometry_msgs::Pose alignpose2;
  //alignpose2=initialpose;
  //alignpose2.position.x+=0.07;
  //alignpose2.position.z-=0.37;

  /*
  ros::WallDuration(1.0).sleep();
  move_group.setNamedTarget("ready");
  move_group.move();
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "panda_link8";
  ocm.header.frame_id = "panda_link0";
  ocm.orientation.w = 1.0;//move_group.getCurrentPose().pose.orientation.w;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 1.0;
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group.setPathConstraints(test_constraints);
  geometry_msgs::Pose setgoals1;//=move_group.getCurrentPose().pose;
  setgoals1.orientation.w=1.0;
  setgoals1.position=move_group.getCurrentPose().pose.position;
  setgoals1.position.x-=0.05;
  setgoals1.position.y-=0.1;
  setgoals1.position.z-=0.1;
  move_group.setPoseTarget(setgoals1);
  move_group.move();
  */



  ROS_INFO("robot_control_node is ready");
  //spinner.stop();
  ros::waitForShutdown();

  return 0;

}
