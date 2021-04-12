#!/usr/bin/env python
from franka_msgs.msg import FrankaState, Errors
import rospy
import actionlib
import actionlib_msgs
import moveit_commander
import controller_manager_msgs
import franka_gripper.msg
import franka_control.srv
import geometry_msgs.msg

class PandaController():
    def __init__(self, move_group_name):
        #self.reset_client = actionlib.SimpleActionClient('/franka_control/error_recovery', franka_msgs.msg.Errors)
        state_sub = rospy.Subscriber("franka_state_controller/franka_states",
                                     FrankaState, self.franka_state_callback)

        rospy.loginfo('Waiting for move_group/status')
        rospy.wait_for_message('move_group/status',
                               actionlib_msgs.msg.GoalStatusArray)

        self.commander = moveit_commander.MoveGroupCommander(move_group_name)

        rospy.loginfo('Waiting for franka_gripper/grasp')
        self.gripper_grasp = actionlib.SimpleActionClient(
            'franka_gripper/grasp',
            franka_gripper.msg.GraspAction)
        self.gripper_grasp.wait_for_server()

        rospy.loginfo('Waiting for franka_gripper/move')
        self.gripper_move = actionlib.SimpleActionClient(
            'franka_gripper/move',
            franka_gripper.msg.MoveAction)
        self.gripper_move.wait_for_server()

        rospy.loginfo('Waiting for controller_manager/switch_controller')
        self.switch_controller = rospy.ServiceProxy(
            'controller_manager/switch_controller',
            controller_manager_msgs.srv.SwitchController)
        self.switch_controller.wait_for_service()

        rospy.loginfo('Waiting for '
                      'franka_control/set_force_torque_collision_behavior')
        self.set_collision_behavior = rospy.ServiceProxy(
            'franka_control/set_force_torque_collision_behavior',
            franka_control.srv.SetForceTorqueCollisionBehavior)
        self.set_collision_behavior.wait_for_service()
        self.active_controllers = []


def gripper_move(ctx, width,
                 epsilon_inner=0.005, epsilon_outer=0.005,
                 speed=0.1, force=10):
    goal = franka_gripper.msg.MoveGoal(width=width, speed=speed)
    rospy.loginfo('Moving gripper:\n{}'.format(goal))
    ctx.gripper_move.send_goal(goal)
    ctx.gripper_move.wait_for_result()
    if not ctx.gripper_move.get_result().success:
        rospy.logerr("Couldn't move gripper")
        sys.exit(1)


def gripper_grasp(ctx, width,
                  epsilon_inner=0.005, epsilon_outer=0.005,
                  speed=0.1, force=10):
    epsilon = franka_gripper.msg.GraspEpsilon(inner=epsilon_inner,
                                              outer=epsilon_outer)
    goal = franka_gripper.msg.GraspGoal(width=width,
                                        epsilon=epsilon,
                                        speed=speed,
                                        force=force)
    rospy.loginfo('Grasping:\n{}'.format(goal))
    ctx.gripper_grasp.send_goal(goal)
    ctx.gripper_grasp.wait_for_result()

    while not ctx.gripper_grasp.get_result().success:
        rospy.logerr('Failed to grasp, retry')
        gripper_move(ctx, width=0.08)
        ctx.gripper_grasp.send_goal(goal)
        ctx.gripper_grasp.wait_for_result()
        #if rospy.is_shutdown():
            #sys.exit(0)



def moveit_cart(ctx, pos, rot, acc=0.1, vel=0.1):
    ctx.load_controllers(['position_joint_trajectory_controller'])

    print('Moving to Cartesian pose: pos {}, rot {}'.format(pos, rot))
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = rot[0]
    pose_goal.orientation.x = rot[1]
    pose_goal.orientation.y = rot[2]
    pose_goal.orientation.z = rot[3]
    pose_goal.position.x = pos[0]
    pose_goal.position.y = pos[1]
    pose_goal.position.z = pos[2]
    ctx.commander.set_max_acceleration_scaling_factor(acc)
    ctx.commander.set_max_velocity_scaling_factor(vel)
    ctx.commander.set_end_effector_link('panda_link8')
    ctx.commander.set_pose_target(pose_goal)
    ctx.commander.go(wait=True)
    ctx.commander.stop()
    ctx.commander.clear_pose_targets()
