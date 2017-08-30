#!/usr/bin/env python

import rospy
import smach
import smach_ros

import moveit_msgs.msg

### XXX: check the way to import things
from injerrobot_grafting_pipeline.place import Place
from injerrobot_grafting_pipeline.pick import Pick
from injerrobot_grafting_pipeline.goto import GoTo
from injerrobot_grafting_pipeline.gotogrid import GoToGrid
from injerrobot_grafting_pipeline.cut import Cut
from injerrobot_grafting_pipeline.clip import Clip
from injerrobot_grafting_pipeline.inspection import Inspection
from injerrobot_grafting_pipeline.dispense import Dispense
from injerrobot_grafting_pipeline.reject import Reject

import io_module 

from moveit_python import MoveGroupInterface
import geometry_msgs.msg

# gets called when ANY child state terminates
def child_termination_cb(outcome_map):
    # if some of the states has terminated with failed, stop the concurrence
    if 'failed' in outcome_map.values():
        return True
    # else, keep running
    return False
    
def outcome_cb(outcome_map):
    # if some of the states has terminated with failed, return failed
    if 'failed' in outcome_map.values():
        return 'failed'
    # else, return None, to return the defined outcome in the map
    return None

def main():
    rospy.init_node('injerrobot_grafting_pipeline')
    
    io_mod = io_module.IoModule(sim = True)
   
    #rootstock_params = rospy.get_param('/rootstock') ### XXX: this must be on userdata??
      

    both_arm_move_group = MoveGroupInterface(group = "both_arms", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "left_gripper")
    both_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
    
    joint_names = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    joint_positions = [-0.2285631371337891, 1.5056021415893555, -0.2051699301452637, 0.026077673364257814, 2.15025756696167, -1.3905535826293947, -0.09433981834716798, -0.041033986029052734, 1.491029324121094, 0.6304661031005859, 0.06902913537597656, -1.7571749905151368, -1.5692623442138673, 3.0530052596008304]
    
    #both_arm_move_group.moveToJointPoseCommander(joint_names, joint_positions, max_velocity_scaling_factor=1)


    rootstock_arm_move_group = MoveGroupInterface(group = "right_arm", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "right_gripper")
    rootstock_arm_move_group.setPlannerId('RRTConnectkConfigDefault')

    scion_arm_move_group = MoveGroupInterface(group = "left_arm", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "left_gripper")
    scion_arm_move_group.setPlannerId('RRTConnectkConfigDefault')

    goal_right = geometry_msgs.msg.PoseStamped()
    goal_right.header.frame_id = 'right_gripper'
    goal_right.pose.position.z = -0.1
    goal_right.pose.orientation.w = 1.0
    
    goal_left = geometry_msgs.msg.PoseStamped()
    goal_left.header.frame_id = 'left_gripper'
    goal_left.pose.position.z = -0.1
    goal_left.pose.orientation.w = 1.0


    goal = geometry_msgs.msg.Pose()
    goal.position.y = -0.1
    goal.orientation.w = 1.0
    
    plan_rootstock = rootstock_arm_move_group.plan_cartesian(goal_right)
    plan_scion = scion_arm_move_group.plan_cartesian(goal_left)
    
    print 'plan_rootstock is', len(plan_rootstock.joint_trajectory.points)
    print 'plan_scion is', len(plan_scion.joint_trajectory.points)
    
    both_arm_move_group.moveToPoseCartesianPathBothArmsCommander([plan_rootstock, plan_scion], max_velocity_scaling_factor=1)

    goal_right = geometry_msgs.msg.PoseStamped()
    goal_right.header.frame_id = 'right_gripper'
    goal_right.pose.position.x = -0.1
    goal_right.pose.orientation.w = 1.0
    
    goal_left = geometry_msgs.msg.PoseStamped()
    goal_left.header.frame_id = 'left_gripper'
    goal_left.pose.position.x = -0.1
    goal_left.pose.orientation.w = 1.0


    goal = geometry_msgs.msg.Pose()
    goal.position.y = -0.1
    goal.orientation.w = 1.0
    
    plan_rootstock = rootstock_arm_move_group.plan_cartesian(goal_right)
    plan_scion = scion_arm_move_group.plan_cartesian(goal_left)
    
    print 'plan_rootstock is', len(plan_rootstock.joint_trajectory.points)
    print 'plan_scion is', len(plan_scion.joint_trajectory.points)
    
    both_arm_move_group.moveToPoseCartesianPathBothArmsCommander([plan_rootstock, plan_scion], max_velocity_scaling_factor=1)

    goal_right = geometry_msgs.msg.PoseStamped()
    goal_right.header.frame_id = 'right_gripper'
    goal_right.pose.position.z = 0.1
    goal_right.pose.orientation.w = 1.0
    
    goal_left = geometry_msgs.msg.PoseStamped()
    goal_left.header.frame_id = 'left_gripper'
    goal_left.pose.position.z = 0.1
    goal_left.pose.orientation.w = 1.0


    goal = geometry_msgs.msg.Pose()
    goal.position.y = -0.1
    goal.orientation.w = 1.0
    
    plan_rootstock = rootstock_arm_move_group.plan_cartesian(goal_right)
    plan_scion = scion_arm_move_group.plan_cartesian(goal_left)
    
    print 'plan_rootstock is', len(plan_rootstock.joint_trajectory.points)
    print 'plan_scion is', len(plan_scion.joint_trajectory.points)
    
    both_arm_move_group.moveToPoseCartesianPathBothArmsCommander([plan_rootstock, plan_scion], max_velocity_scaling_factor=1)

    goal_right = geometry_msgs.msg.PoseStamped()
    goal_right.header.frame_id = 'right_gripper'
    goal_right.pose.position.x = 0.1
    goal_right.pose.orientation.w = 1.0
    
    goal_left = geometry_msgs.msg.PoseStamped()
    goal_left.header.frame_id = 'left_gripper'
    goal_left.pose.position.x = 0.1
    goal_left.pose.orientation.w = 1.0


    goal = geometry_msgs.msg.Pose()
    goal.position.y = -0.1
    goal.orientation.w = 1.0
    
    plan_rootstock = rootstock_arm_move_group.plan_cartesian(goal_right)
    plan_scion = scion_arm_move_group.plan_cartesian(goal_left)
    
    print 'plan_rootstock is', len(plan_rootstock.joint_trajectory.points)
    print 'plan_scion is', len(plan_scion.joint_trajectory.points)
    
    both_arm_move_group.moveToPoseCartesianPathBothArmsCommander([plan_rootstock, plan_scion], max_velocity_scaling_factor=1)



    rospy.spin()

if __name__ == '__main__':
    main()
