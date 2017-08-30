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
   
    rootstock_params = rospy.get_param('/rootstock') ### XXX: this must be on userdata??
      

    both_arm_move_group = MoveGroupInterface(group = "both_arms", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "left_gripper")
    both_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
    
    joint_names = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    joint_positions = [-0.02032524541625977, -0.34783014325561523, 1.4714710690979005, -0.6350680454589844, 0.13920875634155275, 1.629087594873047, -1.1742622917846681, 0.04180097642211914, -0.11504855895996094, 1.931281809741211, 0.4640291878051758, -0.36010198954467776, -1.7472041154052735, -1.5389662236877442, 3.0299955478088383]
    
    #both_arm_move_group.moveToJointPoseCommander(joint_names, joint_positions, max_velocity_scaling_factor=1)


    
    rootstock_arm_move_group = MoveGroupInterface(group = "right_arm", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "right_gripper")
    rootstock_arm_move_group.setPlannerId('RRTConnectkConfigDefault')

    scion_arm_move_group = MoveGroupInterface(group = "left_arm", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "left_gripper")
    scion_arm_move_group.setPlannerId('RRTConnectkConfigDefault')

    goal_right = geometry_msgs.msg.PoseStamped()
    goal_right.header.frame_id = 'right_gripper'
    goal_right.pose.position.x = 0.05
    goal_right.pose.orientation.w = 1.0
    

    goal_left = geometry_msgs.msg.PoseStamped()
    goal_left.header.frame_id = 'left_gripper'
    goal_left.pose.position.x = 0.05
    goal_left.pose.orientation.w = 1.0

    plan_rootstock = rootstock_arm_move_group.plan_cartesian(goal_right)
    plan_scion = scion_arm_move_group.plan_cartesian(goal_left)
    
    print 'plan_rootstock is', len(plan_rootstock.joint_trajectory.points)
    print 'plan_scion is', len(plan_scion.joint_trajectory.points)
    
    both_arm_move_group.moveToPoseCartesianPathBothArmsCommander([plan_rootstock, plan_scion])

    both_arm_move_group.moveToPoseCartesianPathBothArmsCommander([plan_rootstock, plan_scion])

    rospy.spin()

if __name__ == '__main__':
    main()
