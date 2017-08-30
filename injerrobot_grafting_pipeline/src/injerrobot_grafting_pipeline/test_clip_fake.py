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

import baxter_interface

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
    gripper = baxter_interface.Gripper("right")
    
    rootstock_params = rospy.get_param('/rootstock') ### XXX: this must be on userdata??

    both_arm_move_group = MoveGroupInterface(group = "both_arms", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "left_gripper")
    both_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
        
    rootstock_arm_move_group = MoveGroupInterface(group = "right_arm", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "right_gripper")
    rootstock_arm_move_group.setPlannerId('RRTConnectkConfigDefault')

    scion_arm_move_group = MoveGroupInterface(group = "left_arm", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "left_gripper")
    scion_arm_move_group.setPlannerId('RRTConnectkConfigDefault')

    # pose de inicio
    # -0.22204371879272464, 0.7976700087890626, -0.4479223895507813, 0.40957286989746094, 0.49777676510009766, -1.2014904507385256, 0.07861651528930665, -0.02569417816772461, 1.9542915215332033, 0.468247634967041, -0.39423306203613284, -0.33325732578735356, -1.570796325, 3.05415574519043,

    # pose de ataque
    # -0.29797576770629886, 1.2072428786865235, -0.6925923249389649, 0.25885925765991213, 1.7809516927001954, -1.2505778358947754, -0.10392719826049805, -0.09970875109863282, 2.0739420228515626, 0.44332044719238284, -0.4498398655334473, -1.723427413220215, -1.570796325, 2.9057431041320805

    # pose de clipado
    # -0.32328645067749023, 1.4066603808837892, -0.6619127092163086, 0.11658253974609376, 1.7579419809082033, -1.2060923930969238, -0.050237870745849615, -0.09242234236450196, 1.9397187040649415, 0.43373306727905275, -0.31331557556762696, -1.7176749852722168, -1.5692623442138673, 2.9026751425598145,
    
    gripper.close()

    joint_names = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    
    joint_positions = [-0.22204371879272464, 0.7976700087890626, -0.4479223895507813, 0.40957286989746094, 0.49777676510009766, -1.2014904507385256, 0.07861651528930665, -0.02569417816772461, 1.9542915215332033, 0.468247634967041, -0.39423306203613284, -0.33325732578735356, -1.570796325, 3.05415574519043]
    both_arm_move_group.moveToJointPoseCommander(joint_names, joint_positions, max_velocity_scaling_factor=0.5)

    joint_positions = [-0.29797576770629886, 1.2072428786865235, -0.6925923249389649, 0.25885925765991213, 1.7809516927001954, -1.2505778358947754, -0.10392719826049805, -0.09970875109863282, 2.0739420228515626, 0.44332044719238284, -0.4498398655334473, -1.723427413220215, -1.570796325, 2.9057431041320805]
    both_arm_move_group.moveToJointPoseCommander(joint_names, joint_positions, max_velocity_scaling_factor=0.5)

    rospy.sleep(1)

    goal_right = geometry_msgs.msg.PoseStamped()
    goal_right.header.frame_id = 'right_gripper'
    goal_right.pose.orientation.w = 1.0
    
    goal_left = geometry_msgs.msg.PoseStamped()
    goal_left.header.frame_id = 'left_gripper'
    goal_left.pose.orientation.w = 1.0

    
    goal_right.pose.position.x = -0.05
    
    goal_left.pose.position.x = -0.03
    goal_left.pose.position.z = -0.005
    goal_left.pose.position.y = 0.005
    
    plan_rootstock = rootstock_arm_move_group.plan_cartesian(goal_right)
    plan_scion = scion_arm_move_group.plan_cartesian(goal_left)
    
    both_arm_move_group.moveToPoseCartesianPathBothArmsCommander([plan_rootstock, plan_scion])


    rospy.sleep(8)
    gripper.open()

    goal_right = geometry_msgs.msg.PoseStamped()
    goal_right.header.frame_id = 'right_gripper'
    goal_right.pose.orientation.w = 1.0
    
    goal_left = geometry_msgs.msg.PoseStamped()
    goal_left.header.frame_id = 'left_gripper'
    goal_left.pose.orientation.w = 1.0

    
    goal_right.pose.position.z = -0.1
    
    plan_rootstock = rootstock_arm_move_group.plan_cartesian(goal_right)
    plan_scion = scion_arm_move_group.plan_cartesian(goal_left)
    
    both_arm_move_group.moveToPoseCartesianPathBothArmsCommander([plan_rootstock, plan_scion])



    goal_right = geometry_msgs.msg.PoseStamped()
    goal_right.header.frame_id = 'right_gripper'
    goal_right.pose.orientation.w = 1.0
    
    goal_left = geometry_msgs.msg.PoseStamped()
    goal_left.header.frame_id = 'left_gripper'
    goal_left.pose.orientation.w = 1.0

    goal_left.pose.position.y = 0.1
    
    plan_rootstock = rootstock_arm_move_group.plan_cartesian(goal_right)
    plan_scion = scion_arm_move_group.plan_cartesian(goal_left)
    
    both_arm_move_group.moveToPoseCartesianPathBothArmsCommander([plan_rootstock, plan_scion])


    goal_right = geometry_msgs.msg.PoseStamped()
    goal_right.header.frame_id = 'right_gripper'
    goal_right.pose.orientation.w = 1.0
    
    goal_left = geometry_msgs.msg.PoseStamped()
    goal_left.header.frame_id = 'left_gripper'
    goal_left.pose.orientation.w = 1.0

    goal_left.pose.position.z = -0.1
    
    plan_rootstock = rootstock_arm_move_group.plan_cartesian(goal_right)
    plan_scion = scion_arm_move_group.plan_cartesian(goal_left)
    
    both_arm_move_group.moveToPoseCartesianPathBothArmsCommander([plan_rootstock, plan_scion])



    rospy.spin()

if __name__ == '__main__':
    main()
