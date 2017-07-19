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
    
    io_mod = io_module.IoModule(sim = False)
    gripper = baxter_interface.Gripper("right")
    
    rootstock_params = rospy.get_param('/rootstock') ### XXX: this must be on userdata??
      

    rootstock_arm_move_group = MoveGroupInterface(group = "right_arm", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "right_gripper")
    rootstock_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
    
    joint_names = ['right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    joint_positions = [-0.0015339807861328126, 1.4208497031555176, 0.4118738410766602, 0.13882526114501953, -0.2780340174865723, -1.4960147616760255, 3.0349809853637697] # pick
    
    #joint_positions = [-0.14342720350341798, 1.4039759145080568, 1.2202817153686525, -0.09165535197143555, 0.5813787179443359, -1.2870098795654297, 3.0518547740112307] # place

    rootstock_arm_move_group.moveToJointPoseCommander(joint_names, joint_positions, max_velocity_scaling_factor=0.2)
    
    pose = geometry_msgs.msg.Pose()
    pose.position.x = 0#0.05
    pose.position.y = 0#.045
    pose.position.z = 0.09
    pose.orientation.w = 1

    rootstock_arm_move_group.moveToPoseCartesianPathCommander(pose, max_velocity_scaling_factor=0.01, wait=False)
    
    while not rospy.is_shutdown():
        rospy.sleep(0.01)
        if io_mod.get_input(1): # 0 index
            rospy.loginfo('todavia no hay planta')
        else:
            rospy.loginfo('HAY PLANTA')
            break
        
    rospy.loginfo('stopping!')
    
    rootstock_arm_move_group.stop()
    
    gripper.close()
    
    rospy.spin()

if __name__ == '__main__':
    main()
