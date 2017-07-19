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
      

    rootstock_arm_move_group = MoveGroupInterface(group = "both_arms", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "right_gripper")
    rootstock_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
    
    joint_names = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    joint_positions = [-0.11389807337036134, 1.9370342376892091, 0.3535825712036133, -0.1430437083068848, 0.33402431618041994, -0.7788787441589355, 0.29644178692016604, -0.013038836682128907, 1.606461378277588, 1.0676506271484376, 0.08360195284423828, 0.21667478604125978, -1.5508545747802736, 3.054539240386963]
    
    
    rootstock_arm_move_group.moveToJointPoseCommander(joint_names, joint_positions, max_velocity_scaling_factor=0.2)
    
    rospy.spin()

if __name__ == '__main__':
    main()
