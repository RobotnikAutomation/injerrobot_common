#!/usr/bin/env python

import rospy
import smach
import smach_ros

import moveit_msgs.msg

### XXX: check the way to import things
from injerrobot_operation.place import Place
from injerrobot_operation.pick import Pick
from injerrobot_operation.goto import GoTo
from injerrobot_operation.gotogrid import GoToGrid
from injerrobot_operation.cut import Cut
from injerrobot_operation.clip import Clip
from injerrobot_operation.inspection import Inspection
from injerrobot_operation.dispense import Dispense
from injerrobot_operation.reject import Reject

import io_module 

from moveit_python import MoveGroupInterface

# main

def create_path_contraints():
    orientation_constraint = moveit_msgs.msg.OrientationConstraint()
    orientation_constraint.header.frame_id = 'left_arm_link_6'
    orientation_constraint.link_name = 'left_arm_link_6'
    orientation_constraint.orientation.w = 1.0
    orientation_constraint.absolute_x_axis_tolerance = 0.1
    orientation_constraint.absolute_y_axis_tolerance = 0.1
    orientation_constraint.absolute_z_axis_tolerance = 3.14
    orientation_constraint.weight = 1.0
    
    joint_contraint = moveit_msgs.msg.JointConstraint()
    joint_contraint.joint_name = 'left_arm_joint_a2'
    joint_contraint.position = -0.35
    joint_contraint.tolerance_above = 1.2
    joint_contraint.tolerance_below = 1.2
    joint_contraint.weight = 1
    
    path_constraints = moveit_msgs.msg.Constraints()
    path_constraints.name = "keep horizontal"
    path_constraints.orientation_constraints.append(orientation_constraint)
    path_constraints.joint_constraints.append(joint_contraint)
    
    return path_constraints


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
    rospy.init_node('injerrobot_operation')
    
    io_mod = io_module.IoModule(sim = True)
    
    rootstock_params = rospy.get_param('/rootstock') ### XXX: this must be on userdata??
    
    path_constraints = None
    
    rootstock_arm_move_group = MoveGroupInterface("left_arm", fixed_frame = "left_arm_base_link", gripper_frame = "left_arm_link_6")
    rootstock_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
    rootstock_arm_move_group.setPathConstraints(path_constraints)

    rootstock_goto_pick_feeder = GoToGrid(sim = True)
    rootstock_goto_pick_feeder.move_group = rootstock_arm_move_group
    rootstock_goto_pick_feeder.params = rootstock_params['feeder'] ### XXX: name: feeder???
    rootstock_goto_pick_feeder.label = 'rootstock'
    
    rootstock_goto_cut = GoTo(sim = True)
    rootstock_goto_cut.move_group = rootstock_arm_move_group
    rootstock_goto_cut.params = rootstock_params['cut']
    rootstock_goto_cut.label = 'rootstock'
    
    rootstock_goto_place_clip = GoTo(sim = True)
    rootstock_goto_place_clip.move_group = rootstock_arm_move_group
    rootstock_goto_place_clip.params = rootstock_params['clip']
    rootstock_goto_place_clip.label = 'rootstock'
    
    rootstock_goto_dispense = GoTo(sim = True)
    rootstock_goto_dispense.move_group = rootstock_arm_move_group
    rootstock_goto_dispense.params = rootstock_params['dispense']
    rootstock_goto_dispense.label = 'rootstock'
    
    rootstock_pick = Pick(sim = True)
    rootstock_pick.io_module = io_mod
    rootstock_pick.label = 'rootstock'
    
    rootstock_cut = Cut(sim = True)
    rootstock_cut.io_module = io_mod
    rootstock_cut.label = 'rootstock'
    
    rootstock_place = Place(sim = True)
    rootstock_place.io_module = io_mod
    rootstock_place.label = 'rootstock'
    
    rootstock_clip = Clip(sim = True)
    rootstock_clip.io_module = io_mod
    rootstock_clip.label = 'rootstock'
    
    rootstock_dispense = Dispense(sim = True)
    rootstock_dispense.io_module = io_mod
    rootstock_dispense.label = 'rootstock'
    

    ### XXX: right arm? why not scion arm?? the same applies for the left arm
    scion_params = rospy.get_param('/scion') ### XXX: this must be on userdata??
    scion_arm_move_group = MoveGroupInterface("right_arm", fixed_frame = "right_arm_base_link", gripper_frame = "right_arm_link_6")
    scion_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
    scion_arm_move_group.setPathConstraints(path_constraints)

    scion_goto_pick_feeder = GoToGrid(sim = True)
    scion_goto_pick_feeder.move_group = scion_arm_move_group
    scion_goto_pick_feeder.params = scion_params['feeder'] ### XXX: name: feeder???
    scion_goto_pick_feeder.label = 'scion'
    
    scion_goto_cut = GoTo(sim = True)
    scion_goto_cut.move_group = scion_arm_move_group
    scion_goto_cut.params = scion_params['cut']
    scion_goto_cut.label = 'scion'
    
    scion_goto_place_clip = GoTo(sim = True)
    scion_goto_place_clip.move_group = scion_arm_move_group
    scion_goto_place_clip.params = scion_params['clip']
    scion_goto_place_clip.label = 'scion'
    
    scion_pick = Pick(sim = True)
    scion_pick.io_module = io_mod
    scion_pick.label = 'scion'
    
    scion_cut = Cut(sim = True)
    scion_cut.io_module = io_mod
    scion_cut.label = 'scion'
    
    scion_place = Place(sim = True)
    scion_place.io_module = io_mod
    scion_place.label = 'scion'
    
    # Create a SMACH state machine
    sm_rootstock = smach.StateMachine(outcomes=['placed', 'failed', 'completed', 'rejected'])
    sm_rootstock.userdata.params = rootstock_params
    
    # Open the container
    with sm_rootstock:
        # Add states to the container
        smach.StateMachine.add('GOTO_PICK', rootstock_goto_pick_feeder, 
                               transitions={'reached':'PICK', 
                                            'failed':'failed',
                                            'grid_completed': 'completed'})
                                            
        smach.StateMachine.add('PICK', rootstock_pick, 
                               transitions={'picked':'GOTO_CUT', 
                                            'failed':'failed'})
                                            
        smach.StateMachine.add('GOTO_CUT', rootstock_goto_cut, 
                               transitions={'reached':'CUT', 
                                            'failed':'failed'})

        smach.StateMachine.add('CUT', rootstock_cut, 
                               transitions={'cutted':'GOTO_PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_PLACE', rootstock_goto_place_clip, 
                               transitions={'reached':'PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('PLACE', rootstock_place,
                                transitions={'placed':'placed', 
                                            'failed':'failed'})
 
 
     # Create a SMACH state machine
    sm_scion = smach.StateMachine(outcomes=['placed', 'failed', 'completed', 'rejected'])
    sm_scion.userdata.params = scion_params
    
    # Open the container
    with sm_scion:
        # Add states to the container
        smach.StateMachine.add('GOTO_PICK', scion_goto_pick_feeder, 
                               transitions={'reached':'PICK', 
                                            'failed':'failed',
                                            'grid_completed': 'completed'})
                                            
        smach.StateMachine.add('PICK', scion_pick, 
                               transitions={'picked':'GOTO_CUT', 
                                            'failed':'failed'})
                                            
        smach.StateMachine.add('GOTO_CUT', scion_goto_cut, 
                               transitions={'reached':'CUT', 
                                            'failed':'failed'})

        smach.StateMachine.add('CUT', scion_cut, 
                               transitions={'cutted':'GOTO_PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_PLACE', scion_goto_place_clip, 
                               transitions={'reached':'PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('PLACE', scion_place,
                                transitions={'placed':'placed', 
                                            'failed':'failed'})
        
    sm_concurrent_place = smach.Concurrence(outcomes=['failed', 'completed', 'placed'],
                            default_outcome='failed',
                            outcome_map = {'completed':{'PLACE_ROOTSTOCK':'completed','PLACE_SCION':'completed'},
                                            'placed':{'PLACE_ROOTSTOCK':'placed','PLACE_SCION':'placed'},
                                            'failed':{'PLACE_ROOTSTOCK':'failed','PLACE_SCION':'failed'}}, # we put them here to see the transition in the introspection
                            child_termination_cb = child_termination_cb,
                            outcome_cb = outcome_cb)
    
    with sm_concurrent_place:
        smach.Concurrence.add('PLACE_ROOTSTOCK', sm_rootstock)
        smach.Concurrence.add('PLACE_SCION', sm_scion)

    sm_full_operation = smach.StateMachine(outcomes=['failed', 'completed'])
    sm_full_operation.userdata.params = rootstock_params
    with sm_full_operation:
        smach.StateMachine.add('CONCURRENT_PLACE', sm_concurrent_place,
                               transitions={'placed':'CLIP',
                                'completed': 'completed',
                                'failed':'failed'})
                               
        smach.StateMachine.add('CLIP', rootstock_clip, 
                                transitions={'clipped':'GOTO_DISPENSE', 
                                'failed':'failed'})
    
        smach.StateMachine.add('GOTO_DISPENSE', rootstock_goto_dispense, 
                                transitions={'reached':'DISPENSE', 
                                'failed':'failed'})
    
        smach.StateMachine.add('DISPENSE', rootstock_dispense, 
                                transitions={'dispensed':'CONCURRENT_PLACE', 
                                'failed':'failed'})
    
    sis = smach_ros.IntrospectionServer('server_name', sm_full_operation, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm_full_operation.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
