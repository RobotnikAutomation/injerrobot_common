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
def main():
    rospy.init_node('injerrobot_operation')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['dispensed', 'rejected', 'failed', 'completed'])
    sm.userdata.params = rospy.get_param('/rootstock') # XXX: this must be on userdata??
    
    io_mod = io_module.IoModule(sim = True)

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

    #path_constraints = None

    goto_pick_feeder = GoToGrid(sim = True)
    goto_pick_feeder.move_group = MoveGroupInterface("left_arm", "left_arm_base_link")
    goto_pick_feeder.move_group.setPlannerId('RRTConnectkConfigDefault')
    goto_pick_feeder.move_group.setPathConstraints(path_constraints)
    goto_pick_feeder.params = sm.userdata.params['feeder'] ### XXX: name: feeder???

    goto_cut = GoTo(sim = True)
    goto_cut.move_group = MoveGroupInterface("left_arm", "left_arm_base_link")
    goto_cut.move_group.setPlannerId('RRTConnectkConfigDefault')
    goto_cut.move_group.setPathConstraints(path_constraints)
    goto_cut.params = sm.userdata.params['cut']

    goto_place_clip = GoTo(sim = True)
    goto_place_clip.move_group = MoveGroupInterface("left_arm", "left_arm_base_link")
    goto_place_clip.move_group.setPlannerId('RRTConnectkConfigDefault')
    goto_place_clip.move_group.setPathConstraints(path_constraints)
    goto_place_clip.params = sm.userdata.params['clip']
    
    goto_dispense = GoTo(sim = True)
    goto_dispense.move_group = MoveGroupInterface("left_arm", "left_arm_base_link")
    goto_dispense.move_group.setPlannerId('RRTConnectkConfigDefault')
    goto_dispense.move_group.setPathConstraints(path_constraints)
    goto_dispense.params = sm.userdata.params['dispense']

    
    pick = Pick(sim = True)
    pick.io_module = io_mod

    cut = Cut(sim = True)
    cut.io_module = io_mod

    place = Place(sim = True)
    place.io_module = io_mod
    
    clip = Clip(sim = True)
    clip.io_module = io_mod
    
    dispense = Dispense(sim = True)
    dispense.io_module = io_mod
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GOTO_PICK', goto_pick_feeder, 
                               transitions={'reached':'PICK', 
                                            'failed':'failed',
                                            'grid_completed': 'completed'})
                                            
        smach.StateMachine.add('PICK', pick, 
                               transitions={'picked':'GOTO_CUT', 
                                            'failed':'failed'})
                                            
        smach.StateMachine.add('GOTO_CUT', goto_cut, 
                               transitions={'reached':'CUT', 
                                            'failed':'failed'})

        smach.StateMachine.add('CUT', cut, 
                               transitions={'cutted':'GOTO_PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_PLACE', goto_place_clip, 
                               transitions={'reached':'PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('PLACE', place,
                                transitions={'placed':'CLIP', 
                                            'failed':'failed'})

        smach.StateMachine.add('CLIP', clip, 
                               transitions={'clipped':'GOTO_DISPENSE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_DISPENSE', goto_dispense, 
                               transitions={'reached':'DISPENSE', 
                                            'failed':'failed'})

        smach.StateMachine.add('DISPENSE', dispense, 
                               transitions={'dispensed':'GOTO_PICK', 
                                            'failed':'failed'})
                                            
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
