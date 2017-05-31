#!/usr/bin/env python

import rospy
import smach
import smach_ros


### XXX: check the way to import things
from injerrobot_operation.place import Place
from injerrobot_operation.pick import Pick
from injerrobot_operation.goto import GoTo
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
    sm = smach.StateMachine(outcomes=['dispensed', 'rejected', 'failed'])
    sm.userdata.params = rospy.get_param('/rootstock') # XXX: this must be on userdata??
    
    io_mod = io_module.IoModule(sim = True)

    goto_pick = GoTo(sim = True)
    goto_pick.move_group = MoveGroupInterface("left_arm", "left_arm_base_link")
    goto_pick.params = sm.userdata.params['pick']
    
    goto = GoTo(sim = True)
    goto.move_group = MoveGroupInterface("left_arm", "left_arm_base_link")
    goto.params = sm.userdata.params['conveyor']
    
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
        smach.StateMachine.add('GOTO_PICK', goto_pick, 
                               transitions={'reached':'PICK', 
                                            'failed':'failed'})
                                            
        smach.StateMachine.add('PICK', pick, 
                               transitions={'picked':'GOTO_CUT', 
                                            'failed':'failed'})
                                            
        smach.StateMachine.add('GOTO_CUT', goto, 
                               transitions={'reached':'CUT', 
                                            'failed':'failed'})

        smach.StateMachine.add('CUT', cut, 
                               transitions={'cutted':'GOTO_PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_PLACE', goto, 
                               transitions={'reached':'PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('PLACE', place,
                                transitions={'placed':'CLIP', 
                                            'failed':'failed'})

        smach.StateMachine.add('CLIP', clip, 
                               transitions={'clipped':'GOTO_DISPENSE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_DISPENSE', goto, 
                               transitions={'reached':'DISPENSE', 
                                            'failed':'failed'})

        smach.StateMachine.add('DISPENSE', dispense, 
                               transitions={'dispensed':'dispensed', 
                                            'failed':'failed'})
                                            
    # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()

    ## Execute SMACH plan
    outcome = sm.execute()
    # rospy.spin()
    # sis.stop()

if __name__ == '__main__':
    main()
