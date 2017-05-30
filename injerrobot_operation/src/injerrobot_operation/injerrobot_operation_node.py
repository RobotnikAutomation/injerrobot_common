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

# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['dispensed', 'rejected', 'failed'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('GOTO_PICK', GoTo(), 
                               transitions={'reached':'PICK', 
                                            'failed':'failed'})
                                            
        smach.StateMachine.add('PICK', Pick(), 
                               transitions={'picked':'GOTO_CUT', 
                                            'failed':'failed'})
                                            
        smach.StateMachine.add('GOTO_CUT', GoTo(), 
                               transitions={'reached':'CUT', 
                                            'failed':'failed'})

        smach.StateMachine.add('CUT', Cut(), 
                               transitions={'cutted':'GOTO_PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_PLACE', GoTo(), 
                               transitions={'reached':'PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('PLACE', Place(), 
                               transitions={'placed':'CLIP', 
                                            'failed':'failed'})

        smach.StateMachine.add('CLIP', Clip(), 
                               transitions={'clipped':'GOTO_DISPENSE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_DISPENSE', GoTo(), 
                               transitions={'reached':'DISPENSE', 
                                            'failed':'failed'})

        smach.StateMachine.add('DISPENSE', Dispense(), 
                               transitions={'dispensed':'dispensed', 
                                            'failed':'failed'})
                                            
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
