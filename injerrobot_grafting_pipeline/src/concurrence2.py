#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO (counter %d)' % self.counter)
        rospy.sleep(1.)
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.sleep(2.)
        return 'outcome1'
        


# define state Bas
class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAS')
        rospy.sleep(3.)
        return 'outcome3'




def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome6'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('BAS', Bas(),
                               transitions={'outcome3':'CON'})

        # Create the sub SMACH state machine
        sm_con = smach.Concurrence(outcomes=['outcome4','outcome5'],
                                   default_outcome='outcome4',
                                   outcome_map={'outcome5':
                                       { 'FOO':'outcome2',
                                         'BAR':'outcome1'}})

        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('FOO', Foo())
            smach.Concurrence.add('BAR', Bar())

        smach.StateMachine.add('CON', sm_con,
                               transitions={'outcome4':'CON',
                                            'outcome5':'outcome6'})


    sis = smach_ros.IntrospectionServer('server_name', sm_top, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm_top.execute()

    rospy.spin()
    sis.stop()



if __name__ == '__main__':
    main()
