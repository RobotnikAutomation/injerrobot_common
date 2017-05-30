#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Dispense
class Dispense(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dispensed','failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state DISPENSE')
        if self.counter < 3:
            self.counter += 1
            return 'dispensed'
        else:
            return 'failed'
