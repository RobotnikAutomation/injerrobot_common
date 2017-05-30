#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Pick
class Pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['picked','failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state PICK')
        if self.counter < 3:
            self.counter += 1
            return 'picked'
        else:
            return 'failed'
