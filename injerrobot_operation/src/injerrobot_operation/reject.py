#!/usr/bin/env python

import rospy
import smach
import smach_ros

# define state Reject
class Reject(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rejected','failed'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state REJECT')
        if self.counter < 3:
            self.counter += 1
            return 'rejected'
        else:
            return 'failed'
