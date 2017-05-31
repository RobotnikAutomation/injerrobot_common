#!/usr/bin/env python

import sys

import rospy

import tf
import tf.transformations

import smach
import smach_ros

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg



# define state GoTo
class GoTo(smach.State):
    def __init__(self, sim):
        self._sim = sim
        
        smach.State.__init__(self, outcomes=['reached','failed'], input_keys=['params'])
        
        
    def execute(self, userdata):
        rospy.loginfo('Executing state GOTO')
        #print userdata.params
        
        for pose in self.params['pose']:            
            goal = geometry_msgs.msg.PoseStamped()
            goal.header.frame_id = 'left_arm_base_link'
            
            goal.pose.position.x = pose[0][0]
            goal.pose.position.y = pose[0][1]
            goal.pose.position.z = pose[0][2]
            
            tf_quat = tf.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
            goal.pose.orientation.x = tf_quat[0]
            goal.pose.orientation.y = tf_quat[1]
            goal.pose.orientation.z = tf_quat[2]
            goal.pose.orientation.w = tf_quat[3]
            
            self.move_group.moveToPose(goal, 'left_arm_link_6')
            rospy.loginfo("move to pose")
            rospy.sleep(1.)
            
        
        return 'reached'
            
