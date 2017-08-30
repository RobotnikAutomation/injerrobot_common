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



# define state MoveCartesian
class MoveCartesian(smach.State):
    def __init__(self, sim):
        self._sim = sim
        self.constraints = None
        
        smach.State.__init__(self, outcomes=['reached','failed'], input_keys=['params'])
        
        
    def execute(self, userdata):
        rospy.loginfo('Executing state MOVECARTESIAN')
        #print userdata.params
        
        pose = geometry_msgs.msg.Pose()
        pose.position.z = 0.1
        
        #print self.move_group.moveToPoseCartesian(pose, max_velocity_scaling_factor=0.01)
        #self.move_group.moveToJointPoseCommander(self.joint_names,self.params['joints'][0], max_velocity_scaling_factor=0.2)
        
        #if self.params.has_key('joints'):
        #    print self.move_group.moveToJointPosition(self.joint_names,self.params['joints'][0], max_velocity_scaling_factor=0.5)
        #    print "RECORD NOW THE CARTESIAN POSITION FOR:",  self.label, self.params['joints'][0]
        #    rospy.sleep(5)
        #    return 'reached'
            
            
        self.move_group.setPathConstraints(self.constraints)
        for pose in self.params['poses']:            
            goal = geometry_msgs.msg.PoseStamped()
            goal.header.frame_id = self.move_group.getFixedFrame()
            
            goal.pose.position.x = pose[0][0]
            goal.pose.position.y = pose[0][1]
            goal.pose.position.z = pose[0][2]
            
            tf_quat = tf.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
            goal.pose.orientation.x = tf_quat[0]
            goal.pose.orientation.y = tf_quat[1]
            goal.pose.orientation.z = tf_quat[2]
            goal.pose.orientation.w = tf_quat[3]
            
            
            rospy.loginfo("must go to: %f %f %f, %f %f %f %f" % (goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w))

            print self.move_group.moveToPoseCommander(goal, max_velocity_scaling_factor=0.2)
            rospy.loginfo("move to pose")
            rospy.sleep(1.)
        
        return 'reached'
            