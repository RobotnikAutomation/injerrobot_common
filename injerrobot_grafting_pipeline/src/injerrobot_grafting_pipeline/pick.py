#!/usr/bin/env python

import rospy
import smach
import smach_ros

import geometry_msgs.msg

import baxter_interface

import moveit_commander
import tf
import tf.transformations

# define state Pick
class Pick(smach.State):
    def __init__(self, sim):
        self._sim = sim
        
        smach.State.__init__(self, outcomes=['picked','failed','no_plant'], input_keys=['params'])
        
        self._wait_for_gripper = 1.0
        self._gripper = baxter_interface.Gripper("right")
        
    def execute(self, userdata):
        rospy.loginfo('Executing state PICK')

        self._gripper.open()
        rospy.sleep(self._wait_for_gripper)


        if 'prepick' in self.params.keys():
            rospy.loginfo('PICK: executing PREPICK CARTESIAN MOVEMENT.')
            for pose in self.params['prepick']:
                goal = geometry_msgs.msg.Pose()
            
                goal.position.x = pose[0][0]
                goal.position.y = pose[0][1]
                goal.position.z = pose[0][2]
                
                tf_quat = tf.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
                goal.orientation.x = tf_quat[0]
                goal.orientation.y = tf_quat[1]
                goal.orientation.z = tf_quat[2]
                goal.orientation.w = tf_quat[3]
                
                self.move_group.moveToPoseCartesianPathCommander(goal, max_velocity_scaling_factor=0.01, wait=False)


        if self._sim == True:
            self.io_module.set_input(userdata.params['input']['gripper'], True)
            rospy.sleep(self._wait_for_gripper)

        #~ if self.io_module.get_input(userdata.params['input']['gripper']) != True:
            #~ rospy.logerr('PICK: there is not plant. ')
            #~ return 'no_plant'

        plant_detected = False
        wait_time = rospy.Duration(3)
        init_time = rospy.Time.now()
        while (rospy.Time.now() - init_time) < wait_time :
            rospy.sleep(0.01)
            if self.io_module.get_input(1): # 0 index
                rospy.loginfo('PICK: still no plant')
            else:
                rospy.loginfo('PICK: plant detected, closing gripper')
                plant_detected = True
                break

        self.move_group.stop()

        if not plant_detected:
            rospy.logerr('PICK: there is not plant. ')
            return 'no_plant'
                
        rospy.loginfo('PICK: plant detected, closing gripper')
        #self.io_module.set_output(userdata.params['output']['gripper'], True)
        self._gripper.close()
        
        rospy.sleep(self._wait_for_gripper)
        
        rospy.loginfo('PICK: Closed gripper')
        
        
        if 'postpick' in self.params.keys():
            rospy.loginfo('PICK: executing POSTPICK CARTESIAN MOVEMENT.')
            for pose in self.params['postpick']:
                goal = geometry_msgs.msg.Pose()
                
                goal.position.x = pose[0][0]
                goal.position.y = pose[0][1]
                goal.position.z = pose[0][2]
                
                tf_quat = tf.transformations.quaternion_from_euler(pose[1][0], pose[1][1], pose[1][2])
                goal.orientation.x = tf_quat[0]
                goal.orientation.y = tf_quat[1]
                goal.orientation.z = tf_quat[2]
                goal.orientation.w = tf_quat[3]
                
                self.move_group.moveToPoseCartesianPathCommander(goal, max_velocity_scaling_factor=0.01)

        return 'picked'
