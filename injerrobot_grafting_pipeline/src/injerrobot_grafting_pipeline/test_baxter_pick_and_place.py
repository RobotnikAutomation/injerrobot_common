#!/usr/bin/env python

import rospy
import smach
import smach_ros

import moveit_msgs.msg

### XXX: check the way to import things
from injerrobot_grafting_pipeline.place import Place
from injerrobot_grafting_pipeline.pick import Pick
from injerrobot_grafting_pipeline.goto import GoTo
from injerrobot_grafting_pipeline.gotogrid import GoToGrid
from injerrobot_grafting_pipeline.cut import Cut
from injerrobot_grafting_pipeline.clip import Clip
from injerrobot_grafting_pipeline.inspection import Inspection
from injerrobot_grafting_pipeline.dispense import Dispense
from injerrobot_grafting_pipeline.reject import Reject

import io_module 

from moveit_python import MoveGroupInterface

# gets called when ANY child state terminates
def child_termination_cb(outcome_map):
    # if some of the states has terminated with failed, stop the concurrence
    if 'failed' in outcome_map.values():
        return True
    # else, keep running
    return False
    
def outcome_cb(outcome_map):
    # if some of the states has terminated with failed, return failed
    if 'failed' in outcome_map.values():
        return 'failed'
    # else, return None, to return the defined outcome in the map
    return None

def main():
    rospy.init_node('injerrobot_grafting_pipeline')

    io_mod = io_module.IoModule(sim = True)
   
    rootstock_params = rospy.get_param('/rootstock') ### XXX: this must be on userdata??
  
    path_constraints = None
    

    rootstock_arm_move_group = MoveGroupInterface(group = "right_arm", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "right_gripper")
    rootstock_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
    #rootstock_arm_move_group.setPathConstraints(path_constraints)

    rootstock_goto_init = GoTo(sim = True)
    rootstock_goto_init.move_group = rootstock_arm_move_group
    rootstock_goto_init.params = rootstock_params['init']
    rootstock_goto_init.joint_names = rootstock_params['joint_names']
    rootstock_goto_init.label = 'rootstock'
    rootstock_goto_init.constraints = None
    
    rootstock_goto_pick_feeder = GoToGrid(sim = True)
    #rootstock_goto_pick_feeder = GoTo(sim = True)
    rootstock_goto_pick_feeder.move_group = rootstock_arm_move_group
    rootstock_goto_pick_feeder.params = rootstock_params['feeder'] ### XXX: name: feeder???
    rootstock_goto_pick_feeder.joint_names = rootstock_params['joint_names']
    rootstock_goto_pick_feeder.label = 'rootstock'
    rootstock_goto_pick_feeder.constraints = path_constraints
    
    rootstock_goto_place_clip = GoToGrid(sim = True)
    rootstock_goto_place_clip.move_group = rootstock_arm_move_group
    rootstock_goto_place_clip.params = rootstock_params['clip']
    rootstock_goto_place_clip.joint_names = rootstock_params['joint_names']
    rootstock_goto_place_clip.label = 'rootstock'
    rootstock_goto_place_clip.constraints = path_constraints
        
    rootstock_pick = Pick(sim = True)
    rootstock_pick.move_group = rootstock_arm_move_group
    rootstock_pick.params = rootstock_params['pick']
    rootstock_pick.joint_names = rootstock_params['joint_names']
    rootstock_pick.label = 'rootstock'
    rootstock_pick.io_module = io_mod
    rootstock_pick.label = 'rootstock'
        
    rootstock_place = Place(sim = True)
    rootstock_place.move_group = rootstock_arm_move_group
    rootstock_place.params = rootstock_params['place']
    rootstock_place.joint_names = rootstock_params['joint_names']
    rootstock_place.label = 'rootstock'
    rootstock_place.io_module = io_mod
    rootstock_place.label = 'rootstock'
        
    
    # Create a SMACH state machine
    sm_rootstock = smach.StateMachine(outcomes=['placed', 'failed', 'completed', 'rejected'])
    sm_rootstock.userdata.params = rootstock_params
    
    # Open the container
    with sm_rootstock:
        # Add states to the container
        smach.StateMachine.add('INIT_ROOTSTOCK', rootstock_goto_init,
                               transitions={'reached':'GOTO_PICK', 
                               'failed':'failed'})
                               
        smach.StateMachine.add('GOTO_PICK', rootstock_goto_pick_feeder, 
                               transitions={'reached':'PICK', 
                                            'failed':'failed',
                                            'grid_completed': 'completed'})
                                            
        smach.StateMachine.add('PICK', rootstock_pick, 
                               transitions={'picked':'GOTO_PLACE',
                                            'no_plant': 'GOTO_PICK',
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_PLACE', rootstock_goto_place_clip, 
                               transitions={'reached':'PLACE', 
                                            'failed':'failed',
                                            'grid_completed': 'completed'})

        smach.StateMachine.add('PLACE', rootstock_place,
                                transitions={'placed':'GOTO_PICK', 
                                            'failed':'failed'})
 
    sis = smach_ros.IntrospectionServer('server_name', sm_rootstock, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm_rootstock.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
