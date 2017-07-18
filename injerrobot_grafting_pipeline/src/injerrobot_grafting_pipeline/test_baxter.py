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
  
    path_constraints = create_path_contraints()
    

    rootstock_arm_move_group = MoveGroupInterface(group = "right_arm", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "right_gripper")
    rootstock_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
    #rootstock_arm_move_group.setPathConstraints(path_constraints)

    rootstock_goto_init = GoTo(sim = True)
    rootstock_goto_init.move_group = rootstock_arm_move_group
    rootstock_goto_init.params = rootstock_params['init']
    rootstock_goto_init.joint_names = rootstock_params['joint_names']
    rootstock_goto_init.label = 'rootstock'
    rootstock_goto_init.constraints = None
    
    #rootstock_goto_pick_feeder = GoToGrid(sim = True)
    rootstock_goto_pick_feeder = GoTo(sim = True)
    rootstock_goto_pick_feeder.move_group = rootstock_arm_move_group
    rootstock_goto_pick_feeder.params = rootstock_params['feeder'] ### XXX: name: feeder???
    rootstock_goto_pick_feeder.joint_names = rootstock_params['joint_names']
    rootstock_goto_pick_feeder.label = 'rootstock'
    rootstock_goto_pick_feeder.constraints = path_constraints
    
    rootstock_goto_cut = GoTo(sim = True)
    rootstock_goto_cut.move_group = rootstock_arm_move_group
    rootstock_goto_cut.params = rootstock_params['cut']
    rootstock_goto_cut.joint_names = rootstock_params['joint_names']
    rootstock_goto_cut.label = 'rootstock'
    rootstock_goto_cut.constraints = path_constraints
    
    rootstock_goto_place_clip = GoTo(sim = True)
    rootstock_goto_place_clip.move_group = rootstock_arm_move_group
    rootstock_goto_place_clip.params = rootstock_params['clip']
    rootstock_goto_place_clip.joint_names = rootstock_params['joint_names']
    rootstock_goto_place_clip.label = 'rootstock'
    rootstock_goto_place_clip.constraints = path_constraints
    
    rootstock_goto_dispense = GoTo(sim = True)
    rootstock_goto_dispense.move_group = rootstock_arm_move_group
    rootstock_goto_dispense.params = rootstock_params['dispense']
    rootstock_goto_dispense.label = 'rootstock'
    
    rootstock_goto_inspection = GoTo(sim = True)
    rootstock_goto_inspection.move_group = rootstock_arm_move_group
    rootstock_goto_inspection.params = rootstock_params['inspection']
    rootstock_goto_inspection.label = 'rootstock'
    
    rootstock_goto_reject = GoTo(sim = True)
    rootstock_goto_reject.move_group = rootstock_arm_move_group
    rootstock_goto_reject.params = rootstock_params['reject']
    rootstock_goto_reject.label = 'rootstock'
    
    rootstock_pick = Pick(sim = True)
    rootstock_pick.move_group = rootstock_arm_move_group
    rootstock_pick.params = rootstock_params['pick']
    rootstock_pick.joint_names = rootstock_params['joint_names']
    rootstock_pick.label = 'rootstock'
    rootstock_pick.io_module = io_mod
    rootstock_pick.label = 'rootstock'
    
    rootstock_cut = Cut(sim = True)
    rootstock_cut.io_module = io_mod
    rootstock_cut.label = 'rootstock'
    
    rootstock_place = Place(sim = True)
    rootstock_place.move_group = rootstock_arm_move_group
    rootstock_place.params = rootstock_params['place']
    rootstock_place.joint_names = rootstock_params['joint_names']
    rootstock_place.label = 'rootstock'
    rootstock_place.io_module = io_mod
    rootstock_place.label = 'rootstock'
    
    rootstock_clip = Clip(sim = True)
    rootstock_clip.io_module = io_mod
    rootstock_clip.label = 'rootstock'
    
    rootstock_inspection = Inspection(sim = True)
    rootstock_inspection.io_module = io_mod
    rootstock_inspection.label = 'rootstock'
    
    rootstock_reject = Reject(sim = True)
    rootstock_reject.io_module = io_mod
    rootstock_reject.label = 'rootstock'
    
    rootstock_dispense = Dispense(sim = True)
    rootstock_dispense.io_module = io_mod
    rootstock_dispense.label = 'rootstock'
    

    ### XXX: right arm? why not scion arm?? the same applies for the left arm
    
    path_constraints = None
    
    scion_params = rospy.get_param('/scion') ### XXX: this must be on userdata??
    scion_arm_move_group = MoveGroupInterface(group = "left_arm", action_ns = "/move_group", fixed_frame = "base", gripper_frame = "left_gripper")
    scion_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
    scion_arm_move_group.setPathConstraints(path_constraints)

    scion_goto_init = GoTo(sim = True)
    scion_goto_init.move_group = scion_arm_move_group
    scion_goto_init.params = scion_params['init']
    scion_goto_init.label = 'scion'

    scion_goto_pick_feeder = GoToGrid(sim = True)
    scion_goto_pick_feeder.move_group = scion_arm_move_group
    scion_goto_pick_feeder.params = scion_params['feeder'] ### XXX: name: feeder???
    scion_goto_pick_feeder.label = 'scion'
    
    scion_goto_cut = GoTo(sim = True)
    scion_goto_cut.move_group = scion_arm_move_group
    scion_goto_cut.params = scion_params['cut']
    scion_goto_cut.label = 'scion'
    
    scion_goto_place_clip = GoTo(sim = True)
    scion_goto_place_clip.move_group = scion_arm_move_group
    scion_goto_place_clip.params = scion_params['clip']
    scion_goto_place_clip.label = 'scion'
    
    scion_pick = Pick(sim = True)
    scion_pick.io_module = io_mod
    scion_pick.label = 'scion'
    
    scion_cut = Cut(sim = True)
    scion_cut.io_module = io_mod
    scion_cut.label = 'scion'
    
    scion_place = Place(sim = True)
    scion_place.io_module = io_mod
    scion_place.label = 'scion'
    
    # Create a SMACH state machine
    sm_rootstock = smach.StateMachine(outcomes=['placed', 'failed', 'completed', 'rejected'])
    sm_rootstock.userdata.params = rootstock_params
    
    # Open the container
    with sm_rootstock:
        # Add states to the container
        smach.StateMachine.add('GOTO_PICK', rootstock_goto_pick_feeder, 
                               transitions={'reached':'PICK', 
                                            'failed':'failed',
                                            'grid_completed': 'completed'})
                                            
        smach.StateMachine.add('PICK', rootstock_pick, 
                               transitions={'picked':'GOTO_CUT',
                                            'no_plant': 'GOTO_PICK',
                                            'failed':'failed'})
                                            
        smach.StateMachine.add('GOTO_CUT', rootstock_goto_cut, 
                               transitions={'reached':'CUT', 
                                            'failed':'failed'})

        smach.StateMachine.add('CUT', rootstock_cut, 
                               transitions={'cutted':'GOTO_PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_PLACE', rootstock_goto_place_clip, 
                               transitions={'reached':'PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('PLACE', rootstock_place,
                                transitions={'placed':'placed', 
                                            'failed':'failed'})
 
 
     # Create a SMACH state machine
    sm_rootstock_with_esteroids = smach.StateMachine(outcomes=['placed', 'failed', 'completed', 'rejected'])
    sm_rootstock_with_esteroids.userdata.params = rootstock_params
    with sm_rootstock_with_esteroids:
        smach.StateMachine.add('INIT_ROOTSTOCK', rootstock_goto_init,
                                transitions={'reached':'ROOTSTOCK', 
                                'failed':'failed'})
                                
        smach.StateMachine.add('ROOTSTOCK', sm_rootstock,
                                transitions={'placed':'placed', 
                                'failed':'failed'})
                                  
 
 
     # Create a SMACH state machine
    sm_scion = smach.StateMachine(outcomes=['placed', 'failed', 'completed', 'rejected'])
    sm_scion.userdata.params = scion_params
    
    # Open the container
    with sm_scion:
        # Add states to the container
        smach.StateMachine.add('GOTO_PICK', scion_goto_pick_feeder, 
                               transitions={'reached':'PICK', 
                                            'failed':'failed',
                                            'grid_completed': 'completed'})
                                            
        smach.StateMachine.add('PICK', scion_pick, 
                               transitions={'picked':'GOTO_CUT',
                                            'no_plant': 'GOTO_PICK',
                                            'failed':'failed'})
                                            
        smach.StateMachine.add('GOTO_CUT', scion_goto_cut, 
                               transitions={'reached':'CUT', 
                                            'failed':'failed'})

        smach.StateMachine.add('CUT', scion_cut, 
                               transitions={'cutted':'GOTO_PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_PLACE', scion_goto_place_clip, 
                               transitions={'reached':'PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('PLACE', scion_place,
                                transitions={'placed':'placed', 
                                            'failed':'failed'})
        
    sm_concurrent_place = smach.Concurrence(outcomes=['failed', 'completed', 'placed'],
                            default_outcome='failed',
                            outcome_map = {'completed':{'PLACE_ROOTSTOCK':'completed','PLACE_SCION':'completed'},
                                            'placed':{'PLACE_ROOTSTOCK':'placed','PLACE_SCION':'placed'},
                                            'failed':{'PLACE_ROOTSTOCK':'failed','PLACE_SCION':'failed'}}, # we put them here to see the transition in the introspection
                            child_termination_cb = child_termination_cb,
                            outcome_cb = outcome_cb)
    
    with sm_concurrent_place:
        smach.Concurrence.add('PLACE_ROOTSTOCK', sm_rootstock)
        smach.Concurrence.add('PLACE_SCION', sm_scion)
        
    
    sm_sequential_place = smach.StateMachine(outcomes=['failed', 'completed', 'placed'])
    sm_sequential_place.userdata.params = rootstock_params
    
    with sm_sequential_place:
        smach.StateMachine.add('PLACE_ROOTSTOCK', sm_rootstock,
                                transitions = {'placed': 'PLACE_SCION',
                                 'failed':'failed',
                                 'completed':'PLACE_SCION',
                                 'rejected':'PLACE_ROOTSTOCK'})

        smach.StateMachine.add('PLACE_SCION', sm_scion,
                                transitions = {'placed': 'placed',
                                 'failed':'failed',
                                 'completed':'completed',
                                 'rejected':'PLACE_SCION'})

    sm_full_grafting_pipeline = smach.StateMachine(outcomes=['failed', 'completed'])
    sm_full_grafting_pipeline.userdata.params = rootstock_params
    
    

    with sm_full_grafting_pipeline:
        smach.StateMachine.add('INIT_ROOTSTOCK', rootstock_goto_init,
                                transitions={'reached':'INIT_SCION', 
                                'failed':'failed'})
                                
        smach.StateMachine.add('INIT_SCION', scion_goto_init,
                                transitions={'reached':'PLACE', 
                                'failed':'failed'})
                                
        smach.StateMachine.add('PLACE', sm_sequential_place,
                               transitions={'placed':'CLIP',
                                'completed': 'completed',
                                'failed':'failed'})
                               
        smach.StateMachine.add('CLIP', rootstock_clip, 
                                transitions={'clipped':'GOTO_INSPECTION', 
                                'failed':'failed'})
                                
        smach.StateMachine.add('GOTO_INSPECTION', rootstock_goto_inspection, 
                                transitions={'reached':'INSPECTION', 
                                'failed':'failed'})

        smach.StateMachine.add('INSPECTION', rootstock_inspection, 
                                transitions={'good':'GOTO_DISPENSE',
                                'bad': 'GOTO_REJECT',
                                'failed':'failed'})

        smach.StateMachine.add('GOTO_REJECT', rootstock_goto_reject, 
                                transitions={'reached':'REJECT',
                                'failed':'failed'})
    
        smach.StateMachine.add('REJECT', rootstock_reject, 
                                transitions={'rejected':'PLACE', 
                                'failed':'failed'})
                                    
        smach.StateMachine.add('GOTO_DISPENSE', rootstock_goto_dispense, 
                                transitions={'reached':'DISPENSE', 
                                'failed':'failed'})
    
        smach.StateMachine.add('DISPENSE', rootstock_dispense, 
                                transitions={'dispensed':'PLACE', 
                                'failed':'failed'})
    
    sm_to_execute = sm_rootstock_with_esteroids
    sis = smach_ros.IntrospectionServer('server_name', sm_to_execute, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm_to_execute.execute()
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
