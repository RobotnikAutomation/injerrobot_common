def main():
    rospy.init_node('injerrobot_grafting_pipeline')

    # Create a SMACH state machine
    sm_rootstock = smach.StateMachine(outcomes=['dispensed', 'rejected', 'failed', 'completed'])
    sm_rootstock.userdata.params = rospy.get_param('/rootstock') # XXX: this must be on userdata??
    
    io_mod = io_module.IoModule(sim = True)

    path_constraints = None
    
    left_arm_move_group = MoveGroupInterface("left_arm", fixed_frame = "left_arm_base_link", gripper_frame = "left_arm_link_a6")
    left_arm_move_group.setPlannerId('RRTConnectkConfigDefault')
    left_arm_move_group.setPathConstraints(path_constraints)

    goto_pick_feeder = GoToGrid(sim = True)
    goto_pick_feeder.move_group = left_arm_move_group
    goto_pick_feeder.params = sm_rootstock.userdata.params['feeder'] ### XXX: name: feeder???

    goto_cut = GoTo(sim = True)
    goto_cut.move_group = left_arm_move_group
    goto_cut.params = sm_rootstock.userdata.params['cut']

    goto_place_clip = GoTo(sim = True)
    goto_place_clip.move_group = left_arm_move_group
    goto_place_clip.params = sm_rootstock.userdata.params['clip']
    
    goto_dispense = GoTo(sim = True)
    goto_dispense.move_group = left_arm_move_group
    goto_dispense.params = sm_rootstock.userdata.params['dispense']

    
    pick = Pick(sim = True)
    pick.io_module = io_mod

    cut = Cut(sim = True)
    cut.io_module = io_mod

    place = Place(sim = True)
    place.io_module = io_mod
    
    clip = Clip(sim = True)
    clip.io_module = io_mod
    
    dispense = Dispense(sim = True)
    dispense.io_module = io_mod
    
    # Open the container
    with sm_rootstock:
        # Add states to the container
        smach.StateMachine.add('GOTO_PICK', goto_pick_feeder, 
                               transitions={'reached':'PICK', 
                                            'failed':'failed',
                                            'grid_completed': 'completed'})
                                            
        smach.StateMachine.add('PICK', pick, 
                               transitions={'picked':'GOTO_CUT', 
                                            'failed':'failed'})
                                            
        smach.StateMachine.add('GOTO_CUT', goto_cut, 
                               transitions={'reached':'CUT', 
                                            'failed':'failed'})

        smach.StateMachine.add('CUT', cut, 
                               transitions={'cutted':'GOTO_PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_PLACE', goto_place_clip, 
                               transitions={'reached':'PLACE', 
                                            'failed':'failed'})

        smach.StateMachine.add('PLACE', place,
                                transitions={'placed':'CLIP', 
                                            'failed':'failed'})

        smach.StateMachine.add('CLIP', clip, 
                               transitions={'clipped':'GOTO_DISPENSE', 
                                            'failed':'failed'})

        smach.StateMachine.add('GOTO_DISPENSE', goto_dispense, 
                               transitions={'reached':'DISPENSE', 
                                            'failed':'failed'})

        smach.StateMachine.add('DISPENSE', dispense, 
                               transitions={'dispensed':'GOTO_PICK', 
                                            'failed':'failed'})
                                            
    sis = smach_ros.IntrospectionServer('server_name', sm_rootstock, '/SM_ROOT')
    sis.start()

    ## Execute SMACH plan
    outcome = sm_rootstock.execute()
    rospy.spin()
    sis.stop()
