import time 
import numpy as np
import copy
import yaml
import matplotlib
import matplotlib.pyplot as plt
import scipy.io as sio 
import math
import sys
import pandas as pd

import csv 
from datetime import datetime

matplotlib.use('TkAgg') # plotting with tkinter

import pybullet as p
import pybullet_data

import os
import sys
import pytest
from pathlib import Path

# 9/6 ONE FINGER CURVE-PRESSURE TEST Automating Data Collection --> creates csv file for each location and desired error
# 1 Controllers, 1 Manipulators, 1 cylinder. 3 Controller States 

path = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, path)
from somo.sm_manipulator_definition import SMManipulatorDefinition
from somo.sm_continuum_manipulator import SMContinuumManipulator
from somo.utils import load_constrained_urdf



def getBendingMeasurement(manip: SMContinuumManipulator):
    s_i = []
    for ind in manip.flexible_joint_indices:
        jointState = p.getJointState(
            manip.bodyUniqueId, ind, physicsClientId=manip.physics_client
        )
        angle  = jointState[0]
        d = 0.2
        l = 0.2 
        if (abs(angle) < 1e-5):
            s=  angle * d 
        elif (angle > 0):
            s = angle*(l/np.tan(angle/2)+d) - 2*l #why are these the same?
        else:
            s = angle*(l/np.tan(angle/2)+d) - 2*l #why are these the same?
        s_i.append(s)
    delta_R = sum(s_i)
    return delta_R

# need to include def for normal forces 


def manipulator_actuation_tester(gui: bool = False, total_sim_steps = 150, num_axes=None, locationInput = [], multiplierInput=[]):
    """
    tests whether a manipulator can be instantiated in pybullet and whether a sinusoidal torque actuation can be applied
    """

    palmBaseHeight = 0.05
    startPositions = [0, 0, palmBaseHeight + 0.5]
    startPosition_two = [1.5, 0, palmBaseHeight + 0.5]
    startOrientations = p.getQuaternionFromEuler([-np.pi/1.5, -np.pi/2, np.pi])
    startOrientations_two = p.getQuaternionFromEuler([-np.pi/1.5, np.pi/2, -np.pi])

    # load the manipulator definition
    manipulater_def_path = os.path.join(Path(__file__).parent, "manipulator_test_def.yaml")
   # manipulater_two_def_path = os.path.join(Path(__file__).parent, "manipulator_two_test_def.yaml")

    with open(manipulater_def_path, "r") as manipulater_def_file:
        manipulater_def_dict_template = yaml.safe_load(manipulater_def_file)

   # with open(manipulater_two_def_path, "r") as manipulater_def_file:
    #    manipulater_two_def_dict_template = yaml.safe_load(manipulater_def_file)

    manipulater_def_dict = copy.deepcopy(manipulater_def_dict_template)
    #manipulater_two_def_dict = copy.deepcopy(manipulater_two_def_dict_template)
    
    # if num_axes input is None, use the template directly.
    # num_axes = 1 
    if not num_axes:
        pass
    else:  
        # if num_axes is provided, overwrite the number of axes that are actually instantiated
        # import pdb; pdb.set_trace()
        manipulater_def_dict["actuator_definitions"][0][
            "joint_definitions"
        ] = manipulater_def_dict_template["actuator_definitions"][0][
            "joint_definitions"
        ][
            :num_axes
        ]
        if (
            num_axes == 1
        ):  # if this is 1, we have a planar actuator in the test and need to set the planar_flag accordingly
            manipulater_def_dict["actuator_definitions"][0]["planar_flag"] = 1

    # if num_axes input is None, use the template directly.
  #  if not num_axes:
  #      pass
  #  else:  
        # if num_axes is provided, overwrite the number of axes that are actually instantiated
        # import pdb; pdb.set_trace()
   #     manipulater_two_def_dict["actuator_definitions"][0][
            "joint_definitions"
   #     ] = manipulater_two_def_dict_template["actuator_definitions"][0][
            "joint_definitions"
   # ][
    #        :num_axes
    #    ]
   #     if (
   #         num_axes == 1
   #     ):  # if this is 1, we have a planar actuator in the test and need to set the planar_flag accordingly
   #         manipulater_two_def_dict["actuator_definitions"][0]["planar_flag"] = 1


    manipulator_def = SMManipulatorDefinition(**manipulater_def_dict)
    #manipulator_two_def = SMManipulatorDefinition(**manipulater_two_def_dict)

    #----- instantiate manipulator
    manipulator = SMContinuumManipulator(manipulator_def)
    #manipulator_two = SMContinuumManipulator(manipulator_two_def)

    if gui:
        physicsClient = p.connect(p.GUI)
    else:
        physicsClient = p.connect(p.DIRECT)

    # setting up the physics client, parameters
    p.setGravity(0, 0, -10)
    p.setPhysicsEngineParameter(enableConeFriction=1)
    p.setRealTimeSimulation(
        0
    )  # only if this is set to 0 and the simulation is done with explicit steps will the torque control work correctly
    p.setPhysicsEngineParameter(
        enableFileCaching=0
    )  # x todo: check if this makes things faster
    bullet_time_step = 0.0001
    p.setTimeStep(bullet_time_step)

    # load the ground plane
    planeId = p.loadURDF(
        "floor.urdf", 
        flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
    )
    p.changeDynamics(planeId, -1, lateralFriction=1)  # set ground plane friction


    # add a cylinder 
    boxStartPos = locationInput
    boxStartOr = p.getQuaternionFromEuler([0, 0, 0])
    radius = 1.0
    boxId = p.loadURDF(
        "smallCylinder.urdf",
        boxStartPos,
        boxStartOr,
        useFixedBase = True,
        flags=p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
        globalScaling=0.5,
        #physicsClient=physicsClient,
    )
    p.changeDynamics(boxId, -1, lateralFriction=0.6)

    # Load both manipulators to pybullet
    # if constrained, can move manipulator in simulation loop
    manipulator.load_to_pybullet(
        baseStartPos=startPositions,
        baseStartOrn=startOrientations,
        baseConstraint="static", # options: "static" or "constrained"
        physicsClient=physicsClient,
    )  # todo: something looks funny for constrained and free baseConstraint

  #  manipulator_two.load_to_pybullet(
  #      baseStartPos=startPosition_two,
  #      baseStartOrn=startOrientations_two,
  #      baseConstraint="static",
  #      physicsClient=physicsClient, 
  #  )

    p.changeDynamics(manipulator.bodyUniqueId, -1, lateralFriction=0.6)
    p.changeDynamics(manipulator.bodyUniqueId, -1, restitution=1)

    #p.changeDynamics(manipulator_two.bodyUniqueId, -1, lateralFriction=0.6)
    #p.changeDynamics(manipulator_two.bodyUniqueId, -1, restitution=1)

    # Define the gain and zero of controller 
    kr = 80.0
    zc = 0.95
    rampHalfPeriod = 50
    ramp_slope = 0.5/rampHalfPeriod
    # stores finger states
    recfing1State = []
    recfing2State = []
    # stores bending measurements
    recMes1 = []
    recMes2 = []
    # stores references
    recRef1 = []
    recRef2 = []
    # stores output
    recU1 = []
    recU2 = []
    # stores error 
    recErr1 = []
    recErr2 = []
    recErr1DesS3 = []
    recErr2DesS3 = []
    # records time
    tRec = []
    # Time step parameters
    timeStepN = 1000
    e_ramp = 0 
    iTimeStep = 0
    iCtrlStep = 0
    ctrlStepCnt = 0
    print("Total simulation steps:\n",total_sim_steps)
    # Finger reference intialization
    ref1 = 0
    ref2 = 0
    normal_forceTip = 0.0
    normal_forceTip_2 = 0.0
    lateral_forceTip = 0.0  

    # Variables for State 4
    mode = 1 # 1 - firmer contact
    error_threshold = .023
    kc = 22.24
    zc_c = 0.955
    stepHalfPeriod = 20
    step_slope = 0.3/stepHalfPeriod
    e1_c = 0.0
    e1m1_c = 0.0
    u1m1_c = 0.0
    u1_c = 0.0

    e2_c = 0.0 # manipulator two 
    e2m1_c = 0.0
    u2m1_c = 0.0
    u2_c = 0.0
    recErr1Con = []
    recErr2Con = []
    recU1Con = []
    recU2Con = []
    recMesCon1 = []
    recMesCon2 = []
    recRefCon1 = []
    recRefCon2 = []

    Mes1_rec = []
    Mes2_rec = []
    r = 1 
    

    # Normal Force recordings
    rec_normal_forces = []
    #rec_normal_forces_lastLink = []
    rec_normal_forces_2 = []
    #rec_normal_forces_lastLink_2 = []


    csv_now = datetime.now()
    csv_time = csv_now.strftime("%m_%d_%Y %H_%M_%S")


    locationList = [[0.75, -0.5, 0.5],[0.75, -0.8, 0.5],[0.75, -1, 0.5]]
    multiplierList = [0,1,3,4,6,8,9,10,11]

    # --- Begin CSV ---- 
    for location in locationList:
        for multipliers in multiplierList:
            location_str = '_'.join(map(str,locationInput))
            multipliers_str = (multiplierInput)
            fileName1= f'r=1_{multipliers_str}multi_{location_str}loc_{csv_time}.csv' 
   
   
    with open(fileName1, 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile, delimiter=',')
        csv_writer.writerow(['Time Stamp', 'Simulation Step', 'Ref 1', 'Pressure 1', 'Curvature 1', 'Ref Error 1','Finger 1 Force', str(boxStartPos), str(radius)])
        
    fing1State = 1
    fing2State = 1  
    iTimeStep = 0
    ctrlStepCnt = 0
    iCtrlStep = 1
    # simulation loop 
    while iTimeStep < total_sim_steps:
        # readin measurements
        mes1 = getBendingMeasurement(manipulator) 
        #mes2 = getBendingMeasurement(manipulator_two) 

        ##############################################
        #       Finger 1 state machine and control  
        ##############################################
        # Finger 1 STATE 1: Initialization
        if (fing1State == 1):
            ref1 = mes1 # sets reference to current bending measurement
            e1 = 0.0
            e1m1 = 0.0
            e1m2 = 0.0
            u1m1 = 0.0
            u1m2 = 0.0
            u1 = 0.0
            action1 = u1
            total_e1 = e1
            slope_sgn1 = 1.0
            fing1State = 2
            eDes1 = 0 # Desired reference in State 3
        # end of state 1         
        # STATE 2: ramp reference (positive/negative) 
        elif (fing1State==2):
            # sample time 
            if (ctrlStepCnt >= timeStepN) :          
                # the slope change 
                if (int(iCtrlStep/rampHalfPeriod)*rampHalfPeriod == iCtrlStep ):
                    slope_sgn1 = -slope_sgn1
                ref1 = ref1 + slope_sgn1*ramp_slope                
                e1 = ref1 - mes1
                u1 = 2*u1m1 - u1m2 + kr * e1m1 - kr * zc *e1m2                
                action1 = u1 # send to the actuator
                # prepare for the next time step
                e1m2 = e1m1
                e1m1 = e1 
                u1m2 = u1m1
                u1m1 = u1
                # if the theshold is reched jump to state 3  
                if (abs(e1) >= error_threshold) :
                    fing1State=3                # go to state 3 
                    contact_ref1 = ref1         # reference at the time of contact
                    contact_action1 = u1        # action at the time of contact
                    
                    multiplier = [0,1,3,4,6,8,9,10,11]
                    m = 0
                    while m < len(multiplier):
                        multi = multiplier[m]
                        if total_sim_steps == 80000:
                           m = m + 1
                           
                    eDes1 = error_threshold*(multiplierInput)   # Desired error in State 3              
                    
                    u1m1_c = 0                 # initial value for State 3
                    e1m1_c = 0                  # initial value for e1m1_c in State 3
        #end of state 2
        # STATE 3: contact        
        elif (fing1State ==3):   
            # sample time 
            if (ctrlStepCnt >= timeStepN):             
                # START EXPERIMENT 1 HERE ------
                ref1 = contact_ref1
                e1_c = (ref1 - (mes1-eDes1))               
                e1 = (ref1 - mes1)               
                u1_c = u1m1_c + kc * e1_c - kc * zc_c *e1m1_c # step input controller
                action1 = u1_c + contact_action1 # send to the actuator
                # action1 = contact_action1
                # prepare for the next time step
                e1m1_c = e1_c
                u1m1_c = u1_c
                # end of the sample timein state 3
                # end of state 3

        ##############################################
        #       Finger 2 state machine and control  
        ##############################################
        # Finger 2 STATE 1: Initialization
 #       if (fing2State == 1):
 #           ref2 = mes2 # sets reference to current bending measurement
 #           e2 = 0.0
 #           e2m1 = 0.0
 #           e2m2 = 0.0
 #           u2m1 = 0.0
 #           u2m2 = 0.0
 #           u2 = 0.0
 #           action2 = u2
 #           slope_sgn2 = 1.0
 #           fing2State = 2
 #           eDes2 = 0.0 # Desired reference in State 3
        # end of state 1         
        # STATE 2: ramp reference (positive/negative) 
 #       elif (fing2State==2):
            # sample time 
 #            if (ctrlStepCnt >= timeStepN):          
 #               # the slope change 
 #               if (int(iCtrlStep/rampHalfPeriod)*rampHalfPeriod == iCtrlStep ):
 #                   slope_sgn2 = -slope_sgn2
 #               ref2 = ref2 + slope_sgn2*ramp_slope                
 #               e2 = ref2 - mes1
 #               u2 = 2*u2m1 - u2m2 + kr * e2m1 - kr * zc *e2m2                
 #               action2 = u2 # send to the actuator
 #               # prepare for the next time step
 #               e2m2 = e2m1
 #               e2m1 = e2 
 #               u2m2 = u2m1
 #               u2m1 = u2
                # if the theshold is reched jump to state 3
 #               if (abs(e2) >= error_threshold) :
 #                   fing2State=3                # go to state 3 
 #                   contact_ref2 = ref2         # reference at the time of contact
 #                   contact_action2 = u2        # action at the time of contact
 #                   eDes2 = error_threshold*(multiplierInput) # Desired error in State 3
 #                   u2m1_c = 0                  # initial value for State 3
 #                   e2m1_c = 0                  # initial value for e1m1_c in State 3
            # end of the sample time update in state 2
        #end of state 2
        # STATE 3: contact        
 #       elif (fing2State ==3):   
            # sample time 
 #           if (ctrlStepCnt >= timeStepN):             
 #               # START EXPERIMENT 1 HERE ------
 #               ref2 = contact_ref2
 #               e2_c = (ref2 - (mes2-eDes2))                 
 #               e2 = (ref2 - mes2)             
 #               u2_c = u2m1_c + kc * e2_c - kc * zc_c *e2m1_c # step input controller
 #               action2 = u2_c + contact_action2 # send to the actuator
                # prepare for the next time step
 #               e2m1_c = e2_c 
 #               u2m1_c = u2_c
                # reset countes for the sample time
                
            # end of the sample timein state 3
        # end of state 3'''

        
        for axis_nr in range(num_axes): 
            if (axis_nr != 4):        
                manipulator.apply_actuation_torques(
                    actuator_nrs=[0],
                    axis_nrs=[axis_nr],
                    actuation_torques=[action1],
                    )
                
                '''manipulator_two.apply_actuation_torques(
                    actuator_nrs=[0],
                    axis_nrs=[axis_nr],
                    actuation_torques=[action2],
                    )'''
                
            else:
                manipulator.apply_actuation_torques(
                    actuator_nrs=[0],
                    axis_nrs=[axis_nr],
                    actuation_torques=[action1],
                    )
                
                '''manipulator_two.apply_actuation_torques(
                    actuator_nrs=[0],
                    axis_nrs=[axis_nr],
                    actuation_torques=[action2],
                    )'''
        
        

        # ---- START NORMAL FORCE ----       

        last_link = p.getNumJoints(manipulator.bodyUniqueId) - 1  # last link of the finger
        #last_link_2 = p.getNumJoints(manipulator_two.bodyUniqueId) - 1
        
        normal_forces = np.zeros((total_sim_steps,))
        normal_forces_2 = np.zeros((total_sim_steps,))

        normal_forces_lastLink = np.zeros((total_sim_steps,))
        normal_forces_lastLink_2 = np.zeros((total_sim_steps,))

        contactPoints = p.getContactPoints(
            boxId, manipulator.bodyUniqueId, physicsClientId=physicsClient
        )

      #  contactPoints_2 = p.getContactPoints(
      #      boxId, manipulator_two.bodyUniqueId, physicsClientId=physicsClient
      #  )

        contactPointsTip = p.getContactPoints(
            bodyA=boxId,
            bodyB=manipulator.bodyUniqueId,
            linkIndexB=last_link,
            physicsClientId=physicsClient,
        )

        '''  contactPointsTip_2 = p.getContactPoints(
            bodyA=boxId,
            bodyB=manipulator_two.bodyUniqueId,
            linkIndexB=last_link_2,
            physicsClientId=physicsClient,
        )'''
        
        total_normal_force = 0
        total_normal_force_2 = 0
        
        for contactPoint in contactPoints:
            normal_force = contactPoint[9] 
            total_normal_force += normal_force

      #  for contactPoint in contactPoints_2:
      #      normal_force_2 = contactPoint[9] 
      #      total_normal_force_2 += normal_force_2

        total_normal_force_lastLink = 0
        #print("length of points, tip \n",len(contactPoints), len(contactPointsTip))

        total_normal_force_lastLink_2 = 0
        #print("length of points, tip \n",len(contactPoints_2), len(contactPointsTip_2))

        now = datetime.now()
        date_time = now.strftime("%m/%d/%Y %H:%M:%S")
           
        for contactPointTip in contactPointsTip:
            normal_forceTip = contactPointTip[9]
            lateral_forceTip = contactPointTip[10]
            total_normal_force_lastLink += normal_forceTip
            
      #  for contactPointTip in contactPointsTip_2:
      #      normal_forceTip_2 = contactPointTip[9]
      #      #print("Tip force M2\n",normal_forceTip_2)
      #      total_normal_force_lastLink_2 += normal_forceTip_2

        normal_forces[axis_nr] = total_normal_force
        normal_forces_2[axis_nr] = total_normal_force_2

        normal_forces_lastLink[axis_nr] = total_normal_force_lastLink
        normal_forces_lastLink_2[axis_nr] = total_normal_force_lastLink_2
        
        # sample time counter
        ############################################################################################################################################
        
        if (ctrlStepCnt >= timeStepN):
            ctrlStepCnt = 0
            iCtrlStep = iCtrlStep +1
            print('Force figer1',normal_forceTip, 'Force figer2',normal_forceTip_2)
            #mes1, 
            #insert into numpy data structure, curvature with respects to pressure.
            #list()
            #numpy
            #export pandas -> csv
        ############################################################################################################################################    
            
            
        ctrlStepCnt = ctrlStepCnt + 1

        # recording 
        recfing1State.append(fing1State)
        recfing2State.append(fing2State)
        recMes1.append(mes1) 
        #recMes2.append(mes2)
        tRec.append(iTimeStep*0.0001)
        recRef1.append(ref1)
        recRef2.append(ref2)      
        recU1.append(action1) 
       # recU2.append(action2)     
        # to plot the error 
        e1cont=ref1-mes1
        # to plot the error of State 3 controller  
        # e1cont=ref1-(mes1+eDes1)
      #  e2cont=ref2-mes2
        # to plot the error of State 3 controller  
        # e2cont=ref2-(mes2+eDes2)
        recErr1.append(e1cont)
        recErr1DesS3.append(eDes1) 
      #  recErr2.append(e2cont)        
       # recErr2DesS3.append(eDes2)  
        if normal_forceTip > 50:
            normal_forceTip = 50
        if normal_forceTip_2 > 50:
            normal_forceTip_2 = 50
        rec_normal_forces.append(normal_forceTip)
        rec_normal_forces_2.append(normal_forceTip_2)
        
        csv_var = date_time + "," + str(round(iTimeStep*0.0001, 2)) + "," + str(round(ref1, 4)) + "," + str(round(action1, 4)) + "," + str(mes1) + ',' + str(round(e1cont, 4)) + ',' + str(round(normal_forceTip, 4)) + '\n'
        
        # --- Begin CSV ----
        with open(fileName1, 'a', newline='') as csvfile:
            csvfile.write(csv_var)
            
        p.stepSimulation(physicsClient)
        iTimeStep = iTimeStep+1
        #print('fing1State, iTimeStep, ctrStepCnt, iCtrlStep, e1 ',fing1State, iTimeStep, ctrlStepCnt, iCtrlStep, e1,slope_sgn1)        

    
    p.disconnect(physicsClient)
   
    # delete the test urdf
    os.remove(manipulator.manipulator_definition.urdf_filename)
    #os.remove(manipulator_two.manipulator_definition.urdf_filename)


def test_manipulator_actuation_gui():
    locationList = [[0.75, -0.5, 0.5],[0.75, -0.8, 0.5],[0.75, -1, 0.5]]
    multiplierList = [0,1,3,4,6,8,9,10,11]
    for location in locationList:
        for multipliers in multiplierList:
            manipulator_actuation_tester(gui=True, total_sim_steps=int(80000), num_axes=1, locationInput=location, multiplierInput=multipliers)
    manipulator_actuation_tester(gui=True, total_sim_steps=int(80000), num_axes=1)

if __name__ == '__main__':
    test_manipulator_actuation_gui()
