#!/usr/bin/env python3
import rospy
import os 
from multiprocessing import Lock

from custom_planner_msgs.msg import polygonArray

#Import for solver

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from time import perf_counter as clock

from sl1m.generic_solver import solve_L1_combinatorial, solve_MIP
from sl1m.problem_definition import Problem 

from sl1m.solver import Solvers
import sl1m
import sl1m.tools.plot_tools as plot

#Imports for the action server
import actionlib
from pal_footstep_planner_msgs.msg import PlanWalkActionGoal, PlanWalkActionResult, PlanWalkAction
from gazebo_msgs.msg import ModelStates
from pal_footstep_planner_msgs.msg import FootstepData
from geometry_msgs.msg import Vector3


SOLVER = Solvers.GUROBI #GUROBI

GAIT = [np.array([1, 0]), np.array([0, 1])]

#constraints path
paths = ["/home/alouahadj/developpement/sl1m/sl1m/stand_alone_scenarios/constraints_files/","/home/alouahadj/developpement/sl1m/sl1m/stand_alone_scenarios/constraints_files/"]

limbs = ["LF", "RF"]
suffix_com = "_effector_frame_quasi_static_reduced.obj"
suffix_feet = "_reduced.obj"

#number of steps
N_STEPS = 12



class FootstepPlannerSl1m: 

    #Constructor
    def __init__(self):
        self.surface = []
        rospy.Subscriber("/elevation_mapping/Obstacle/polygons",polygonArray,self.callback)
        rospy.Subscriber("/gazebo/model_states",ModelStates,self.positionCallback)
        self.lockSurface = Lock()
        self.lockContacts = Lock()
        self.initialContacts = []
    
    def getSurface(self):
        self.lockSurface.acquire()
        surface = self.surface
        self.lockSurface.release()
        return surface
    
    def setSurface(self,surface):
        self.lockSurface.acquire()
        self.surface=surface
        self.lockSurface.release()
    
    def getInitialContacts(self):
        self.lockContacts.acquire()
        initialContacts = self.initialContacts
        self.lockContacts.release()
        return initialContacts
    
    def setInitialContacts(self,initialContacts):
        self.lockContacts.acquire()
        self.initialContacts=initialContacts
        self.lockContacts.release()    

    #Callback function to get the initial contacts
    def positionCallback(self,msg):
        index = msg.name.index('talos')
        
        x = msg.pose[index].position.x
        y = msg.pose[index].position.y
        z = 0.0

        #initialPosition = [x,y,z]
        lPosition = [x, y-0.1, z]
        rPosition = [x, y+0.1, z]
        initial_contacts = [np.array(lPosition), np.array(rPosition)]

        self.setInitialContacts(initial_contacts)
        
    #Callback function to get the surfaces
    def callback(self,msg):
        #goal=[]
        #zmax = 0.0
        surfaces = []
        allPlan = []
        nbSurfaces = len(msg.polygonArray)
        nbPointsInSurface = 0

        #TODO delete it
        #Adding the floor
        floor = [[3., 5., 0.], [-3, 5., 0.], [-3, -5., 0.], [3., -5., 0.]]
        allPlan.append(np.array(floor).T)
        #first stair
        stair = [[-1, -1, 0.12], [-1, 1., 0.12], [-1.5, 1., 0.12], [-1.5, -1., 0.12]]
        allPlan.append(np.array(stair).T)

        nb=len(msg.polygonArray[0].polygon.points)
        rospy.loginfo('New message')
        rospy.loginfo('Number of surfaces : {}'.format(nbSurfaces))

        #Getting the list of surfaces
        for i in range(nbSurfaces):
            nbPointsInSurface = len(msg.polygonArray[i].polygon.points)
            l=[]

            if nbPointsInSurface > 2 :
                for j in range(nbPointsInSurface):
                    lbis=[]
                    lbis=[msg.polygonArray[i].polygon.points[j].x,msg.polygonArray[i].polygon.points[j].y,msg.polygonArray[i].polygon.points[j].z]
                    l.append(lbis)
                    
                #get goal surface (highest)
                """
                if lbis[2] > zmax :
                    goal = l
                    zmax = lbis[2]"""

                al=np.array(l).T
                allPlan.append(al)
        #add goal surface
        #goal = np.array(goal).T

        surfaces =[[allPlan] for _ in range (N_STEPS-1)]
        #Adding the goal surface at the end
        #surfaces.append([[goal]])

        self.setSurface(surfaces)
    
    #Using the solver
    def plan(self,cost):
        t_init = clock()
        R = [np.identity(3)] * N_STEPS
        t_1 = clock()

        #initial_contacts = [np.array([-2,  -0.1,  0]), np.array([-2,  0.1,  0])]
        initial_contacts = self.getInitialContacts()
        t_2 = clock()

        pb = Problem(limb_names=limbs, constraint_paths=paths, suffix_com=suffix_com, suffix_feet=suffix_feet)
        pb.generate_problem(R, self.getSurface(), GAIT, initial_contacts)
        t_3 = clock()

        result = solve_MIP(pb, costs=cost, com=True, solver = SOLVER)
        t_end = clock()

        #Test
        leftSteps = result.all_feet_pos[0]
        rightSteps = result.all_feet_pos[1]
        steps = [leftSteps,rightSteps]
        liste =[]


        for i in range(N_STEPS-1) : 
            footStepDat = FootstepData()
            footStepDat.location.x = steps[i%2][i+1][0]
            footStepDat.location.y = steps[i%2][i+1][1]
            footStepDat.location.z = steps[i%2][i+1][2]
            footStepDat.robot_side = i%2
            liste.append(footStepDat)
        
        #End Test

        print(result)

        print("Optimized number of steps:              ", pb.n_phases)
        print("Total time is:                          ", 1000. * (t_end-t_init))
        print("Computing the surfaces takes            ", 1000. * (t_1 - t_init))
        print("Computing the initial contacts takes    ", 1000. * (t_2 - t_1))
        print("Generating the problem dictionary takes ", 1000. * (t_3 - t_2))
        print("Solving the problem takes               ", 1000. * (t_end - t_3))
        print("The LP and QP optimizations take        ", result.time)

        ax = plot.draw_scene(self.getSurface(), GAIT)
        plot.plot_initial_contacts(initial_contacts, ax=ax)
        if(result.success):
            plot.plot_planner_result(result.coms, result.all_feet_pos, ax, True)
        else:
            plt.show()
        
        return liste
    

class ActionServ(object):

    _result = PlanWalkActionResult()

    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PlanWalkAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        self.footStepPlan = FootstepPlannerSl1m()
        self.cost = {"final_com": [1.0, [0.0,0.0,0.0]]}
    
    #set the final position
    def setCost(self,g):
        x = g.target_pose.position.x
        y = g.target_pose.position.y
        z = g.target_pose.position.z
        self.cost = {"final_com": [1.0, [x,y,z]]}

    #get the final position
    def getCost(self):
        cost=self.cost
        return cost

    #Callback function
    def execute_cb(self,goal):
        self.setCost(goal)

        if self.footStepPlan.getSurface() != [] :
            self._result.result.footsteps=self.footStepPlan.plan(self.getCost())
            self._result.result.error_type = 0
        
        self._result.header.stamp = rospy.get_rostime()
        self._as.set_succeeded(self._result)



if __name__ == '__main__':
    rospy.init_node("plan_walk")
    server = ActionServ(rospy.get_name())
    rospy.spin()