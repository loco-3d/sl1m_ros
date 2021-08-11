#!/usr/bin/env python3
import rospy
import os 
from multiprocessing import Lock
from math import sqrt, inf

from custom_planner_msgs.msg import polygonArray

#Imports for solver

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
from pal_footstep_planner_msgs.msg import FootStepPlanAction, FootStepPlanActionGoal, FootStepPlanResult
from gazebo_msgs.msg import ModelStates
from pal_footstep_planner_msgs.msg import FootstepData
from geometry_msgs.msg import Vector3
from std_srvs.srv import TriggerRequest, TriggerResponse, Trigger

SOLVER = Solvers.GUROBI #GUROBI

GAIT = [np.array([1, 0]), np.array([0, 1])]

#constraints path
paths = ["/home/alouahadj/developpement/sl1m/sl1m/stand_alone_scenarios/constraints_files/","/home/alouahadj/developpement/sl1m/sl1m/stand_alone_scenarios/constraints_files/"]

limbs = ["LF", "RF"]
suffix_com = "_effector_frame_quasi_static_reduced.obj"
suffix_feet = "_reduced.obj"

#number of steps
N_STEPS = 25



class FootstepPlannerSl1m: 

    #Constructor
    def __init__(self):
        #numpy array
        self.surface = []
        #normal list
        self.listSurfaces = []
        rospy.Subscriber("/elevation_mapping/Obstacle/polygons",polygonArray,self.callback)
        rospy.Subscriber("/gazebo/model_states",ModelStates,self.positionCallback)
        self.lockSurface = Lock()
        self.lockListSurfaces = Lock()
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
    
    def getListSurfaces(self):
        self.lockListSurfaces.acquire()
        listSurfaces = self.listSurfaces
        self.lockListSurfaces.release()
        return listSurfaces
    
    def setListSurfaces(self,listSurfaces):
        self.lockListSurfaces.acquire()
        self.listSurfaces=listSurfaces
        self.lockListSurfaces.release()
    
    def getInitialContacts(self):
        self.lockContacts.acquire()
        initialContacts = self.initialContacts
        self.lockContacts.release()
        return initialContacts
    
    def setInitialContacts(self,initialContacts):
        self.lockContacts.acquire()
        self.initialContacts=initialContacts
        self.lockContacts.release()  

    def isPointRight(self,a,b,c) :
        return ((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))<0
    
    #is surface s2 included in surface s1
    def isSurfaceIncluded(self,s1,s2):
        for i in range (len(s2)) :
            p = s2[i]
            for j in range (len(s1)-1) :
                isTrue =self.isPointRight(s1[j],s1[j+1],p)
                if not isTrue :
                    return False
            
            isTrue = self.isPointRight(s1[len(s1)-1],s1[0],p)
            if not isTrue :
                return False
        
        return True
    
    def isColinear(self,vec1,vec2) : 
        prod1 = vec1[1]*vec2[2] - vec2[1]*vec1[2] 
        prod2 = vec2[0]*vec1[2] - vec1[0]*vec2[2]
        prod3 = vec1[0]*vec2[1] - vec2[0]*vec1[1]

        return (prod1==0 and prod2==0 and prod3==0)
    
    def getPlanEquation(self,surface):
        #point A
        x1 = surface[0][0]
        y1 = surface[0][1]
        z1 = surface[0][2]
        #point B
        x2 = surface[1][0]
        y2 = surface[1][1]
        z2 = surface[1][2]
        #vector AB
        vec1 = [x2-x1, y2-y1, z2-z1]
        for i in range(len(surface)-2) :
            #point C
            x = surface[i+2][0]
            y = surface[i+2][1]
            z = surface[i+2][2]
            #vector AC
            vec2 = vec1 = [x-x1, y-y1, z-z1]
            if not self.isColinear(vec1,vec2) :
                break

        a = (y2 - y1)*(z - z1) - (z2 - z1)*(y - y1)
        b = - ( (x2 - x1)*(z - z1) - (z2 - z1)*(x - x1) )
        c = (x2 - x1)*(y - y1) - (y2 - y1)*(x - x1)
        d = - (a*x1 + b*y1 + c*z1)

        return [a,b,c,d]

    def getGoalSurface(self,surfaces,point):
        index = -1
        distance = float('inf')
        for i in range (len(surfaces)) :
            surface = surfaces[i]
            planeCoef = self.getPlanEquation(surface)
            
            numerateur = abs(point[0]*planeCoef[0] + point[1]*planeCoef[1] + point[2]*planeCoef[2] + planeCoef[3])
            denominateur = sqrt(planeCoef[0]*planeCoef[0] + planeCoef[1]*planeCoef[1] + planeCoef[2]*planeCoef[2])
            dist = numerateur/denominateur

            if dist < distance :
                distance = dist
                index = i
        
        return surfaces[index]
    
    def addGoalSurface(self,cost) :
        listeSurface = self.getListSurfaces()
        allPlan = []
        surfaces = []
        goalSurface = self.getGoalSurface(listeSurface,cost)

        for i in range(len(listeSurface)):
            al = np.array(listeSurface[i]).T
            allPlan.append(al)
        
        surfaces =[[allPlan] for _ in range (N_STEPS-1)]
        """
        goalSurface = np.array(goalSurface).T
        surfaces.append([[goalSurface]])
        """
        self.setSurface(surfaces)

        return surfaces    
    
    def barycentre(self,surface) :
        x = 0
        y = 0
        nbPoints = len(surface)
        for i in range(nbPoints) :
            x = x + surface[i][0]
            y = y + surface[i][1]
        
        x = x/nbPoints
        y = y/nbPoints

        return [x,y]
    
    def surfaceArea(self, surface) :
        area = 0
        nbPoints = len(surface)
        C = self.barycentre(surface)
        for i in range(nbPoints) :
            if i < nbPoints - 1 :
                #Vectors coordinates
                vectAB = [ surface[i+1][0]-surface[i][0] , surface[i+1][1]-surface[i][1] ]
                vectBC = [ C[0]-surface[i+1][0] , C[1]-surface[i+1][1] ]
                vectCA = [ surface[i][0]-C[0] , surface[i][1]-C[1] ]
            else :
                #Vectors coordinates
                vectAB = [ surface[0][0]-surface[i][0] , surface[0][1]-surface[i][1] ]
                vectBC = [ C[0]-surface[0][0] , C[1]-surface[0][1] ]
                vectCA = [ surface[i][0]-C[0] , surface[i][1]-C[1] ]
            #Lengths of the vectors
            a = sqrt(vectAB[0]*vectAB[0] + vectAB[1]*vectAB[1])
            b = sqrt(vectBC[0]*vectBC[0] + vectBC[1]*vectBC[1])
            c = sqrt(vectCA[0]*vectCA[0] + vectCA[1]*vectCA[1])
            #Heron formula
            S = sqrt(((a+b+c)/2)*(((a+b+c)/2)-a)*(((a+b+c)/2)-b)*(((a+b+c)/2)-c))
            area = area + S
        
        return area
    
    def reduceSurface(self,surface,L) :
        C = self.barycentre(surface)
        nbPoints = len(surface)

        for i in range(nbPoints) : 
            vect = [ C[0]-surface[i][0] , C[0]-surface[i][1] ]
            denomintor = sqrt((C[0]-surface[i][0])*(C[0]-surface[i][0])) + sqrt((C[1]-surface[i][1])*(C[1]-surface[i][1]))
            x = (L*vect[0])/denomintor
            y = (L*vect[1])/denomintor
            surface[i][0] = x
            surface[i][1] = y



    #Callback function to get the initial contacts
    def positionCallback(self,msg):
        index = msg.name.index('talos')
        
        x = msg.pose[index].position.x
        y = msg.pose[index].position.y
        z = 0.0

        #initialPosition = [x,y,z]
        lPosition = [x, y+0.1, z]
        rPosition = [x, y-0.1, z]
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
        listeSurface = []

        #TODO delete it
        #Adding the floor
        floor = [[1.5, 5., 0.], [-3, 5., 0.], [-3, -5., 0.], [1.5, -5., 0.]]
        allPlan.append(np.array(floor).T)

        nb=len(msg.polygonArray[0].polygon.points)
        rospy.loginfo('New message')
        rospy.loginfo('Number of surfaces : {}'.format(nbSurfaces))

        #Getting the list of surfaces
        for i in range(nbSurfaces):
            nbPointsInSurface = len(msg.polygonArray[i].polygon.points)
            l=[]

            if nbPointsInSurface > 2 :
                #if msg.polygonArray[i].polygon.points[0].z > 0.09 :
                for j in range(nbPointsInSurface):
                    lbis=[]
                    lbis=[msg.polygonArray[i].polygon.points[j].x,msg.polygonArray[i].polygon.points[j].y,msg.polygonArray[i].polygon.points[j].z]
                    l.append(lbis)
                    
                listeSurface.append(l)    

        #TODO : transform into method
        """
        i = 0
        while i < (len(listeSurface)-1) :
            j = 0
            while j < (len(listeSurface)-1) :
                if i!=j :
                    s1 = listeSurface[i]
                    s2 = listeSurface[j]
                    isIncluded = self.isSurfaceIncluded(s1,s2)
                    if isIncluded :
                        if listeSurface[i][0][2] < listeSurface[j][0][2] :
                            listeSurface.remove(listeSurface[i])
                        else :
                            listeSurface.remove(listeSurface[j])
                        
                        i = i-1
                        break
                        
                j = j + 1
            i = i + 1
        """
        self.setListSurfaces(listeSurface)

        
        
        for i in range(len(listeSurface)):
            al = np.array(listeSurface[i]).T
            allPlan.append(al)
        
        #end test

        #al=np.array(l).T
        #allPlan.append(al)

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

        initial_contacts = self.getInitialContacts()
        t_2 = clock()

        pb = Problem(limb_names=limbs, constraint_paths=paths, suffix_com=suffix_com, suffix_feet=suffix_feet)
        pb.generate_problem(R, self.getSurface(), GAIT, initial_contacts)
        #pb.generate_problem(R, surfaces, GAIT, initial_contacts)
        t_3 = clock()

        result = solve_MIP(pb, costs=cost, com=True, solver = SOLVER)
        t_end = clock()

        print("Feet pose :")
        print(result.all_feet_pos)

        leftSteps = result.all_feet_pos[0]
        rightSteps = result.all_feet_pos[1]
        steps = [leftSteps,rightSteps]
        liste =[]
        
        print(result)
        print("LeftSteps : \n{} \n\nRighSteps : \n{}".format(leftSteps,rightSteps))
        print(self.initialContacts)

        
        #Adding the first ignored right step :
        ignoredStep = FootstepData()
        ignoredStep.location.x = 0.0
        ignoredStep.location.y = 0.0
        ignoredStep.location.z = 0.0
        ignoredStep.robot_side = 1
        ignoredStep.orientation.w = 1.
        ignoredStep.origin = 1
        liste.append(ignoredStep)
        

        for i in range(N_STEPS-1) : 
            footStepDat = FootstepData()
            footStepDat.location.x = steps[i%2][i+1][0]
            footStepDat.location.y = steps[i%2][i+1][1]
            footStepDat.location.z = steps[i%2][i+1][2]
            footStepDat.robot_side = i%2
            footStepDat.orientation.w = 1.
            footStepDat.origin = 1
            liste.append(footStepDat)
        

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


    def __init__(self,name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FootStepPlanAction, execute_cb=self.execute_cb, auto_start=False)
        self.configure_srv = rospy.Service("robot_footstep_planner/configure", Trigger, self.configure_cb)
        self._result = FootStepPlanResult()
        self.footStepPlan = FootstepPlannerSl1m()
        self.cost = {"final_com": [1.0, [0.0,0.0,0.0]]}
        self._as.start()
    
    #set the final position
    def setCost(self,g):
        x = g.target_pose.position.x
        y = g.target_pose.position.y
        z = g.target_pose.position.z
        self.cost = {"final_com": [1.0, [x,y,z+0.87]],"posture":[1]}

        return [x,y,z]

    #get the final position
    def getCost(self):
        cost=self.cost
        return cost

    #Callback function
    def execute_cb(self,goal):
        rospy.loginfo('Executing callback of %s' % self._action_name)
        cost = self.setCost(goal)
        print("Cost : {}".format(self.getCost()))

        if self.footStepPlan.getListSurfaces() != [] :
            self._result.footsteps=self.footStepPlan.plan(self.getCost())
            self._result.error_type = 0
        
        rospy.loginfo('Callback return succeed for Action %s' % self._action_name)
        self._as.set_succeeded(self._result)

    def configure_cb(self, req):
        rospy.loginfo('Executing Configure of %s' % self._action_name)
        res = TriggerResponse()
        res.success = True
        res.message = ""
        return res


if __name__ == '__main__':
    rospy.init_node("robot_footstep_planner")
    server = ActionServ("robot_footstep_planner")
    rospy.spin()