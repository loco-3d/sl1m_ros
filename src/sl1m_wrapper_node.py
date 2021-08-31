#!/usr/bin/env python3
import rospy
import os 

from custom_planner_msgs.msg import polygonArray

#Import for solver

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from time import perf_counter as clock

from sl1m.generic_solver import solve_L1_combinatorial, solve_MIP
#from sl1m.stand_alone_scenarios.problem_definition_talos import Problem
from sl1m.problem_definition import Problem 

#Choose the solver that will be used, CBC or gurobi 
from sl1m.solver import Solvers

import sl1m
import sl1m.tools.plot_tools as plot


SOLVER = Solvers.GUROBI #GUROBI

GAIT = [np.array([1, 0]), np.array([0, 1])]

#constraints path
paths = ["/home/alouahadj/developpement/sl1m/sl1m/stand_alone_scenarios/constraints_files/","/home/alouahadj/developpement/sl1m/sl1m/stand_alone_scenarios/constraints_files/"]

limbs = ["LF", "RF"]
suffix_com = "_effector_frame_quasi_static_reduced.obj"
suffix_feet = "_reduced.obj"

#number of steps
N_STEPS = 12

def callback(msg):
    #Variables declaration
    goal=[]
    zmax = 0.0
    surfaces = []
    allPlan = []
    nbSurfaces = len(msg.polygonArray)
    nbPointsInSurface = 0
"""
    #TODO delete it
    #Adding the floor
    floor = [[0., 3., 0.], [-3, 3., 0.], [-3, -3., 0.], [0., -3., 0.]]
    allPlan.append(np.array(floor).T)
    #first stair
    stair = [[-1, -1, 0.12], [-1, 1., 0.12], [-1.5, 1., 0.12], [-1.5, -1., 0.12]]
    allPlan.append(np.array(stair).T)
"""
    nb=len(msg.polygonArray[0].polygon.points)
    rospy.loginfo('New message')
    rospy.loginfo('Number of surfaces : {}'.format(nbSurfaces))

    #Getting the list of surfaces
    for i in range(nbSurfaces):
        nbPointsInSurface = len(msg.polygonArray[i].polygon.points)
        l=[]
        for j in range(nbPointsInSurface):
            lbis=[]
            lbis=[msg.polygonArray[i].polygon.points[j].x,msg.polygonArray[i].polygon.points[j].y,msg.polygonArray[i].polygon.points[j].z]
            l.append(lbis)
            
        #get goal surface (highest)
        if lbis[2] > zmax :
            goal = l
            zmax = lbis[2]

        al=np.array(l).T
        allPlan.append(al)
    #add goal surface
    goal = np.array(goal).T

    surfaces =[[allPlan] for _ in range (N_STEPS-1)]
    #Adding the goal surface at the end
    surfaces.append([[goal]])

    #Using solver
    t_init = clock()
    R = [np.identity(3)] * N_STEPS
    t_1 = clock()

    initial_contacts = [np.array([-2,  -0.1,  0]), np.array([-2,  0.1,  0])]
    t_2 = clock()

    pb = Problem(limb_names=limbs, constraint_paths=paths, suffix_com=suffix_com, suffix_feet=suffix_feet)
    pb.generate_problem(R, surfaces, GAIT, initial_contacts)
    t_3 = clock()

    COSTS = {"final_com": [1.0, [0.3,0,1.4]]}
    result = solve_MIP(pb, costs=COSTS, com=True, solver = SOLVER)
    t_end = clock()
    print(result)

    print("Optimized number of steps:              ", pb.n_phases)
    print("Total time is:                          ", 1000. * (t_end-t_init))
    print("Computing the surfaces takes            ", 1000. * (t_1 - t_init))
    print("Computing the initial contacts takes    ", 1000. * (t_2 - t_1))
    print("Generating the problem dictionary takes ", 1000. * (t_3 - t_2))
    print("Solving the problem takes               ", 1000. * (t_end - t_3))
    print("The LP and QP optimizations take        ", result.time)

    ax = plot.draw_scene(surfaces, GAIT)
    plot.plot_initial_contacts(initial_contacts, ax=ax)
    if(result.success):
        plot.plot_planner_result(result.coms, result.all_feet_pos, ax, True)
    else:
        plt.show()
        


def main():
    rospy.init_node('change_msg')

    rospy.Subscriber("/obstacle_detection/Obstacle/polygons",polygonArray,callback)
    rospy.spin()

if __name__ == '__main__':
    main()