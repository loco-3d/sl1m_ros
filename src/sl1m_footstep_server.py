#! /usr/bin/env python

import rospy

import actionlib

from pal_footstep_planner_msgs.msg import FootStepPlanAction, FootStepPlanActionGoal, FootStepPlanResult, FootstepData
from std_srvs.srv import TriggerRequest, TriggerResponse, Trigger

class Sl1mFootstepServer(object):
    # create messages that are used to publish feedback/result

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, FootStepPlanAction, execute_cb=self.execute_cb, auto_start = False)
        self.configure_srv = rospy.Service("robot_footstep_planner/configure", Trigger, self.configure_cb)
        self._result = FootStepPlanResult()

        self._as.start()
      
    def execute_cb(self, goal):
        # publish info to the console for the user
        rospy.loginfo('Executing callback of %s' % self._action_name)
        success = True
        
        #CALL SL1M
        
        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._result = FootStepPlanResult()
            # TODO with sl1m
            stepR = FootstepData()
            stepR.robot_side = FootstepData.LEFT
            stepR.location.x = 0.
            stepR.location.y = 0.
            stepR.location.z = 0.
            stepR.orientation.w = 1.
            stepR.origin = 1
            stepL = FootstepData()
            stepL.robot_side = FootstepData.RIGHT
            stepL.location.x = 0.
            stepL.location.y = 0.
            stepL.location.z = 0.
            stepL.orientation.w = 1.
            stepL.origin = 1
            """
            stepR2 = FootstepData()
            stepR2.robot_side = FootstepData.RIGHT
            stepR2.location.x = 0.4
            stepR2.location.y = -0.1
            stepR2.location.z = 0.
            stepR2.orientation.w = 1.
            stepR2.origin = 1
            """
            steps = [stepR, stepL]
            self._result.footsteps = steps
            self._as.set_succeeded(self._result)

    
    def configure_cb(self, req):
        rospy.loginfo('Executing Configure of %s' % self._action_name)
        res = TriggerResponse()
        res.success = True
        res.message = ""
        return res
        
if __name__ == '__main__':
    rospy.init_node('robot_footstep_planner')
    server = Sl1mFootstepServer('robot_footstep_planner')
    rospy.spin()
