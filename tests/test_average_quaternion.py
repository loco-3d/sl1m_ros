import unittest
from math import pi
import numpy as np
import pinocchio

from sl1m_ros.average_quaternion import average_quaternion


class TestAverageQuaternion(unittest.TestCase):

    def rpyToQuaternion(self, rpy):
        return pinocchio.Quaternion(pinocchio.rpy.rpyToMatrix(rpy))

    def quaternionToRpy(self, q):
        return pinocchio.rpy.matrixToRpy(q.matrix())

    def test_two_quaternions(self):

        q1 = pinocchio.Quaternion.Identity()
        q2 = self.rpyToQuaternion(np.random.random(3))

        q_avg = pinocchio.Quaternion(average_quaternion([q1.coeffs(), q2.coeffs()]))
        q_avg_slerp = q1.slerp(0.5, q2)
        
        np.testing.assert_almost_equal(q_avg.coeffs(), q_avg_slerp.coeffs())
