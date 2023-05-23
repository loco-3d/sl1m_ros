import os
import rospy
import rospkg
import numpy as np
from pathlib import Path
from sl1m.solver import Solvers


class Sl1mParameters:
    def __init__(self):

        # public params
        self.use_sl1m = "MIP"
        self.paths = []
        self.package = ""
        self.limbs = []
        self.suffix_com = ""
        self.suffix_feet = ""
        self.gait = []
        self.plot = False
        self.costs = {}
        self.step_length = [0.1, 0.087]  # 10cm linear and 5degree yaw
        self.rate = 20
        self.optimize_com = True
        self.final_base_orientation = np.eye(3)
        self.nb_steps_max = 100
        self.com_height = 0.87
        self.initial_surface_width = 0.8
        self.initial_surface_length = 1.0
        self.max_goal_distance = 1.
        self.end_effector_cost_weight = 1.
        self.step_length_cost_weight = 1.
        self.step_length_cost_ref = [0.1, 0.]

        # private params
        self._numerical_solver = "GUROBI"

    def get_solver_type(self):
        if self._numerical_solver == "QUADPROG":
            return Solvers.QUADPROG
        if self._numerical_solver == "LINPROG":
            return Solvers.LINPROG
        if self._numerical_solver == "GLPK":
            return Solvers.GLPK
        if self._numerical_solver == "GUROBI":
            return Solvers.GUROBI
        if self._numerical_solver == "CVXPY":
            return Solvers.CVXPY

    def get_ros_param(self):
        self.use_sl1m = self._get_param("/sl1m/use_sl1m")
        self._numerical_solver = self._get_param("/sl1m/numerical_solver")
        self.package = self._get_param("sl1m/package")
        self.limbs = self._get_param("sl1m/limbs")
        self.suffix_com = self._get_param("sl1m/suffix_com")
        self.suffix_feet = self._get_param("sl1m/suffix_feet")
        self.gait = self._get_param("sl1m/gait")
        self.plot = self._get_param("sl1m/plot")
        self.costs = self._get_param("sl1m/costs")
        self.step_length = self._get_param("sl1m/step_length")
        self.rate = self._get_param("sl1m/rate")
        self.optimize_com = self._get_param("sl1m/optimize_com")
        self.final_base_orientation = self._get_param("sl1m/final_base_orientation")
        self.nb_steps_max = self._get_param("sl1m/nb_steps_max")
        self.com_height = self._get_param("sl1m/com_height")
        self.initial_surface_width = self._get_param("sl1m/initial_surface_width")
        self.initial_surface_length = self._get_param("sl1m/initial_surface_length")
        self.max_goal_distance = self._get_param("sl1m/max_goal_distance")
        self.end_effector_cost_weight = self._get_param("sl1m/end_effector_cost_weight")
        self.step_length_cost_weight = self._get_param("sl1m/step_length_cost_weight")
        self.step_length_cost_ref = self._get_param("sl1m/step_length_cost_ref")


        # Fetch the data from the paths
        assert type(self.paths) is list
        rp = rospkg.RosPack()
        package_path = Path(rp.get_path(self.package))
        def get_path_in_package(suffix):
            ret = Path()
            for p in package_path.rglob("*" + suffix):
                if p.is_file():
                    ret = p.parent
                    break
            assert(ret.exists())
            return str(ret) + os.path.sep
        self.paths = [
            get_path_in_package(self.suffix_com),
            get_path_in_package(self.suffix_feet),
        ]

        # sanity checks on types
        assert type(self.limbs) is list
        for limb in self.limbs:
            assert type(limb) is str
        assert type(self.suffix_com) is str
        assert type(self.suffix_feet) is str
        assert type(self.gait) is list
        for gait_el in self.gait:
            assert type(gait_el) is list
        assert type(self.nb_steps_max) is int
        assert type(self.com_height) is float
        assert type(self.initial_surface_width) is float
        assert type(self.initial_surface_length) is float
        assert type(self.max_goal_distance) is float
        assert type(self.end_effector_cost_weight) is float
        assert type(self.step_length_cost_weight) is float
        assert type(self.step_length_cost_ref) is list
        assert(len(self.step_length_cost_ref) == 2)
        for val in self.step_length_cost_ref:
            assert type(val) is float


        for i in range(len(self.gait)):
            self.gait[i] = np.array(self.gait[i])

        # assert sanity checks on content
        if "end_effector_positions" in self.costs:
            self.costs.pop("end_effector_positions")

    def _get_param(self, name):
        if rospy.has_param(name):
            return rospy.get_param(name)
        else:
            var_name = str(Path(name).name)
            if var_name == "numerical_solver":
                return self.__dict__["_" + var_name]
            else:
                return self.__dict__[var_name]

    def __repr__(self):
        ret = "Sl1mParameters\n"
        ret += "    - use_sl1m = " + str(self.use_sl1m) + "\n"
        ret += "    - numerical_solver = " + str(self._numerical_solver) + "\n"
        ret += "    - paths = " + str(self.paths) + "\n"
        ret += "    - limbs = " + str(self.limbs) + "\n"
        ret += "    - suffix_com = " + str(self.suffix_com) + "\n"
        ret += "    - suffix_feet = " + str(self.suffix_feet) + "\n"
        ret += "    - gait = " + str(self.gait) + "\n"
        ret += "    - plot = " + str(self.plot) + "\n"
        ret += "    - costs = " + str(self.costs) + "\n"
        ret += "    - step_length = " + str(self.step_length) + "\n"
        ret += "    - nb_steps_max = " + str(self.nb_steps_max) + "\n"
        ret += "    - rate = " + str(self.rate) + "\n"
        ret += "    - optimize_com = " + str(self.optimize_com) + "\n"
        ret += "    - com_height = " + str(self.com_height) + "\n"
        ret += "    - initial_surface_width = " + str(self.initial_surface_width) + "\n"
        ret += "    - initial_surface_length = " + str(self.initial_surface_length) + "\n"
        ret += "    - max_goal_distance = " + str(self.max_goal_distance) + "\n"
        ret += "    - end_effector_cost_weight = " + str(self.end_effector_cost_weight) + "\n"
        ret += "    - step_length_cost_weight = " + str(self.step_length_cost_weight) + "\n"
        ret += "    - step_length_cost_ref = " + str(self.step_length_cost_ref) + "\n"

        return ret
