import rospy
import numpy as np
from pathlib import Path
from sl1m.solver import Solvers


class Sl1mParameters:
    def __init__(self) -> None:
            
        # public params
        self.paths = []
        self.limbs = []
        self.suffix_com = ""
        self.suffix_feet = ""
        self.gait = []
        self.plot = False
        self.cost = {}
        self.nb_steps = 0
        self.rate = 20
        self.optimize_com = True

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
        self._numerical_solver = self._get_param("/sl1m/numerical_solver")
        self.paths = self._get_param("sl1m/paths")
        self.limbs = self._get_param("sl1m/limbs")
        self.suffix_com = self._get_param("sl1m/suffix_com")
        self.suffix_feet = self._get_param("sl1m/suffix_feet")
        self.gait = self._get_param("sl1m/gait")
        self.plot = self._get_param("sl1m/plot")
        self.cost = self._get_param("sl1m/cost")
        self.nb_steps = self._get_param("sl1m/nb_steps")
        self.rate = self._get_param("sl1m/rate")
        self.optimize_com = self._get_param("sl1m/optimize_com")

        # sanity checks on types
        assert type(self.paths) is list
        for path in self.paths:
            assert Path(path).exists()
        assert type(self.limbs) is list
        for limb in self.limbs:
            assert type(limb) is str
        assert type(self.suffix_com) is str
        assert type(self.suffix_feet) is str
        assert type(self.gait) is list
        for gait_el in self.gait:
            assert type(gait_el) is list

        # assert sanity checks on content
        if self.cost:
            pass

    def _get_param(self, name):
        if rospy.has_param(name):
            return rospy.get_param(name)
        else:
            print(self.__dict__)
            var_name = str(Path(name).name)
            if var_name == "numerical_solver":
                return self.__dict__["_" + var_name]
            else:
                return self.__dict__[var_name]

    def __repr__(self):
        ret = "Sl1mParameters\n"
        ret += "    - numerical_solver = " + str(self._numerical_solver) + "\n"
        ret += "    - paths = " + str(self.paths) + "\n"
        ret += "    - limbs = " + str(self.limbs) + "\n"
        ret += "    - suffix_com = " + str(self.suffix_com) + "\n"
        ret += "    - suffix_feet = " + str(self.suffix_feet) + "\n"
        ret += "    - gait = " + str(self.gait) + "\n"
        ret += "    - plot = " + str(self.plot) + "\n"
        ret += "    - cost = " + str(self.cost) + "\n"
        ret += "    - nb_steps = " + str(self.nb_steps) + "\n"
        ret += "    - rate = " + str(self.rate) + "\n"
        ret += "    - optimize_com = " + str(self.optimize_com) + "\n"
        return ret