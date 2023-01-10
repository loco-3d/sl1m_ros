import numpy as np
from sl1m_ros.sl1m_params import Sl1mParameters


class Sl1mParametersTalos(Sl1mParameters):
    def __init__(self):
        super(Sl1mParametersTalos).__init__()

        # simple gait
        self.gait = [np.array([1, 0]), np.array([0, 1])]
        # constraints path
        self.paths = [
            "/opt/openrobots/share/talos-rbprm/com_inequalities/feet_quasi_flat/",
            "/opt/openrobots/share/talos-rbprm/relative_effector_positions/",
        ]
        # Suffixes for finding the constraints files.
        self.suffix_com = "_effector_frame_REDUCED.obj"
        self.suffix_feet = "_quasi_flat_REDUCED.obj"

        # Number and limb name
        self.limbs = ["LF", "RF"]

        # number of steps
        self.nb_steps = 4

        # Cost
        self.cost = {
            # Compute a cost to keep the final CoM position close to a target one
            "end_effector_positions": [
                1.0,
                [[0.3, 0.1, 0.0, 0, 0, 0, 1], [0.3, -0.1, 0.0, 0, 0, 0, 1]],
            ],
        }

        self.optimize_com = False

        self.plot = True
