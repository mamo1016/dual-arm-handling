"""
openarm.py — minimal, self-contained loader for the 7-DOF OpenArm manipulator.

Wraps the URDF in a roboticstoolbox ``Robot`` so the planner can run forward and
inverse kinematics. No visualization or project-specific dependencies — just the
model. The robot model (URDF + meshes) comes from the open-source OpenArm
project; see the README for attribution and the mesh-package dependency.
"""
import logging
from pathlib import Path

import numpy as np
from numpy import pi
from roboticstoolbox import Robot
from spatialmath import SE3  # noqa: F401  (re-exported for convenience)

logger = logging.getLogger(__name__)

# The single-arm URDF lives at the repo root, next to this package. Callers that
# run inside a ROS 2 install (where the URDF is installed to the package's share
# directory) should pass urdf_dir explicitly.
_DEFAULT_URDF_DIR = Path(__file__).resolve().parent.parent
_URDF_FILE = 'openarm_single_control.urdf'


class OpenArm(Robot):
    """7-DOF OpenArm (+ 2 gripper finger joints) loaded from URDF."""

    def __init__(self, urdf_dir=None, urdf_file=_URDF_FILE):
        urdf_dir = Path(urdf_dir) if urdf_dir is not None else _DEFAULT_URDF_DIR
        logger.info('Loading OpenArm URDF from %s/%s', urdf_dir, urdf_file)
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            tld=urdf_dir, file_path=urdf_file)
        super().__init__(
            links, name=name,
            urdf_string=urdf_string, urdf_filepath=urdf_filepath)

        self.manufacturer = 'OpenArm'
        self.ee_link = self.ee_links[0]

        # Named joint configurations (9 = 7 arm joints + 2 gripper fingers).
        # qr is the "ready" pose used to seed inverse kinematics.
        self.qz = np.zeros(9)
        self.qr = np.array([pi / 2, pi / 2, 0, 0, 0, 0, 0, 0, 0])
        self.addconfiguration('qr', self.qr)
        self.addconfiguration('qz', self.qz)
