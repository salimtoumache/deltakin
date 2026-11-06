"""deltakin: forward and inverse kinematics for 3-DOF Delta parallel robots."""
from .errors import DeltaKinematicsError, SingularityError, UnreachableError
from .robot import DeltaGeometry, DeltaRobot

__all__ = [
    "DeltaRobot",
    "DeltaGeometry",
    "DeltaKinematicsError",
    "UnreachableError",
    "SingularityError",
]
__version__ = "0.1.0"
__author__ = "Salim Toumache"
__email__ = "salimtoumache@gmail.com"
