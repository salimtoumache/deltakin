"""Exceptions raised by deltakin."""


class DeltaKinematicsError(Exception):
    """Base class for all deltakin errors."""


class UnreachableError(DeltaKinematicsError, ValueError):
    """The requested position (or joint configuration) has no valid solution."""


class SingularityError(DeltaKinematicsError, ArithmeticError):
    """The configuration is singular and the solution is undefined."""
