"""Kinematics of a 3-DOF Delta parallel robot.

Notation and equations follow:
    R. L. Williams II, "The Delta Parallel Robot: Kinematics Solutions",
    Ohio University.

Frame: origin at the centre of the base, z axis pointing up, so a platform
hanging below the base has a negative z. Leg 1 sits on the -y side, legs 2
and 3 on the +x / -x sides.

All lengths share one (arbitrary) unit; the equations are scale free.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

from .errors import SingularityError, UnreachableError

Vec3 = Tuple[float, float, float]

_SQRT3 = math.sqrt(3.0)
_UNITS = ("deg", "rad")
_BRANCHES = ("primary", "alternate")


@dataclass(frozen=True)
class DeltaGeometry:
    """Physical dimensions of the robot.

    Sb: base equilateral triangle side
    Sp: platform equilateral triangle side
    L:  upper arm length
    l:  lower arm (parallelogram) length
    """

    Sb: float
    Sp: float
    L: float
    l: float

    def __post_init__(self) -> None:
        for name in ("Sb", "Sp", "L", "l"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a positive finite number, got {value!r}")

    @property
    def Wb(self) -> float:
        """Planar distance from the base centre to the nearest base side."""
        return _SQRT3 / 6.0 * self.Sb

    @property
    def Ub(self) -> float:
        """Planar distance from the base centre to a base vertex."""
        return _SQRT3 / 3.0 * self.Sb

    @property
    def Wp(self) -> float:
        """Planar distance from the platform centre to the nearest platform side."""
        return _SQRT3 / 6.0 * self.Sp

    @property
    def Up(self) -> float:
        """Planar distance from the platform centre to a platform vertex."""
        return _SQRT3 / 3.0 * self.Sp


def _wrap(angle: float) -> float:
    """Wrap an angle in radians to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class DeltaRobot:
    """Forward and inverse kinematics of a Delta robot.

    >>> robot = DeltaRobot(Sb=567, Sp=76, L=524, l=1244)
    >>> angles = robot.inverse(0, 0, -900)      # degrees
    >>> x, y, z = robot.forward(*angles)        # back to (0, 0, -900)
    """

    def __init__(self, Sb: float, Sp: float, L: float, l: float, *, angle_unit: str = "deg") -> None:
        if angle_unit not in _UNITS:
            raise ValueError(f"angle_unit must be one of {_UNITS}, got {angle_unit!r}")
        self.geometry = DeltaGeometry(float(Sb), float(Sp), float(L), float(l))
        self.angle_unit = angle_unit

    def __repr__(self) -> str:
        g = self.geometry
        return f"DeltaRobot(Sb={g.Sb:g}, Sp={g.Sp:g}, L={g.L:g}, l={g.l:g}, angle_unit={self.angle_unit!r})"

    # ------------------------------------------------------------------ units
    def _to_rad(self, angle: float) -> float:
        return math.radians(angle) if self.angle_unit == "deg" else float(angle)

    def _from_rad(self, angle: float) -> float:
        return math.degrees(angle) if self.angle_unit == "deg" else angle

    # ---------------------------------------------------------------- forward
    def forward(self, theta1: float, theta2: float, theta3: float) -> Vec3:
        """Platform centre (x, y, z) for the three actuated joint angles.

        Raises UnreachableError if the three legs cannot be assembled, and
        SingularityError at a singular configuration.
        """
        g = self.geometry
        t1, t2, t3 = self._to_rad(theta1), self._to_rad(theta2), self._to_rad(theta3)
        Wb, Wp, Up, Sp, L, l = g.Wb, g.Wp, g.Up, g.Sp, g.L, g.l

        # Centres of the three spheres (elbows shifted by the platform joint offsets).
        y1 = -Wb - L * math.cos(t1) + Up
        z1 = -L * math.sin(t1)

        x2 = (_SQRT3 / 2.0) * (Wb + L * math.cos(t2)) - Sp / 2.0
        y2 = 0.5 * (Wb + L * math.cos(t2)) - Wp
        z2 = -L * math.sin(t2)

        x3 = -(_SQRT3 / 2.0) * (Wb + L * math.cos(t3)) + Sp / 2.0
        y3 = 0.5 * (Wb + L * math.cos(t3)) - Wp
        z3 = -L * math.sin(t3)

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2
        if abs(dnm) < 1e-12 * L * L:
            raise SingularityError("Singular configuration (the three sphere centres are aligned).")

        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3

        # x = (a1 z + b1) / dnm
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
        # y = (a2 z + b2) / dnm
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        # a z^2 + b z + c = 0 (sphere of radius l around the first centre)
        a = a1 * a1 + a2 * a2 + dnm * dnm
        b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = (b2 - y1 * dnm) ** 2 + b1 * b1 + dnm * dnm * (z1 * z1 - l * l)

        disc = b * b - 4.0 * a * c
        if disc < 0:
            if disc > -1e-9 * b * b:
                disc = 0.0
            else:
                raise UnreachableError("These joint angles do not give a valid platform position.")

        z0 = -0.5 * (b + math.sqrt(disc)) / a
        x0 = (a1 * z0 + b1) / dnm
        y0 = (a2 * z0 + b2) / dnm
        return (x0, y0, z0)

    # ---------------------------------------------------------------- inverse
    def _inverse_rad(self, x: float, y: float, z: float) -> Tuple[List[float], List[float]]:
        g = self.geometry
        L, l = g.L, g.l
        a = g.Wb - g.Up
        b = g.Sp / 2.0 - (_SQRT3 / 2.0) * g.Wb
        c = g.Wp - g.Wb / 2.0

        r2 = x * x + y * y + z * z
        E = (
            2.0 * L * (y + a),
            -L * (_SQRT3 * (x + b) + y + c),
            L * (_SQRT3 * (x - b) - y - c),
        )
        F = (2.0 * z * L,) * 3
        G = (
            r2 + a * a + L * L + 2.0 * y * a - l * l,
            r2 + b * b + c * c + L * L + 2.0 * (x * b + y * c) - l * l,
            r2 + b * b + c * c + L * L - 2.0 * (x * b - y * c) - l * l,
        )

        primary: List[float] = []
        alternate: List[float] = []
        for i in range(3):
            disc = E[i] ** 2 + F[i] ** 2 - G[i] ** 2
            if disc < 0:
                if disc > -1e-9 * (E[i] ** 2 + F[i] ** 2 + G[i] ** 2 + 1.0):
                    disc = 0.0
                else:
                    raise UnreachableError(
                        f"Position ({x:g}, {y:g}, {z:g}) is outside the workspace "
                        f"(leg {i + 1} cannot reach it)."
                    )
            root = math.sqrt(disc)
            den = G[i] - E[i]
            # atan2 keeps this well defined even when den == 0.
            primary.append(_wrap(2.0 * math.atan2(-F[i] - root, den)))
            alternate.append(_wrap(2.0 * math.atan2(-F[i] + root, den)))
        return primary, alternate

    def inverse_all(self, x: float, y: float, z: float) -> Tuple[Vec3, Vec3]:
        """Both joint-angle solutions: (primary, alternate)."""
        p, a = self._inverse_rad(x, y, z)
        return (
            tuple(self._from_rad(t) for t in p),  # type: ignore[return-value]
            tuple(self._from_rad(t) for t in a),  # type: ignore[return-value]
        )

    def inverse(self, x: float, y: float, z: float, branch: str = "primary") -> Vec3:
        """Joint angles (theta1, theta2, theta3) for a platform position.

        branch="primary" is the usual elbow-out configuration (about -20.5
        degrees at (0, 0, -900) for the example geometry); "alternate" is the
        other mathematical solution.

        Raises UnreachableError if the position is outside the workspace.
        """
        if branch not in _BRANCHES:
            raise ValueError(f"branch must be one of {_BRANCHES}, got {branch!r}")
        primary, alternate = self.inverse_all(x, y, z)
        return primary if branch == "primary" else alternate

    # ------------------------------------------------------------------ extras
    def is_reachable(self, x: float, y: float, z: float) -> bool:
        """True if the platform centre can be placed at (x, y, z)."""
        try:
            self._inverse_rad(x, y, z)
        except UnreachableError:
            return False
        return True

    def forward_many(self, angles: Iterable[Sequence[float]]) -> List[Vec3]:
        """forward() for many (theta1, theta2, theta3) triples."""
        return [self.forward(*t) for t in angles]

    def inverse_many(self, points: Iterable[Sequence[float]], branch: str = "primary") -> List[Vec3]:
        """inverse() for many (x, y, z) points."""
        return [self.inverse(*p, branch=branch) for p in points]
