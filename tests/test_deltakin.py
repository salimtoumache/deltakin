import math
import random
import unittest

from deltakin import DeltaGeometry, DeltaRobot, UnreachableError


def make(**kw):
    return DeltaRobot(Sb=567, Sp=76, L=524, l=1244, **kw)


class TestInverse(unittest.TestCase):
    def test_paper_example(self):
        for t in make().inverse(0, 0, -900):
            self.assertAlmostEqual(t, -20.5473, places=3)

    def test_axis_symmetry(self):
        t = make().inverse(0, 0, -1000)
        self.assertAlmostEqual(t[0], t[1], places=9)
        self.assertAlmostEqual(t[1], t[2], places=9)

    def test_unreachable_raises(self):
        robot = make()
        with self.assertRaises(UnreachableError):
            robot.inverse(0, 0, 0)
        self.assertFalse(robot.is_reachable(0, 0, 0))
        self.assertFalse(robot.is_reachable(5000, 0, -900))
        self.assertTrue(robot.is_reachable(0, 0, -900))

    def test_bad_branch(self):
        with self.assertRaises(ValueError):
            make().inverse(0, 0, -900, branch="nope")


class TestRoundTrip(unittest.TestCase):
    def test_inverse_then_forward(self):
        robot = make()
        rng = random.Random(1234)
        tested = 0
        for _ in range(400):
            p = (rng.uniform(-250, 250), rng.uniform(-250, 250), rng.uniform(-1250, -800))
            try:
                primary, alternate = robot.inverse_all(*p)
            except UnreachableError:
                continue
            for sol in (primary, alternate):
                q = robot.forward(*sol)
                for u, v in zip(p, q):
                    self.assertAlmostEqual(u, v, delta=1e-4)
            tested += 1
        self.assertGreater(tested, 100)

    def test_forward_then_inverse(self):
        robot = make()
        rng = random.Random(7)
        for _ in range(100):
            angles = tuple(rng.uniform(-30, 20) for _ in range(3))
            p = robot.forward(*angles)
            back = robot.inverse(*p)
            for u, v in zip(angles, back):
                self.assertAlmostEqual(u, v, delta=1e-6)

    def test_mirror_symmetry(self):
        # swapping legs 2 and 3 mirrors the platform across the yz-plane
        robot = make()
        x, y, z = robot.forward(-10, -15, -5)
        mx, my, mz = robot.forward(-10, -5, -15)
        self.assertAlmostEqual(mx, -x, places=9)
        self.assertAlmostEqual(my, y, places=9)
        self.assertAlmostEqual(mz, z, places=9)


class TestApi(unittest.TestCase):
    def test_radians_match_degrees(self):
        deg = make().inverse(50, -30, -950)
        rad = make(angle_unit="rad").inverse(50, -30, -950)
        for d, r in zip(deg, rad):
            self.assertAlmostEqual(math.radians(d), r, places=12)

    def test_many(self):
        robot = make()
        pts = [(0, 0, -900), (50, 20, -1000)]
        angles = robot.inverse_many(pts)
        back = robot.forward_many(angles)
        for p, q in zip(pts, back):
            for u, v in zip(p, q):
                self.assertAlmostEqual(u, v, delta=1e-6)

    def test_geometry_validation(self):
        for bad in (0, -1, float("nan"), float("inf")):
            with self.assertRaises(ValueError):
                DeltaGeometry(Sb=bad, Sp=76, L=524, l=1244)
        with self.assertRaises(ValueError):
            DeltaRobot(1, 1, 1, 1, angle_unit="grad")

    def test_derived_dimensions(self):
        g = DeltaGeometry(Sb=567, Sp=76, L=524, l=1244)
        self.assertAlmostEqual(g.Wb, 163.679, places=2)
        self.assertAlmostEqual(g.Ub, 327.358, places=2)
        self.assertAlmostEqual(g.Wp, 21.939, places=2)
        self.assertAlmostEqual(g.Up, 43.878, places=2)


if __name__ == "__main__":
    unittest.main()
