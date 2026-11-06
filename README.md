# deltakin

Forward and inverse kinematics for 3-DOF Delta parallel robots, in pure Python (no dependencies).

Equations follow R. L. Williams II, *The Delta Parallel Robot: Kinematics Solutions* (Ohio University).

## Install

```bash
# Install from PyPI
pip install deltakin

# Install from source
git clone https://github.com/YOUR_USERNAME/deltakin.git
cd deltakin
pip install .
```

## Usage

```python
from deltakin import DeltaRobot, UnreachableError

robot = DeltaRobot(Sb=567, Sp=76, L=524, l=1244)    # lengths in mm

angles = robot.inverse(0, 0, -900)                   # degrees -> about (-20.55, -20.55, -20.55)
x, y, z = robot.forward(*angles)                     # -> (0, 0, -900)

robot.is_reachable(0, 0, 0)                          # False

try:
    robot.inverse(0, 0, 0)
except UnreachableError as err:
    print(err)                                       # clear error, no fake angles

primary, alternate = robot.inverse_all(100, -50, -1000)   # both solutions
robot.inverse_many([(0, 0, -900), (50, 20, -1000)])        # batches
DeltaRobot(567, 76, 524, 1244, angle_unit="rad")           # radians instead of degrees
```

## Conventions

- Origin at the centre of the base, z up (a hanging platform has negative z).
- Leg 1 on the -y side; legs 2 and 3 on the +x and -x sides.
- Lengths use any single unit; angles default to degrees.
- `inverse(..., branch="primary")` returns the usual elbow-out solution; `"alternate"` returns the other one.

## Tests

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

The test suite includes round trips (inverse then forward, and forward then inverse), the example from the Williams paper, and a mirror-symmetry check.

## Author

Salim Toumache, mechatronics engineer: <salimtoumache@gmail.com>

## License

MIT
