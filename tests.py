from mhsboat_ctrl.utils.math_util import angle, distance, midpoint

print("Testing math_util.py...")

assert angle(0, 0, 1, 0, 0, 1) == 90
assert angle(0, 0, 1, 0, 1, 1) == 45
assert angle(0, 0, 1, 0, 1, -1) == 135

assert distance(0, 0, 0, 0) == 0
assert distance(0, 0, 1, 0) == 1
assert distance(0, 0, 0, 1) == 1

assert midpoint(0, 0, 0, 0) == (0, 0)
assert midpoint(0, 0, 1, 0) == (0.5, 0)
assert midpoint(0, 0, 0, 1) == (0, 0.5)
assert midpoint(0, 0, 1, 1) == (0.5, 0.5)

print("Tests completed")