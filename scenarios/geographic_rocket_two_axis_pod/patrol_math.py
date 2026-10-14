"""依据实际位置的往返速度调度；不生成或回写飞机真值位姿。"""

import math


def desired_velocity(error, speed, acceleration, gain):
    """远处限速，近处按制动距离及位置误差减速。"""
    magnitude = min(speed, math.sqrt(2 * acceleration * abs(error)), gain * abs(error))
    return math.copysign(magnitude, error)


def slew_velocity(previous, target, acceleration, dt):
    change = acceleration * max(0., dt)
    return previous + max(-change, min(change, target - previous))


def at_endpoint(error, velocity, position_tolerance, speed_tolerance):
    return abs(error) <= position_tolerance and abs(velocity) <= speed_tolerance
