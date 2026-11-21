"""Collect arm, waist or standard ROS joint feedback."""
from .run import run


def collect(adapter, session, duration_s):
    return run(adapter, session, duration_s)
