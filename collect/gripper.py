"""Collect actual gripper feedback through the configured adapter."""
from .run import run


def collect(adapter, session, duration_s):
    return run(adapter, session, duration_s)
