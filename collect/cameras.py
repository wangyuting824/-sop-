"""Capture frames, calibration data and operator-triggered samples."""
def collect(adapter, session, camera, tag_arm=None):
    return adapter.get_camera_stream(session, camera, tag_arm)
