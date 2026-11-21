"""Optional capabilities fail explicitly; do not fabricate telemetry."""
from abc import ABC, abstractmethod


class Unsupported(RuntimeError):
    pass


class Adapter(ABC):
    def __init__(self, config):
        self.config = config

    @abstractmethod
    def decode(self, message):
        """Decode a raw message into SI rows; keep original alongside it."""

    def read_state(self):
        from .ros2 import read_once
        return read_once(self)

    def observe_actual_command(self):
        raise Unsupported('Controller accepted-target readback is not verified for this adapter')

    def stop_motion(self, motion_id=None):
        raise Unsupported('Reliable controller stop is not implemented; use the original control system')

    def get_joint_map(self):
        return {'mapping': self.config.get('joint_map'), 'verified': self.config.get('joint_map_verified', False)}

    def get_model(self):
        from pathlib import Path
        model=self.config.get('model_path')
        if not model:
            raise Unsupported('Real robot model is not configured')
        path=Path(model).expanduser()
        if not path.is_absolute(): path=Path(self.config['_path']).parent/path
        if not path.is_file(): raise FileNotFoundError(path)
        return path.resolve()
