from ..base import Adapter,Unsupported


class XuchuangCameras(Adapter):
    def decode(self,message):
        raise Unsupported('Binary camera frames are collected through the preserved camera v3 path')

    def get_camera_stream(self,session,camera,tag_arm=None):
        from ...control.run import run_camera
        return run_camera(self.config,session,camera,tag_arm)

    def read_state(self):
        raise Unsupported('Use collect --camera to acquire/save camera frames and CameraInfo')
