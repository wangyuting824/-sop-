"""Only explicitly registered interfaces can be selected by configuration."""
from pathlib import Path
import copy
import math
import yaml


class ConfigError(ValueError):
    pass


def finite(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigError(f'{label} must be a finite number')
    return float(value)


def required(config, *keys):
    value = config
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            raise ConfigError('Missing configuration: ' + '.'.join(keys))
        value = value[key]
    if value is None or value == '' or isinstance(value, dict) and not value:
        raise ConfigError('Unfilled configuration: ' + '.'.join(keys))
    return value


def load_config(path):
    path = Path(path).expanduser().resolve()
    class UniqueLoader(yaml.SafeLoader):
        pass
    def mapping(loader,node,deep=False):
        result={}
        for key_node,value_node in node.value:
            key=loader.construct_object(key_node,deep=deep)
            if key in result:raise ConfigError(f'Duplicate YAML key: {key}')
            result[key]=loader.construct_object(value_node,deep=deep)
        return result
    UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,mapping)
    try:
        with path.open(encoding='utf-8-sig') as stream:
            config = yaml.load(stream,Loader=UniqueLoader)
    except yaml.YAMLError as exc:
        raise ConfigError(f'Invalid YAML: {exc}') from exc
    if not isinstance(config, dict):
        raise ConfigError('Configuration must be a YAML mapping')
    config = copy.deepcopy(config)
    config['_path'] = str(path)
    if config.get('schema_version') != 1:
        raise ConfigError('schema_version must be 1')
    return config


def public_config(config):
    return {key: value for key, value in config.items() if not key.startswith('_')}


def validate(config):
    adapter = required(config, 'adapter')
    if adapter not in {'ros2_joint_state', 'xuchuang_arms_ros2', 'xuchuang_waist_ros2',
                       'xuchuang_gripper_ros2', 'xuchuang_camera_driver_ros2'}:
        raise ConfigError(f'Adapter not implemented: {adapter}')
    required(config, 'robot')
    if adapter == 'xuchuang_camera_driver_ros2':
        cameras = required(config, 'cameras')
        if not isinstance(cameras, dict):
            raise ConfigError('cameras must be a mapping')
        for name in cameras:
            if name not in {'head', 'left_wrist', 'right_wrist'}:
                raise ConfigError('The existing camera protocol supports head/left_wrist/right_wrist')
            for field in ('image_topic', 'info_topic', 'h264_topic'):
                topic = required(config, 'cameras', name, field)
                if not isinstance(topic, str) or not topic.startswith('/'):
                    raise ConfigError('Camera topic must be absolute')
        for field in ('acquire_service', 'release_service'):
            required(config, 'stream', field)
        if type(config['stream'].get('acquire')) is not bool:
            raise ConfigError('stream.acquire must be boolean')
        if config['stream'].get('image_source') not in ('auto','image','h264'):
            raise ConfigError('image_source must be auto/image/h264')
        if config['stream'].get('codec')!='h264' or config['stream'].get('stream_mask')!=1:
            raise ConfigError('Existing camera stream implementation requests color H264 mask=1')
        if type(config['stream'].get('priority')) is not int or not 0<=config['stream']['priority']<=255:
            raise ConfigError('Stream priority must be uint8')
        expected={'image':'sensor_msgs/msg/Image','camera_info':'sensor_msgs/msg/CameraInfo','h264':'camera_driver/msg/H264Packet'}
        for key,kind in expected.items():
            if required(config,'message_types',key)!=kind:raise ConfigError('Unsupported camera message schema')
        required(config,'board_path')
        return config
    topic = required(config, 'feedback', 'topic')
    if not isinstance(topic, str) or not topic.startswith('/'):
        raise ConfigError('feedback.topic must be an absolute ROS name')
    kind = required(config, 'feedback', 'type')
    expected = {'ros2_joint_state': 'sensor_msgs/msg/JointState',
                'xuchuang_arms_ros2': 'manipulator/msg/ArmStateArray',
                'xuchuang_waist_ros2': 'std_msgs/msg/Float32MultiArray',
                'xuchuang_gripper_ros2': 'end_effector/msg/EndEffectorStatus'}[adapter]
    if kind != expected:
        raise ConfigError(f'{adapter} implements {expected}; a different schema needs adapter code')
    timeout = finite(config.get('feedback_timeout_s', 2), 'feedback_timeout_s')
    if timeout <= 0:
        raise ConfigError('feedback_timeout_s must be positive')
    if adapter in {'ros2_joint_state', 'xuchuang_arms_ros2'}:
        if required(config, 'feedback', 'position_unit') not in {'deg', 'rad'}:
            raise ConfigError('Revolute arm position unit must be deg or rad')
        if required(config, 'feedback', 'velocity_unit') not in {'deg/s', 'rad/s'}:
            raise ConfigError('Velocity unit must be deg/s or rad/s')
    if adapter == 'xuchuang_arms_ros2':
        ids=[]
        for name in ('left', 'right'):
            arm=required(config, 'arms', name)
            if type(arm.get('id')) is not int or arm.get('joint_count') != 7:
                raise ConfigError('Existing Xuchuang protocol requires two integer IDs and seven axes per arm')
            ids.append(arm['id'])
        if len(set(ids)) != 2:
            raise ConfigError('Left and right IDs must differ')
    if adapter == 'xuchuang_waist_ros2':
        if config['feedback'].get('position_unit')!='deg' or config['feedback'].get('velocity_unit')!='deg/s':
            raise ConfigError('Existing MotorState decoder requires deg and deg/s')
    if adapter == 'xuchuang_gripper_ros2':
        if required(config, 'units', 'position') != 'mm':
            raise ConfigError('Existing gripper protocol uses mm')
        for name in ('left','right'):
            required(config,'slots',name)
        if config['slots']['left']==config['slots']['right']:raise ConfigError('Gripper slots must differ')
        for key,kind in {'properties':'end_effector/srv/GetProperties','set_position':'end_effector/srv/SetPosition',
                         'open':'std_srvs/srv/Trigger','close':'std_srvs/srv/Trigger'}.items():
            if required(config,'services',key,'type')!=kind:raise ConfigError(f'Unsupported service schema for {key}')
            required(config,'services',key,'name')
    return config
