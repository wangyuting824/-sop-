import json,tempfile,unittest,time
from pathlib import Path
from unittest.mock import patch
from test_adapters import config,message
from calibration_platform.config import load_config,ConfigError,validate
from calibration_platform.adapters.loader import make_adapter


class ErrorTests(unittest.TestCase):
    def test_duplicate_yaml_keys(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'c.yaml';p.write_text('schema_version: 1\nadapter: one\nadapter: two\n')
            with self.assertRaises(ConfigError):load_config(p)

    def test_yaml_cannot_execute_python(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'c.yaml';p.write_text('!!python/object/apply:os.system ["echo NO"]')
            with self.assertRaises(ConfigError):load_config(p)

    def test_camera_wrong_schema_rejected(self):
        c=config('cameras');c['message_types']['h264']='wrong/msg/Type'
        with self.assertRaises(ConfigError):validate(c)

    def test_waist_wrong_units_rejected(self):
        c=config('waist');c['feedback']['position_unit']='rad'
        with self.assertRaises(ConfigError):validate(c)

    def test_gripper_wrong_service_schema(self):
        c=config('gripper');c['services']['close']['type']='std_srvs/srv/SetBool'
        with self.assertRaises(ConfigError):validate(c)

    def test_invalid_message_preserved(self):
        from calibration_platform.collect.run import run
        raw=message('arms');raw['arms'][0]['joint_pos_act'][0]=float('nan')
        class Fake:
            def __init__(self,adapter,callback):self.callback=callback
            def spin(self):self.callback(raw,{'received_unix_ns':time.time_ns()})
            def close(self):pass
        with tempfile.TemporaryDirectory() as d,patch('calibration_platform.adapters.ros2.Subscription',Fake):
            with self.assertRaises(ValueError):run(make_adapter(config()),d,1)
            saved=json.loads((Path(d)/'raw/joints/arms/messages.jsonl').read_text(encoding='utf-8'))
            self.assertEqual(saved['message']['arms'][0]['joint_pos_act'][0],{'nonfinite_float':'nan'})
            state=json.loads((Path(d)/'health/arms.json').read_text())
            self.assertEqual(state['status'],'failed')

    def test_negative_duration_rejected(self):
        from calibration_platform.collect.run import run
        with tempfile.TemporaryDirectory() as d,self.assertRaises(ValueError):run(make_adapter(config()),d,-1)


if __name__=='__main__':unittest.main()
