import copy,importlib.util,json,math,os,socket,subprocess,sys,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch

from calibration_platform.config import load_config,validate,ConfigError
from calibration_platform.adapters.loader import make_adapter,main
from calibration_platform.adapters.base import Unsupported
from calibration_platform.collect.writer import session_root,snapshot,atomic_json,health_path,config_digest,assert_ready
from calibration_platform.collect.replay import replay
from calibration_platform.control.run import load_source,ROOT as VENDOR,ReplaceEndpoints
from calibration_platform.compute.gripper_fit import solve as gripper_fit
from calibration_platform.compute.static_fit import solve as static_fit
from calibration_platform.compute.dynamic_fit import solve as dynamic_fit
from calibration_platform.compute.scene import solve as scene_fit
from calibration_platform.compute.validate import validate as validate_metrics

ROOT=Path(__file__).resolve().parents[1]
SAMPLES=ROOT/'tests/samples'


def config(name='arms'):
    folder=ROOT/'configs/xuchuang'
    if not folder.is_dir():folder=ROOT/'tests/configs/xuchuang'
    return load_config(folder/f'{name}.yaml')

def known_path(name='arms'):
    return config(name)['_path']
def message(name):return json.loads((SAMPLES/f'{name}.jsonl').read_text(encoding='utf-8'))['message']


class ConfigTests(unittest.TestCase):
    def test_known_configurations(self):
        for name in ('arms','waist','gripper','cameras'):validate(config(name))

    def test_blank_templates_fail_explicitly(self):
        for p in (ROOT/'configs/generic').glob('*.yaml'):
            if p.stem not in ('robot','acceptance'):
                with self.assertRaises(ConfigError):validate(load_config(p))

    def test_no_arbitrary_import(self):
        c=config();c['adapter']='os:system'
        with self.assertRaises(ConfigError):make_adapter(c)

    def test_bad_type(self):
        c=config();c['feedback']['type']='std_msgs/msg/String'
        with self.assertRaises(ConfigError):validate(c)

    def test_duplicate_arm_ids(self):
        c=config();c['arms']['right']['id']=0
        with self.assertRaises(ConfigError):validate(c)

    def test_bad_units(self):
        c=config();c['feedback']['position_unit']='ticks'
        with self.assertRaises(ConfigError):validate(c)

    def test_nan_timeout(self):
        c=config();c['feedback_timeout_s']=float('nan')
        with self.assertRaises(ConfigError):validate(c)


class DecodeTests(unittest.TestCase):
    def test_actual_arms_against_field_implementation(self):
        c=config();raw=message('arms');rows=make_adapter(c).decode(raw)
        legacy=load_source('_field',VENDOR.parents[1]/'control/arms.py',c)
        old=legacy.feedback_rows(raw,0)
        self.assertEqual(len(rows),14)
        for new,previous in zip(rows,old):
            self.assertAlmostEqual(new['q'],previous[-2]);self.assertAlmostEqual(new['dq'],previous[-1])

    def test_arm_unit_conversion(self):
        c=config();raw=message('arms');raw['arms'][0]['joint_pos_act'][0]=180
        self.assertAlmostEqual(make_adapter(c).decode(raw)[0]['q'],math.pi)

    def test_missing_velocity_is_not_zero(self):
        raw=message('arms');raw['arms'][0].pop('joint_vel_act',None)
        self.assertIsNone(make_adapter(config()).decode(raw)[0]['dq'])

    def test_bad_axis_count(self):
        raw=message('arms');raw['arms'][0]['joint_pos_act']=[0]
        with self.assertRaises(ValueError):make_adapter(config()).decode(raw)

    def test_nonfinite_feedback(self):
        raw=message('arms');raw['arms'][0]['joint_pos_act'][0]=float('nan')
        with self.assertRaises(ValueError):make_adapter(config()).decode(raw)

    def test_missing_arm(self):
        raw=message('arms');raw['arms']=raw['arms'][:1]
        with self.assertRaises(ValueError):make_adapter(config()).decode(raw)

    def test_actual_waist(self):
        rows=make_adapter(config('waist')).decode(message('waist'))
        self.assertEqual([r['joint'] for r in rows][2],'WAIST_YAW');self.assertEqual(len(rows),6)
        self.assertIsNone(rows[0]['source_stamp'])

    def test_waist_offset(self):
        raw=message('waist');raw['data']=[99]+raw['data'];raw.setdefault('layout',{})['data_offset']=1
        self.assertEqual(len(make_adapter(config('waist')).decode(raw)),6)

    def test_bad_waist_length(self):
        raw=message('waist');raw['data'].pop()
        with self.assertRaises(ValueError):make_adapter(config('waist')).decode(raw)

    def test_actual_gripper(self):
        raw=message('gripper');rows=make_adapter(config('gripper')).decode(raw)
        self.assertEqual(len(rows),2)
        self.assertAlmostEqual(rows[0]['q'],raw['units'][0]['position'][0]/1000)

    def test_bad_gripper_status(self):
        raw=message('gripper');raw['units'][0]['position']=[-1]
        with self.assertRaises(ValueError):make_adapter(config('gripper')).decode(raw)

    def test_standard_joint_state(self):
        c=config();c.update(adapter='ros2_joint_state');c['feedback'].update(type='sensor_msgs/msg/JointState',position_unit='rad',velocity_unit='rad/s')
        rows=make_adapter(c).decode({'name':['j1'],'position':[1],'velocity':[]})
        self.assertEqual(rows[0]['q'],1);self.assertIsNone(rows[0]['dq'])

    def test_unsupported_methods_are_explicit(self):
        a=make_adapter(config())
        for f in (a.stop_motion,a.observe_actual_command,a.get_model):
            with self.assertRaises(Unsupported):f()


class FieldBridgeTests(unittest.TestCase):
    def test_config_changes_real_read_topic_and_action(self):
        c=config();c['feedback']['topic']='/test/feedback';c['motion']['action']='/test/move'
        module=load_source('_field',VENDOR.parents[1]/'control/arms.py',c)
        class Result:stdout=(json.dumps(message('arms'))+'\n')
        with patch.object(module.subprocess,'run',return_value=Result()) as call:
            module.read_state()
        self.assertEqual(call.call_args.args[0][3],'/test/feedback')
        self.assertEqual(module.ACTION,'/test/move')

    def test_real_action_success_result(self):
        module=load_source('_field',VENDOR.parents[1]/'control/arms.py',config())
        self.assertEqual(module.result_ok((SAMPLES/'move_joint_success.txt').read_text(),0)['code'],0)

    def test_action_rejection(self):
        module=load_source('_field',VENDOR.parents[1]/'control/arms.py',config())
        with self.assertRaises(RuntimeError):module.result_ok('Goal was rejected.',0)

    def test_motion_168_targets(self):
        module=load_source('_field',VENDOR.parents[1]/'control/arms.py',config())
        seq=module.targets([0]*7,0,math.degrees(.05),3)
        self.assertEqual((len(seq)-1)*14,168);self.assertEqual(seq[-1][1],[0]*7)

    def test_gripper_endpoint_remapping(self):
        c=config('gripper');c['services']['set_position']['name']='/test/set'
        m=load_source('_gripper',VENDOR.parents[1]/'control/gripper.py',c)
        self.assertEqual(m.SET_SERVICE,'/test/set')
        self.assertEqual(m.parse_measurement('12.3')[0],12.3)

    def test_waist_head_fault_policy_preserved(self):
        c=config('waist');m=load_source('_waist',VENDOR.parents[1]/'control/waist.py',c)
        self.assertEqual(m.COMMAND_TOPIC,c['motion']['topic'])
        text=(VENDOR.parents[1]/'control/waist.py').read_text(encoding='utf-8')
        self.assertIn('head_status_policy',text)

    def test_camera_dynamic_topics_use_config(self):
        import ast
        c=config('cameras');mapper=ReplaceEndpoints({});mapper.cameras=c['cameras']
        tree=ast.parse('image_topic = f"/camera_driver/slots/{args.camera}/color"')
        out=ast.fix_missing_locations(mapper.visit(tree))
        from types import SimpleNamespace
        env={'args':SimpleNamespace(camera='head'),'_configured_camera_topic':lambda n,k:'/custom/'+k}
        exec(compile(out,'<test>','exec'),env)
        self.assertEqual(env['image_topic'],'/custom/image_topic')


class SessionTests(unittest.TestCase):
    def test_replay_cli_end_to_end(self):
        with tempfile.TemporaryDirectory() as d:
            code=main(['collect','--config',known_path(),'--session',d,'--replay',str(SAMPLES/'arms.jsonl')])
            self.assertEqual(code,0)
            rows=(Path(d)/'raw/joints/arms/states.jsonl').read_text(encoding='utf-8').splitlines()
            self.assertEqual(len(rows),14);self.assertTrue((Path(d)/'manifest.sha256').is_file())
            with self.assertRaises(RuntimeError):assert_ready(d,config())

    def test_config_change_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            root=session_root(d);c=config();snapshot(root,c);c['motion']['vel_percent']=.02
            with self.assertRaises(ValueError):snapshot(root,c)

    def test_ready_heartbeat(self):
        with tempfile.TemporaryDirectory() as d:
            c=config();h={'status':'ready','monotonic_s':time.monotonic(),'host':socket.gethostname(),'config_sha256':config_digest(c)}
            atomic_json(health_path(d,c),h);assert_ready(d,c)
            h['monotonic_s']-=30;atomic_json(health_path(d,c),h)
            with self.assertRaises(RuntimeError):assert_ready(d,c)

    def test_no_overwrite_raw(self):
        with tempfile.TemporaryDirectory() as d:
            replay(make_adapter(config()),d,SAMPLES/'arms.jsonl')
            with self.assertRaises(FileExistsError):replay(make_adapter(config()),d,SAMPLES/'arms.jsonl')

    def test_live_collection_with_injected_transport(self):
        from calibration_platform.collect.run import run
        class Fake:
            def __init__(self,adapter,callback):self.callback=callback
            def spin(self):self.callback(message('arms'),{'received_unix_ns':time.time_ns(),'received_monotonic_ns':time.monotonic_ns()});time.sleep(.002)
            def close(self):pass
        with tempfile.TemporaryDirectory() as d,patch('calibration_platform.adapters.ros2.Subscription',Fake):
            result=run(make_adapter(config()),d,.005)
            self.assertEqual(result['status'],'recorded');self.assertGreaterEqual(result['samples'],1)
            self.assertEqual(len((Path(d)/'raw/joints/arms/states.jsonl').read_text().splitlines()),14*result['samples'])


class ComputeTests(unittest.TestCase):
    def test_gripper_known_scale(self):
        rows=[{'slot':'left','command_mm':x,'measured_mm':.98*x+.6,'split':s} for x,s in [(10,'fit'),(40,'fit'),(80,'fit'),(20,'holdout')]]
        r=gripper_fit({'points':rows})['gripper']['left'];self.assertAlmostEqual(r['scale'],.98);self.assertLess(r['holdout_rmse_mm'],1e-10)

    def test_gripper_no_holdout(self):
        with self.assertRaises(ValueError):gripper_fit({'points':[{'slot':'left','split':'fit'}]*3})

    def test_static_independent_reference(self):
        doc={'reference_source':'external_angle_meter','joint_names':['j1'],
             'samples':[{'q_measured_rad':[i*.1],'q_reference_rad':[i*.1+.02],'split':'fit' if i<3 else 'holdout'} for i in range(4)]}
        self.assertAlmostEqual(static_fit(doc)['joint_zero_offsets_rad']['j1'],.02)

    def test_static_encoder_only_rejected(self):
        with self.assertRaises(ValueError):static_fit({'reference_source':'encoder'})

    def test_dynamic_no_fake_pd(self):
        doc={'time_basis':'shared_clock_seconds','samples':[{'t_s':i*.1,'target_rad':0 if i<2 else .1,
              'position_rad':0 if i<3 else .1} for i in range(10)]}
        r=dynamic_fit(doc);self.assertIsNone(r['Kp']);self.assertAlmostEqual(r['observed_response_delay_s'],.1)

    def test_dynamic_bad_clock(self):
        with self.assertRaises(ValueError):dynamic_fit({'samples':[]})

    def test_scene_identity(self):
        import numpy as np
        doc={'parent_frame':'base','child_frame':'table','observations':[{'matrix':np.eye(4).tolist(),'split':'fit' if i<3 else 'holdout'} for i in range(4)]}
        r=scene_fit(doc);self.assertEqual(r['holdout_translation_errors_m'],[0.])

    def test_unknown_threshold_not_passed(self):
        self.assertIsNone(validate_metrics({'metrics':{'error':.1}},{'thresholds':{'error':None}})['passed'])

    def test_missing_required_metric_not_passed(self):
        self.assertIsNone(validate_metrics({'metrics':{'a':.1}},{'thresholds':{'a':.2,'b':.3}})['passed'])

    def test_candidate_export_rejected(self):
        from calibration_platform.compute.export import export
        with tempfile.TemporaryDirectory() as d,self.assertRaises(ValueError):export({'validated':False},Path(d)/'out')

    def test_explicit_urdf_origin_writeback(self):
        from calibration_platform.compute.export import export
        with tempfile.TemporaryDirectory() as d:
            src=Path(d)/'robot.urdf';src.write_text('<robot name="r"><joint name="j" type="fixed"><parent link="a"/><child link="b"/></joint></robot>')
            out=Path(d)/'out';export({'validated':True,'urdf':{'source':str(src),'absolute_joint_origins':{'j':{'xyz_m':[0,0,.1],'rpy_rad':[0,0,0]}}}},out)
            self.assertIn('0.1',(out/'robot_calibrated.urdf').read_text())


class CLITests(unittest.TestCase):
    def test_check_offline(self):self.assertEqual(main(['check','--config',known_path()]),0)
    def test_generic_unfilled(self):self.assertEqual(main(['check','--config',str(ROOT/'configs/generic/arms.yaml')]),2)
    def test_execute_only_control(self):self.assertEqual(main(['collect','--execute','--config',known_path()]),2)
    def test_control_requires_session(self):self.assertEqual(main(['control','--config',known_path()]),2)
    def test_execute_without_live_collector_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(main(['control','--execute','--session',d,'--config',known_path()]),2)


if __name__=='__main__':unittest.main()
