"""Run trusted preserved field implementations with explicit endpoint substitutions.

Only constants in repository-owned source are rewritten, never Python from YAML.
Protocol structure/result semantics remain in the field implementation.
"""
import ast, importlib.util, json, sys, time
from pathlib import Path
from ..collect.writer import assert_ready,session_root,snapshot,atomic_json,device_name,device_dir,manifest
from ..config import required

ROOT=Path(__file__).resolve().parents[1]/'vendor'/'xuchuang'


def replacements(config):
    result={}
    if config['adapter']=='xuchuang_arms_ros2':
        result={'/manipulator/arm_state':config['feedback']['topic'],
                '/manipulator/move_joint':required(config,'motion','action')}
        if required(config,'motion','type')!='manipulator/action/MoveJoint':
            raise ValueError('Existing arms motion implementation requires MoveJoint schema')
        # Original field controller performs degree arithmetic on raw feedback.
        if config['feedback']['position_unit']!='deg' or config['feedback']['velocity_unit']!='deg/s':
            raise ValueError('Preserved field motion requires degree-native feedback')
    elif config['adapter']=='xuchuang_waist_ros2':
        result={'/MotorState':config['feedback']['topic'],
                '/WaistCtrl':required(config,'motion','topic')}
        if config['feedback']['topic'] not in ('/MotorState','/motor_state/array'):
            raise ValueError('Preserved waist selector supports its two known topic names')
    elif config['adapter']=='xuchuang_gripper_ros2':
        result={'/end_effector/status':config['feedback']['topic']}
        for key,old in [('properties','get_properties'),('set_position','set_position'),('open','open'),('close','close')]:
            result['/end_effector/'+old]=required(config,'services',key,'name')
        if config['slots']!={'left':'left','right':'right'} or config.get('endpoint_scope')!='module':
            raise ValueError('Preserved fixed gripper trial requires left/right and module-wide endpoints')
    elif config['adapter']=='xuchuang_camera_driver_ros2':
        for name,cam in config['cameras'].items():
            prefix='/camera_driver/slots/'+name
            for old,key in [('/color','image_topic'),('/color/camera_info','info_topic'),('/h264','h264_topic')]:
                result[prefix+old]=cam[key]
        result['/camera_driver/acquire_stream']=config['stream']['acquire_service']
        result['/camera_driver/release_stream']=config['stream']['release_service']
        result['/manipulator/joint_states']=required(config,'joint_feedback_topic')
        result['/manipulator/arm_state']=required(config,'arm_feedback_topic')
    return result


class ReplaceEndpoints(ast.NodeTransformer):
    def __init__(self,mapping):self.mapping=mapping
    def visit_Constant(self,node):
        if isinstance(node.value,str) and node.value in self.mapping:
            return ast.copy_location(ast.Constant(value=self.mapping[node.value]),node)
        return node

    def visit_Assign(self,node):
        keys={'image_topic','h264_topic','info_topic'}
        if (hasattr(self,'cameras') and len(node.targets)==1 and isinstance(node.targets[0],ast.Name)
                and node.targets[0].id in keys):
            node.value=ast.Call(func=ast.Name(id='_configured_camera_topic',ctx=ast.Load()),
                                args=[ast.Attribute(value=ast.Name(id='args',ctx=ast.Load()),attr='camera',ctx=ast.Load()),
                                      ast.Constant(node.targets[0].id)],keywords=[])
            return node
        return self.generic_visit(node)


def load_source(name,path,config):
    module=importlib.util.module_from_spec(importlib.util.spec_from_file_location(name,path))
    tree=ast.parse(path.read_text(encoding='utf-8-sig'),filename=str(path))
    mapper=ReplaceEndpoints(replacements(config))
    if config['adapter']=='xuchuang_camera_driver_ros2':
        mapper.cameras=config['cameras']
        module._configured_camera_topic=lambda name,key:config['cameras'][name][key]
    tree=mapper.visit(tree);ast.fix_missing_locations(tree)
    exec(compile(tree,str(path),'exec'),module.__dict__)
    if config['adapter']=='xuchuang_arms_ros2':
        module.ARMS={k:v['id'] for k,v in config['arms'].items()}
    return module


def run_field(config,session,arm='right',joint='1',mode='step',direction=1,execute=False):
    root=session_root(session);snapshot(root,config)
    kind=device_name(config)
    if kind not in {'arms','waist','gripper'}:raise ValueError('No field controller for this adapter')
    if execute:assert_ready(root,config)
    path=Path(__file__).resolve().parent/(kind+'.py')
    output=device_dir(root,kind)/'control';output.mkdir(parents=True,exist_ok=True)
    stamp=str(time.time_ns());record=output/f'invocation_{stamp}.json'
    if kind=='arms':
        command_unit=required(config,'motion','position_unit')
        args=[arm,str(joint)]
        if mode=='step':args+=['--step-rad',str(.05*direction)]
        else:args+=['--delta-rad','0.05','--cycles','3']
        args+=['--command-unit',command_unit,'--vel-percent',str(required(config,'motion','vel_percent')),
               '--acc-percent',str(required(config,'motion','acc_percent')),'--output',str(output)]
    elif kind=='waist':args=['0.05','--delta-unit','rad','--cycles','3','--state-topic',config['feedback']['topic'],'--output',str(output)]
    else:args=['both','--output',str(output)]
    if execute:args+=['--execute']
    report={'implementation':path.name,'arguments':args,'execute':execute,'status':'starting',
            'stop_status':'not_verified; CLI interruption is not hardware stop'}
    atomic_json(record,report)
    oldargv=sys.argv[:]
    try:
        module=load_source('_robotcal_field_motion',path,config)
        if kind=='arms':
            original_send=module.send
            def send(*a,**kw):assert_ready(root,config);return original_send(*a,**kw)
            module.send=send
            previous_rows=module.feedback_rows
            last_received=[None]
            def rows(*a,**kw):
                result=previous_rows(*a,**kw);last_received[0]=time.monotonic();return result
            module.feedback_rows=rows
            previous_check=module.FeedbackRecorder.check
            def check(recorder):
                previous_check(recorder)
                if last_received[0] is not None and time.monotonic()-last_received[0]>config.get('feedback_timeout_s',2):
                    raise RuntimeError('Native feedback expired; no more targets will be sent')
            module.FeedbackRecorder.check=check
        sys.argv=[str(path),*args]
        code=module.main()
        report['status']='completed' if code in (None,0) else 'failed'
        if code not in (None,0):raise RuntimeError(f'Field controller returned {code}')
    except BaseException as exc:
        report.update(status='interrupted' if isinstance(exc,KeyboardInterrupt) else 'failed',error=str(exc));raise
    finally:
        sys.argv=oldargv;atomic_json(record,report)
        # Collector may still be appending. Final manifest is generated after collection ends.
    return report


def run_camera(config,session,camera,tag_arm=None,solve=False,source=None,frames=None):
    root=session_root(session);snapshot(root,config)
    if camera not in config['cameras']:raise ValueError('Camera is not configured')
    if camera=='head' and not tag_arm and not solve:raise ValueError('head requires --tag-arm left/right')
    if camera!='head' and tag_arm:raise ValueError('--tag-arm is only for head')
    scripts=ROOT/'camera'/'scripts'
    names=('camera_common','capture_data','camera_stream','solve_extrinsic')
    saved={n:sys.modules.get(n) for n in names};oldargv=sys.argv[:]
    try:
        for name in names:sys.modules[name]=load_source(name,scripts/(name+'.py'),config)
        name='solve_camera' if solve else 'capture_camera'
        module=load_source('_robotcal_camera_entry',scripts/(name+'.py'),config)
        if solve:
            if not source or not frames:raise ValueError('Hand-eye requires --input session and --frames real mapping/model config')
            args=[str(source),'--config',str(frames)]
        else:
            output=root/'raw'/'cameras'/f'{camera}_{time.time_ns()}'
            args=[camera,'--output',str(output),'--image-source',config['stream']['image_source'],
                  '--stream-priority',str(config['stream']['priority'])]
            if tag_arm:args+=['--tag-arm',tag_arm]
            if not config['stream']['acquire']:args+=['--no-acquire-stream']
            board=config.get('board_path')
            if board:
                board=Path(config['_path']).parent/board
                args+=['--board',str(board.resolve())]
        sys.argv=[str(scripts/(name+'.py')),*args]
        if solve:return module.main()
        return module.run(module.arguments())
    finally:
        sys.argv=oldargv
        for name,value in saved.items():
            if value is None:sys.modules.pop(name,None)
            else:sys.modules[name]=value
        manifest(root)
