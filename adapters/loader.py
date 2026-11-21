"""One executable entry point; loading configuration never enables robot motion."""
import argparse,json,sys
from ..config import load_config,validate,ConfigError
from .base import Unsupported


def make_adapter(config):
    validate(config)
    from .joint_state import JointState
    from .xuchuang.arms import XuchuangArms
    from .xuchuang.waist import XuchuangWaist
    from .xuchuang.gripper import XuchuangGripper
    from .xuchuang.cameras import XuchuangCameras
    classes={'ros2_joint_state':JointState,'xuchuang_arms_ros2':XuchuangArms,
             'xuchuang_waist_ros2':XuchuangWaist,'xuchuang_gripper_ros2':XuchuangGripper,
             'xuchuang_camera_driver_ros2':XuchuangCameras}
    return classes[config['adapter']](config)


def parser():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation',choices=('check','collect','control','stop','compute'))
    p.add_argument('--config',required=True)
    p.add_argument('--session',help='Same explicit session directory in both terminals')
    p.add_argument('--live',action='store_true',help='check: connect and read one real message')
    p.add_argument('--duration-s',type=float,default=1200,help='0 means until Ctrl+C')
    p.add_argument('--camera',choices=('head','left_wrist','right_wrist'),default='head')
    p.add_argument('--tag-arm',choices=('left','right'))
    p.add_argument('--arm',choices=('left','right','both'),default='right')
    p.add_argument('--joint',default='1',help='1..7 or all')
    p.add_argument('--mode',choices=('step','oscillation'),default='step')
    p.add_argument('--direction',type=int,choices=(-1,1),default=1)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--task',choices=('static','dynamic','gripper','intrinsics','handeye','scene','export','validate'))
    p.add_argument('--input');p.add_argument('--output');p.add_argument('--frames')
    p.add_argument('--replay',help='collect: archived JSONL; never authorizes live control')
    return p


def main(argv=None):
    p=parser();args=p.parse_args(argv)
    try:
        config=load_config(args.config)
        if args.execute and args.operation!='control':raise ConfigError('--execute is only valid for control')
        if args.operation=='compute':
            from ..compute.run import run
            if not args.task or not args.input or not args.output:raise ConfigError('compute requires --task --input --output')
            result=run(args,config)
        else:
            adapter=make_adapter(config)
            if args.operation=='check':
                result={'status':'configuration_valid','hardware_verified':False,'adapter':config['adapter'],
                        'capabilities':{'controller_target_readback':False,'reliable_stop':False}}
                if args.live:result={'status':'live_sample_received','sample':adapter.read_state()}
            elif args.operation=='stop':
                from ..control.stop import stop
                result=stop(adapter)
            else:
                if not args.session:raise ConfigError('collect/control requires --session')
                if args.operation=='collect':
                    if args.replay:
                        from ..collect.replay import replay
                        result=replay(adapter,args.session,args.replay)
                    elif config['adapter']=='xuchuang_camera_driver_ros2':
                        from ..collect.cameras import collect
                        result=collect(adapter,args.session,args.camera,args.tag_arm)
                    else:
                        if config['adapter']=='xuchuang_gripper_ros2':
                            from ..collect.gripper import collect
                        else:
                            from ..collect.joints import collect
                        result=collect(adapter,args.session,args.duration_s)
                else:
                    from ..control.run import run_field
                    result=run_field(config,args.session,args.arm,args.joint,args.mode,args.direction,args.execute)
        print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
        return 0
    except KeyboardInterrupt:
        print('Interrupted; no claim of hardware stop',file=sys.stderr);return 130
    except (ConfigError,Unsupported,ValueError,RuntimeError,OSError,ImportError,KeyError,TypeError) as exc:
        print(json.dumps({'status':'failed','error':str(exc)},ensure_ascii=False),file=sys.stderr);return 2


if __name__=='__main__':raise SystemExit(main())
