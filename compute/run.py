"""Offline entry points consume documented JSON; never connect to the robot."""
from pathlib import Path
from .prepare import read,write_result


def run(args,config):
    document=read(args.input)
    if args.task=='export':
        from .export import export
        return export(document,args.output)
    if args.task=='validate':
        from .validate import validate
        result=validate(document,config)
    else:
        if args.task=='static':from .static_fit import solve
        elif args.task=='dynamic':from .dynamic_fit import solve
        elif args.task=='gripper':from .gripper_fit import solve
        elif args.task=='intrinsics':from .camera_intrinsics import solve
        elif args.task=='handeye':
            from .handeye import solve,with_real_fk
            if args.frames:document=with_real_fk(document,args.frames)
        elif args.task=='scene':from .scene import solve
        else:raise ValueError('Unknown compute task')
        result=solve(document)
    return write_result(args.output,result,args.input)
