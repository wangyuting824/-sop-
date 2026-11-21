"""Reuse the existing solver, requiring real FK poses and explicit 10/5 split."""
from .prepare import split


def with_real_fk(document,frames_path):
    import copy,json
    from pathlib import Path
    import numpy as np
    from ..vendor.xuchuang.camera.scripts.camera_common import UrdfKinematics
    document=copy.deepcopy(document);path=Path(frames_path).expanduser().resolve()
    config=json.loads(path.read_text(encoding='utf-8-sig'))
    if not config.get('urdf_path'):raise ValueError('Real urdf_path is missing')
    urdf=Path(config['urdf_path']).expanduser()
    if not urdf.is_absolute():urdf=path.parent/urdf
    chain=config['arms'][document['arm']]
    base,tool=chain.get('reference_frame'),chain.get('tool_frame')
    if not base or not tool or base==tool:raise ValueError('Real base/tool frames are required')
    mapping=chain.get('joint_map',{})
    for sample in document['samples']:
        if not sample.get('use_for_calibration'):continue
        raw=sample['robot_feedback']['q_rad']
        q={mapping.get(k,k):float(v) for k,v in raw.items()}
        if len(q)!=len(raw):raise ValueError('Duplicate URDF map destinations')
        fk=UrdfKinematics(urdf,list(q),base,tool)
        if any(j.find('mimic') is not None or j.attrib.get('type')=='prismatic' for j in fk.chain):
            raise ValueError('This camera-arm FK path does not infer mimic/prismatic states')
        sample['T_base_tool']={'matrix':fk.fk(np.asarray(list(q.values()))).tolist()}
    return document


def solve(document):
    from ..vendor.xuchuang.camera.scripts.solve_extrinsic import solve_eye_in_hand,solve_eye_to_hand,matrix
    import numpy as np
    import cv2
    fit,hold=split([s for s in document['samples'] if s.get('use_for_calibration')],10,5)
    poses=[matrix(s['T_base_tool']['matrix'],'T_base_tool') for s in fit]
    rotations=np.asarray([cv2.Rodrigues(poses[0][:3,:3].T@t[:3,:3])[0].reshape(3) for t in poses[1:]])
    singular=np.linalg.svd(rotations,compute_uv=False)
    if len(singular)<2 or singular[1]<.01 or singular[1]/max(singular[0],1e-12)<.05:
        raise ValueError('Insufficient independent rotation axes')
    method=document.get('method','park')
    if document['mode']=='eye_in_hand':result=solve_eye_in_hand(fit,hold,method)
    elif document['mode']=='eye_to_hand':result=solve_eye_to_hand(fit,hold,method)
    else:raise ValueError('mode must be eye_in_hand or eye_to_hand')
    return {**result,'status':'candidate_holdout_scored','passed':None,
            'convention':'T_A_B maps B into A, translation metres; optical frame must be explicit'}
