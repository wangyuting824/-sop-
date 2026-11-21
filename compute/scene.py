"""Estimate one fixed transform from repeated independent observations."""
import numpy as np
from .prepare import transform,split


def solve(document):
    if not document.get('parent_frame') or not document.get('child_frame') or document['parent_frame']==document['child_frame']:
        raise ValueError('Distinct parent/child frames required')
    fit,hold=split(document['observations'])
    ts=[transform(r['matrix']) for r in fit]
    u,_,vt=np.linalg.svd(sum(t[:3,:3] for t in ts))
    r=u@np.diag([1,1,np.linalg.det(u@vt)])@vt
    t=np.eye(4);t[:3,:3]=r;t[:3,3]=np.mean([a[:3,3] for a in ts],axis=0)
    holdouts=[transform(row['matrix']) for row in hold]
    translation=[float(np.linalg.norm(t[:3,3]-a[:3,3])) for a in holdouts]
    rotation=[float(np.degrees(np.arccos(np.clip((np.trace(t[:3,:3].T@a[:3,:3])-1)/2,-1,1)))) for a in holdouts]
    return {'status':'candidate_holdout_scored','parent_frame':document['parent_frame'],'child_frame':document['child_frame'],
            'matrix':t.tolist(),'holdout_translation_errors_m':translation,'holdout_rotation_errors_deg':rotation,'passed':None}
