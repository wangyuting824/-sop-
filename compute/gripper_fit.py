"""Physical opening = scale * commanded opening + offset; no full-stroke claim."""
import numpy as np
from .prepare import array,split


def solve(document):
    groups={}
    for row in document['points']:groups.setdefault(row['slot'],[]).append(row)
    if not groups:raise ValueError('No gripper measurements')
    result={}
    for slot,rows in groups.items():
        fit,hold=split(rows)
        x=array([r['command_mm'] for r in fit],'command');y=array([r['measured_mm'] for r in fit],'measurement')
        if np.any(x<0) or np.any(y<0) or len(np.unique(x))<3:raise ValueError('Need three distinct nonnegative measurement points')
        design=np.column_stack([x,np.ones(len(x))]);scale,offset=np.linalg.lstsq(design,y,rcond=None)[0]
        if scale<=0:raise ValueError('Opening map has nonpositive scale')
        hx=array([r['command_mm'] for r in hold],'holdout command');hy=array([r['measured_mm'] for r in hold],'holdout measurement')
        if np.any(hx<x.min()) or np.any(hx>x.max()) or np.any(hy<0):raise ValueError('Holdout must lie in measured command domain')
        result[slot]={'scale':float(scale),'offset_mm':float(offset),'command_domain_mm':[float(x.min()),float(x.max())],
                      'holdout_rmse_mm':float(np.sqrt(np.mean((scale*hx+offset-hy)**2))),
                      'physical_zero_mm':None,'full_stroke_mm':None,'contact_threshold':None}
    return {'status':'candidate_holdout_scored','gripper':result,'passed':None}
