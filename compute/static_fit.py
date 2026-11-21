"""Fit joint-zero offsets from independent joint references, not encoder self-comparison."""
import numpy as np
from .prepare import array,split


def solve(document):
    if document.get('reference_source') in (None,'encoder','same_feedback'):
        raise ValueError('Independent joint reference source required; no zero from encoder-only data')
    names=document['joint_names']
    if not names or len(set(names))!=len(names):raise ValueError('Unique joint names required')
    fit,hold=split(document['samples'])
    def pairs(rows):
        q=array([r['q_measured_rad'] for r in rows],'measured',(len(rows),len(names)))
        ref=array([r['q_reference_rad'] for r in rows],'reference',(len(rows),len(names)))
        return q,ref
    q,ref=pairs(fit);offset=np.mean(ref-q,axis=0)
    hq,hr=pairs(hold);rmse=np.sqrt(np.mean((hq+offset-hr)**2,axis=0))
    return {'status':'candidate_independent_joint_reference_fit','joint_zero_offsets_rad':dict(zip(names,offset.tolist())),
            'holdout_rmse_rad':dict(zip(names,rmse.tolist())),
            'application':'q_model = q_measured + offset; runtime only, not also baked into joint origin',
            'tcp':None,'link_geometry':None,'passed':None,
            'limitations':'This implemented path fits joint references only; TCP/link geometry fitting is not implemented'}
