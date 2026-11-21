from pathlib import Path
import hashlib,json
import numpy as np


def read(path):return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def digest(path):
    h=hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def array(value,label,shape=None):
    a=np.asarray(value,dtype=float)
    if not np.isfinite(a).all() or shape is not None and a.shape!=shape:
        raise ValueError(f'{label}: invalid shape or nonfinite values')
    return a


def transform(value):
    t=array(value,'transform',(4,4));r=t[:3,:3]
    if not np.allclose(t[3],[0,0,0,1],atol=1e-9) or not np.allclose(r.T@r,np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(r),1,atol=1e-6):
        raise ValueError('Invalid rigid transform')
    return t


def split(rows,minimum_fit=3,minimum_holdout=1):
    if any(r.get('split') not in ('fit','holdout') for r in rows):
        raise ValueError('Every sample requires explicit fit/holdout assignment')
    fit=[r for r in rows if r['split']=='fit'];hold=[r for r in rows if r['split']=='holdout']
    if len(fit)<minimum_fit or len(hold)<minimum_holdout:raise ValueError('Insufficient explicit fit/holdout data')
    return fit,hold


def write_result(path,result,source):
    from ..collect.writer import atomic_json
    path=Path(path)
    if path.exists():raise FileExistsError('Output exists; do not overwrite an earlier result')
    result={**result,'source_sha256':digest(source),'source_file':Path(source).name}
    atomic_json(path,result);return result
