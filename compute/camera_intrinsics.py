"""Fit OpenCV pinhole from explicit 3D/2D correspondences and independent holdouts."""
import numpy as np
from .prepare import array,split


def solve(document):
    import cv2
    fit,hold=split(document['views'],minimum_fit=8,minimum_holdout=3)
    width,height=document['resolution']
    if type(width) is not int or type(height) is not int or min(width,height)<=0:raise ValueError('Invalid resolution')
    def points(row):
        obj=array(row['object_points_m'],'object points');img=array(row['image_points_px'],'image points')
        if obj.ndim!=2 or obj.shape[1]!=3 or len(obj)<6 or img.shape!=(len(obj),2):raise ValueError('Invalid point arrays')
        return obj.astype(np.float32),img.astype(np.float32)
    obj,img=zip(*(points(r) for r in fit))
    rms,K,D,_,_=cv2.calibrateCamera(list(obj),list(img),(width,height),None,None)
    if not np.isfinite(K).all() or not np.isfinite(D).all() or K[0,0]<=0 or K[1,1]<=0:raise ValueError('Nonphysical camera fit')
    errors=[]
    for row in hold:
        o,x=points(row);ok,r,t=cv2.solvePnP(o,x,K,D)
        if not ok:raise ValueError('Holdout pose estimation failed')
        pred,_=cv2.projectPoints(o,r,t,K,D)
        errors.append(float(np.sqrt(np.mean(np.sum((pred.reshape(-1,2)-x)**2,axis=1)))))
    return {'status':'candidate_holdout_scored','model':'opencv_pinhole','resolution':[width,height],
            'K':K.tolist(),'distortion':D.reshape(-1).tolist(),'training_rmse_px':float(rms),
            'holdout_reprojection_rmse_px':errors,'depth_scale':None,'passed':None}
