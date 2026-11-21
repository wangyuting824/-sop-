"""Measured step-response statistics. Absolute PD/friction require separate SysID."""
import numpy as np
from .prepare import array


def solve(document):
    if document.get('time_basis')!='shared_clock_seconds':
        raise ValueError('Explicit synchronized command/feedback clock declaration required')
    rows=document['samples'];t=array([r['t_s'] for r in rows],'time')
    q=array([r['position_rad'] for r in rows],'position');u=array([r['target_rad'] for r in rows],'target')
    if len(t)<5 or not np.all(np.diff(t)>0):raise ValueError('Need at least five strictly increasing samples')
    if len(t)!=len(q) or len(t)!=len(u):raise ValueError('Array length mismatch')
    change=np.where(np.abs(np.diff(u))>1e-9)[0]+1
    if len(change)!=1:raise ValueError('This summary requires exactly one step; split multi-step trials first')
    start=int(change[0]);base=float(np.median(q[:start]));amplitude=float(u[-1]-base)
    if abs(amplitude)<1e-8:raise ValueError('Step amplitude too small')
    response=(q[start:]-base)/amplitude
    onset=np.where(response>.02)[0]
    settled=np.abs(q[start:]-u[-1])<=.02*abs(amplitude)
    settling=None
    for i in range(len(settled)):
        if settled[i:].all():settling=float(t[start+i]-t[start]);break
    dq=np.gradient(q,t)
    return {'status':'observed_step_metrics_only','position_rmse_rad':float(np.sqrt(np.mean((u-q)**2))),
            'observed_response_delay_s':float(t[start+onset[0]]-t[start]) if len(onset) else None,
            'sample_resolution_s':float(np.max(np.diff(t))),
            'settling_time_s':settling,'overshoot_pct':max(0.,float((response.max()-1)*100)),
            'observed_peak_velocity_rad_s':float(np.abs(dq).max()),
            'Kp':None,'Kd':None,'max_effort_nm':None,'rated_max_velocity':None,'friction':None,
            'armature':None,'separate_feedback_delay':None,
            'next_step':'Use compute.nvidia_sysid with actual USD and model-specific official RunSpec', 'passed':None}
