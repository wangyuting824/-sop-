from .prepare import array


def validate(document,config):
    """Evaluate explicitly named scalar metrics against configured maximum errors."""
    thresholds=config.get('thresholds',{})
    metrics=document.get('metrics')
    if not isinstance(metrics,dict) or not metrics:raise ValueError('Input must contain named scalar metrics')
    checks={}
    for key,value in metrics.items():
        scalar=array(value,key)
        if scalar.ndim or float(scalar)<0:raise ValueError('Error metrics must be nonnegative scalars')
        limit=thresholds.get(key)
        if limit is None:checks[key]={'value':float(scalar),'limit':None,'passed':None}
        else:
            limit=float(array(limit,key+' threshold'))
            if limit<0:raise ValueError('Negative threshold')
            checks[key]={'value':float(scalar),'limit':limit,'passed':float(scalar)<=limit}
    for key,limit in thresholds.items():
        if limit is not None and key not in checks:checks[key]={'value':None,'limit':limit,'passed':None}
    flags=[r['passed'] for r in checks.values()]
    passed=False if False in flags else None if None in flags else True
    runtime_verified=document.get('runtime_readback_verified') is True
    return {'status':'thresholds_evaluated','checks':checks,'metric_thresholds_passed':passed,
            'passed':passed if passed is not True or runtime_verified else None,
            'runtime_readback_verified':runtime_verified}
