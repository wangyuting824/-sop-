"""Standard ROS 2 JointState, revolute-only until prismatic units are configured."""
import math
from .base import Adapter
from ..config import finite


class JointState(Adapter):
    def decode(self,msg):
        names=msg.get('name');q=msg.get('position');dq=msg.get('velocity',[]);effort=msg.get('effort',[])
        if not isinstance(names,list) or not names or any(not isinstance(n,str) or not n for n in names):
            raise ValueError('Joint names missing')
        if len(set(names))!=len(names) or not isinstance(q,list) or len(q)!=len(names):
            raise ValueError('JointState names/positions mismatch')
        if any(not isinstance(v,list) or len(v) not in (0,len(names)) for v in (dq,effort)):
            raise ValueError('Optional arrays must be empty or match names')
        pf=math.pi/180 if self.config['feedback']['position_unit']=='deg' else 1
        vf=math.pi/180 if self.config['feedback']['velocity_unit']=='deg/s' else 1
        return [{'joint':n,'q':finite(q[i],n)*pf,'dq':finite(dq[i],n)*vf if dq else None,
                 'effort_raw':finite(effort[i],n) if effort else None,'position_unit':'rad','velocity_unit':'rad/s',
                 'source_stamp':msg.get('header',{}).get('stamp')} for i,n in enumerate(names)]
