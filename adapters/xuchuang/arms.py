"""Xuchuang ArmStateArray, using the field script's actual names and units."""
import math
from ..base import Adapter
from ...config import finite


def vector(values, label, size=7):
    if not isinstance(values, (list,tuple)) or len(values)!=size:
        raise ValueError(f'{label}: expected {size} elements')
    return [finite(x,label) for x in values]


class XuchuangArms(Adapter):
    def decode(self, message):
        arms=message.get('arms')
        if not isinstance(arms,list): raise ValueError('ArmStateArray.arms is missing')
        ids=[arm.get('arm_id') for arm in arms]
        if any(type(i) is not int for i in ids) or len(set(ids))!=len(ids):
            raise ValueError('Arm IDs must be unique integers')
        roles={v['id']:k for k,v in self.config['arms'].items()}
        if not set(roles).issubset(ids): raise ValueError('Configured left/right arm feedback missing')
        rows=[]
        pf=math.pi/180 if self.config['feedback']['position_unit']=='deg' else 1
        vf=math.pi/180 if self.config['feedback']['velocity_unit']=='deg/s' else 1
        for arm in arms:
            if arm['arm_id'] not in roles: continue
            role=roles[arm['arm_id']]
            q=vector(arm['joint_pos_act'],'joint_pos_act')
            dq=arm.get('joint_vel_act')
            dq=vector(dq,'joint_vel_act') if dq is not None and dq!=[] else [None]*7
            for i,p in enumerate(q):
                key=f'{role}:{i}'
                rows.append({'joint':key,'urdf_joint':(self.config.get('joint_map') or {}).get(key),
                             'arm_id':arm['arm_id'],'index':i,'q':p*pf,
                             'dq':None if dq[i] is None else dq[i]*vf,
                             'q_raw':p,'dq_raw':dq[i],'position_unit':'rad','velocity_unit':'rad/s',
                             'source_stamp':message.get('stamp'),'frame_seq':arm.get('frame_seq'),
                             'traj_plan_state':arm.get('traj_plan_state')})
        return rows
