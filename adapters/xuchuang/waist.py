"""Six native channels; photo-module to output-joint mapping remains unverified."""
import math
from ..base import Adapter
from ...config import finite


class XuchuangWaist(Adapter):
    def decode(self,message):
        values=message.get('data'); offset=message.get('layout',{}).get('data_offset',0)
        if type(offset) is not int or offset<0 or not isinstance(values,list) or len(values)-offset!=36:
            raise ValueError('Expected 36 MotorState values after data_offset')
        values=[finite(x,'MotorState') for x in values[offset:]]
        names=('HEAD_YAW','HEAD_PITCH','WAIST_YAW','WAIST_PITCH_1','WAIST_PITCH_2','WAIST_PITCH_3')
        rows=[]
        for i,name in enumerate(names):
            q,dq,current,temperature,fault,enabled=values[6*i:6*i+6]
            rows.append({'joint':name,'index':i,'q':math.radians(q),'dq':math.radians(dq),
                         'q_raw':q,'dq_raw':dq,'position_unit':'rad','velocity_unit':'rad/s',
                         'current_mA':current,'temperature_reserved_raw':temperature,
                         'fault_raw':fault,'enabled_raw':enabled,'source_stamp':None})
        return rows
