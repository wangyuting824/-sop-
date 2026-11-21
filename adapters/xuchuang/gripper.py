"""Use the confirmed units/position/slot_id structure; preserve missing fields."""
from ..base import Adapter
from ...config import finite


class XuchuangGripper(Adapter):
    def decode(self,message):
        units=message.get('units')
        if not isinstance(units,list):raise ValueError('EndEffectorStatus.units missing')
        by_slot={}
        for unit in units:
            slot=unit.get('slot_id')
            if slot in by_slot:raise ValueError('Duplicate gripper slot')
            by_slot[slot]=unit
        rows=[]
        for role,slot in self.config['slots'].items():
            if slot not in by_slot:raise ValueError(f'Missing slot {slot}')
            unit=by_slot[slot];q=unit.get('position')
            if unit.get('category')!='gripper' or not isinstance(q,list) or len(q)!=1:
                raise ValueError('Expected scalar gripper position')
            p=finite(q[0],'opening_mm')
            if p<0:raise ValueError('Negative reported opening')
            for flag in ('connected','moving'):
                if type(unit.get(flag)) is not bool:raise ValueError(f'Missing {flag}')
            rows.append({'joint':role,'slot_id':slot,'q':p/1000,'q_raw':p,'position_unit':'m',
                         'velocity_raw':unit.get('velocity'),'effort_raw':unit.get('effort'),
                         'connected':unit['connected'],'moving':unit['moving'],
                         'estopped':message.get('estopped'),'source_stamp':message.get('header',{}).get('stamp')})
        return rows
