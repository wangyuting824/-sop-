"""Offline decoder regression; never advertises live readiness to control."""
import json
from pathlib import Path
from .writer import Writer,atomic_json,health_path,manifest


def replay(adapter,session,source):
    writer=Writer(session,adapter.config);count=0;status='failed'
    try:
        for line in Path(source).read_text(encoding='utf-8-sig').splitlines():
            if not line.strip():continue
            record=json.loads(line);message=record.get('message',record)
            rows=adapter.decode(message)
            clocks={k:record.get(k) for k in ('received_unix_ns','received_monotonic_ns','subscriber_ros_time_ns')}
            writer.write(message,rows,{**clocks,'offline_replay':True});count+=1
        if not count:raise ValueError('Replay input is empty')
        status='offline_replay_complete'
        return {'status':status,'messages':count,'hardware_verified':False}
    finally:
        writer.event('finished',status=status,messages=count)
        atomic_json(health_path(writer.root,adapter.config),{'status':status,'offline_replay':True})
        writer.close();manifest(writer.root)
