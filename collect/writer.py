"""One session, explicit source and config snapshots, append-only raw records."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib, json, os, socket, time, math
from .. import __version__
from ..config import public_config


def atomic_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+f'.{os.getpid()}.tmp')
    tmp.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    os.replace(tmp,path)


def raw_json(value):
    if isinstance(value,float) and not math.isfinite(value):return {'nonfinite_float':repr(value)}
    if isinstance(value,dict):return {key:raw_json(v) for key,v in value.items()}
    if isinstance(value,(list,tuple)):return [raw_json(v) for v in value]
    return value


def config_digest(config):
    return hashlib.sha256(json.dumps(public_config(config),sort_keys=True,ensure_ascii=False).encode()).hexdigest()


def session_root(path):
    root=Path(path).expanduser().resolve();root.mkdir(parents=True,exist_ok=True)
    for folder in ('config','raw/bags','raw/joints','raw/gripper','raw/cameras','derived',
                   'bundles/urdf','bundles/usd','bundles/cameras','reports'):
        (root/folder).mkdir(parents=True,exist_ok=True)
    meta=root/'session.json'
    try:
        with meta.open('x',encoding='utf-8') as f:
            json.dump({'schema_version':1,'version':__version__,'status':'created','created_utc':datetime.now(timezone.utc).isoformat(),
                       'host':socket.gethostname(),'session':root.name},f,ensure_ascii=False,indent=2)
    except FileExistsError:
        old=json.loads(meta.read_text(encoding='utf-8'))
        if old.get('host')!=socket.gethostname():raise ValueError('Session belongs to another host')
    return root


def device_name(config):
    return {'xuchuang_arms_ros2':'arms','xuchuang_waist_ros2':'waist','xuchuang_gripper_ros2':'gripper',
            'xuchuang_camera_driver_ros2':'cameras','ros2_joint_state':'joints'}[config['adapter']]

def device_dir(root, kind):
    raw=Path(root)/'raw'
    return raw/'joints'/kind if kind in ('arms','waist') else raw/kind


def snapshot(root,config):
    path=root/'config'/f'{device_name(config)}.json'
    if path.exists():
        if json.loads(path.read_text(encoding='utf-8'))!=public_config(config):
            raise ValueError('Configuration changed within the session; use a new session')
    else:atomic_json(path,public_config(config))


def health_path(root,config):return Path(root)/'health'/f'{device_name(config)}.json'


def assert_ready(root,config):
    path=health_path(root,config)
    if not path.is_file():raise RuntimeError('Start collection for this device and session first')
    h=json.loads(path.read_text(encoding='utf-8'))
    age=time.monotonic()-h.get('monotonic_s',-1e20)
    if h.get('host')!=socket.gethostname() or h.get('config_sha256')!=config_digest(config):
        raise RuntimeError('Collector host/configuration does not match')
    if h.get('status')!='ready' or not 0<=age<=2.5:
        raise RuntimeError('Collector is not ready or its heartbeat expired')


def manifest(root):
    root=Path(root)
    meta=root/'session.json'
    if meta.is_file():
        data=json.loads(meta.read_text(encoding='utf-8'))
        states={p.stem:json.loads(p.read_text(encoding='utf-8')).get('status')
                for p in sorted((root/'health').glob('*.json'))}
        data.update(device_status=states,
                    status=next(iter(states.values())) if len(set(states.values()))==1 else
                           ('mixed' if states else 'no_feedback_status'),
                    control_records=[p.relative_to(root).as_posix()
                                     for p in sorted((root/'raw').rglob('invocation_*.json'))])
        atomic_json(meta,data)
    rows=[]
    for p in sorted(Path(root).rglob('*')):
        if p.is_file() and p.name!='manifest.sha256' and '.tmp' not in p.name:
            h=hashlib.sha256()
            with p.open('rb') as f:
                for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
            rows.append(f'{h.hexdigest()}  {p.relative_to(root).as_posix()}')
    (Path(root)/'manifest.sha256').write_text('\n'.join(rows)+'\n',encoding='utf-8')


class Writer:
    def __init__(self,root,config):
        self.root=session_root(root);self.config=config;snapshot(self.root,config)
        folder=device_dir(self.root,device_name(config));folder.mkdir(parents=True,exist_ok=True)
        self.raw=(folder/'messages.jsonl').open('x',encoding='utf-8')
        try:self.rows=(folder/'states.jsonl').open('x',encoding='utf-8')
        except BaseException:self.raw.close();raise
        self.events=(folder/'events.jsonl').open('x',encoding='utf-8')

    def write(self,message,rows,clocks):
        self.raw.write(json.dumps({**clocks,'topic':self.config['feedback']['topic'],
                                   'type':self.config['feedback']['type'],'message':raw_json(message)},ensure_ascii=False,allow_nan=False)+'\n');self.raw.flush()
        for row in rows:
            self.rows.write(json.dumps({**clocks,**row},ensure_ascii=False,allow_nan=False)+'\n')
        self.rows.flush()

    def event(self,event,**fields):
        line=json.dumps({'event':event,'device':device_name(self.config),'received_unix_ns':time.time_ns(),**fields},ensure_ascii=False)+'\n'
        self.events.write(line);self.events.flush()
        with (self.root/'raw/events.jsonl').open('a',encoding='utf-8') as events:
            events.write(line)

    def close(self):
        for stream in (self.raw,self.rows,self.events):stream.close()
