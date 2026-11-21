"""Collect only configured state topics; no all-topics bag and no motion calls."""
import socket,time
from .writer import Writer,atomic_json,health_path,config_digest,manifest


def run(adapter,session,duration_s=1200):
    from ..adapters.ros2 import Subscription
    from ..config import finite
    duration_s=finite(duration_s,'duration_s')
    if duration_s<0:raise ValueError('duration must be nonnegative; 0 means until Ctrl+C')
    writer=Writer(session,adapter.config);count=0;last=None;first=None;status='failed';failure=None
    timeout=adapter.config.get('feedback_timeout_s',2.0)
    def health(state):
        atomic_json(health_path(writer.root,adapter.config),{'status':state,'monotonic_s':time.monotonic(),
                    'host':socket.gethostname(),'samples':count,'config_sha256':config_digest(adapter.config)})
    def receive(message,clocks):
        nonlocal count,last,first
        try:rows=adapter.decode(message)
        except (ValueError,KeyError,TypeError) as exc:
            writer.write(message,[],clocks);writer.event('invalid_message',error=str(exc));raise
        writer.write(message,rows,clocks);count+=1;last=time.monotonic()
        if first is None:first=last;print('Collection ready: valid feedback saved',flush=True)
    sub=None
    try:
        health('starting');start=time.monotonic();sub=Subscription(adapter,receive)
        while True:
            sub.spin();now=time.monotonic()
            if last is None and now-start>10:raise TimeoutError('No valid feedback; collector not ready')
            if last is not None and now-last>timeout:raise TimeoutError('Feedback stream expired')
            health('ready' if last is not None else 'starting')
            if first is not None and duration_s and now-first>=duration_s:status='recorded';break
    except KeyboardInterrupt:status='recorded' if count else 'interrupted'
    except BaseException as exc:failure=str(exc);raise
    finally:
        health(status)
        if sub is not None:sub.close()
        writer.event('finished',status=status,error=failure,samples=count);writer.close();manifest(writer.root)
    return {'status':status,'samples':count,'session':str(writer.root)}
