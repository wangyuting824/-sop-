"""Explicit validated writeback; never silently apply candidate fit fields."""
from pathlib import Path
import yaml
import xml.etree.ElementTree as ET
from .prepare import array,digest


def export(document,destination):
    dest=Path(destination)
    session_layout=dest.name=='bundles'
    if dest.exists() and not (session_layout and dest.is_dir() and
                              all(p.is_dir() and p.parent==dest and p.name in {'urdf','usd','cameras'}
                                  for p in dest.rglob('*'))):
        raise FileExistsError(dest)
    if document.get('validated') is not True:raise ValueError('Only independently validated, reviewed writeback bundles are accepted')
    urdf=document.get('urdf');usd=document.get('usd')
    if not urdf and not usd:raise ValueError('No explicit supported writeback changes')
    # Validate all available source paths and changes before creating output.
    tree=None;stage=None;source_stage=None
    if urdf:
        source=Path(urdf['source']).expanduser().resolve()
        tree=ET.parse(source)
        joints={j.attrib['name']:j for j in tree.getroot().findall('joint')}
        for name,origin in urdf['absolute_joint_origins'].items():
            if name not in joints:raise ValueError(f'URDF joint not found: {name}')
            xyz=array(origin['xyz_m'],'xyz',(3,));rpy=array(origin['rpy_rad'],'rpy',(3,))
            el=joints[name].find('origin')
            if el is None:el=ET.SubElement(joints[name],'origin')
            el.set('xyz',' '.join(map(str,xyz)));el.set('rpy',' '.join(map(str,rpy)))
    if usd:
        from pxr import Usd
        source_stage=Path(usd['source']).expanduser().resolve()
        stage=Usd.Stage.Open(str(source_stage))
        if not stage:raise ValueError('Cannot open USD stage')
        allowed={'physics:mass','physxJoint:armature','drive:angular:physics:stiffness','drive:angular:physics:damping',
                 'drive:angular:physics:maxForce','drive:linear:physics:stiffness','drive:linear:physics:damping','drive:linear:physics:maxForce'}
        for change in usd['attributes']:
            if change['name'] not in allowed or change.get('units')!='usd_native':
                raise ValueError('Only explicitly converted usd_native attributes from whitelist are supported')
            attr=stage.GetPrimAtPath(change['prim_path']).GetAttribute(change['name'])
            if not attr:raise ValueError('Requested prim/schema attribute does not exist')
            array(change['value'],'USD attribute')
    dest.mkdir(parents=True,exist_ok=session_layout)
    urdf_dir=dest/'urdf' if session_layout else dest
    usd_dir=dest/'usd' if session_layout else dest
    sources={}
    if tree is not None:
        urdf_dir.mkdir(exist_ok=True)
        target=urdf_dir/'robot_calibrated.urdf';tree.write(target,encoding='utf-8',xml_declaration=True)
        sources['urdf_sha256']=digest(urdf['source'])
    if stage is not None:
        from pxr import Sdf,Usd
        usd_dir.mkdir(exist_ok=True)
        layer=Sdf.Layer.CreateNew(str(usd_dir/'robot_physics.usda'))
        layer.subLayerPaths=[str(source_stage)]
        result=Usd.Stage.Open(layer)
        for change in usd['attributes']:
            attr=result.GetPrimAtPath(change['prim_path']).GetAttribute(change['name'])
            if not attr.Set(change['value']):raise ValueError('USD Set rejected value')
        layer.Save();sources['usd_sha256']=digest(source_stage)
    if session_layout:
        files=sorted(p.relative_to(dest).as_posix() for p in dest.rglob('*') if p.is_file())
        with (dest/'calibration_bundle.yaml').open('x',encoding='utf-8') as f:
            yaml.safe_dump({'schema_version':1,'files':files,'sources':sources,'runtime_verified':False,
                            'applied_parameters':document},f,allow_unicode=True,sort_keys=False)
    return {'status':'explicit_writeback_created_not_runtime_validated','output':str(dest),'sources':sources,
            'limitations':'Mesh references and USD base sublayer must remain resolvable. Camera writeback and SI conversion are not inferred.'}
