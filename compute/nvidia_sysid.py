"""Invoke NVIDIA SysID 6.1 tools. This file does not implement a PD optimizer.

Requires an actual robot USD and a model-specific UI-exported RunSpec.
Only official stiffness/damping scale parameters are accepted.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import subprocess


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))


def validate_spec(spec, base):
    spec = copy.deepcopy(spec)
    if spec.get('schema_version') != 2:
        raise ValueError('Expected NVIDIA 6.1 RunSpec schema_version=2; export from the actual model.')
    rows = spec.get('parameters', {}).get('selected', [])
    if not rows:
        raise ValueError('No parameters selected. Select mapped joint stiffness and damping in SysID.')
    seen = set()
    for row in rows:
        key = (row['param_type'], row['dof_index'])
        if key[0] not in ('joint_stiffness', 'joint_damping') or key[1] < 0 or key in seen:
            raise ValueError('PD-only run requires unique, mapped joint_stiffness/joint_damping entries.')
        seen.add(key)
        lo, initial, hi = (float(row[k]) for k in ('min', 'initial', 'max'))
        if not all(math.isfinite(v) for v in (lo, initial, hi)) or not 0 <= lo <= initial <= hi or lo == hi:
            raise ValueError('Invalid PD scale bounds.')
        if initial != 1.0:
            raise ValueError('Use initial=1 for authored-gain baseline and candidate replay; these are scales.')
    residuals = spec.get('residuals', {})
    if any(float(residuals.get(k, 0)) != 0 for k in ('torque_weight', 'end_effector_pose_weight', 'contact_force_weight')):
        raise ValueError('This dataset supports position/velocity residuals only.')
    if not all(float(residuals.get(k, 0)) > 0 for k in ('position_weight', 'velocity_weight')):
        raise ValueError('Specify positive position and velocity weights explicitly.')
    telemetry = spec['telemetry']
    chunks = telemetry.get('chunks', [])
    if telemetry.get('source_type') != 'csv' or not chunks:
        raise ValueError('Use the exported CSV plus explicit trial chunks; do not replay across separators.')
    if {c['role'] for c in chunks} != {'train', 'validation'}:
        raise ValueError('Both train and validation chunks are required.')
    intervals = sorted((float(c['start']), float(c['end'])) for c in chunks)
    if any(not a < b for a, b in intervals) or any(b >= c for (_, b), (c, _) in zip(intervals, intervals[1:])):
        raise ValueError('Chunk windows must be nonoverlapping, positive-duration intervals.')
    for section, key in (('stage', 'input_path'), ('telemetry', 'source_path'), ('telemetry', 'mapping_path')):
        path = Path(spec[section][key]).expanduser()
        path = (base / path).resolve() if not path.is_absolute() else path.resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        spec[section][key] = str(path)
    mapping = read(spec['telemetry']['mapping_path'])
    if not mapping.get('command_columns') or not mapping.get('velocity_columns'):
        raise ValueError('Map actual commands and velocities; no feedback-as-command fallback.')
    if not spec.get('simulation', {}).get('robot_prim_path', '').startswith('/'):
        raise ValueError('Export the actual articulation prim path from Isaac Sim.')
    # Do not silently select a different engine, actuator model, DOF order or timestep.
    spec.setdefault('solver', {})['use_analytical_presolve'] = False
    spec.setdefault('outputs', {}).update(apply_parameters_to_usd=False, write_usd_provenance=False)
    return spec


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('phase', choices=('analyze', 'solve', 'apply', 'validate'))
    p.add_argument('--isaac-python', required=True, help='Installed Isaac Sim python.sh / python.bat')
    p.add_argument('--tools-dir', required=True, help='Installed matching SysID extension tools directory')
    p.add_argument('--run-spec', required=True, help='Model-specific exported official RunSpec JSON')
    p.add_argument('--output', required=True, help='Run artifacts directory')
    p.add_argument('--candidate-stage', help='New USD path; required for apply/validate')
    args = p.parse_args()
    src = Path(args.run_spec).resolve()
    spec = validate_spec(read(src), src.parent)
    launcher = Path(args.isaac_python).expanduser().resolve()
    if not launcher.is_file():
        raise FileNotFoundError(launcher)
    out = Path(args.output).expanduser().resolve()
    out.mkdir(parents=True, exist_ok=True)
    prepared = out / 'prepared_run_spec.json'
    # Keep the configuration identical across phases so saved theta cannot be
    # accidentally applied with reordered parameters or a different baseline.
    if prepared.exists() and read(prepared) != spec:
        raise ValueError('RunSpec changed: use a new output directory for this fit.')
    prepared.write_text(json.dumps(spec, ensure_ascii=False, indent=2), encoding='utf-8')
    if args.phase in ('apply', 'validate') and not args.candidate_stage:
        p.error('--candidate-stage is required for apply/validate')
    candidate = Path(args.candidate_stage).expanduser().resolve() if args.candidate_stage else None
    if candidate and candidate == Path(spec['stage']['input_path']):
        raise ValueError('Candidate USD must differ from original USD.')
    if args.phase == 'apply' and candidate.exists():
        raise FileExistsError(candidate)
    if args.phase == 'validate' and not candidate.is_file():
        raise FileNotFoundError(candidate)
    tools = Path(args.tools_dir).expanduser().resolve()
    name = {'analyze': 'analyze', 'solve': 'solve', 'apply': 'apply_theta_to_stage', 'validate': 'analyze'}[args.phase]
    entry = tools / ('headless_sysid_' + name + '.py')
    if not entry.is_file():
        raise FileNotFoundError(entry)
    cmd = [str(launcher), str(entry), '--run-spec', str(prepared)]
    if args.phase == 'analyze':
        cmd += ['--mode', 'all', '--result-json', str(out / 'baseline_sensitivity.json')]
    elif args.phase == 'solve':
        cmd += ['--result-json', str(out / 'solve_result.json'), '--checkpoint-json', str(out / 'checkpoint.json')]
    elif args.phase == 'apply':
        result = out / 'solve_result.json'
        theta = read(result).get('selected_status', {}).get('theta', [])
        if len(theta) != len(spec['parameters']['selected']) or not all(math.isfinite(x) for x in theta):
            raise ValueError('Official solver did not return a finite, matching selected_status.theta.')
        cmd += ['--result-json', str(result), '--output-stage', str(candidate), '--summary-json', str(out / 'candidate_writeback.json')]
    else:
        # initial=1 makes this evaluate the newly authored candidate gains,
        # without multiplying the fitted scales for a second time.
        cmd += ['--stage', str(candidate), '--mode', 'baseline', '--result-json', str(out / 'candidate_replay.json')]
    (out / (args.phase + '_command.json')).write_text(json.dumps(cmd, indent=2), encoding='utf-8')
    print('Official NVIDIA tool:', entry, flush=True)
    print('This run only operates on local simulation assets.', flush=True)
    subprocess.run(cmd, cwd=src.parent, check=True)


if __name__ == '__main__':
    main()
