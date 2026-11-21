#!/usr/bin/env python3
"""Create paired v3 adapters with one shared session-only physics configuration."""

from pathlib import Path


BUNDLE = Path(__file__).resolve().parent
ROOT = BUNDLE.parents[1]
CALL = '''    # The same session-only physics configuration is used by tune and replay.
    bundle_path = str(Path(__file__).resolve().parent)
    if bundle_path not in sys.path:
        sys.path.insert(0, bundle_path)
    from physics_session_config import apply_physics_session_config
    physics_config = apply_physics_session_config(
        context.get_stage(), stage_path=spec.stage.input_path, output=OUT, tag=ARGS.tag,
    )
'''


def main() -> None:
    tune = (BUNDLE / "run_official_sysid_tune_active_v2.py").read_text(encoding="utf-8")
    anchor = "    timeline = omni.timeline.get_timeline_interface()\n    timeline.play()"
    if tune.count(anchor) != 1:
        raise ValueError("Unexpected tune timeline anchor")
    tune = tune.replace(anchor, CALL + anchor)
    tune = tune.replace('            "physics_backend": prepared.physics_backend,',
                        '            "physics_backend": prepared.physics_backend,\n'
                        '            "physics_configuration": physics_config,')
    (BUNDLE / "run_official_sysid_tune_active_v3.py").write_text(tune, encoding="utf-8", newline="\n")

    replay_source = ROOT / "outputs/paired_pd_comparison_20261114/scripts/run_official_sysid_validate_replay_only.py"
    replay = replay_source.read_text(encoding="utf-8")
    if replay.count(anchor) != 1:
        raise ValueError("Unexpected replay timeline anchor")
    replay = replay.replace(anchor, CALL + anchor)
    replay = replay.replace("    return parser.parse_args()", '''    parser.add_argument(
        "--skip-check-report", action="store_true",
        help="Skip only the startup diagnostic report using the existing replay-only hook",
    )
    return parser.parse_args()''')
    old_hook = '''    import isaacsim.robot_setup.sysid.check_report as check_report_module
    class ReplayOnlyCheckReport:
        def summary_lines(self):
            return []
    check_report_module.build_sysid_check_report = lambda *args, **kwargs: ReplayOnlyCheckReport()
'''
    if replay.count(old_hook) != 1:
        raise ValueError("Unexpected replay diagnostic hook")
    new_hook = '''    if ARGS.skip_check_report:
        import isaacsim.robot_setup.sysid.check_report as check_report_module

        class ReplayOnlyCheckReport:
            def summary_lines(self):
                return []

        check_report_module.build_sysid_check_report = lambda *args, **kwargs: ReplayOnlyCheckReport()
        print("Diagnostic startup report skipped by explicit option", flush=True)
'''
    replay = replay.replace(old_hook, new_hook)
    replay = replay.replace('                "pre_solve_check_report": "skipped for fixed-parameter replay; no effect on simulator rollout",',
                            '                "diagnostic_check_report_skipped": ARGS.skip_check_report,\n'
                            '                "physics_configuration": physics_config,')
    replay = replay.replace('                    "status": "running",',
                            '                    "status": "running",\n'
                            '                    "physics_configuration": physics_config,')
    (BUNDLE / "run_official_sysid_validate_replay_v3.py").write_text(replay, encoding="utf-8", newline="\n")
    print("Prepared tune_v3 and replay_v3; both use physics_session_config.py")


if __name__ == "__main__":
    main()
