import tempfile
import unittest
from pathlib import Path

import yaml

from calibration_platform.collect.writer import session_root
from calibration_platform.compute.export import export


class SessionLayoutTests(unittest.TestCase):
    def test_reviewed_bundle_in_session(self):
        with tempfile.TemporaryDirectory() as folder:
            session = session_root(Path(folder) / "example")
            model = Path(folder) / "input.urdf"
            model.write_text('<robot name="sample"><joint name="j"/></robot>', encoding="utf-8")
            request = {"validated": True, "urdf": {"source": str(model),
                       "absolute_joint_origins": {"j": {"xyz_m": [0, 0, 0], "rpy_rad": [0, 0, 0]}}}}
            dest = session / "bundles"
            export(request, dest)
            index = yaml.safe_load((dest / "calibration_bundle.yaml").read_text(encoding="utf-8"))
            self.assertEqual(index["files"], ["urdf/robot_calibrated.urdf"])
            self.assertTrue((dest / index["files"][0]).is_file())
            with self.assertRaises(FileExistsError):
                export(request, dest)
