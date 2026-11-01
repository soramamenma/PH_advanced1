import os
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch


@unittest.skipUnless(sys.platform == 'linux', 'ROS process locks use Linux flock')
class ProcessLockTests(unittest.TestCase):
    def test_another_process_cannot_claim_the_active_shield(self):
        from pai_ai_navigation.locks import claim
        with tempfile.TemporaryDirectory(prefix='navigation_lock_test_') as directory:
            with patch.dict(os.environ, {'NAV_ROOT': directory}):
                lock = claim('shield')
                command = [sys.executable, '-c', "from pai_ai_navigation.locks import claim; f=claim('shield'); f.close()"]
                try:
                    blocked = subprocess.run(command, capture_output=True, text=True, timeout=10)
                    self.assertNotEqual(blocked.returncode, 0)
                    self.assertIn('Another navigation shield process is active', blocked.stderr)
                finally:
                    lock.close()
                released = subprocess.run(command, capture_output=True, text=True, timeout=10)
                self.assertEqual(released.returncode, 0, released.stderr)

    def test_role_guards_are_held_for_the_entire_operation(self):
        from pai_ai_navigation.locks import hold_roles
        with tempfile.TemporaryDirectory(prefix='navigation_lock_test_') as directory:
            with patch.dict(os.environ, {'NAV_ROOT': directory}):
                command = [sys.executable, '-c',
                           "from pai_ai_navigation.locks import claim; f=claim('episode'); f.close()"]
                with hold_roles():
                    blocked = subprocess.run(command, capture_output=True, text=True, timeout=10)
                    self.assertNotEqual(blocked.returncode, 0)
                    self.assertIn('Another navigation episode process is active', blocked.stderr)
                released = subprocess.run(command, capture_output=True, text=True, timeout=10)
                self.assertEqual(released.returncode, 0, released.stderr)


if __name__ == '__main__':
    unittest.main()