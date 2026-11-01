from pathlib import Path
import tempfile
import time
import unittest
from pai_ai_navigation.manual_control import ManualControl, read_json, request_start, request_stop, write_json


class ManualControlTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.directory = self.root / 'runtime' / ('capture_control_' + 'a' * 32)
        self.directory.mkdir(parents=True)
        self.control = ManualControl(self.directory, self.root)

    def tearDown(self):
        self.temporary.cleanup()

    def test_start_before_readiness_is_rejected(self):
        self.control.write_state('preparing', sensor_ready=False)
        with self.assertRaises(RuntimeError): request_start(self.directory)
        self.assertFalse(self.control.start_requested())

    def test_previous_session_token_cannot_start_a_new_wait(self):
        old = ManualControl(self.directory, self.root)
        old.write_state('ready', sensor_ready=True)
        request_start(self.directory)
        self.assertTrue(old.start_requested())
        self.assertFalse(self.control.start_requested())

    def test_start_requires_fresh_sensors_and_status(self):
        self.control.write_state('ready', sensor_ready=False)
        with self.assertRaises(RuntimeError): request_start(self.directory)
        self.control.write_state('ready', sensor_ready=True)
        state = read_json(self.directory/'status.json');state['updated_at'] = time.time()-3
        write_json(self.directory/'status.json',state)
        with self.assertRaises(RuntimeError): request_start(self.directory)

    def test_only_ready_state_accepts_the_start_request(self):
        self.control.write_state('ready',sensor_ready=True)
        request_start(self.directory)
        self.assertTrue(self.control.start_requested())
        self.control.write_state('running',sensor_ready=True)
        with self.assertRaises(RuntimeError): request_start(self.directory)

    def test_stop_is_scoped_to_the_owned_session(self):
        write_json(self.directory/'stop.json',{'session':'another-session','action':'stop'})
        self.assertFalse(self.control.stop_requested())
        request_stop(self.directory)
        self.assertTrue(self.control.stop_requested())

    def test_unrelated_directory_and_malformed_commands_are_rejected(self):
        with self.assertRaises(ValueError): ManualControl(self.root,self.root)
        (self.directory/'start.json').write_text('{broken',encoding='utf-8')
        self.assertFalse(self.control.start_requested())


if __name__ == '__main__':
    unittest.main()