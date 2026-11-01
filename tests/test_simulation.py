import json
from pathlib import Path
import signal
import tempfile
import unittest
from unittest.mock import patch

from pai_ai_navigation import simulation


class FakeProcess:
    pid = 12345

    def __init__(self):
        self.waited = []

    def wait(self, timeout=None):
        self.waited.append(timeout)
        return 0

    def poll(self):
        return None


class SimulationTests(unittest.TestCase):
    def test_process_record_is_written_atomically(self):
        with tempfile.TemporaryDirectory(prefix='navigation_simulation_test_') as directory:
            runtime = Path(directory)
            log_path = runtime / 'simulation.log'
            process = FakeProcess()

            simulation._write_process_record(runtime, process, log_path)

            record = json.loads((runtime / 'simulation_process.json').read_text(encoding='utf-8'))
            self.assertEqual(record, {'pid': process.pid, 'log': log_path.name})
            self.assertEqual(list(runtime.glob('.simulation_process.*.tmp')), [])

    def test_failed_record_write_preserves_old_record_and_cleans_temp_file(self):
        with tempfile.TemporaryDirectory(prefix='navigation_simulation_test_') as directory:
            runtime = Path(directory)
            record = runtime / 'simulation_process.json'
            record.write_text('{"pid": 7}', encoding='utf-8')

            with patch.object(simulation.os, 'replace', side_effect=OSError('disk error')):
                with self.assertRaisesRegex(OSError, 'disk error'):
                    simulation._write_process_record(runtime, FakeProcess(), runtime / 'simulation.log')

            self.assertEqual(record.read_text(encoding='utf-8'), '{"pid": 7}')
            self.assertEqual(list(runtime.glob('.simulation_process.*.tmp')), [])

    def test_start_stops_launch_if_recording_its_pid_fails(self):
        with tempfile.TemporaryDirectory(prefix='navigation_simulation_test_') as directory:
            root = Path(directory)
            (root / 'runtime').mkdir()
            process = FakeProcess()
            with (patch.object(simulation.subprocess, 'Popen', return_value=process),
                  patch.object(simulation, '_write_process_record', side_effect=OSError('disk error')),
                  patch.object(simulation.os, 'killpg') as killpg,
                  patch.object(simulation.time, 'sleep') as sleep):
                with self.assertRaisesRegex(OSError, 'disk error'):
                    simulation.start(root)

            killpg.assert_called_once_with(process.pid, signal.SIGINT)
            self.assertEqual(process.waited, [15])
            sleep.assert_not_called()


if __name__ == '__main__':
    unittest.main()
