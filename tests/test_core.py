"""Safety behavior tests, independent of ROS/Gazebo installations."""
import math
import unittest
from pai_ai_navigation.core import Command, process_scan, supervise, goal_features, feature_vector, expert


def scan(values=None, start=-math.pi, step=None):
    values = [5.0] * 360 if values is None else values
    return process_scan(values, start, 2 * math.pi / len(values) if step is None else step, .05, 8.0)


class ProcessingTests(unittest.TestCase):
    def test_front_uses_message_angles(self):
        values = [5.0] * 720
        values[0] = .6  # zero angle for this message
        result = scan(values, start=0)
        self.assertTrue(result.valid)
        self.assertEqual(result.front, .6)
        self.assertEqual(min(result.distances), .6)

    def test_infinite_no_returns_are_free(self):
        self.assertTrue(scan([math.inf] * 360).valid)
        self.assertEqual(scan([math.inf] * 360).minimum, 8)

    def test_nan_and_negative_infinity_are_invalid(self):
        for bad in [math.nan, -math.inf, -.1, 20]:
            values = [5.0] * 360; values[180] = bad
            self.assertFalse(scan(values).valid)
            self.assertEqual(supervise(Command(.2, .4), scan(values), 0, 0).command, Command())

    def test_partial_field_of_view_fails_closed(self):
        self.assertFalse(scan([5.0] * 20, start=-.1, step=.01).valid)

    def test_empty_and_bad_metadata_fail_closed(self):
        for values, increment in [([], .1), ([1], 0), ([1], math.nan)]:
            self.assertFalse(process_scan(values, 0, increment, .05, 8).valid)

    def test_feature_order_and_goal_wrap(self):
        distance, angle = goal_features(0, 0, math.pi, (1, 0))
        self.assertEqual(distance, 1)
        self.assertAlmostEqual(abs(angle), math.pi)
        self.assertEqual(len(feature_vector(scan(), distance, angle)), 14)


class ShieldTests(unittest.TestCase):
    def test_close_side_stops_rotation_too(self):
        values = [5.0] * 360; values[270] = .3
        result = supervise(Command(.2, .8), scan(values), 0, 0)
        self.assertEqual(result.command, Command())
        self.assertEqual(result.reason, 'emergency_stop')

    def test_approach_slows_forward_speed(self):
        values = [5.0] * 360; values[180] = .7
        result = supervise(Command(.2, .1), scan(values), 0, 0)
        self.assertGreater(result.command.linear, 0)
        self.assertLess(result.command.linear, .2)
        self.assertEqual(result.command.angular, .1)

    def test_dead_or_invalid_inputs_stop(self):
        for age in [.7, math.inf, math.nan, -.1]:
            self.assertEqual(supervise(Command(.2, .8), scan(), age, 0).command, Command())
            self.assertEqual(supervise(Command(.2, .8), scan(), 0, age).command, Command())
        self.assertEqual(supervise(Command(math.nan, .8), scan(), 0, 0).command, Command())

    def test_watchdog_is_retained_for_ai_only_comparison(self):
        self.assertEqual(supervise(Command(.2, .8), scan(), 1, 0, False).command, Command())

    def test_clear_command_and_limits(self):
        self.assertEqual(supervise(Command(.2, .1), scan(), 0, 0).command, Command(.2, .1))
        self.assertEqual(supervise(Command(10, -10), scan(), 0, 0).command, Command(.2, -.8))

    def test_expert_goal_and_emergency_stop(self):
        self.assertEqual(expert(scan(), .1, 0), Command())
        self.assertEqual(expert(scan([.3] * 360), 2, 0), Command())
        self.assertGreater(expert(scan(), 3, 0).linear, 0)


if __name__ == '__main__':
    unittest.main()