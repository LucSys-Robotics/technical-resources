# Copyright 2026 LucSys Robotics - integration and maintenance changes only.
#
# LucSys Robotics distributes this source under supplier authorization.
# LucSys Robotics is responsible for integration and maintenance only.
# This notice does not grant an additional open-source license.
# Third-party components retain their respective licenses and notices.

"""Test explicit model selection and independent customer configuration."""
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch

from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]


def load(name):
    """Load a launch module directly from the package source."""
    spec = importlib.util.spec_from_file_location(name, ROOT / 'launch' / (name + '.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(**overrides):
    """Construct only the arguments required by the entry under test."""
    value = LaunchContext()
    value.launch_configurations.update({
        'lucsys_param_yaml': str(ROOT / 'config/lucsys_param.yaml'),
        'robot_model_yaml': str(ROOT / 'config/robot_model.yaml'),
        'imu_mode_yaml': '', 'imu_mode': '', 'car_mode': '',
        'usart_port_name': '/dev/lucsys_controller', 'serial_baud_rate': '115200',
        'ultrasonic_package': '', 'node_name': 'lucsys_robot',
        'start_state_publisher': 'false', 'publish_static_tf': 'false',
        'model_package': '', 'model_name': '',
        **overrides,
    })
    return value


class LaunchContracts(unittest.TestCase):
    """Prevent implicit hardware/model assumptions and device substitution."""

    def base_params(self, **args):
        """Return the driver parameters without spawning processes."""
        module = load('base_serial')
        with patch.object(module, 'get_package_share_directory', return_value=str(ROOT)), \
                patch.object(module, 'Node', side_effect=lambda **kwargs: kwargs):
            return module.launch_nodes(context(**args))[0]['parameters'][0]

    def test_explicit_serial_port_wins(self):
        """Never replace an explicitly selected device."""
        self.assertEqual(self.base_params(usart_port_name='/dev/test_robot')['usart_port_name'],
                         '/dev/test_robot')

    def test_confirmed_mode_is_not_rewritten(self):
        """Keep explicit LucSys mode values unchanged."""
        for mode in ('LSR550A_mec_arm', 'LSR550A_4wd_arm', 'LSR200_4wd'):
            self.assertEqual(self.base_params(car_mode=mode)['car_mode'], mode)

    def test_default_device_is_not_substituted(self):
        """A missing device must not silently select another controller."""
        with patch.object(Path, 'exists', return_value=False):
            self.assertEqual(self.base_params()['usart_port_name'], '/dev/lucsys_controller')

    def test_disabled_model_has_no_dependencies(self):
        """Base operation does not require a model package or sensor transforms."""
        module = load('robot_mode_description')
        with patch.object(module, 'get_package_share_directory', side_effect=AssertionError):
            self.assertEqual(module.spawn_robot_nodes(context()), [])

    def test_model_selection_is_required(self):
        """Never guess a model from a chassis name."""
        module = load('robot_mode_description')
        with self.assertRaisesRegex(ValueError, 'model_package'):
            module.spawn_robot_nodes(context(start_state_publisher='true'))

    def test_model_path_cannot_escape_package(self):
        """Reject traversal in the URDF basename."""
        module = load('robot_mode_description')
        with self.assertRaisesRegex(ValueError, 'model_name'):
            module.spawn_robot_nodes(context(start_state_publisher='true',
                                             model_package='lucsys_lsr550a_mec_description',
                                             model_name='../other'))


if __name__ == '__main__':
    unittest.main()
