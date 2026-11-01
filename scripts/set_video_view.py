"""Set only the navigation GUI camera; does not publish a robot command."""
from __future__ import annotations
import math
import os
import subprocess


def main() -> None:
    if os.environ.get('ROS_DOMAIN_ID') != '42' or os.environ.get('GZ_PARTITION') != 'pai_navigation':
        raise RuntimeError('Source scripts/env.sh first')
    # Look from (2,-4,5) toward (2,-0.3,0), with local +X pointing forward.
    yaw, pitch = math.pi / 2, math.atan2(5, 3.7)
    x = -math.sin(pitch / 2) * math.sin(yaw / 2)
    y = math.sin(pitch / 2) * math.cos(yaw / 2)
    z = math.cos(pitch / 2) * math.sin(yaw / 2)
    w = math.cos(pitch / 2) * math.cos(yaw / 2)
    request = (f'pose: {{position: {{x: 2, y: -4, z: 5}}, '
               f'orientation: {{x: {x}, y: {y}, z: {z}, w: {w}}}}}')
    result = subprocess.run(['gz', 'service', '-s', '/gui/move_to/pose',
                             '--reqtype', 'gz.msgs.GUICamera', '--reptype', 'gz.msgs.Boolean',
                             '--timeout', '4000', '--req', request],
                            capture_output=True, text=True, timeout=8, check=True)
    if 'true' not in result.stdout.lower():
        raise RuntimeError(f'Camera pose was not acknowledged. Start an episode with --gui. {result.stdout}')
    print('Video camera set; world/model/robot pose unchanged; no robot commands sent')


if __name__ == '__main__':
    main()