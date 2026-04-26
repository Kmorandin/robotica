#!/usr/bin/env python3

import math
import numpy as np
import rclpy

from phi_p3dx_navigation.main import NavigationNode


class TesteLidar(NavigationNode):
    def __init__(self):
        super().__init__(node_name='teste_lidar', timer_period=0.5)

    def _control_loop(self):
        if not self.has_laser_data():
            return

        ranges = np.array(self.laser_ranges)

        linhas = []
        for i, dist in enumerate(ranges):
            angle_rad = self.laser_angle_min + i * self.laser_angle_increment
            angle_deg = math.degrees(angle_rad)

            if np.isfinite(dist):
                linhas.append(f'[{i:03d}] {angle_deg:7.2f}° -> {dist:.3f} m')
            else:
                linhas.append(f'[{i:03d}] {angle_deg:7.2f}° -> inf')

        self.get_logger().info('\n' + '\n'.join(linhas))


def main(args=None):
    rclpy.init(args=args)
    node = TesteLidar()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()