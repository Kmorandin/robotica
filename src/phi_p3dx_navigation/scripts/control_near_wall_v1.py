#!/usr/bin/env python3

import math
import rclpy
import numpy as np

from phi_p3dx_navigation.main import NavigationNode


class WallFollowerPIDV2(NavigationNode):

    def __init__(self):
        super().__init__(node_name='wall_follower_pid_v2', timer_period=0.05)

        self.wall_side = 'right'
        self.target_distance = 0.4
        self.front_trigger_distance = 0.35
        self.front_turn_distance = 0.25

        self.search_linear_speed = 0.15
        self.follow_linear_speed = 0.10
        self.max_angular_speed = 0.6

        self.kp = 1.2
        self.ki = 0.0
        self.kd = 0.12

        self.dt = 0.05
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.integral_limit = 1.0

        self.state = 'SEARCH_WALL'
        self.turn_target_theta = None
        self.turn_speed = 0.6
        self.turn_tolerance = math.radians(4.0)

        self.last_print_time = self.get_clock().now()
        self.print_interval = 2.0

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, v, vmin, vmax):
        return max(vmin, min(vmax, v))

    def reset_pid(self):
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0

    def get_side_distance(self):
        """
        Convenção corrigida:
        - início do array = -90°
        - fim do array    = +90°
        """
        if not self.has_laser_data():
            return float('inf')

        k = 5
        n = len(self.laser_ranges)

        if n == 0:
            return float('inf')

        if self.wall_side == 'right':
            region = self.laser_ranges[0:k]
        else:
            region = self.laser_ranges[n - k:n]

        finite = region[np.isfinite(region)]

        if len(finite) == 0:
            return float('inf')

        return float(np.median(finite))

    def estimate_wall_distance(self):
        d = self.get_side_distance()
        return d if math.isfinite(d) else float('inf')

    def estimate_wall_error(self):
        d = self.estimate_wall_distance()
        if not math.isfinite(d):
            return float('nan')
        return self.target_distance - d

    def start_turn(self, angle_offset, next_state):
        self.turn_target_theta = self.normalize_angle(self.theta + angle_offset)
        self.state = next_state
        self.reset_pid()

    def control_turn(self, next_state):
        err = self.normalize_angle(self.turn_target_theta - self.theta)

        if abs(err) < self.turn_tolerance:
            self.stop()
            self.turn_target_theta = None
            self.state = next_state
            return

        w = self.clamp(2.0 * err, -self.turn_speed, self.turn_speed)
        self.publish_velocity(0.0, w)

    def control_search_wall(self):
        front = self.get_front_distance(15.0)

        if front <= self.front_trigger_distance:
            self.stop()
            if self.wall_side == 'right':
                self.start_turn(math.pi / 2.0, 'ALIGN_TO_WALL')
            else:
                self.start_turn(-math.pi / 2.0, 'ALIGN_TO_WALL')
            return

        self.publish_velocity(self.search_linear_speed, 0.0)

    def control_follow_wall(self):
        front = self.get_front_distance(15.0)

        if front <= self.front_turn_distance:
            self.stop()
            if self.wall_side == 'right':
                self.start_turn(math.pi / 2.0, 'TURN_CORNER')
            else:
                self.start_turn(-math.pi / 2.0, 'TURN_CORNER')
            return

        side_dist = self.get_side_distance()
        error = self.estimate_wall_error()

        if math.isnan(error):
            w = -0.10 if self.wall_side == 'right' else 0.10
            self.publish_velocity(0.05, w)
            self.print_debug(side_dist, float('nan'), 0.05, w)
            return

        self.pid_integral += error * self.dt
        self.pid_integral = self.clamp(self.pid_integral, -1.0, 1.0)

        derivative = (error - self.pid_prev_error) / self.dt
        self.pid_prev_error = error

        u = self.kp * error + self.kd * derivative

        w = u if self.wall_side == 'right' else -u
        w = self.clamp(w, -self.max_angular_speed, self.max_angular_speed)

        v = self.follow_linear_speed * max(0.45, 1 - abs(w) / self.max_angular_speed)

        self.print_debug(side_dist, error, v, w)
        self.publish_velocity(v, w)

    def print_debug(self, side_dist, error, v, w):
        now = self.get_clock().now()
        elapsed = (now - self.last_print_time).nanoseconds * 1e-9

        if elapsed >= self.print_interval:
            sd = "inf" if not math.isfinite(side_dist) else f"{side_dist:.3f}"
            er = "nan" if math.isnan(error) else f"{error:.3f}"

            self.get_logger().info(
                f"Lateral={sd} m | erro={er} | v={v:.3f} | w={w:.3f}"
            )

            self.last_print_time = now

    def _control_loop(self):
        if not self.has_laser_data():
            self.stop()
            return

        if self.state == 'SEARCH_WALL':
            self.control_search_wall()

        elif self.state == 'ALIGN_TO_WALL':
            self.control_turn('FOLLOW_WALL')

        elif self.state == 'FOLLOW_WALL':
            self.control_follow_wall()

        elif self.state == 'TURN_CORNER':
            self.control_turn('FOLLOW_WALL')

        else:
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerPIDV2()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()