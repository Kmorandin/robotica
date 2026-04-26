#!/usr/bin/env python3

import math
import rclpy
import numpy as np

from phi_p3dx_navigation.main import NavigationNode


class VFHWindowNavigator(NavigationNode):
    """
    Navegação por objetivo com desvio local por janelas livres.

    Estratégia:
      - tenta ir na direção do objetivo final
      - se a direção do objetivo estiver livre até a distância X, segue nela
      - se estiver bloqueada, escolhe a janela livre mais próxima do azimute do objetivo
      - projeta um ponto intermediário a distância X no centro da janela
      - navega até esse ponto
      - só então recalcula
      - repete até chegar ao objetivo final
    """

    def __init__(self):
        super().__init__(node_name='vfh_window_navigator', timer_period=0.05)

        # =========================================================
        # Parâmetros principais
        # =========================================================
        self.goal_tolerance = 0.20

        # Distância X de avaliação/projeção local
        self.lookahead_distance = 1.0
        self.local_point_tolerance = 0.12

        # Controle
        self.max_linear_speed = 0.22
        self.min_linear_speed = 0.06
        self.max_angular_speed = 0.9
        self.k_ang = 1.8

        # Histograma
        self.num_sectors = 36
        self.min_free_block_size = 2
        self.obstacle_enlarge_sectors = 1

        # Estado
        self.mode = 'GOAL'  # GOAL ou LOCAL_POINT
        self.local_target_point = None
        self.local_target_dir = None
        self.local_target_window = None

        # Debug
        self.debug_print_interval = 2.0
        self.last_debug_time = self.get_clock().now()

        self.get_logger().info(
            f'VFHWindowNavigator iniciado | X={self.lookahead_distance:.2f} m'
        )

    # =========================================================
    # Utilidades
    # =========================================================

    def normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def clamp(self, value: float, vmin: float, vmax: float) -> float:
        return max(vmin, min(vmax, value))

    def goal_angle_robot_frame(self) -> float:
        return self.normalize_angle(self.angle_to_goal())

    def world_point_from_robot_polar(self, distance: float, angle_robot_frame: float):
        """
        Constrói um ponto no mundo a partir de uma direção no referencial do robô.
        """
        world_angle = self.theta + angle_robot_frame
        px = self.x + distance * math.cos(world_angle)
        py = self.y + distance * math.sin(world_angle)
        return (px, py)

    def distance_to_point(self, point) -> float:
        if point is None:
            return float('inf')
        return math.hypot(point[0] - self.x, point[1] - self.y)

    def angle_to_point(self, point) -> float:
        if point is None:
            return 0.0
        desired = math.atan2(point[1] - self.y, point[0] - self.x)
        err = desired - self.theta
        return math.atan2(math.sin(err), math.cos(err))

    def clear_local_target(self):
        self.local_target_point = None
        self.local_target_dir = None
        self.local_target_window = None

    # =========================================================
    # Histograma VFH simplificado
    # =========================================================

    def build_sector_histogram(self):
        """
        Constrói histograma binário:
          0 = setor livre até distância X
          1 = setor bloqueado antes de distância X
        """
        if not self.has_laser_data():
            return np.array([]), np.array([])

        angle_min = self.laser_angle_min
        angle_max = self.laser_angle_max

        sector_edges = np.linspace(angle_min, angle_max, self.num_sectors + 1)
        sector_centers = 0.5 * (sector_edges[:-1] + sector_edges[1:])
        histogram = np.zeros(self.num_sectors, dtype=int)

        for i in range(self.num_sectors):
            a0 = sector_edges[i]
            a1 = sector_edges[i + 1]

            idx0 = int(max(0, math.floor((a0 - angle_min) / self.laser_angle_increment)))
            idx1 = int(min(
                len(self.laser_ranges) - 1,
                math.ceil((a1 - angle_min) / self.laser_angle_increment)
            ))

            if idx1 < idx0:
                histogram[i] = 1
                continue

            region = self.laser_ranges[idx0:idx1 + 1]
            finite = region[np.isfinite(region)]

            # sem leitura válida -> bloqueado por segurança
            if len(finite) == 0:
                histogram[i] = 1
                continue

            min_dist = float(np.min(finite))
            histogram[i] = 1 if min_dist < self.lookahead_distance else 0

        # Alarga obstáculos para dar folga lateral
        if self.obstacle_enlarge_sectors > 0:
            expanded = histogram.copy()
            occupied_idxs = np.where(histogram == 1)[0]

            for idx in occupied_idxs:
                start = max(0, idx - self.obstacle_enlarge_sectors)
                end = min(len(histogram) - 1, idx + self.obstacle_enlarge_sectors)
                expanded[start:end + 1] = 1

            histogram = expanded

        return sector_centers, histogram

    def find_free_intervals(self, histogram: np.ndarray):
        """
        Identifica intervalos contíguos de setores livres.
        """
        free_intervals = []
        start = None

        for i, value in enumerate(histogram):
            if value == 0 and start is None:
                start = i
            elif value == 1 and start is not None:
                end = i - 1
                if end - start + 1 >= self.min_free_block_size:
                    free_intervals.append((start, end))
                start = None

        if start is not None:
            end = len(histogram) - 1
            if end - start + 1 >= self.min_free_block_size:
                free_intervals.append((start, end))

        return free_intervals

    # =========================================================
    # Relação direção <-> setores
    # =========================================================

    def sector_index_of_angle(self, sector_centers: np.ndarray, angle: float):
        if len(sector_centers) == 0:
            return None
        return min(
            range(len(sector_centers)),
            key=lambda i: abs(self.normalize_angle(sector_centers[i] - angle))
        )

    def is_direction_free(self, sector_centers: np.ndarray, histogram: np.ndarray, direction: float) -> bool:
        idx = self.sector_index_of_angle(sector_centers, direction)
        if idx is None:
            return False
        return histogram[idx] == 0

    def interval_center_angle(self, sector_centers: np.ndarray, i_start: int, i_end: int) -> float:
        return float(0.5 * (sector_centers[i_start] + sector_centers[i_end]))

    def interval_distance_to_goal(self, sector_centers: np.ndarray, i_start: int, i_end: int, goal_dir: float) -> float:
        """
        Distância angular entre uma janela e o azimute do objetivo.
        """
        left_angle = min(sector_centers[i_start], sector_centers[i_end])
        right_angle = max(sector_centers[i_start], sector_centers[i_end])

        if left_angle <= goal_dir <= right_angle:
            return 0.0

        return min(
            abs(self.normalize_angle(goal_dir - left_angle)),
            abs(self.normalize_angle(goal_dir - right_angle))
        )

    def select_best_window(self, sector_centers: np.ndarray, free_intervals, goal_dir: float):
        """
        Escolhe a janela livre mais próxima do azimute do objetivo final.
        Retorna:
          - intervalo escolhido
          - centro angular da janela
        """
        if len(free_intervals) == 0:
            return None, None

        best_interval = min(
            free_intervals,
            key=lambda interval: self.interval_distance_to_goal(
                sector_centers, interval[0], interval[1], goal_dir
            )
        )

        center_dir = self.interval_center_angle(
            sector_centers, best_interval[0], best_interval[1]
        )

        return best_interval, center_dir

    # =========================================================
    # Controle
    # =========================================================

    def compute_cmd_to_direction(self, target_dir: float):
        ang_error = self.normalize_angle(target_dir)

        w = self.clamp(
            self.k_ang * ang_error,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        # anda mais quando está bem alinhado
        if abs(ang_error) < math.radians(15.0):
            v = self.max_linear_speed
        elif abs(ang_error) < math.radians(35.0):
            v = 0.10
        else:
            v = 0.0

        if v > 0.0:
            v = max(v, self.min_linear_speed)

        return v, w

    def compute_cmd_to_point(self, point):
        target_dir = self.angle_to_point(point)
        dist = self.distance_to_point(point)

        w = self.clamp(
            self.k_ang * target_dir,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        if abs(target_dir) < math.radians(15.0):
            v = self.max_linear_speed
        elif abs(target_dir) < math.radians(35.0):
            v = 0.10
        else:
            v = 0.0

        if dist < 0.35:
            v = min(v, 0.08)

        if v > 0.0:
            v = max(v, self.min_linear_speed)

        return v, w

    def print_debug(self, goal_dir: float, histogram: np.ndarray):
        now = self.get_clock().now()
        elapsed = (now - self.last_debug_time).nanoseconds * 1e-9

        if elapsed < self.debug_print_interval:
            return

        hist_str = ''.join(str(int(x)) for x in histogram.tolist())
        dir_txt = 'None' if self.local_target_dir is None else f'{math.degrees(self.local_target_dir):.1f}°'
        point_txt = 'None' if self.local_target_point is None else f'({self.local_target_point[0]:.2f},{self.local_target_point[1]:.2f})'

        self.get_logger().info(
            f'modo={self.mode} | goal={math.degrees(goal_dir):.1f}° | '
            f'local_dir={dir_txt} | ponto={point_txt} | hist={hist_str}'
        )

        self.last_debug_time = now

    # =========================================================
    # Loop principal
    # =========================================================

    def _control_loop(self) -> None:
        if not self.has_laser_data():
            self.stop()
            return

        if not self.has_goal():
            self.stop()
            self.mode = 'GOAL'
            self.clear_local_target()
            return

        if self.distance_to_goal() <= self.goal_tolerance:
            self.stop()
            self.clear_goal()
            self.mode = 'GOAL'
            self.clear_local_target()
            return

        goal_dir = self.goal_angle_robot_frame()

        # Se o objetivo estiver fora do campo frontal, gira até trazê-lo ao FOV
        if goal_dir < self.laser_angle_min or goal_dir > self.laser_angle_max:
            v = 0.0
            w = self.clamp(self.k_ang * goal_dir, -self.max_angular_speed, self.max_angular_speed)
            self.publish_velocity(v, w)
            return

        # =====================================================
        # MODO GOAL: tenta ir diretamente ao objetivo
        # =====================================================
        if self.mode == 'GOAL':
            sector_centers, histogram = self.build_sector_histogram()

            if len(histogram) == 0:
                self.stop()
                return

            free_intervals = self.find_free_intervals(histogram)

            if self.is_direction_free(sector_centers, histogram, goal_dir):
                v, w = self.compute_cmd_to_direction(goal_dir)
                self.print_debug(goal_dir, histogram)
                self.publish_velocity(v, w)
                return

            # Goal bloqueado: escolhe janela mais próxima do azimute do objetivo
            best_window, best_center_dir = self.select_best_window(
                sector_centers, free_intervals, goal_dir
            )

            if best_center_dir is None:
                self.stop()
                self.print_debug(goal_dir, histogram)
                return

            self.local_target_dir = best_center_dir
            self.local_target_window = best_window
            self.local_target_point = self.world_point_from_robot_polar(
                self.lookahead_distance,
                self.local_target_dir
            )
            self.mode = 'LOCAL_POINT'

            v, w = self.compute_cmd_to_point(self.local_target_point)
            self.print_debug(goal_dir, histogram)
            self.publish_velocity(v, w)
            return

        # =====================================================
        # MODO LOCAL_POINT:
        # vai até o ponto local já escolhido
        # NÃO recalcula nada até chegar
        # =====================================================
        elif self.mode == 'LOCAL_POINT':
            dist_to_local = self.distance_to_point(self.local_target_point)

            if dist_to_local <= self.local_point_tolerance:
                self.mode = 'GOAL'
                self.clear_local_target()
                self.stop()
                return

            # segue até o ponto intermediário escolhido
            v, w = self.compute_cmd_to_point(self.local_target_point)
            self.publish_velocity(v, w)
            return

        else:
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    navigator = VFHWindowNavigator()

    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    finally:
        navigator.stop()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()