#!/usr/bin/env python3
"""
Exemplo simples de controle de navegação.

Este script implementa um algoritmo básico: gira para alinhar com o objetivo,
depois anda em linha reta. Para se encontra obstáculo à frente ou se chega ao objetivo.
"""

import math
import rclpy
from phi_p3dx_navigation.main import NavigationNode


class ControlExample(NavigationNode):
    """
    Navegador de exemplo: gira para o goal, anda em frente, para se encontra obstáculo.
    """

    def __init__(self):
        super().__init__(node_name='control_example_navigator', timer_period=0.1)

        # Parâmetros do controle
        self.front_threshold = 0.5  # Distância para parar por obstáculo (m)
        self.angle_threshold = math.radians(5.0)  # Ângulo para considerar alinhado (graus)
        self.linear_speed = 0.2    # Velocidade linear (m/s)
        self.angular_speed = 0.1   # Velocidade angular (rad/s)

    def on_goal(self) -> None:
        """Chamado quando um novo goal é definido."""
        self.get_logger().info('Novo goal recebido! Iniciando controle simples.')

    def _control_loop(self) -> None:
        """Loop de controle"""
        if not self.has_goal():
            self.stop()
            return

        if not self.has_laser_data():
            self.stop()
            return
        
        # Checa se esta proximo do objetivo
        if self.distance_to_goal() < self.front_threshold:
            self.clear_goal()
            self.stop()
            return

        # Calcular ângulo para o objetivo
        angle_err = self.angle_to_goal()
        self.get_logger().info(f'Angulo para objetivo={math.degrees(angle_err):.1f}°')


        if abs(angle_err) > self.angle_threshold:
            # Girar para alinhar
            w = self.angular_speed 
            if angle_err > 0:
                w = self.angular_speed 
            else:
                w = -self.angular_speed
            self.publish_velocity(0.0, w)
            self.get_logger().info(f'Girando: vel angular={w:.1f}')
        else:

            # Verificar obstáculo à frente
            front_dist = self.get_front_distance()
            if front_dist < self.front_threshold:
                self.stop()
                self.get_logger().info('Obstáculo à frente: parando.')
                return


            # Andar em frente
            self.publish_velocity(self.linear_speed, 0.0)
            self.get_logger().info('Andando em frente.')


def main(args=None):
    rclpy.init(args=args)
    navigator = ControlExample()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()