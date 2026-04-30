#!/usr/bin/env python3
"""
INF01034 - Robótica Móvel Inteligente - 2026/1
Planejamento de caminhos usando RRT com Restrições Cinemáticas.
Alunos: Matheus Silveira e Kairan Morandin
"""

import math
import random
import rclpy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from phi_p3dx_planning.main import PlanningNode


class RRTKinematicPlanner(PlanningNode):
    """
    Planejador RRT Cinemático: respeita as restrições do robô diferencial.
    Os nodos da árvore incluem orientação (x, y, θ), e a expansão é feita
    simulando o modelo cinemático com velocidade linear constante.
    """

    def __init__(self):
        super().__init__(node_name='rrt_kinematic_planner', timer_period=0.1)

        # Parâmetros cinemáticos do robô
        self.v0      = 0.4   # Velocidade linear constante usada no planejamento (m/s)
        self.w_max   = 0.5   # Velocidade angular máxima admissível (rad/s)
        self.dt      = 1.0   # Intervalo de simulação cinemática (s)
        self.w_samples  = 7      # Número de valores de ω testados por expansão
        self.w_epsilon  = 1e-4   # Limiar abaixo do qual ω é tratado como zero (evita div/0)

        # Parâmetros do RRT
        self.max_iterations = 8000   # Limite de iterações antes de desistir
        self.goal_tolerance = 0.5    # Distância para considerar o goal alcançado (m)
        self.goal_bias      = 0.10   # Probabilidade de amostrar o goal diretamente

        # Parâmetros do controlador de seguimento
        self.angle_threshold = math.radians(8.0)  # Tolerância angular para considerar alinhado
        self.dist_threshold  = 0.35               # Tolerância de chegada a cada waypoint (m)
        self.linear_speed    = 0.2                # Velocidade linear de seguimento (m/s)
        self.angular_speed   = 0.4                # Velocidade angular de seguimento (rad/s)

        # Estrutura da árvore RRT
        # Cada nodo é (x, y, θ); tree_parents[i] guarda o índice do pai do nodo i
        self.tree_nodes   = []
        self.tree_parents = []

        # Caminho extraído por backtracking: lista de waypoints (x, y, θ)
        self.path       = []
        self.path_index = 0   # Índice do próximo waypoint a seguir

        self.rrt_ready = False  # Indica se existe um caminho válido para seguir

    def publish_tree(self) -> None:
        """
        Publica a árvore RRT atual no RViz como marcadores visuais.

        Utiliza as configurações da classe base: nodos como esferas (SPHERE_LIST)
        e arestas como linhas (LINE_LIST). Cada par de pontos consecutivos na
        lista de arestas representa uma conexão pai → filho na árvore.
        """
        marker_array = MarkerArray()

        self.nodes_marker.points.clear()
        self.edges_marker.points.clear()

        # Adiciona cada nodo da árvore como uma esfera (usa apenas x, y)
        for node in self.tree_nodes:
            self.nodes_marker.points.append(Point(x=node[0], y=node[1], z=0.0))

        # Adiciona cada aresta como um par de pontos (pai, filho)
        for i, parent_idx in enumerate(self.tree_parents):
            if parent_idx is None:
                continue
            p_child  = self.tree_nodes[i]
            p_parent = self.tree_nodes[parent_idx]
            self.edges_marker.points.append(Point(x=p_parent[0], y=p_parent[1], z=0.0))
            self.edges_marker.points.append(Point(x=p_child[0],  y=p_child[1],  z=0.0))

        marker_array.markers.append(self.nodes_marker)
        marker_array.markers.append(self.edges_marker)
        self.marker_pub.publish(marker_array)

    def on_goal(self) -> None:
        """
        Callback chamado automaticamente ao receber um novo objetivo via RViz.

        Reinicia o estado do planejador e dispara a construção de uma nova
        árvore RRT cinemática a partir da posição e orientação atuais do robô.
        """
        if self.goal is None:
            return

        if self.map_msg is None:
            self.get_logger().warn('Novo goal recebido (nenhum mapa recebido ainda), ignorando.')
            self.goal = None
            self.stop()
            return

        # Reinicia o estado antes de planejar
        self.rrt_ready  = False
        self.path       = []
        self.path_index = 0

        self.get_logger().info('Novo goal recebido. Computando RRT cinemático...')

        found = self._build_rrt()

        if found:
            self.get_logger().info(f'Caminho encontrado com {len(self.path)} waypoints.')
            self.rrt_ready = True
        else:
            self.get_logger().warn('RRT cinemático não encontrou caminho dentro do limite de iterações.')
            self.goal = None

        # Publica a árvore gerada no RViz independente do resultado
        self.publish_tree()

    def _build_rrt(self) -> bool:
        """
        Constrói a árvore RRT respeitando as restrições cinemáticas do robô.

        A cada iteração, amostra um ponto aleatório, encontra o nodo mais próximo,
        calcula uma velocidade angular admissível baseada na diferença de orientação
        necessária, simula o movimento do robô pelas equações cinemáticas e verifica
        colisão antes de inserir o novo nodo na árvore.

        Retorna True se um caminho até o goal foi encontrado, False caso contrário.
        """
        # Inicializa a árvore com o estado atual do robô (posição + orientação)
        self.tree_nodes   = [(self.x, self.y, self.theta)]
        self.tree_parents = [None]

        gx, gy   = self.goal
        info     = self.map_msg.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y
        map_xmax = origin_x + info.width  * info.resolution
        map_ymax = origin_y + info.height * info.resolution

        for _ in range(self.max_iterations):

            # 1. Amostra um ponto aleatório no mapa (com viés para o goal)
            if random.random() < self.goal_bias:
                qrand = (gx, gy)
            else:
                qrand = (random.uniform(origin_x, map_xmax),
                         random.uniform(origin_y, map_ymax))

            # 2. Encontra o nodo mais próximo de qrand na árvore atual (distância em x,y)
            near_idx = self._nearest_node(qrand)
            qnear    = self.tree_nodes[near_idx]   # (x, y, θ)

            # 3. Calcula o ω ideal para apontar qnear em direção a qrand
            #    e amostra candidatos em torno desse valor
            dx_goal   = qrand[0] - qnear[0]
            dy_goal   = qrand[1] - qnear[1]
            desired_θ = math.atan2(dy_goal, dx_goal)
            dθ        = math.atan2(math.sin(desired_θ - qnear[2]),
                                   math.cos(desired_θ - qnear[2]))

            # Velocidade angular ideal (limitada por w_max) + amostras aleatórias adicionais
            w_ideal  = max(-self.w_max, min(self.w_max, dθ / self.dt))
            w_values = [w_ideal] + [random.uniform(-self.w_max, self.w_max)
                                    for _ in range(self.w_samples - 1)]

            # 4. Testa cada candidato de ω e usa o primeiro que resultar em posição livre
            qnew = None
            for w in w_values:
                candidate = self._simulate(qnear, w)
                if self._is_free((candidate[0], candidate[1])):
                    qnew = candidate
                    break

            if qnew is None:
                continue

            # 5. Insere o novo nodo na árvore
            self.tree_nodes.append(qnew)
            self.tree_parents.append(near_idx)
            new_idx = len(self.tree_nodes) - 1

            # 6. Verifica se chegou suficientemente perto do goal
            if math.hypot(qnew[0] - gx, qnew[1] - gy) <= self.goal_tolerance:
                # Adiciona o goal exato como nodo final e reconstrói o caminho
                self.tree_nodes.append((gx, gy, qnew[2]))
                self.tree_parents.append(new_idx)
                self.path = self._backtrack(len(self.tree_nodes) - 1)
                return True

        return False

    def _simulate(self, qnear: tuple, w: float) -> tuple:
        """
        Simula o movimento do robô diferencial a partir de qnear com velocidade angular w.

        Aplica as equações de cinemática do robô diferencial por um intervalo dt.
        Para valores de ω muito pequenos (abaixo de w_epsilon), aproxima o movimento
        como retilíneo para evitar instabilidade numérica na divisão por ω.

        Retorna o novo estado (x_new, y_new, θ_new) após a simulação.
        """
        x, y, θ = qnear

        if abs(w) < self.w_epsilon:
            # Movimento retilíneo: robô avança na direção atual
            x_new = x + self.v0 * self.dt * math.cos(θ)
            y_new = y + self.v0 * self.dt * math.sin(θ)
            θ_new = θ
        else:
            # Movimento curvilíneo: arco de círculo com raio v0/ω
            x_new = x + (self.v0 / w) * (math.sin(θ + w * self.dt) - math.sin(θ))
            y_new = y - (self.v0 / w) * (math.cos(θ + w * self.dt) - math.cos(θ))
            θ_new = θ + w * self.dt

        # Normaliza o ângulo resultante para o intervalo [-π, π]
        θ_new = math.atan2(math.sin(θ_new), math.cos(θ_new))

        return (x_new, y_new, θ_new)

    def _nearest_node(self, q: tuple) -> int:
        """
        Retorna o índice do nodo da árvore mais próximo do ponto q.

        Utiliza apenas as coordenadas (x, y) para o cálculo da distância,
        ignorando a orientação θ dos nodos.
        """
        best_idx  = 0
        best_dist = float('inf')
        for i, node in enumerate(self.tree_nodes):
            d = math.hypot(node[0] - q[0], node[1] - q[1])
            if d < best_dist:
                best_dist = d
                best_idx  = i
        return best_idx

    def _is_free(self, q: tuple) -> bool:
        """
        Verifica se a posição q está em região livre no mapa de ocupação.

        Converte coordenadas do mundo para índices do mapa e consulta o valor
        de ocupação da célula correspondente. Considera livre apenas células
        com valor entre 0 e 50 (exclusive obstáculos e regiões desconhecidas).
        """
        if self.map_msg is None:
            return False

        info     = self.map_msg.info
        origin_x = info.origin.position.x
        origin_y = info.origin.position.y

        col = int((q[0] - origin_x) / info.resolution)
        row = int((q[1] - origin_y) / info.resolution)

        if not (0 <= col < info.width and 0 <= row < info.height):
            return False

        val = self.map_msg.data[row * info.width + col]
        return val != -1 and val <= 50

    def _backtrack(self, leaf_idx: int) -> list:
        """
        Reconstrói o caminho do goal até a raiz percorrendo os ponteiros de pai.

        Retorna a lista de nodos ordenada da raiz até o goal.
        """
        path = []
        idx  = leaf_idx
        while idx is not None:
            path.append(self.tree_nodes[idx])
            idx = self.tree_parents[idx]
        path.reverse()
        return path

    def _control_loop(self) -> None:
        """
        Loop de controle executado periodicamente (timer_period = 0.1s).

        Implementa o seguimento do caminho RRT nodo a nodo usando a estratégia
        gira-para-anda reto-gira: primeiro alinha a orientação com o próximo
        waypoint, depois avança em linha reta até alcançá-lo.
        """
        # Aguarda um caminho válido antes de iniciar o movimento
        if not self.has_goal() or not self.rrt_ready:
            self.stop()
            return

        # Verifica se todos os waypoints foram percorridos
        if not self.path or self.path_index >= len(self.path):
            self.clear_goal()
            self.stop()
            return

        # Obtém o waypoint atual (ignora θ no seguimento, usa apenas x, y)
        wx, wy, _ = self.path[self.path_index]
        dx        = wx - self.x
        dy        = wy - self.y
        dist      = math.hypot(dx, dy)
        desired   = math.atan2(dy, dx)
        angle_err = math.atan2(math.sin(desired - self.theta),
                               math.cos(desired - self.theta))

        # Chegou ao waypoint atual: avança para o próximo
        if dist < self.dist_threshold:
            self.path_index += 1
            self.get_logger().info(
                f'Waypoint {self.path_index}/{len(self.path)} alcançado.')
            self.stop()
            return

        # Precisa girar antes de avançar
        if abs(angle_err) > self.angle_threshold:
            w = self.angular_speed if angle_err > 0 else -self.angular_speed
            self.publish_velocity(0.0, w)
            self.get_logger().info(f'Girando: erro angular={math.degrees(angle_err):.1f}°')
        else:
            # Já está alinhado: avança em linha reta
            self.publish_velocity(self.linear_speed, 0.0)
            self.get_logger().info(f'Andando em frente. Distância ao waypoint: {dist:.2f}m')

        # Atualiza a visualização da árvore no RViz
        self.publish_tree()


def main(args=None):
    rclpy.init(args=args)
    planner = RRTKinematicPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()