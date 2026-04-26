#include "navigation_node.hpp"

/**
 * @brief Exemplo simples de navegação reativa em C++.
 *
 * Esta classe implementa um algoritmo básico de navegação: vira em direção ao objetivo,
 * anda em linha reta se alinhado, e para se houver obstáculo à frente.
 *
 * Thresholds configuráveis:
 * - DIST_THRESHOLD: Distância para considerar objetivo atingido (0.1 m).
 * - ANGLE_THRESHOLD: Ângulo para considerar alinhado (0.1 rad ~ 5.7°).
 * - OBSTACLE_THRESHOLD: Distância para detectar obstáculo (0.5 m).
 */
class ControlExample : public NavigationNode
{
public:
  ControlExample() : NavigationNode("control_example_cpp") {}

private:
  /**
   * @brief Loop de controle executado a 20 Hz.
   *
   * Implementa a lógica de navegação: verifica objetivo, calcula erros,
   * decide entre virar, andar ou parar.
   */
  void control_loop() override
  {
    if (!has_goal()) {
      stop();
      return;
    }

    double dist_to_goal = distance_to_goal();
    double angle_err = angle_to_goal();
    double front_dist = get_front_distance();

    // Thresholds
    const double DIST_THRESHOLD = 0.1;     // metros
    const double ANGLE_THRESHOLD = 0.1;    // radianos (~5.7 graus)
    const double OBSTACLE_THRESHOLD = 0.5; // metros

    if (dist_to_goal < DIST_THRESHOLD) {
      // Objetivo alcançado
      clear_goal();
      stop();
      RCLCPP_INFO(this->get_logger(), "Objetivo alcançado!");
      return;
    }

    if (front_dist < OBSTACLE_THRESHOLD) {
      // Obstáculo à frente, parar
      stop();
      RCLCPP_WARN(this->get_logger(), "Obstáculo à frente, parando.");
      return;
    }

    if (std::abs(angle_err) > ANGLE_THRESHOLD) {
      // Virar em direção ao objetivo
      double w = (angle_err > 0) ? 0.5 : -0.5; // velocidade angular
      publish_velocity(0.0, w);
    } else {
      // Andar em linha reta
      publish_velocity(0.2, 0.0); // velocidade linear
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlExample>());
  rclcpp::shutdown();
  return 0;
}