#ifndef NAVIGATION_NODE_HPP_
#define NAVIGATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <cmath>
#include <limits>

/**
 * @brief Classe base para nós de navegação em ROS2.
 *
 * Esta classe gerencia a leitura de sensores (odometria e laser), recebimento de objetivos (goals)
 * do RViz, e publicação de comandos de velocidade. Deve ser herdada para implementar algoritmos
 * específicos de navegação.
 *
 * Atributos principais:
 * - x, y, theta: Posição e orientação atual do robô.
 * - laser_ranges: Array com distâncias do laser (inf para inválidos).
 * - goal: Tupla (x, y) do objetivo atual, ou None.
 *
 * Exemplo de uso:
 * @code
 * class MeuNavegador : public NavigationNode {
 * public:
 *   void on_goal() override {
 *     RCLCPP_INFO(this->get_logger(), "Novo objetivo recebido!");
 *   }
 *
 *   void control_loop() override {
 *     if (has_goal()) {
 *       // Lógica de navegação aqui
 *     }
 *   }
 * };
 * @endcode
 */
class NavigationNode : public rclcpp::Node
{
public:
  /**
   * @brief Inicializa o nó de navegação.
   *
   * @param node_name Nome do nó ROS2 (padrão: "navigation_node").
   * @param timer_period Período do loop de controle em segundos (padrão: 0.05, i.e., 20 Hz).
   */
  NavigationNode(const std::string &node_name = "navigation_node", double timer_period = 0.05);

  /**
   * @brief Retorna a pose atual do robô.
   *
   * @return Tupla (x, y, theta) com posição e orientação.
   */
  std::tuple<double, double, double> get_pose() const;

  /**
   * @brief Verifica se há dados válidos do laser.
   *
   * @return True se o array de ranges não estiver vazio.
   */
  bool has_laser_data() const;

  /**
   * @brief Verifica se há dados válidos do sonar.
   *
   * @return True se o array de ranges não estiver vazio.
   */
  bool has_sonar_data() const;

  /**
   * @brief Calcula a distância mínima em uma região específica do scan laser.
   *
   * @param idx_start Índice inicial da região.
   * @param idx_end Índice final da região.
   * @return Distância mínima na região, ou inf se inválida.
   */
  double get_region_distance(size_t idx_start, size_t idx_end) const;

  /**
   * @brief Calcula a distância mínima na frente do robô.
   *
   * @param half_angle_deg Metade do ângulo frontal em graus (padrão: 15°).
   * @return Distância mínima na frente, ou inf se sem dados.
   */
  double get_front_distance(double half_angle_deg = 15.0) const;

  /**
   * @brief Verifica se há um objetivo ativo.
   *
   * @return True se goal estiver definido.
   */
  bool has_goal() const;

  /**
   * @brief Remove o objetivo atual (e.g., quando atingido).
   */
  void clear_goal();

  /**
   * @brief Calcula a distância euclidiana até o objetivo.
   *
   * @return Distância até o goal, ou inf se sem goal.
   */
  double distance_to_goal() const;

  /**
   * @brief Calcula o ângulo relativo até o objetivo (erro de orientação).
   *
   * @return Ângulo em radianos (-pi a pi), ou 0 se sem goal.
   */
  double angle_to_goal() const;

  /**
   * @brief Publica comando de velocidade no tópico /cmd_vel.
   *
   * @param v Velocidade linear (m/s).
   * @param w Velocidade angular (rad/s).
   */
  void publish_velocity(double v, double w);

  /**
   * @brief Para o robô imediatamente.
   */
  void stop();

  /**
   * @brief Hook chamado após receber odometria.
   *
   * Sobrescreva em subclasses para reagir a mudanças de pose.
   */
  virtual void on_odom() {}

  /**
   * @brief Hook chamado após receber dados do laser.
   *
   * Sobrescreva em subclasses para processar leituras do sensor.
   */
  virtual void on_laser() {}

  /**
   * @brief Hook chamado após receber dados do sonar.
   *
   * Sobrescreva em subclasses para processar leituras do sensor.
   */
  virtual void on_sonar() {}

  /**
   * @brief Hook chamado após receber um novo objetivo.
   *
   * Sobrescreva em subclasses para iniciar navegação.
   */
  virtual void on_goal() {}

protected:
  // Estado do robô (odometria)
  double x_, y_, theta_;

  // Dados do laser
  std::vector<float> laser_ranges_;
  double laser_angle_min_, laser_angle_max_, laser_angle_increment_;

  // Dados do sonar
  std::vector<float> sonar_ranges_;
  std::vector<int> sonar_angles_;

  // Objetivo de navegação (definido pelo RViz)
  std::tuple<double, double> goal_;
  double goal_theta_;
  bool has_goal_;

  // Publishers e subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sonar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void sonar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  /**
   * @brief Loop de controle executado periodicamente (padrão 20 Hz).
   *
   * Sobrescreva em subclasses para implementar a lógica de navegação.
   */
  virtual void control_loop() = 0;
};

#endif  // NAVIGATION_NODE_HPP_