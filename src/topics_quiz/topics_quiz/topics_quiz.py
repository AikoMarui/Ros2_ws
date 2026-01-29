import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf_transformations


class TopicsQuizNode(Node):
    def __init__(self):
        super().__init__('topics_quiz_node')

        # Publicador de velocidades para mover o robô
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # (Opcional) publicador de status /mars_rover_status
        self.status_pub = self.create_publisher(String, '/mars_rover_status', 10)

        # Assinantes: odometria, laser e /nasa_mission
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.mission_sub = self.create_subscription(
            String,
            '/nasa_mission',
            self.nasa_mission_callback,
            10
        )

        # Estados internos
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.yaw = 0.0

        self.home_position = {'x': 0.0, 'y': 0.0}
        self.pickup_position = {'x': -2.342, 'y': -2.432}
        self.goal_tolerance = 0.1

        self.current_goal = None
        self.mission_active = False

        # Guarda o último scan para desviar de obstáculos
        self.latest_scan = None

        self.get_logger().info('topics_quiz_node started.')

    def nasa_mission_callback(self, msg: String):
        """Recebe comandos Go-Home ou Go-Pickup em /nasa_mission."""
        command = msg.data.strip()
        self.get_logger().info(f'Received mission command: {command}')

        # Se já estou lá, não faço nada
        if command == 'Go-Home':
            if self.is_at_position(self.home_position):
                self.get_logger().info('Already at home. Not moving.')
                return
            self.current_goal = self.home_position
            self.mission_active = True
            self.get_logger().info('Going to HOME.')

        elif command == 'Go-Pickup':
            if self.is_at_position(self.pickup_position):
                self.get_logger().info('Already at pickup. Not moving.')
                return
            self.current_goal = self.pickup_position
            self.mission_active = True
            self.get_logger().info('Going to PICKUP.')

        else:
            self.get_logger().warn('Unknown command, ignoring.')
            return

        self.status_pub.publish(String(data=f'mission: {command}'))

    def odom_callback(self, msg: Odometry):
        """Atualiza posição e checa se chegou ao objetivo."""
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ]
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(orientation_list)
        self.yaw = yaw

        # Se há objetivo ativo, checa distância
        if self.current_goal:
            distance_to_goal = math.sqrt(
                (self.current_position['x'] - self.current_goal['x']) ** 2 +
                (self.current_position['y'] - self.current_goal['y']) ** 2
            )

            if distance_to_goal < self.goal_tolerance:
                self.stop_robot()
                self.mission_active = False
                self.current_goal = None
                self.get_logger().info('Goal reached. Robot stopped.')
                self.status_pub.publish(String(data='goal_reached'))

    def is_at_position(self, target_position: dict) -> bool:
        """Verifica se já está dentro da tolerância do alvo."""
        distance_to_target = math.sqrt(
            (self.current_position['x'] - target_position['x']) ** 2 +
            (self.current_position['y'] - target_position['y']) ** 2
        )
        return distance_to_target < self.goal_tolerance

    def laser_callback(self, msg: LaserScan):
        """Guarda o laser e, se estiver em missão, manda comandos evitando obstáculos."""
        self.latest_scan = msg

        if not self.mission_active or self.current_goal is None:
            return

        twist = Twist()

        # Lógica super simples de desvio:
        # Se algo muito perto à frente, gira; senão, vai em frente.
        min_range = min(msg.ranges)

        if min_range < 0.5:
            # Obstáculo muito perto: girar
            twist.linear.x = 0.0
            twist.angular.z = 0.5
        else:
            # Ir em frente na direção geral do objetivo
            twist.linear.x = 0.2
            twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    def stop_robot(self):
        """Publica velocidade zero para parar o robô."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = TopicsQuizNode()

    # Processa odometria inicial para saber onde está
    rclpy.spin_once(node)

    # Exemplo de lógica extra (opcional, baseado no hint):
    # se não está em casa ao iniciar, pode mandar voltar para home automaticamente.
    if not node.is_at_position(node.home_position):
        node.get_logger().info('Initial position not at home, setting goal HOME.')
        node.current_goal = node.home_position
        node.mission_active = True

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
