import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_custom_msgs.msg import DetectedBalls  # Mensagem customizada com lista de bolas
import time

class FollowBall(Node):

    def __init__(self):
        super().__init__('follow_ball')
        self.subscription = self.create_subscription(
            DetectedBalls,
            '/detected_balls_3d',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.declare_parameter("rcv_timeout_secs", 1.0)
        self.declare_parameter("angular_chase_multiplier", 0.7)
        self.declare_parameter("forward_chase_speed", 0.1)
        self.declare_parameter("search_angular_speed", 0.5)
        self.declare_parameter("max_size_thresh", 0.1)
        self.declare_parameter("filter_value", 0.9)

        self.rcv_timeout_secs = self.get_parameter('rcv_timeout_secs').get_parameter_value().double_value
        self.angular_chase_multiplier = self.get_parameter('angular_chase_multiplier').get_parameter_value().double_value
        self.forward_chase_speed = self.get_parameter('forward_chase_speed').get_parameter_value().double_value
        self.search_angular_speed = self.get_parameter('search_angular_speed').get_parameter_value().double_value
        self.max_size_thresh = self.get_parameter('max_size_thresh').get_parameter_value().double_value
        self.filter_value = self.get_parameter('filter_value').get_parameter_value().double_value

        self.selected_ball_idx = None  # Índice da bola escolhida pelo usuário
        self.target_val = 0.0
        self.target_dist = 0.0
        self.lastrcvtime = time.time() - 10000

    def timer_callback(self):
        msg = Twist()
        if (time.time() - self.lastrcvtime < self.rcv_timeout_secs) and self.selected_ball_idx is not None:
            self.get_logger().info(f'Seguindo a bola: {self.selected_ball_idx}')
            print(self.target_dist)
            if self.target_dist < self.max_size_thresh:
                msg.linear.x = self.forward_chase_speed
            msg.angular.z = -self.angular_chase_multiplier * self.target_val
        else:
            self.get_logger().info('Bola perdida ou não selecionada. Buscando...')
            msg.angular.z = self.search_angular_speed
        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        if len(msg.balls) == 0:
            self.get_logger().info("Nenhuma bola detectada.")
            return

        # Pergunta ao usuário para escolher qual bola seguir, se ainda não houver uma escolha
        if self.selected_ball_idx is None:
            print("\nBolas detectadas:")
            for idx, ball in enumerate(msg.balls):
                print(f"Bola {idx}: x={ball.x:.2f}, y={ball.y:.2f}, z(tamanho)={ball.z:.2f}")

            # Espera a escolha do usuário
            while True:
                try:
                    choice = int(input("Digite o índice da bola que deseja seguir: "))
                    if 0 <= choice < len(msg.balls):
                        self.selected_ball_idx = choice
                        self.get_logger().info(f"Bola escolhida: {self.selected_ball_idx}")
                        break
                    else:
                        print(f"Por favor, insira um índice válido entre 0 e {len(msg.balls)-1}.")
                except ValueError:
                    print("Entrada inválida. Digite um número inteiro.")

        # Definir os valores alvo com base na bola selecionada
        selected_ball = msg.balls[self.selected_ball_idx]
        f = self.filter_value
        self.target_val = self.target_val * f + selected_ball.x * (1-f)
        self.target_dist = self.target_dist * f + selected_ball.z * (1-f)
        self.lastrcvtime = time.time()


def main(args=None):
    rclpy.init(args=args)
    follow_ball = FollowBall()

    # Criar um timer para o controle do robô com base na bola escolhida
    timer_period = 0.1
    follow_ball.timer = follow_ball.create_timer(timer_period, follow_ball.timer_callback)

    rclpy.spin(follow_ball)
    follow_ball.destroy_node()
    rclpy.shutdown()
