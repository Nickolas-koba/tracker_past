# detect_ball_3d.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from my_custom_msgs.msg import DetectedBalls  # Mensagem customizada com lista de bolas detectadas
import math

class DetectBall3d(Node):

    def __init__(self):
        super().__init__('detect_ball_3d')
        
        # Mensagem de inicialização
        self.get_logger().info('Detecção de bolas em 3D iniciada...')
        
        # Assina o tópico que contém a lista de bolas detectadas em 2D
        self.ball2d_sub = self.create_subscription(
            DetectedBalls, "/detected_balls", self.ball_rcv_callback, 10)
        
        # Publica a posição das bolas em 3D
        self.ball3d_pub = self.create_publisher(DetectedBalls, "/detected_balls_3d", 1)
        
        # Publica marcadores para visualização no rviz
        self.ball_marker_pub = self.create_publisher(Marker, "/ball_3d_marker", 1)

        # Declaração dos parâmetros da câmera e configuração da bola
        self.declare_parameter("h_fov", 1.089)
        self.declare_parameter("ball_radius", 0.083)
        self.declare_parameter("aspect_ratio", 4.0/3.0)
        self.declare_parameter("camera_frame", 'camera_link_optical')

        self.h_fov = self.get_parameter('h_fov').get_parameter_value().double_value
        self.v_fov = self.h_fov / self.get_parameter('aspect_ratio').get_parameter_value().double_value
        self.ball_radius = self.get_parameter('ball_radius').get_parameter_value().double_value
        self.camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value           # utiliza os parametros calculados em relação à câmera e aos obsjetos

    def ball_rcv_callback(self, data: DetectedBalls):
        # Mensagem para armazenar as bolas em 3D
        detected_balls_3d_msg = DetectedBalls()
        
        # Itera sobre cada bola detectada na lista recebida
        for idx, ball_2d in enumerate(data.balls):
            # Calcula a projeção 3D com base nas coordenadas 2D e parâmetros da câmera
            ang_size = ball_2d.z * self.h_fov
            d = self.ball_radius / (math.atan(ang_size / 2))

            # Calcula desvios angulares e a projeção de distância em X e Y
            y_ang = ball_2d.y * self.v_fov / 2
            y = d * math.sin(y_ang)
            d_proj = d * math.cos(y_ang)

            x_ang = ball_2d.x * self.h_fov / 2
            x = d_proj * math.sin(x_ang)
            z = d_proj * math.cos(x_ang)

            # Cria a posição 3D da bola como um ponto
            ball_3d_point = Point()
            ball_3d_point.x = x
            ball_3d_point.y = y
            ball_3d_point.z = z

            # Adiciona o ponto 3D ao array de bolas 3D
            detected_balls_3d_msg.balls.append(ball_3d_point)

            # Publica um marcador para visualizar cada bola no rviz
            marker = Marker()
            marker.header.frame_id = self.camera_frame
            marker.id = idx  # Cada marcador precisa de um ID único
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = ball_3d_point  # Define a posição 3D da bola
            marker.scale.x = self.ball_radius * 2
            marker.scale.y = self.ball_radius * 2
            marker.scale.z = self.ball_radius * 2
            marker.color.r = 1.0  # Define a cor (vermelho)
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            self.ball_marker_pub.publish(marker)

        # Publica a lista completa de bolas em 3D
        if len(detected_balls_3d_msg.balls) > 0:
            self.ball3d_pub.publish(detected_balls_3d_msg)
            self.get_logger().info(f"{len(detected_balls_3d_msg.balls)} bola(s) convertida(s) para 3D e publicada(s).")

def main(args=None):
    rclpy.init(args=args)
    detect_ball_3d = DetectBall3d()
    rclpy.spin(detect_ball_3d)
    detect_ball_3d.destroy_node()
    rclpy.shutdown()
