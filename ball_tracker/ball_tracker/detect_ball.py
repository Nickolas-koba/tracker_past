# detect_ball.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
from my_custom_msgs.msg import DetectedBalls  # Importa a mensagem customizada que representa a lista de bolas
import ball_tracker.process_image as proc

class DetectBall(Node):
    def __init__(self):
        super().__init__('detect_ball')

        self.get_logger().info('Procurando por bolas...')
        
        # Assinatura no tópico de entrada de imagem
        self.image_sub = self.create_subscription(
            Image, "/image_in", self.callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        
        # Publicadores para as imagens de saída e lista de bolas
        self.image_out_pub = self.create_publisher(Image, "/image_out", 1)
        self.image_tuning_pub = self.create_publisher(Image, "/image_tuning", 1)
        self.ball_list_pub = self.create_publisher(DetectedBalls, "/detected_balls", 1)  # Publica a lista de bolas

        # Declaração de parâmetros de configuração
        self.declare_parameter('tuning_mode', False)
        self.declare_parameter("x_min", 0)
        self.declare_parameter("x_max", 100)
        self.declare_parameter("y_min", 37)
        self.declare_parameter("y_max", 100)
        self.declare_parameter("h_min", 0)
        self.declare_parameter("h_max", 10)
        self.declare_parameter("s_min", 0)
        self.declare_parameter("s_max", 255)
        self.declare_parameter("v_min", 255)
        self.declare_parameter("v_max", 255)
        self.declare_parameter("sz_min", 0)
        self.declare_parameter("sz_max", 100)
        
        # Parâmetros de tuning (ajuste fino)
        self.tuning_mode = self.get_parameter('tuning_mode').get_parameter_value().bool_value
        self.tuning_params = {
            'x_min': self.get_parameter('x_min').get_parameter_value().integer_value,
            'x_max': self.get_parameter('x_max').get_parameter_value().integer_value,
            'y_min': self.get_parameter('y_min').get_parameter_value().integer_value,
            'y_max': self.get_parameter('y_max').get_parameter_value().integer_value,
            'h_min': self.get_parameter('h_min').get_parameter_value().integer_value,
            'h_max': self.get_parameter('h_max').get_parameter_value().integer_value,
            's_min': self.get_parameter('s_min').get_parameter_value().integer_value,
            's_max': self.get_parameter('s_max').get_parameter_value().integer_value,
            'v_min': self.get_parameter('v_min').get_parameter_value().integer_value,
            'v_max': self.get_parameter('v_max').get_parameter_value().integer_value,
            'sz_min': self.get_parameter('sz_min').get_parameter_value().integer_value,
            'sz_max': self.get_parameter('sz_max').get_parameter_value().integer_value
        }

        self.bridge = CvBridge()

        # Se estiver em modo de ajuste, cria uma janela interativa
        if self.tuning_mode:
            proc.create_tuning_window(self.tuning_params)

    def callback(self, data):
        try:
            # Converte a imagem recebida para um formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"Erro ao converter a imagem: {e}")
            return

        try:
            if self.tuning_mode:
                # Atualiza os parâmetros de ajuste a partir da interface gráfica
                self.tuning_params = proc.get_tuning_params()

            # Detecta círculos na imagem usando os parâmetros de ajuste
            keypoints_norm, out_image, tuning_image = proc.find_circles(cv_image, self.tuning_params)

            # Converte a imagem de saída para ser publicada
            img_to_pub = self.bridge.cv2_to_imgmsg(out_image, "bgr8")
            img_to_pub.header = data.header
            self.image_out_pub.publish(img_to_pub)

            # Converte a imagem de tuning para ser publicada
            img_to_pub = self.bridge.cv2_to_imgmsg(tuning_image, "bgr8")
            img_to_pub.header = data.header
            self.image_tuning_pub.publish(img_to_pub)

            # Cria uma mensagem customizada para armazenar a lista de bolas detectadas
            detected_balls_msg = DetectedBalls()
            
            # Itera por cada ponto detectado e armazena na lista
            for kp in keypoints_norm:
                point = Point()
                point.x = kp.pt[0]  # Coordenada x normalizada
                point.y = kp.pt[1]  # Coordenada y normalizada
                point.z = kp.size   # Tamanho da bola detectada
                detected_balls_msg.balls.append(point)  # Adiciona à lista de bolas

            # Publica a lista de bolas detectadas
            if len(detected_balls_msg.balls) > 0:
                self.ball_list_pub.publish(detected_balls_msg)
                self.get_logger().info(f"{len(detected_balls_msg.balls)} bola(s) detectada(s) e publicada(s).")

        except CvBridgeError as e:
            self.get_logger().error(f"Erro ao processar a imagem: {e}")  


def main(args=None):
    rclpy.init(args=args)
    detect_ball = DetectBall()
    while rclpy.ok():
        rclpy.spin_once(detect_ball)
        proc.wait_on_gui()  # Aguarda a interação com a interface de ajuste (se ativada)
    detect_ball.destroy_node()
    rclpy.shutdown()
