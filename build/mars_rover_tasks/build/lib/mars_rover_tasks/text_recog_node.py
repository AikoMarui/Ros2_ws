import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from mars_rover_tasks.text_recog import TextRecognition


class TextRecognitionNode(Node):
    def __init__(self):
        # Nome do nó no ROS2
        super().__init__('text_recognition_node')

        # Caminho do modelo EAST de texto (ajuste se no seu ambiente for diferente)
        east_model_path = '/home/user/ros2_ws/src/basic_ros2_extra_files/text_detector/frozen_east_text_detection.pb'

        # Parâmetros do detector de texto
        min_confidence = 0.5
        width = 320
        height = 320
        padding = 0.0

        # Objeto que faz o reconhecimento de texto na imagem
        self.text_recognizer = TextRecognition(
            east_model_path,
            min_confidence,
            width,
            height,
            padding
        )

        # Conversor entre ROS Image e OpenCV
        self.bridge = CvBridge()

        # Assinatura do tópico de imagem da câmera
        self.image_subscriber = self.create_subscription(
            Image,                       # tipo da mensagem
            '/leo/camera/image_raw',     # tópico da câmera
            self.image_callback,         # função callback
            1                            # tamanho da fila (queue size)
        )

    def image_callback(self, msg):
        # Log para confirmar que o callback está sendo chamado
        self.get_logger().info('Recebi uma imagem')

        # Converte a imagem do formato ROS para OpenCV (BGR)
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Roda o reconhecimento de texto na imagem
        results = self.text_recognizer.recognize_text(cv_image)

        # results: lista de ((start_x, start_y, end_x, end_y), text)
        for (start_x, start_y, end_x, end_y), text in results:
            # Limpa o texto de caracteres estranhos
            cleaned_text = text.rstrip(".\n\x0c")
            # Monta string com posição da bounding box
            position = f"{start_x}-{start_y}-{end_x}-{end_y}"

            # Escreve no log o texto encontrado e a posição
            self.get_logger().info(
                f'OCR Result: {cleaned_text}, position: {position}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = TextRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
