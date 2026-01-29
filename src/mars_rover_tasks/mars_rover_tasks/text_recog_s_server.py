#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
import cv2
from mars_rover_tasks.text_recog import TextRecognition


class TextRecognitionService(Node):
    def __init__(self):
        super().__init__('text_recognition_service')

        # Path to the EAST text detection model and parameters
        east_model_path = '/home/user/ros2_ws/src/basic_ros2_opencv/text_detection/frozen_east_text_detection.pb'
        min_confidence = 0.5
        width = 320
        height = 320
        padding = 0.0

        # Objeto que faz o reconhecimento de texto
        self.text_recognizer = TextRecognition(
            east_model_path,
            min_confidence=min_confidence,
            width=width,
            height=height,
            padding=padding
        )

        # Conversor ROS Image <-> OpenCV
        self.bridge = CvBridge()

        # Inscrição no tópico de imagem
        self.image_subscriber = self.create_subscription(
            Image,
            '/leo/camera/image_raw',  # tópico da câmera
            self.image_callback,
            1
        )

        # Criação do service server
        name_service = '/text_recognition_service'
        self.srv = self.create_service(
            Trigger,
            name_service,
            self.handle_text_recognition_request
        )

        # Variável para guardar o último texto detectado
        self.last_detected_text = ""

        self.get_logger().info(name_service + " Service Server Ready...")

    def image_callback(self, msg):
        # Converte a imagem ROS para OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Roda o reconhecimento de texto
        results = self.text_recognizer.recognize_text(cv_image)

        # Pega o primeiro texto reconhecido e guarda
        if results:
            for (start_x, start_y, end_x, end_y), text in results:
                cleaned_text = text.rstrip(".\n\x0c")
                self.last_detected_text = cleaned_text
                break  # só queremos o primeiro texto
        else:
            self.last_detected_text = ""

        self.get_logger().info(f'Result: {self.last_detected_text}')

    def handle_text_recognition_request(self, request, response):
        # Usa o último texto detectado para responder ao serviço
        detected_text = self.last_detected_text.upper()

        # success True se for FOOD ou WASTE
        if detected_text in ['FOOD', 'WASTE']:
            response.success = True
        else:
            response.success = False

        # Sempre devolver o texto na message (pode ser vazio)
        response.message = detected_text if detected_text else ""

        self.get_logger().info(
            f'Service called. Detected text: "{detected_text}", success={response.success}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = TextRecognitionService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
