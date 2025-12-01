#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        # DepthAI v3-style pipeline (tu API)
        self.pipeline = dai.Pipeline()
        cam = self.pipeline.create(dai.node.Camera).build()
        # requestOutput determina la resolución del stream
        self.q = cam.requestOutput((640, 400)).createOutputQueue()
        self.pipeline.start()

        # Timer para publicar, 30Hz (ajusta si quieres)
        self.timer = self.create_timer(1.0/30.0, self.timer_callback)

    def timer_callback(self):
        if not self.pipeline.isRunning():
            return
        frame = self.q.get()   # bloqueante por defecto
        if frame is None:
            return
        img = frame.getCvFrame()
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
