#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from vision_msgs.msg import Detection2DArray, Detection2D
from vision_msgs.msg import ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D

import cv2
import numpy as np
import os


class PersonDetector(Node):
    def __init__(self):
        super().__init__("person_detector")

        self.bridge = CvBridge()

        self.create_subscription(Image, "/camera/image_raw",
                                self.image_callback, 10)

        self.det_pub = self.create_publisher(
            Detection2DArray, "/person_detections", 10
        )
        self.image_pub = self.create_publisher(
            Image, "/camera/detections_image", 10
        )

        # rutas de modelos
        pkg_path = os.path.dirname(os.path.realpath(__file__))
        model_dir = os.path.join(pkg_path, "..", "models")

        cfg = os.path.join(model_dir, "yolov4-tiny.cfg")
        weights = os.path.join(model_dir, "yolov4-tiny.weights")

        self.net = cv2.dnn.readNetFromDarknet(cfg, weights)

        try:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            self.get_logger().info("CUDA habilitado")
        except Exception:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)
            self.get_logger().warn("CUDA no disponible, usando CPU")

        self.output_layers = self.net.getUnconnectedOutLayersNames()

    # ---------------------------------------------------------------------

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        h, w = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(frame, 1/255.0,
                                     (416, 416), swapRB=True, crop=False)
        self.net.setInput(blob)
        outputs = self.net.forward(self.output_layers)

        det_msg = Detection2DArray()
        det_msg.header = msg.header

        boxes, confidences = [], []

        # YOLO detección
        for out in outputs:
            for det in out:
                scores = det[5:]
                class_id = np.argmax(scores)
                conf = scores[class_id]

                if class_id == 0 and conf > 0.5:  # solo personas
                    cx, cy, bw, bh = det[:4] * np.array([w, h, w, h])
                    x = int(cx - bw / 2)
                    y = int(cy - bh / 2)

                    boxes.append([x, y, int(bw), int(bh)])
                    confidences.append(float(conf))

        idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # procesar detecciones
        if len(idxs) > 0:
            for i in idxs.flatten():
                x, y, bw, bh = boxes[i]
                conf = confidences[i]

                # -----------------------------
                # llenar mensaje vision_msgs
                # -----------------------------
                det = Detection2D()
                det.header = msg.header

                obj = ObjectHypothesisWithPose()
                obj.hypothesis.class_id = "person"
                obj.hypothesis.score = conf
                det.results.append(obj)

                bbox = BoundingBox2D()
                bbox.center.position.x = float(x + bw / 2)
                bbox.center.position.y = float(y + bh / 2)
                bbox.size_x = float(bw)
                bbox.size_y = float(bh)
                det.bbox = bbox

                det_msg.detections.append(det)

                # dibujar en imagen
                cv2.rectangle(frame, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                cv2.putText(frame, f"Person {conf:.2f}",
                            (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (0, 255, 0), 2)

        self.det_pub.publish(det_msg)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))


# ---------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

