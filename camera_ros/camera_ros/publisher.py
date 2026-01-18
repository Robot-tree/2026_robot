#!/usr/bin/env python3

################################################################################
##  Node              : video publisher with inference                        ##
##  Author            : 12941516                                              ##
##  Dev date          : 2026/01/16 ~ 2026/01/18                               ##
##  To ready          : ssd-mobilenet.onnx, labels.txt                        ##
##  To run            : ros2 run camera_ros publisher                         ##
##  To test video     : ros2 run camera_ros video_test                        ##
##  To test inference : ros2 topic echo /detectnet/result                     ##
##  To change mode    : ros2 topic pub ~~~~                                   ##
##     ros2 topic pub -1 /inference_switch std_msgs/msg/Bool "{data: true}"   ##
##     ros2 topic pub -1 /inference_switch std_msgs/msg/Bool "{data: false}"  ##
################################################################################

import os
import json
import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool

from jetson_inference import detectNet
from jetson_utils import videoSource


class DetectNetNode(Node):
    def __init__(self):
        super().__init__("detectnet_node")

        # -----------------------------
        # Publishers
        # -----------------------------
        self.img_pub = self.create_publisher(
            CompressedImage, "/camera", 5
        )
        self.result_pub = self.create_publisher(
            String, "/detectnet/result", 5
        )

        # -----------------------------
        # Subscriber (inference switch)
        # -----------------------------
        self.inference_enabled = False
        self.create_subscription(
            Bool,
            "/inference_switch",
            self.switch_callback,
            5
        )

        self.get_logger().info("DetectNet node started")


    def switch_callback(self, msg: Bool):
        self.inference_enabled = msg.data
        self.get_logger().info(
            f"Inference switch: {'ON' if msg.data else 'OFF'}"
        )


def main():
    rclpy.init()
    node = DetectNetNode()

    # -----------------------------
    # Jetson Inference init
    # -----------------------------
    base_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(base_dir, "ssd-mobilenet.onnx")
    labels_path = os.path.join(base_dir, "labels.txt")

    input_uri = "csi://0"
    overlay = "none"
    threshold = 0.5

    input_stream = videoSource(input_uri)

    net = detectNet(
        argv=[
            "--model=" + model_path,
            "--labels=" + labels_path,
            "--input-blob=input_0",
            "--output-cvg=scores",
            "--output-bbox=boxes",
            "--threshold=" + str(threshold),
        ]
    )

    # -----------------------------
    # Main loop
    # -----------------------------
    while rclpy.ok():
        img_cuda = input_stream.Capture()
        if img_cuda is None:
            continue
        detections = net.Detect(img_cuda, overlay=overlay)
        
        # =============================
        # 1) Publish image
        # =============================
        img_np = np.array(img_cuda, copy=True)
        img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGR)

        # =============================
        # 2) Inference
        # =============================
        if node.inference_enabled:
            result = []
            for d in detections:
                xmin = int(d.Left)
                ymin = int(d.Top)
                xmax = int(d.Right)
                ymax = int(d.Bottom)
                class_name = net.GetClassDesc(d.ClassID)
                conf = float(d.Confidence)
                label = f"{class_name} {conf:.2f}"

                cv2.rectangle(img_bgr, (xmin, ymin),
                            (xmax, ymax), (0, 255, 0), 2)
                cv2.putText(img_bgr, label, (xmin, max(ymin-10,0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                result.append({"class": class_name, "confidence": conf,
                            "xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax,})
            
            result_msg = String()
            result_msg.data = json.dumps(result)
            node.result_pub.publish(result_msg)

        _, jpg = cv2.imencode(".jpg", img_bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        img_msg = CompressedImage()
        img_msg.header.stamp = node.get_clock().now().to_msg()
        img_msg.format = "jpeg"
        img_msg.data = jpg.tobytes()
        node.img_pub.publish(img_msg)
        
        # ROS spin
        rclpy.spin_once(node, timeout_sec=0.0)
        if not input_stream.IsStreaming():
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

