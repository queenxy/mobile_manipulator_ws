import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.time import Time
import sensor_msgs
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header, Int32MultiArray
import geometry_msgs
import tinker_vision_msgs
from tinker_vision_msgs.msg import Object, Objects
from tinker_vision_msgs.srv import ObjectDetection, PointDirection

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from message_filters import Subscriber, ApproximateTimeSynchronizer

from ultralytics import YOLO
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt
# import mediapipe as mp
import time
import os
from img2vec_pytorch import Img2Vec
from PIL import Image as PILImage
from sklearn.metrics.pairwise import cosine_similarity

from torchvision.transforms import ToTensor
from deep_sort_realtime.deepsort_tracker import DeepSort
from .third_party.deep_sort_realtime.deepsort_tracker import DeepSort
import torch


class PointDirectionService(Node):

    def __init__(self):
        super().__init__('point_direction_service')

        self.declare_parameter('camera_type', 'realsense')
        self.camera_type = self.get_parameter('camera_type').get_parameter_value().string_value

        self.declare_parameter('visualization_en', False)
        self.visualization_en = self.get_parameter('visualization_en').get_parameter_value().bool_value

        self.action_server = self.create_service(PointDirection, 'point_direction_service', self.point_direction_callback)
        self.bridge = CvBridge()
        
        self.get_logger().info('Point direction service initialized.')


    def point_direction_callback(self, request, response):
        self.visualization_en = self.get_parameter('visualization_en').get_parameter_value().bool_value


def main():
    rclpy.init()
    service = PointDirectionService()
    rclpy.spin(service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()