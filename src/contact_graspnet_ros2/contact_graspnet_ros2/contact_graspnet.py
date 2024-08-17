from tinker_msgs.srv import GraspnetService
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

import rclpy
from rclpy.node import Node

import numpy as np
from cv_bridge import CvBridge
import cv2

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs

from tf_transformations import quaternion_from_matrix, quaternion_from_euler, quaternion_multiply

import argparse
import os
from .contact_graspnet_pytorch.contact_graspnet_pytorch import inference, config_utils, contact_grasp_estimator, checkpoints
from ament_index_python.packages import get_package_share_directory

import time


def image_to_numpy(img: Image) -> np.ndarray:
    assert img.encoding == 'rgb8'
    CvBridge().imgmsg_to_cv2(img, 'passthrough')


class ContactGraspnetService(Node):

    def __init__(self):
        super().__init__('contact_graspnet_ros2')
        self.srv = self.create_service(GraspnetService, 'graspnet_service', self.contact_graspnet_callback)

        # Initialize graspnet model
        package_share_directory = get_package_share_directory('contact_graspnet_ros2')
        # Checkpoints directory
        ckpt_dir = os.path.join(package_share_directory, 'checkpoints/contact_graspnet')

        parser = argparse.ArgumentParser()
        parser.add_argument('--ckpt_dir', default=ckpt_dir, help='Log dir')
        # useless in production
        parser.add_argument('--np_path', default='/home/tinker/tk23_ws/src/tk23_manipulation/src/contact_graspnet_ros2/contact_graspnet_ros2/contact_graspnet_pytorch/test_data/7.npy', help='Input data: npz/npy file with keys either "depth" & camera matrix "K" or just point cloud "pc" in meters. Optionally, a 2D "segmap"')
        parser.add_argument('--K', default=None, help='Flat Camera Matrix, pass as "[fx, 0, cx, 0, fy, cy, 0, 0 ,1]"')
        parser.add_argument('--z_range', default=[0.2,1.8], help='Z value threshold to crop the input point cloud')
        parser.add_argument('--local_regions', action='store_true', default=True, help='Crop 3D local regions around given segments.')
        parser.add_argument('--filter_grasps', action='store_true', default=True,  help='Filter grasp contacts according to segmap.')
        parser.add_argument('--skip_border_objects', action='store_true', default=False,  help='When extracting local_regions, ignore segments at depth map boundary.')
        parser.add_argument('--forward_passes', type=int, default=1,  help='Run multiple parallel forward passes to mesh_utils more potential contact points.')
        parser.add_argument('--arg_configs', nargs="*", type=str, default=[], help='overwrite config parameters')
        self.FLAGS = parser.parse_args()

        global_config = config_utils.load_config(self.FLAGS.ckpt_dir,
                                                 batch_size=self.FLAGS.forward_passes,
                                                 arg_configs=self.FLAGS.arg_configs)
        self.get_logger().info('Graspnet global config: ' + str(global_config))

        # Build the model
        self.grasp_model = contact_grasp_estimator.GraspEstimator(global_config)
        # Load the weights
        model_checkpoint_dir = os.path.join(ckpt_dir, 'checkpoints')
        checkpoint_io = checkpoints.CheckpointIO(checkpoint_dir=model_checkpoint_dir, model=self.grasp_model.model)
        try:
            load_dict = checkpoint_io.load('model.pt')
        except FileExistsError:
            self.get_logger().error(f'Failed to load model.pt from directory {model_checkpoint_dir}.')
            load_dict = {}
            return
        
        # Camera intrinsics
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/aligned_depth_to_color/camera_info', 
            self.camera_info_callback, 
            qos_profile=10
        )
        self.camera_intrinsics = None

        # Utilities
        self.bridge = CvBridge()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.debug_topic = self.create_publisher(PoseStamped, 'graspnet_result', qos_profile=10)

        self.get_logger().info('Graspnet initialized.')

    def camera_info_callback(self, msg):
        self.camera_intrinsics = msg

    def contact_graspnet_callback(self, request, response):
        self.get_logger().info('Incoming request')
        if self.camera_intrinsics is None:
            response.grasp_ids = []
            response.grasp_poses = []
            response.scores = []
            self.get_logger().error('Failed to get the camera intrinsics')
            return response
        
        depth = self.bridge.imgmsg_to_cv2(request.depth_image, '32FC1')
        depth = np.array(depth, dtype=np.float32)

        rgb = self.bridge.imgmsg_to_cv2(request.rgb_image, 'rgb8')
        rgb = np.array(rgb, dtype=np.float32)
        cv2.imwrite('rgb.png', rgb)

        # There are two modes of segmentation, pickup specific object or pickup any object, 
        # len(segments) == 1, means pickup specific object, 
        # len(segments) > 1, means pickup any object by scores
        segments = request.segments
        segment = self.bridge.imgmsg_to_cv2(segments[0], '8UC1')
        if len(segments) > 1:
            for i in range(1, len(segments)):
                seg = self.bridge.imgmsg_to_cv2(segments[i], '8UC1')
                assert seg.max() == 1
                overlap = np.logical_and(segment > 0, seg > 0)
                segment += (i + 1) * (seg - overlap)

        segment = np.array(segment, dtype=np.float32)

        cam_K = np.array(self.camera_intrinsics.k).reshape(3, 3)

        if request.z_max > request.z_min + 1e-3:
            z_range = [request.z_min, request.z_max]
        else:
            z_range = eval(str(self.FLAGS.z_range))

        start_time = time.time()

        pred_grasps, scores, _ = inference.inference_light(self.grasp_model,
                                                           depth=depth,
                                                           cam_K=cam_K,
                                                           segmap=segment,
                                                           rgb=rgb,
                                                           local_regions=self.FLAGS.local_regions,
                                                           filter_grasps=self.FLAGS.filter_grasps,
                                                           skip_border_objects=self.FLAGS.skip_border_objects,
                                                           z_range=z_range,
                                                           forward_passes=self.FLAGS.forward_passes,
                                                           visualize=False)

        self.get_logger().info(f'Inference done in {(time.time() - start_time) * 1000} ms. {len(pred_grasps)} grasps found.')
        
        response.grasp_ids = []
        response.grasp_poses = []
        response.scores = []

        print(pred_grasps.keys())
        for key in pred_grasps.keys():
            for g, score in zip(pred_grasps[key], scores[key]):
                pose = Pose()
                # The original params contact-graspnet used. No need to change to robotiq params 
                gripper = np.array([0, 0, 0.105273142457])
                point = g[:3, 3] + np.matmul(g[:3, :3], gripper)
                pose.position = Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
                # The gripper's direction vector is (0, 0, 1), rotation in ROS is all from (1, 0, 0)
                # q _conv = quaternion_from_euler(0, -np.pi / 2, 0)
                q = quaternion_from_matrix(g)   # tuple                
                # q = quaternion_multiply(q, q_conv)
                pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

                if len(request.source_frame) > 0 and len(request.target_frame) > 0:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            target_frame=request.target_frame,
                            source_frame=request.source_frame,
                            time=rclpy.time.Time()
                        )
                        pose = tf2_geometry_msgs.do_transform_pose(pose, transform)

                    except TransformException as ex:
                        self.get_logger().error(f'Could not transform {request.source_frame} to {request.target_frame}: {ex}')
                
                # v_ref = np.array([0,0,1])  # Horizonal
                # v_real = np.matmul(np.array([0,0,1]),g[:3,:3])
                # v_real = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                # ori_score = np.dot(v_real,v_ref)/np.linalg.norm(v_real)
                try:
                    real_ori = self.tf_buffer.lookup_transform(
                            target_frame="gripper",
                            source_frame="base_link",
                            time=rclpy.time.Time()
                        ).transform.rotation
                    self.get_logger().info(f"Real gripper orientation:  {real_ori}")
                except  TransformException as ex:
                    self.get_logger().error(f'Could not transform base_link to gripper: {ex}')

                # Choose the pose with the orientation closest to the real gripper orientation 
                expect_ori = np.array([0.653,0.653,0.271,0.271])
                score += np.dot([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], expect_ori)

                response.grasp_ids.append(int(key))
                response.grasp_poses.append(pose)
                response.scores.append(score)

            best = response.scores.index(max(response.scores))
            self.debug_topic.publish(PoseStamped(
                    header=Header(stamp=self.get_clock().now().to_msg(), frame_id=request.target_frame),
                    pose=response.grasp_poses[best]
                ))
            self.get_logger().info(f"Best grasp: {response.grasp_poses[best]}")

        self.get_logger().info('Done.')
                
        return response

def main():
    rclpy.init()
    service = ContactGraspnetService()
    rclpy.spin(service)
    rclpy.shutdown()

    # package_share_directory = get_package_share_directory('contact_graspnet_ros2')
    # # checkpoints directory
    # ckpt_dir = os.path.join(package_share_directory, 'checkpoints/contact_graspnet')

    # parser = argparse.ArgumentParser()
    # parser.add_argument('--ckpt_dir', default=ckpt_dir, help='Log dir')
    # # useless in production
    # parser.add_argument('--np_path', default='/home/a/tk23_ws/src/tk23_manipulation/src/contact_graspnet_ros2/contact_graspnet_ros2/contact_graspnet_pytorch/test_data/7.npy', help='Input data: npz/npy file with keys either "depth" & camera matrix "K" or just point cloud "pc" in meters. Optionally, a 2D "segmap"')
    # parser.add_argument('--K', default=None, help='Flat Camera Matrix, pass as "[fx, 0, cx, 0, fy, cy, 0, 0 ,1]"')
    # parser.add_argument('--z_range', default=[0.2,1.8], help='Z value threshold to crop the input point cloud')
    # parser.add_argument('--local_regions', action='store_true', default=True, help='Crop 3D local regions around given segments.')
    # parser.add_argument('--filter_grasps', action='store_true', default=True,  help='Filter grasp contacts according to segmap.')
    # parser.add_argument('--skip_border_objects', action='store_true', default=False,  help='When extracting local_regions, ignore segments at depth map boundary.')
    # parser.add_argument('--forward_passes', type=int, default=1,  help='Run multiple parallel forward passes to mesh_utils more potential contact points.')
    # parser.add_argument('--arg_configs', nargs="*", type=str, default=[], help='overwrite config parameters')
    # FLAGS = parser.parse_args()

    # global_config = config_utils.load_config(FLAGS.ckpt_dir, batch_size=FLAGS.forward_passes, arg_configs=FLAGS.arg_configs)

    # inference.inference(global_config,
    #                     FLAGS.ckpt_dir,
    #                     FLAGS.np_path,
    #                     local_regions=FLAGS.local_regions,
    #                     filter_grasps=FLAGS.filter_grasps,
    #                     skip_border_objects=FLAGS.skip_border_objects,
    #                     z_range=eval(str(FLAGS.z_range)),
    #                     forward_passes=FLAGS.forward_passes,
    #                     K=eval(str(FLAGS.K)))
    

if __name__ == '__main__':
    main()
