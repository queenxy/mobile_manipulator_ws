import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped
from tinker_vision_msgs.srv import ObjectDetection
from tinker_msgs.srv import GraspnetService, Robotiq, URControlService
import argparse


class GraspDemo(Node):
    def __init__(self):
        super().__init__('grasp_demo')
        self.det_cli = self.create_client(ObjectDetection, 'object_detection_service_realsense')
        self.grasp_cli = self.create_client(GraspnetService, 'graspnet_service')
        self.ur_cli = self.create_client(URControlService, 'tinker_arm_control_service')
        self.ur_initial_cli = self.create_client(Trigger, 'tinker_arm_initial_service')
        self.robotiq_cli = self.create_client(Robotiq, 'robotiq_service')
        self.octomap_cli = self.create_client(Trigger,"add_octomap_service")

        while not self.det_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Detection service not available, waiting again...')
        while not self.grasp_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Graspnet service not available, waiting again...')
        while not self.ur_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('UR service not available, waiting again...')
        while not self.octomap_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Add octomap service not available, waiting again...')
        # while not self.robotiq_cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Robotiq service not available, waiting again...')

    def grasp(self, obj_name):
        self.get_logger().info('Grasping...')
        req = ObjectDetection.Request()
        req.flags = 'request_image|request_segments'
        
        future = self.det_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res.status > 0:
            self.get_logger().error('Detection fail with status 1')
            return
        
        depth_img = res.depth_image
        rgb_img = res.rgb_image
        # filter with conf
        # segments = [seg for seg, obj in zip(res.segments, res.objects) if obj.conf > 0.7 and obj.cls == obj_name]
        segments = [seg for seg, obj in zip(res.segments, res.objects)]
        
        if len(segments) == 0:
            self.get_logger().warn('No objects')
            return

        req = GraspnetService.Request()
        req.depth_image = depth_img
        req.rgb_image = rgb_img
        req.segments = segments
        req.source_frame = res.header.frame_id
        req.target_frame = "base_link"
        req.z_min = 0.1
        req.z_max = 1.5

        future = self.grasp_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        assert len(res.grasp_ids) > 0
        best = res.scores.index(max(res.scores))
        pose = Pose()
        pose.position = res.grasp_poses[best].position
        pose.orientation = res.grasp_poses[best].orientation

        req = Trigger.Request()
        future = self.octomap_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        req = URControlService.Request()
        req.target_pose = pose
        future = self.ur_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        # assert res is True, "UR Moveit Failed"

        req = Robotiq.Request()
        req.distance = 250
        future = self.robotiq_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        # assert res is True, "Robotiq Failed"

        req = Trigger.Request()
        future = self.ur_initial_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        # assert res is True, "UR Move Back Failed"

        return
    


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--obj_name', default='bottle', help='Log dir')
    args = parser.parse_args()

    demo = GraspDemo()

    while True:
        try:
            _ = input('Press enter to grasp')
            demo.grasp(args.obj_name)
        except Exception as e:
            print('Error caught: ', e)
            break

    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()