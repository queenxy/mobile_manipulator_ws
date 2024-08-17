import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped
import tinker_vision_msgs
from tinker_vision_msgs.srv import ObjectDetection
from tinker_msgs.srv import GraspnetService


class GraspnetTester(Node):
    def __init__(self):
        super().__init__('graspnet_tester')
        self.det_cli = self.create_client(ObjectDetection, 'object_detection_service')
        self.grasp_cli = self.create_client(GraspnetService, 'graspnet_service')

        while not self.det_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.grasp_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.pub = self.create_publisher(PoseStamped, 'tpose', 10)


    def test(self):
        self.get_logger().info('Testing...')
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
        segments = [seg for seg, obj in zip(res.segments, res.objects) if obj.conf > 0.7]
        
        if len(segments) == 0:
            self.get_logger().warn('No objects')
            return
        else:
            print('Frame: ', res.header.frame_id)
            print('Objects: ')
            for obj in res.objects:
                if obj.conf > 0.7:
                    print(obj.cls, obj.centroid.x, obj.centroid.y, obj.centroid.z)

        req = GraspnetService.Request()
        req.depth_image = depth_img
        req.rgb_image = rgb_img
        req.segments = segments
        req.source_frame = res.header.frame_id
        req.target_frame = res.header.frame_id
        req.z_min = 0.0
        req.z_max = 0.0
        print('Request: ')
        # print(str(req))

        future = self.grasp_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()

        assert len(res.grasp_ids) > 0
        print('grasp_ids: ', res.grasp_ids)
        print('grasp_poses: ', res.grasp_poses)
        print('scores: ', res.scores)

        print('frame: ', req.target_frame)
        self.pub.publish(PoseStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=req.target_frame),
            pose=res.grasp_poses[0]
        ))




def main(args=None):
    rclpy.init(args=args)

    tester = GraspnetTester()

    while True:
        try:
            _ = input('Press enter to test')
            tester.test()
        except Exception as e:
            print('Error caught: ', e)
            break

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()