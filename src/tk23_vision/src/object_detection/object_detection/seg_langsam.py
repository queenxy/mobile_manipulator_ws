import rclpy
from rclpy.node import Node
from rclpy.time import Time
import sensor_msgs
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header, Int32MultiArray
import geometry_msgs
import tinker_vision_msgs
from tinker_vision_msgs.msg import Object, Objects
from tinker_vision_msgs.srv import ObjectDetection

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from message_filters import Subscriber, ApproximateTimeSynchronizer

from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt
import mediapipe as mp
import time
import os
from img2vec_pytorch import Img2Vec
from PIL import Image as PILImage
from sklearn.metrics.pairwise import cosine_similarity

from torchvision.transforms import ToTensor
from deep_sort_realtime.deepsort_tracker import DeepSort
from .third_party.deep_sort_realtime.deepsort_tracker import DeepSort
import torch
import socket

# helper functions
def get_time_sec(stamp):
    if isinstance(stamp, rclpy.time.Time):
        return (stamp.nanoseconds // 1e6) / 1e3 if stamp else None
    
    assert False
    # return (stamp.nanosec // 1e6) / 1e3 if stamp else None


def get_array_from_points(points: PointCloud2) -> tuple[np.array, np.array]:
    '''
    Get the point array from point cloud
    Returns: point_arr: [H, W, 3], valid_mask: [H, W]
    Kinect PointCloud2 format:
        height: 1
        width: 3145728
        fields:
        - name: x
            offset: 0
            datatype: 7
            count: 1
        - name: y
            offset: 4
            datatype: 7
            count: 1
        - name: z
            offset: 8
            datatype: 7
            count: 1
        - name: rgb
            offset: 16
            datatype: 7
            count: 1
        is_bigendian: false
        point_step: 32
        row_step: 100663296
    PointField datatype:
        uint8 INT8    = 1
        uint8 UINT8   = 2
        uint8 INT16   = 3
        uint8 UINT16  = 4
        uint8 INT32   = 5
        uint8 UINT32  = 6
        uint8 FLOAT32 = 7
        uint8 FLOAT64 = 8
    '''
    h, w = 1536, 2048
    # h, w = 720, 1280
    arr = np.frombuffer(points.data, dtype='<f4')
    # print(len(arr), h * w * 8)
    assert(len(arr) == h * w * 8)
    arr = arr.reshape((h, w, 8))[:, :, :3]
    mask = 1 - np.multiply.reduce(np.isnan(arr), axis=2)
    # remove nans
    arr = np.nan_to_num(arr, nan=0)
    return arr, mask


def send_arr_to_tcp(arr, conn):
    arr = arr.astype(np.uint8)
    conn.sendall(arr.shape[0].to_bytes(4, byteorder='big') + arr.shape[1].to_bytes(4, byteorder='big') + arr.shape[2].to_bytes(4, byteorder='big'))
    conn.sendall(arr.tobytes())
    data = conn.recv(2)
    assert data.decode('utf-8') == 'ok'


def recv_arr_from_tcp(conn):
    shape = conn.recv(12)
    shape = (int.from_bytes(shape[:4], byteorder='big'), int.from_bytes(shape[4:8], byteorder='big'), int.from_bytes(shape[8:], byteorder='big'))
    bytes_recv = shape[0] * shape[1] * shape[2]
    arr = b''
    while bytes_recv > 0:
        data = conn.recv(65536)
        arr += data
        bytes_recv -= len(data)
    arr = np.frombuffer(arr, dtype=np.uint8).reshape(shape)
    conn.sendall('ok'.encode('utf-8'))
    return arr


def send_str_to_tcp(s, conn):
    conn.sendall(len(s).to_bytes(4, byteorder='big'))
    conn.sendall(s.encode('utf-8'))
    data = conn.recv(2)
    assert data.decode('utf-8') == 'ok'


def recv_str_from_tcp(conn):
    length = int.from_bytes(conn.recv(4), byteorder='big')
    s = b''
    while length > 0:
        data = conn.recv(65536)
        s += data
        length -= len(data)
    conn.sendall('ok'.encode('utf-8'))
    return s.decode('utf-8')


def bbox_from_mask(mask):
    nonzero = np.nonzero(mask)
    x1, y1, x2, y2 = np.min(nonzero[0]), np.min(nonzero[1]), np.max(nonzero[0]), np.max(nonzero[1])
    return x1, y1, x2, y2


class DetectionService(Node):

    def __init__(self):
        super().__init__(f'detection_service_{int(time.time())}')

        self.declare_parameter('camera_type', 'realsense')
        self.camera_type = self.get_parameter('camera_type').get_parameter_value().string_value

        if self.camera_type == 'realsense':
            self.image_topic = '/camera/camera/color/image_raw'
            self.point_cloud_topic = '/camera/camera/aligned_depth_to_color/image_raw'
        else:
            self.image_topic = 'rgb/image_raw'
            self.point_cloud_topic = 'points2'

        self.declare_parameter('visualization_en', True)   # visualize (cv etc.)
        self.declare_parameter('detection_topic_en', False)  # publish to topic for each frame received
        self.declare_parameter('match_objects_en', True)   # match objects using vector (from a given list)
        self.declare_parameter('publish_rate', 5)
        self.visualization_en = self.get_parameter('visualization_en').get_parameter_value().bool_value
        self.detection_topic_en = self.get_parameter('detection_topic_en').get_parameter_value().bool_value
        self.match_objects_en = self.get_parameter('match_objects_en').get_parameter_value().bool_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().integer_value

        # synchronize between image topic and point cloud topic
        self.image_sub = Subscriber(
            self,
            Image,
            self.image_topic
        )
        self.point_cloud_sub = Subscriber(
            self,
            Image if self.camera_type == 'realsense' else PointCloud2,
            self.point_cloud_topic
        )
        self.image_sync = ApproximateTimeSynchronizer(
            [self.image_sub, self.point_cloud_sub], queue_size=3, slop=0.05
        )
        self.image_sync.registerCallback(self.img_sync_callback)
        
        # The most recent (RGB, Depth) raw message
        self.recent_sync_msg = None
        # The most recent (RGB, Depth) processed data
        self.recent_sync_data = None
        self.recent_publish_time = None


        if self.camera_type == 'realsense':
            self.camera_info_topic = '/camera/camera/aligned_depth_to_color/camera_info'
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                self.camera_info_topic, 
                self.camera_info_callback, 
                qos_profile=10
            )
        else:
            pass
        self.camera_intrinsic = None
        # self.point_cloud_xy = None

        if self.camera_type == 'realsense':
            self.srv = self.create_service(ObjectDetection, 'object_detection_service_realsense', self.detection_srv_callback)
        else:
            self.srv = self.create_service(ObjectDetection, 'object_detection_service', self.detection_srv_callback)

        self.bridge = CvBridge()
        
        self.tracker = DeepSort(max_age=300, embedder="mobilenet", max_cosine_distance=0.12)
        self.fps_time_rec = 0

        self.publisher_ = self.create_publisher(geometry_msgs.msg.PointStamped, 'obj_det', 10)
        if self.detection_topic_en:
            self.detection_publisher = self.create_publisher(Objects, 'detection_publisher', 10)

        if self.match_objects_en:
            self.img2vec = Img2Vec(cuda=True, model='densenet',layer_output_size=1024)

            # target_objects are Bluemoon Oreo Lays Nongfu Cola Shuyuan Sprite Fanta Libai Franzzi Shampoo Bread
            self.n_target_object = 4
            self.target_object_str = ['amendel', 'chocolade', 'soep', 'wine']
            self.target_image_path = [f'/home/tinker/tk23_ws/src/tk23_vision/src/object_detection/object_detection/images/{self.target_object_str[idx]}' for idx in range(self.n_target_object)]
            self.target_vector_path = [f'/home/tinker/tk23_ws/src/tk23_vision/src/object_detection/object_detection/vectors/{self.target_object_str[idx]}.npy' for idx in range(self.n_target_object)]
            self.target_vec = []
            for i in range(self.n_target_object):
                if not os.path.exists(self.target_vector_path[i]):
                    self.get_logger().warn('Target vectors not found, creating from ' + self.target_image_path[i])
                    # create vectors
                    files = []
                    for f in os.listdir(self.target_image_path[i]):
                        files.append(PILImage.open(os.path.join(self.target_image_path[i], f)))
                    img_vec = self.img2vec.get_vec(files, tensor=True)
                    img_vec = img_vec.reshape((len(files), -1))

                    np.save(self.target_vector_path[i], img_vec, allow_pickle=True)
                    self.target_vec.append(torch.tensor(img_vec))

                else:
                    self.target_vec.append(torch.tensor(np.load(self.target_vector_path[i], allow_pickle=True)))

        else:
            self.img2vec = None
            self.n_target_object = 0
            self.target_object_str = []
            self.target_vector_path = []
            self.target_vec = []
        
        self.pose_detector = mp.solutions.pose.Pose(static_image_mode=False,        # 是静态图片还是连续视频帧
                                                    model_complexity=1,             # 取0,1,2；0最快但性能差，2最慢但性能好
                                                    smooth_landmarks=True,          # 是否平滑关键点
                                                    min_detection_confidence=0.5,   # 置信度阈值
                                                    min_tracking_confidence=0.5)    # 追踪阈值

        self.cos_similarity = torch.nn.CosineSimilarity(dim=1)

        self.get_logger().info('Detection service initialized.')


    def camera_info_callback(self, info):
        self.camera_intrinsic = info
    
    
    def img_sync_callback(self, color_msg, depth_msg):
        start_time = time.time()

        # convert realsense depth info to point cloud
        if self.camera_type == 'realsense':
            if self.camera_intrinsic is None:
                return
            
            # color_img = np.ndarray(shape=(color_msg.height, color_msg.width, 3), dtype=np.uint8, buffer=color_msg.data)
            color_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            # depth_img = np.ndarray(shape=(depth_msg.height, depth_msg.width), dtype=np.uint16, buffer=depth_msg.data)
            depth_img = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            depth_img = depth_img.astype(float) / 1000.0

            H, W = depth_img.shape
            fx, fy, cx, cy = tuple(self.camera_intrinsic.k[[0, 4, 2, 5]])
            points_x = np.repeat(np.expand_dims(np.arange(0, H), axis=1), W, axis=1)
            points_y = np.repeat(np.expand_dims(np.arange(0, W), axis=0), H, axis=0)
            points_x = (points_x - cx) * depth_img / fx
            points_y = (points_y - cy) * depth_img / fy
            
            validmask_points = np.ones_like(depth_img)
            validmask_points[depth_img > 10] = 0
            validmask_points[depth_img < 1e-6] = 0
            # depth_img *= validmask_points
            depth_img[depth_img > 10] = 10
            depth_img[depth_img < 1e-6] = 0

            points = np.stack([points_x, points_y, depth_img], axis=2)
        # for kinect (point clodu already given)
        else:
            color_img = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            points, validmask_points = get_array_from_points(depth_msg)

        # print(f'Stage 1, time = {(time.time() - start_time) * 1000: .1f} ms.')

        # publish to topic
        if self.detection_topic_en:
            now = get_time_sec(self.get_clock().now())
            if self.publish_rate <= 0 or self.recent_publish_time is None \
                or now > self.recent_publish_time + 1 / self.publish_rate:

                msg, _, _ = self.detection_segmentation(color_img, points, validmask_points, tracking_fast_mode=True, register_person=False)
                msg.header = depth_msg.header
                self.detection_publisher.publish(msg)
                self.recent_publish_time = now

        # print(f'Stage 2, time = {(time.time() - start_time) * 1000: .1f} ms.')
        self.recent_sync_msg = color_msg, depth_msg
        self.recent_sync_data = (color_img, points, validmask_points, depth_msg.header)

        # print(f'Done, time = {(time.time() - start_time) * 1000: .1f} ms.')


    def call_lang_sam(self, img, prompt='objects'):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as conn:
            conn.connect(('127.0.0.1', 12345))
            send_arr_to_tcp(img, conn)
            send_str_to_tcp(prompt, conn)
            masks = recv_arr_from_tcp(conn)
            # cv2.imshow('mask', img)
            # cv2.waitKey(0)
            # for i in range(masks.shape[0]):
                # cv2.imshow('mask', masks[i] * 255)
                # cv2.waitKey(0)
        return masks
    

    def detection_segmentation(self,
                               img,
                               img_pt,
                               validmask_pt,
                               tracking_fast_mode=False,
                               register_person=False,
                               match_obj_cls=False,
                               find_pointed_object=False,
                               request_img=False,
                               request_segments=False):
        """    
        Args:
            img (np.array) : rgb image
            img_pt (np.array) : point cloud
            validmask_pt (np arr) : 0/1 mask of wether pt in pc is valid
            tracking_fast_mode (bool) : only detect persons
            register_person (bool) : register the closest person
            match_obj_cls (bool) : match objects to a given list using vector 
            find_pointed_object (bool) : find the object pointed to by the closest person in frame
            request_img (bool) : return the original image (otherwise return empty image)
            request_segments (bool) : return segments of each object (otherwise return empty list)
    
        Returns:
            tuple[Objects, tuple[int, int], tuple[np.array, np.array, list[np.array]]] : 
                objects, (register_status, register_id), (rgb_image, depth_image, [segment_1, segment_2, ...])
                
            objects (Objects) : detection result message
            register_status (int) : 1 if successfully registered, 0 otherwise
            register_id (int) : track ID of the registered person
            rgb_image (np.array) : original RGB image
            depth_image (np.array) : original depth image
            [segment_1, segment_2, ...] (list[np.array]) : segments of the objects (as 0/1 masks)     
        """
        start_time = time.time()

        self.visualization_en = self.get_parameter('visualization_en').get_parameter_value().bool_value

        response = Objects()

        response.status = 1
        response.objects = []

        # img_pt = cv2.resize(img_pt, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        # validmask_pt = cv2.resize(validmask_pt, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        assert img_pt.shape[0] == img.shape[0] and img_pt.shape[1] == img.shape[1]

        if self.visualization_en:
            # images for visualization
            vis_img = np.copy(img)
            vis_depth_img = np.copy(img_pt[:, :, 2])

            vis_depth_img = (vis_depth_img - vis_depth_img.min()) / (vis_depth_img.max() - vis_depth_img.min())
            vis_depth_img = cv2.cvtColor((vis_depth_img * 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        else:
            vis_img, vis_depth_img = None, None

        deepsort_detections, deepsort_raw_indices = [], []

        # rgb, depth, segments
        ret_img = np.ones((1, 1, 3)), np.ones((1, 1)), []
        if request_img:
            ret_img = img, img_pt[:, :, 2], []

        # print(f'Stage 2, time = {(time.time() - start_time) * 1000: .1f} ms.')
        masks = self.call_lang_sam(img)
        print(masks.shape)
        bboxes = []
        for i in range(masks.shape[0]):
            conf = 0.0
            cls = 'a'
            bbox = bbox_from_mask(masks[i])
            if tracking_fast_mode:
                assert False

            mask_obj = masks[i]
            
            # centroid
            # img_pt = [H, W, 3], masks[i] = [H, W]

            # mask_pt = mask_obj * validmask_pt
            # if mask_pt.sum() < 10:
            #     self.get_logger().warn(f'Detected {cls} with invalid depth info, skipped.')
            #     continue
            # masked_pts = img_pt * np.expand_dims(mask_pt, 2)
            # cent_pts = masked_pts.sum(axis=(0, 1)) / mask_pt.sum()

            x1, y1, x2, y2 = bbox
            mask_pt = mask_obj[x1: x2, y1: y2] * validmask_pt[x1: x2, y1: y2]
            if mask_pt.sum() < 10:
                self.get_logger().warn(f'Detected {cls} with invalid depth info, skipped.')
                continue
            sum_pt = mask_pt.sum()
            cent_pts = [(img_pt[x1: x2, y1: y2, i] * mask_pt).sum() / sum_pt for i in range(3)]

            if self.camera_type == 'realsense':
                centroid = geometry_msgs.msg.Point(x=cent_pts[2], y=-cent_pts[1], z=-cent_pts[0])
            else:
                centroid = geometry_msgs.msg.Point(x=cent_pts[0], y=cent_pts[1], z=cent_pts[2])

            # compare with existing objects
            object_id, similarity_max = 0, 0.0
            if match_obj_cls:
                cropped_image = PILImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)[bbox[0]:bbox[2], bbox[1]:bbox[3]])
                # cv2.imshow('cropped', cv2.cvtColor(np.array(cropped_image), cv2.COLOR_RGB2BGR))
                # cv2.waitKey(1000)
                img_vec = self.img2vec.get_vec(cropped_image, tensor=True)
                img_vec = img_vec.reshape((1, -1))
                # print('img_vec', img_vec.shape)

                for i in range(self.n_target_object):
                    similarity = self.cos_similarity(img_vec, self.target_vec[i]).max()
                    # print(similarity)
                    if similarity > similarity_max:
                        similarity_max = similarity
                        object_id = i

                print('*********************************************************')
                print(f'Object: {self.target_object_str[object_id]}, similarity: {similarity_max}')
                print('*********************************************************')
            
            response.objects.append(Object(conf=conf, id=0, cls=cls, centroid=centroid, object_id=object_id, similarity=float(similarity_max), being_pointed=0))
            bboxes.append(bbox)

            if request_segments:
                ret_img[2].append(mask_obj.astype(np.uint8))
            
            if cls == 'person':
                deepsort_detections.append((
                    [bbox[1], bbox[0], bbox[3] - bbox[1], bbox[2] - bbox[0]],
                    conf,
                    cls,
                    centroid.z
                ))
                deepsort_raw_indices.append(i)

            if self.visualization_en:
                # Draw bbox on rgb image
                x1, y1, x2, y2 = bbox
                vis_img = cv2.rectangle(vis_img, (y1, x1), (y2, x2), (0, 255, 0), 2)
                if match_obj_cls:
                    cv2.putText(
                        img=vis_img,
                        text=f'{self.target_object_str[object_id]}, {similarity_max:.2f}, {cent_pts[2]:.2f}',
                        org=((y1 + y2) // 2, (x1 + x2) // 2),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.6,
                        color=(0, 0, 255),
                        thickness=2
                    )
                else:
                    cv2.putText(
                        img=vis_img,
                        text=f'{cls}, {conf:.2f}, {cent_pts[2]:.2f}',
                        org=((y1 + y2) // 2, (x1 + x2) // 2),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.6,
                        color=(0, 0, 255),
                        thickness=2
                    )
                
                # Draw contours on depth image
                contours, _ = cv2.findContours(mask_obj.astype(np.uint8) * 255, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                cv2.drawContours(vis_depth_img, contours, -1, (255, 0, 0), 1)

                # if i < 1:
                #     self.publisher_.publish(geometry_msgs.msg.PointStamped(header=response.header, point=centroid))

        # print(f'Stage 3, time = {(time.time() - start_time) * 1000: .1f} ms.')
        # Deepsort tracking begin
        if len(deepsort_detections) > 0:
            min_z, min_idx = 1e9, -1
            for idx, (bbox, conf, cls, z) in enumerate(deepsort_detections):
                if min_z > z:
                    min_z, min_idx = z, idx

            for idx, (bbox, conf, cls, z) in enumerate(deepsort_detections):
                deepsort_detections[idx] = (bbox, conf, cls, (register_person and min_idx == idx))

                
        tracks, detection_ids, reg_info = self.tracker.update_tracks(deepsort_detections, frame=img, register_person=register_person)
        for detection_idx, track_id in detection_ids:
            response.objects[deepsort_raw_indices[detection_idx]].id = track_id
            self.get_logger().info(f'Track {track_id} found.')

        # Deepsort tracking end

        # print(f'Stage 4, time = {(time.time() - start_time) * 1000: .1f} ms.')
        if find_pointed_object:
            # Find the bag being pointed
            person_idx, person_depth = -1, 1e6
            # find the nearest person
            for i, obj in enumerate(response.objects):
                dep = obj.centroid.x if self.camera_type == 'realsense' else obj.centroid.z
                if obj.cls == 'person' and dep < person_depth:
                    person_idx, person_depth = i, dep
            
            if person_idx >= 0:
                # get pose landmarks
                bbox = bboxes[person_idx]

                cropped_image = img[bbox[0]:bbox[2], bbox[1]:bbox[3]]
                results_pose = self.pose_detector.process(cropped_image)

                if results_pose.pose_landmarks:
                    landmarks = results_pose.pose_landmarks.landmark
                    H, W, _ = cropped_image.shape

                    # 左胳膊肘
                    left_elbow = int(landmarks[13].y * H) + bbox[0], int(landmarks[13].x * W) + bbox[1]
                    # 右胳膊肘
                    right_elbow = int(landmarks[14].y * H) + bbox[0], int(landmarks[14].x * W) + bbox[1]
                    # 左手腕
                    left_wrist = int(landmarks[15].y * H) + bbox[0], int(landmarks[15].x * W) + bbox[1]
                    # 右手腕
                    right_wrist = int(landmarks[16].y * H) + bbox[0], int(landmarks[16].x * W) + bbox[1]

                    if self.visualization_en:
                        img__ = img.copy()
                        cv2.circle(img__, (left_elbow[1], left_elbow[0]), radius=5, color=(255, 0, 0), thickness=-1)
                        cv2.circle(img__, (right_elbow[1], right_elbow[0]), radius=5, color=(0, 0, 255), thickness=-1)
                        cv2.circle(img__, (left_wrist[1], left_wrist[0]), radius=5, color=(255, 0, 0), thickness=-1)
                        cv2.circle(img__, (right_wrist[1], right_wrist[0]), radius=5, color=(0, 0, 255), thickness=-1)
                    
                    def distance_to_ray(point, ray_st, ray_ed):
                        x0, y0 = point
                        x1, y1 = ray_st
                        x2, y2 = ray_ed
                        # distance from point to (ray_st, ray_ed)
                        dis = np.abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / (np.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2) + 1e-6)
                        # cross product (ray_ed - ray_st, point - ray_st)
                        position = np.sign((x2 - x1) * (x0 - x1) + (y2 - y1) * (y0 - y1))
                        return dis, position

                    # find the pointed object
                    object_idx, distance, direction = -1, 1e6, 0
                    for i, (obj, bbox) in enumerate(zip(response.objects, bboxes)):
                        if obj.cls in ['bag', 'suitcase'] or True:
                            obj_center = (bbox[0] + bbox[2]) // 2, (bbox[1] + bbox[3]) // 2

                            dis_left, pos_left = distance_to_ray(obj_center, left_elbow, left_wrist)
                            if pos_left > 0 and distance > dis_left:
                                object_idx, distance, direction = i, dis_left, -1

                            dis_right, pos_right = distance_to_ray(obj_center, right_elbow, right_wrist)
                            if pos_right > 0 and distance > dis_right:
                                object_idx, distance, direction = i, dis_right, 1
                    
                    if object_idx > 0:
                        response.objects[object_idx].being_pointed = 1
                        if self.visualization_en:
                            x1, y1, x2, y2 = bboxes[object_idx]
                            cv2.rectangle(img__, (y1, x1), (y2, x2), (0, 0, 255), 3)
                            cv2.imshow('point_object', img__)
                            cv2.waitKey(10)
        # Find the bag end
        
        # print(f'Stage 5, time = {(time.time() - start_time) * 1000: .1f} ms.')
        if self.visualization_en:
            # draw ids
            for track in tracks:
                # print('Track status: ', track.is_confirmed(), track.time_since_update)
                if not track.is_confirmed():# or track.time_since_update > 200:
                    continue
                y1, x1, y2, x2 = track.to_ltrb()
                cls, conf = track.det_class, 0 if track.det_conf is None else track.det_conf
                cv2.putText(
                    img=vis_img,
                    text="%s, ID: %s, age: %d" % (cls, track.track_id, track.time_since_update),
                    org=(int((y1 + y2) // 2), int((x1 + x2) // 2)),
                    # org=(y1, x1 + 25),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.7,
                    color=(255, 0, 0),
                    thickness=2
                )
                # print('Track mean: ')
                # print(track.mean)
                print('Track avg cov: ')
                # print(track.covariance)
                print(np.diag(track.covariance).mean())

            # Draw fps
            time_now = get_time_sec(self.get_clock().now())
            fps = 1 / (time_now - self.fps_time_rec + 1e-6) if self.fps_time_rec is not None else 0
            self.fps_time_rec = time_now
            cv2.putText(
                img=vis_img,
                # text="ID: %s" % (track.track_id),
                text="FPS: %.2f" % (fps),
                # org=(int((y1 + y2) // 2), int((x1 + x2) // 2) + 25),
                org=(0, 25),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.9,
                color=(0, 0, 255),
                thickness=2
            )

            cv2.imshow('color image', vis_img.astype(np.uint8))
            cv2.imshow('depth image', vis_depth_img.astype(np.uint8))
            cv2.waitKey(10)
            # plt.subplot(1, 2, 1)
            # plt.imshow(vis_img.astype(np.uint8))
            # plt.subplot(1, 2, 2)
            # plt.imshow(vis_depth_img.astype(np.uint8))
            # plt.show()
        # print(f'Stage 6, time = {(time.time() - start_time) * 1000: .1f} ms.')

        response.status = 0
        return response, reg_info, ret_img
        # return response, None
    

    def detection_srv_callback(self, request, response):
        self.visualization_en = self.get_parameter('visualization_en').get_parameter_value().bool_value

        # if self.recent_sync_data is None and False:
        if self.recent_sync_data is None:
            response.header = Header(stamp=self.get_clock().now().to_msg())
            response.status = 1
            response.objects = []
            return response
        
        register_person = ('register_person' in request.flags)
        match_obj_cls = ('match_object' in request.flags)
        request_img = ('request_image' in request.flags)
        request_segments = ('request_segments' in request.flags)
        find_pointed_object = ('find_pointed_object' in request.flags)

        self.get_logger().info(f'Incoming detection request (register_person = {register_person}).')

        img, points, validmask_pt, msg_header = self.recent_sync_data
        # img = cv2.imread('/home/tinker/vision_test/chocolade/photo_1.jpg')
        # points = np.ones_like(img)
        # validmask_pt = np.ones((img.shape[0], img.shape[1]))
        # msg_header = Header(stamp=self.get_clock().now().to_msg(), frame_id='map')
        # match_obj_cls = True
        objects, reg_info, ret_img = self.detection_segmentation(img, points, validmask_pt,
                                                                 register_person=register_person,
                                                                 match_obj_cls=match_obj_cls,
                                                                 find_pointed_object=find_pointed_object,
                                                                 request_img=request_img,
                                                                 request_segments=request_segments)
        objects.header = msg_header
        
        response.header = objects.header
        response.status = objects.status# or not (register_person and reg_info[0] == 0)
        response.person_id = reg_info[1]
        response.objects = objects.objects

        # print(ret_img[0].shape, ret_img[0].dtype, ret_img[0][:3, :3])
        response.rgb_image = self.bridge.cv2_to_imgmsg(ret_img[0].astype(np.uint8), "bgr8")
        response.depth_image = self.bridge.cv2_to_imgmsg(ret_img[1].astype(np.float32), "32FC1")
        response.segments = [self.bridge.cv2_to_imgmsg(seg.astype(np.uint8), '8UC1') for seg in ret_img[2]]

        return response



def main():
    rclpy.init()
    detection_service = DetectionService()
    rclpy.spin(detection_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()