# adapted from https://yzqin.github.io/anyteleop/
"""
手部关键点检测模块
"""
import mediapipe as mp
import mediapipe.framework as framework
import numpy as np
from mediapipe.framework.formats import landmark_pb2
from mediapipe.python.solutions import hands_connections
from mediapipe.python.solutions.drawing_utils import DrawingSpec
from mediapipe.python.solutions.hands import HandLandmark
import cv2
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker
import sys

PIPE2LEAP = {
    'thumb_tip_head': 4,
    'index_tip_head': 8,
    'middle_tip_head': 12,
    'ring_tip_head': 16,
}

OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ]
)

OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ]
)

# 你可以根据自己的骨架连线关系定义 connections
HAND_CONNECTIONS = [
    (0, 1), (1, 2), (2, 3), (3, 4),      # Thumb
    (0, 5), (5, 6), (6, 7), (7, 8),      # Index
    (0, 9), (9,10), (10,11), (11,12),    # Middle
    (0,13), (13,14), (14,15), (15,16),   # Ring
    (0,17), (17,18), (18,19), (19,20)    # Pinky
]

class SingleHandDetector:
    def __init__(
        self,
        hand_type="Right",
        min_detection_confidence=0.8,
        min_tracking_confidence=0.8,
        selfie=False,
    ):
        self.hand_detector = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self.selfie = selfie
        self.operator2mano = (
            OPERATOR2MANO_RIGHT if hand_type == "Right" else OPERATOR2MANO_LEFT
        )
        inverse_hand_dict = {"Right": "Left", "Left": "Right"}
        self.detected_hand_type = hand_type if selfie else inverse_hand_dict[hand_type]

    @staticmethod
    def draw_skeleton_on_image(
        image, keypoint_2d: landmark_pb2.NormalizedLandmarkList, style="white"
    ):
        if style == "default":
            mp.solutions.drawing_utils.draw_landmarks(
                image,
                keypoint_2d,
                mp.solutions.hands.HAND_CONNECTIONS,
                mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
                mp.solutions.drawing_styles.get_default_hand_connections_style(),
            )
        elif style == "white":
            landmark_style = {}
            for landmark in HandLandmark:
                landmark_style[landmark] = DrawingSpec(
                    color=(255, 48, 48), circle_radius=4, thickness=-1
                )

            connections = hands_connections.HAND_CONNECTIONS
            connection_style = {}
            for pair in connections:
                connection_style[pair] = DrawingSpec(thickness=2)

            mp.solutions.drawing_utils.draw_landmarks(
                image,
                keypoint_2d,
                mp.solutions.hands.HAND_CONNECTIONS,
                landmark_style,
                connection_style,
            )

        return image

    def detect(self, rgb):
        """
        Finds hands in a BGR image.
        """
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        results = self.hand_detector.process(rgb)
        if not results.multi_hand_landmarks:
            return 0, None, None, None

        desired_hand_num = -1
        for i in range(len(results.multi_hand_landmarks)):
            label = results.multi_handedness[i].ListFields()[0][1][0].label
            if label == self.detected_hand_type:
                desired_hand_num = i
                break
        if desired_hand_num < 0:
            return 0, None, None, None

        keypoint_3d = results.multi_hand_world_landmarks[desired_hand_num]
        keypoint_2d = results.multi_hand_landmarks[desired_hand_num]
        num_box = len(results.multi_hand_landmarks)

        # Parse 3d keypoint from MediaPipe hand detector
        keypoint_3d_array = self.parse_keypoint_3d(keypoint_3d)
        keypoint_3d_array = keypoint_3d_array - keypoint_3d_array[0:1, :]
        mediapipe_wrist_rot = self.estimate_frame_from_hand_points(keypoint_3d_array)
        joint_pos = keypoint_3d_array @ mediapipe_wrist_rot @ self.operator2mano
        # joint_pos = keypoint_3d_array @ mediapipe_wrist_rot

        return num_box, joint_pos, keypoint_2d, mediapipe_wrist_rot

    @staticmethod
    def parse_keypoint_3d(
        keypoint_3d: framework.formats.landmark_pb2.LandmarkList,
    ) -> np.ndarray:
        keypoint = np.empty([21, 3])
        for i in range(21):
            keypoint[i][0] = keypoint_3d.landmark[i].x
            keypoint[i][1] = keypoint_3d.landmark[i].y
            keypoint[i][2] = keypoint_3d.landmark[i].z
        return keypoint

    @staticmethod
    def parse_keypoint_2d(
        keypoint_2d: landmark_pb2.NormalizedLandmarkList, img_size
    ) -> np.ndarray:
        keypoint = np.empty([21, 2])
        for i in range(21):
            keypoint[i][0] = keypoint_2d.landmark[i].x
            keypoint[i][1] = keypoint_2d.landmark[i].y
        keypoint = keypoint * np.array([img_size[1], img_size[0]])[None, :]
        return keypoint

    @staticmethod
    def estimate_frame_from_hand_points(keypoint_3d_array: np.ndarray) -> np.ndarray:
        """
        Compute the 3D coordinate frame (orientation only) from detected 3d key points
        :param points: keypoint3 detected from MediaPipe detector. Order: [wrist, index, middle, pinky]
        :return: the coordinate frame of wrist in MANO convention
        """
        assert keypoint_3d_array.shape == (21, 3)
        points = keypoint_3d_array[[0, 5, 9], :]

        # Compute vector from palm to the first joint of middle finger
        x_vector = points[0] - points[2]

        # Normal fitting with SVD
        points = points - np.mean(points, axis=0, keepdims=True)
        u, s, v = np.linalg.svd(points)

        normal = v[2, :]

        # Gram–Schmidt Orthonormalize
        x = x_vector - np.sum(x_vector * normal) * normal
        x = x / np.linalg.norm(x)
        z = np.cross(x, normal)

        # We assume that the vector from pinky to index is similar the z axis in MANO convention
        if np.sum(z * (points[1] - points[2])) < 0:
            normal *= -1
            z *= -1
        frame = np.stack([x, normal, z], axis=1)
        return frame
    
class HandKeypointPublisher(Node):
    def __init__(self, hand_side):
        self.hand_side = hand_side.lower()
        super().__init__(f'hand_keypoint_publisher_{hand_side.lower()}')
        self.topic_prefix = f"human_hand_{self.hand_side}"

        self.publisher_keypoints = self.create_publisher(PoseArray, f"{self.topic_prefix}/keypoints", 10)
        self.publisher_target = self.create_publisher(PoseArray, f"{self.topic_prefix}/target_pos", 10)
        self.marker_pub = self.create_publisher(Marker, f"/human_hand_{hand_side}/target_markers", 10)

        self.cap = cv2.VideoCapture(0)
        # self.detector = SingleHandDetector(hand_type=self.hand_side.capitalize(), selfie=False)
        self.detector = SingleHandDetector(hand_type="Right", selfie=False) # always use right hand detector
        self.prev_joint_pos = None
        # Manually set values
        self.alpha = 0.4
        self.origin_offset = np.array([0.00524088, -0.00119007, 0.05511909])
        self.tip_extension_ratio = 0.3  # 指尖延长比例
        self.scaling = 3.1 # 手部放大比例 pose 2.1

        timer_period = 0.033  # ~30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)
    def extend_fingertips(self, joint_pos):
        """
        延长所有指尖，使其更接近真实指尖位置
        joint_pos: (21, 3)
        """
        extended = joint_pos.copy()

        tip_pairs = {
            4: 3,   # thumb_tip : thumb_dip
            # 8: 7,   # index_tip : index_dip
            # 12:11,  # middle_tip : middle_dip
            # 16:15,  # ring_tip : ring_dip
            # 20:19,  # pinky_tip : pinky_dip
        }

        for tip, dip in tip_pairs.items():
            vec = joint_pos[tip] - joint_pos[dip]
            extended[tip] = joint_pos[tip] + self.tip_extension_ratio * vec

        return extended
    def timer_callback(self):
        success, rgb = self.cap.read()
        if not success:
            self.get_logger().warn("Failed to read camera frame.")
            return
        
        if self.hand_side == 'left':
            rgb = cv2.flip(rgb, 1)  # 左手图像水平翻转
        
        _, joint_pos, keypoint_2d, _ = self.detector.detect(rgb)

        if joint_pos is None:
            return
        
        if self.prev_joint_pos is None:
            filtered_joint_pos = joint_pos.copy()
        else:
            filtered_joint_pos = self.alpha * joint_pos + (1 - self.alpha) * self.prev_joint_pos
        self.prev_joint_pos = filtered_joint_pos.copy()

        # 处理关键点
        filtered_joint_pos -= self.origin_offset
        filtered_joint_pos = self.extend_fingertips(filtered_joint_pos)

        # 放大
        filtered_joint_pos *= self.scaling

        # 发布所有关键点
        pose_array_all = PoseArray()
        pose_array_all.header.stamp = self.get_clock().now().to_msg()
        pose_array_all.header.frame_id = 'base'  # 与 RViz 中 Fixed Frame 一致

        for pos in filtered_joint_pos:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos.tolist()
            pose.orientation.w = 1.0  # 单位四元数
            pose_array_all.poses.append(pose)

        self.publisher_keypoints.publish(pose_array_all)

        # 发布 target_pos
        """
        target_pos: (4, 3)
        四个指尖位置, 顺序为thumb, index, middle, ring
        """
        pose_array_target = PoseArray()
        pose_array_target.header = pose_array_all.header  # 统一 header

        for name, idx in PIPE2LEAP.items():
            pos = filtered_joint_pos[idx]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = pos.tolist()
            pose.orientation.w = 1.0
            pose_array_target.poses.append(pose)

        self.publisher_target.publish(pose_array_target)

        # 发布可视化 Marker
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base'  # 与 RViz 中 Fixed Frame 一致
        marker.ns = "target_fingertips"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.005
        marker.scale.y = 0.005
        marker.scale.z = 0.005
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.points = []
        for pose in pose_array_target.poses:
            point = Point()
            point.x = pose.position.x
            point.y = pose.position.y
            point.z = pose.position.z
            marker.points.append(point)

        self.marker_pub.publish(marker)


        # 可选择可视化关键点
        rgb = self.detector.draw_skeleton_on_image(rgb, keypoint_2d, style="default")
        cv2.imshow("keypoints", rgb)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2 or sys.argv[1] not in ['left', 'right']:
        print("Usage: ros2 run human_hand_detector hand_keypoint_publisher.py [left|right]")
        return

    node = HandKeypointPublisher(sys.argv[1])
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
