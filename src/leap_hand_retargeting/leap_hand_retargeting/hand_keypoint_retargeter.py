import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Point
from visualization_msgs.msg import Marker
from scipy.optimize import minimize
import pinocchio as pin
import numpy as np
import sys
from ament_index_python.packages import get_package_share_directory
import os
from abc import abstractmethod

class Hand:
    def __init__(self, urdf_path: str, joint_limits):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.joint_limits = np.zeros_like(joint_limits)
        self.last_q = np.zeros(self.model.nq)

        assert len(joint_limits) == self.model.nq

        # 跳过 universe，收集(name, q_index)元组
        joint_info = []
        for idx, joint in enumerate(self.model.joints):
            if idx == 0:  # 跳过 universe
                continue
            joint_name = self.model.names[idx]
            q_index = joint.idx_q
            joint_info.append((joint_name, q_index))

        # 按关节名字升序排序（如果是数字字符串，转为int排序更合理）
        joint_info_sorted = sorted(joint_info, key=lambda x: int(x[0]))
        # 根据舵机ID获得对应q_index motor id -> q index
        self.q_indices_sorted = [q_index for joint_id, q_index in joint_info_sorted]
        self.urdf_joint_names = [joint_id for joint_id, _ in joint_info_sorted]

        # 舵机ID顺序重排到q顺序
        self.joint_limits = self.default_to_urdf(joint_limits)
        # print(self.q_indices_sorted)
    def default_to_urdf(self, default):
        urdf = np.zeros_like(default)
        urdf[self.q_indices_sorted] = default
        return urdf
    def urdf_to_default(self, urdf):
        return urdf[self.q_indices_sorted]

    def get_joint_names(self):
        """
        生成带有手指名称和关节类型的关节名称列表。
        关节顺序为:index, middle, ring, thumb;每指4个关节:MCP, PIP, DIP, TIP。
        返回长度为16的列表,例如:index_mcp_right, ..., thumb_tip_right
        目前已经弃用，因为不符合urdf中的关节名称。
        """
        fingers = ['index', 'middle', 'ring', 'thumb']
        joints = ['mcp', 'pip', 'dip', 'tip']

        joint_names = []
        for finger in fingers:
            for joint in joints:
                joint_names.append(f"{finger}_{joint}")
        return joint_names
    
    def get_urdf_joint_names(self):
        """
        返回与 URDF 中使用的关节名称一致的关节名列表。
        顺序为：index, middle, ring, thumb；每指 4 个关节。
        """
        # urdf_joint_names = [
        #     # index
        #     '0', '1', '2', '3',
        #     # middle
        #     '4', '5', '6', '7',
        #     # ring
        #     '8', '9', '10', '11',
        #     # thumb
        #     '12', '13', '14', '15'
        # ]
        return self.urdf_joint_names

    def get_urdf_tip_names(self):
        """
        返回 URDF 中的指尖 frame 名称列表。
        顺序为：thumb, index, middle, ring。
        """
        return [
            'thumb_tip_head', 'index_tip_head', 'middle_tip_head', 'ring_tip_head'
        ]
        
    @staticmethod
    def huber(x, delta=0.03):
        abs_x = np.abs(x)
        if abs_x < delta:
            return 0.5 * x * x
        else:
            return delta * (abs_x - 0.5 * delta)
        
    @abstractmethod
    def loss(self, q, target_pos: dict, 
         project_dist=0.05, escape_dist=0.05, 
         weight_pos=1.0, weight_dist=5.0, 
         reg_weight=0.01,
         pinch_threshold=0.05, pinch_weight=60.0):
        pass
    
    def parse_target_pos(self, target_pos):
        """
        解析目标位置，返回字典格式。
        输入: target_pos (4, 3) numpy 数组，四个指尖位置。
        输出: dict {'thumb_tip_head': [...], 'index_tip_head': [...], 
                    'middle_tip_head': [...], 'ring_tip_head': [...]}
        """
        assert target_pos.shape == (4, 3), "target_pos must be of shape (4, 3)"
        tip_names = self.get_urdf_tip_names()
        return {
            tip_names[0]: target_pos[0],
            tip_names[1]: target_pos[1],
            tip_names[2]: target_pos[2],
            tip_names[3]: target_pos[3]
        }

    def retarget(self, target_pos):
        """
        target_pos_dict: dict 
        {'index_tip_head': [...], 'thumb_tip_head': [...], 
        'middle_tip_head': [...], 'ring_tip_head': [...]}
        target_pos: (4, 3)
        四个指尖位置, 顺序为thumb, index, middle, ring
        """
        target_pos_dict = dict()

        target_pos_dict = self.parse_target_pos(target_pos)
        q0 = self.last_q.copy()
        bounds = self.joint_limits

        res = minimize(
            lambda q: self.loss(q, target_pos_dict),
            q0,
            method='L-BFGS-B',
            bounds=bounds,
            options={'ftol': 1e-6, 'maxiter': 100}
        )
        # add ema filter
        # 保存真实 IK 解
        self.last_q = res.x.copy()

        # adaptive EMA
        if not hasattr(self, 'filtered_q'):
            self.filtered_q = self.last_q.copy()
        else:
            delta_norm = np.linalg.norm(self.last_q - self.filtered_q)
            # 参数，可调：
            k = 50.0   # 控制 sigmoid 的斜率
            alpha_min = 0.05
            alpha_max = 0.95
            # alpha 随 delta 增大而趋近 alpha_max（即减少平滑）
            alpha = alpha_min + (alpha_max - alpha_min) * (1.0 - np.exp(-k * delta_norm))
            self.filtered_q = alpha * self.last_q + (1 - alpha) * self.filtered_q

        return self.urdf_to_default(self.filtered_q)

class HKVM_Hand(Hand):
    def __init__(self, urdf_path: str, joint_limits):
        super().__init__(urdf_path, joint_limits)

        self.tip_names = self.get_urdf_tip_names()
        self.wrist_name = 'palm_lower'  # 模型中的手腕 frame 名称

        # 指尖间向量（相对方向约束）
        self.vector_pairs = [
            ('thumb_tip_head', 'index_tip_head'),
            ('thumb_tip_head', 'middle_tip_head'),
            ('thumb_tip_head', 'ring_tip_head'),
            ('index_tip_head', 'middle_tip_head'),
            ('index_tip_head', 'ring_tip_head'),
            ('middle_tip_head', 'ring_tip_head')
        ]

    def loss(self, q, target_pos: dict,
             weight_vec=1.0, weight_abs=1.0, reg_weight=0.01,
             pinch_threshold=0.08, pinch_weight=200.0):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

        # 获取模型的关键点位置
        fingertip_poses = {}
        for name in self.tip_names:
            frame_id = self.model.getFrameId(name)
            fingertip_poses[name] = self.data.oMf[frame_id].translation

        # 获取模型手腕位置
        wrist_id = self.model.getFrameId(self.wrist_name)
        wrist_pos = self.data.oMf[wrist_id].translation

        loss_sum = 0.0

        # 向量差异损失（指尖间）
        for origin, target in self.vector_pairs:
            vec_model = fingertip_poses[target] - fingertip_poses[origin]
            vec_target = target_pos[target] - target_pos[origin]
            diff = vec_model - vec_target
            loss_sum += weight_vec * np.linalg.norm(diff)

        # 绝对位置差异（手腕到指尖）
        # for name in self.tip_names:
        #     if name == 'thumb_tip_head':
        #         vec_model = fingertip_poses[name] - wrist_pos
        #         vec_target = target_pos[name] - np.array([0.0, 0.0, 0.0])  # 人手手腕默认为原点
        #         diff = vec_model - vec_target
        #         loss_sum += weight_abs * np.linalg.norm(diff)

        # pinch 加强约束（拇指靠近）
        thumb_name = 'thumb_tip_head'
        for other in ['index_tip_head', 'middle_tip_head', 'ring_tip_head']:
            dist = np.linalg.norm(target_pos[thumb_name] - target_pos[other])
            if dist < pinch_threshold:
                model_dist = np.linalg.norm(fingertip_poses[thumb_name] - fingertip_poses[other])
                loss_sum += pinch_weight * self.huber(model_dist)

        # 正则项（防止过度跳变）
        loss_sum += reg_weight * np.linalg.norm(q - self.last_q)

        return loss_sum

class HandRetargetNode(Node):
    def __init__(self, hand_side, mode, urdf_path, joint_limits):
        super().__init__(f'hand_keypoint_retargeter_{hand_side}')

        self.mode = mode
        self.hand_model = HKVM_Hand(urdf_path=urdf_path, joint_limits=joint_limits)
        self.joint_names = self.hand_model.get_urdf_joint_names()
        self.hand_side = hand_side
        # EMA滤波参数
        self.alpha = 0.2#1.0
        self.filtered_joints = None
        # 初始化最新目标位置和头部信息
        self.latest_target_pos = None
        self.latest_header = None
        # 初始化最新人类指尖坐标和灵巧手指尖坐标
        self.latest_human_pos = None
        self.latest_leap_pos = None
        # 创建发布者
        self.publisher = self.create_publisher(JointState, f"/leap_hand_{hand_side}/target_joint", 10)
        # 创建订阅者
        self.subscription = self.create_subscription(
            PoseArray,
            f"/human_hand_{hand_side}/target_pos",
            self.target_pose_callback,
            10
        )

        # 启动服务客户端
        self.get_logger().info(f"Hand retargeting node started in default mode for {hand_side} hand.")
        self.timer = self.create_timer(1/60, self.timer_callback)  # 60Hz

    def target_pose_callback(self, msg):
        """
        接收(4,3)的人类手指指尖坐标PoseArray消息
        """
        self.latest_target_pos = np.array([[p.position.x, p.position.y, p.position.z] for p in msg.poses])
        self.latest_header = msg.header

    def publish_joint_state(self, joints):
        joint_state_msg = JointState()
        joint_state_msg.header = self.latest_header
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = joints.tolist()

        self.publisher.publish(joint_state_msg)
        # self.get_logger().info(f'Published joint state: {joint_state_msg.position}')

    def EMA_filter(self, qpos):
        """
        使用指数移动平均滤波器对关节角度进行平滑处理。
        """
        if self.filtered_joints is None:
            self.filtered_joints = qpos.copy()
        else:
            self.filtered_joints = self.alpha * qpos + (1 - self.alpha) * self.filtered_joints
        return self.filtered_joints

    def timer_callback(self):
        if self.latest_target_pos is None:
            return  # no data

        target_pos = self.latest_target_pos.copy()
        self.latest_target_pos = None  # clear after copy

        # Step 1: Estimate qpos
        qpos = self.hand_model.retarget(target_pos)
        # Step 2: Publish joint state
        self.filtered_joints = self.EMA_filter(qpos) 
        self.publish_joint_state(self.filtered_joints)
    
def main(args=None):
    rclpy.init(args=args)

    # 默认值
    hand_side = "right"
    mode = "default"

    # 参数合法性检查
    if len(sys.argv) < 2 or sys.argv[1] not in ["left", "right"]:
        print("Usage: ros2 run your_package your_node left|right")
        sys.exit(1)

    hand_side = sys.argv[1]

    print(f"Hand side: {hand_side}, Mode: {mode}")

    package_name = 'leap_hand_retargeting'
    urdf_filename = f'urdf/leap_hand/leap_hand_right_converted.urdf'

    package_share_dir = get_package_share_directory(package_name)
    urdf_path = os.path.join(package_share_dir, urdf_filename)

    joint_limits = [
        (-0.5, 0.3),
        (0.0, 1.3),
        (0.0, 1.6),
        (0.0, 2.0),
        (-0.3, 0.3),
        (0.0, 1.3),
        (0.0, 1.6),
        (0.0, 2.0),
        (-0.3, 1.0),
        (0.0, 1.3),
        (0.0, 1.6),
        (0.0, 2.0),
        (0.0, 1.5),
        (0.0, 1.6),
        (0.0, 0.6),
        (0.0, 1.7),
    ] # normal
    node = HandRetargetNode(hand_side, mode, urdf_path, joint_limits)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()