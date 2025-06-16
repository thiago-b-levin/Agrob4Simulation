from controller import Supervisor
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import numpy as np


def find_robot_node_by_name(supervisor, target_name):
    root = supervisor.getRoot()
    children_field = root.getField("children")
    for i in range(children_field.getCount()):
        child = children_field.getMFNode(i)
        if child.getTypeName() == "Robot":
            name_field = child.getField("name")
            if name_field and name_field.getSFString() == target_name:
                return child
    return None


class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('ground_truth_publisher')

        # Declare and read parameter
        self.declare_parameter('robot_name', 'ModelE')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

        self.get_logger().info(f"Using robot name: {robot_name}")
        
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())

        # Wait for the robot to appear
        self.robot_node = None
        max_attempts = 200
        for attempt in range(max_attempts):
            self.robot_node = find_robot_node_by_name(self.supervisor, robot_name)
            if self.robot_node:
                break
            self.get_logger().warn(f"[GT] Waiting for robot '{robot_name}' to appear... ({attempt + 1}/{max_attempts})")
            self.supervisor.step(self.timestep)

        if not self.robot_node:
            self.get_logger().error(f"Robot '{robot_name}' not found after {max_attempts} attempts. Exiting.")
            exit(1)

        self.get_logger().info(f"[UPdatedGT] Robot '{robot_name}' found, starting publisher.")

        self.translation_field = self.robot_node.getField("translation")
        self.rotation_field = self.robot_node.getField("rotation")

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_gt_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.latest_odom = None
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Frame IDs - note the change of naming for clarity
        self.odom_frame = "odom"
        self.world_frame = "world"
        self.base_frame = "base_link"

        self.timer = self.create_timer(self.timestep / 1000.0, self.step)
        self.total_distance = 0.0
        self.last_position = None


    def odom_callback(self, msg):
        
        self.latest_odom = msg
       

    def step(self):
        self.supervisor.step(self.timestep)
        if self.latest_odom:
            self.publish_ground_truth()

    def publish_ground_truth(self):
        now = self.get_clock().now().to_msg()

        # === Ground Truth Pose: world → base_link ===
        position = self.translation_field.getSFVec3f()
        orientation = self.rotation_field.getSFRotation()
        # Convert Webots axis-angle rotation to quaternion
        gt_quat = tf_transformations.quaternion_about_axis(orientation[3], orientation[:3])
        
        # Create transformation matrix: world → base_link
        world_to_base = tf_transformations.quaternion_matrix(gt_quat)
        world_to_base[0:3, 3] = position

        # === Estimated Odometry: odom → base_link ===
        odom_pos = self.latest_odom.pose.pose.position
        odom_ori = self.latest_odom.pose.pose.orientation
        odom_quat = [odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w]
        
        # Create transformation matrix: odom → base_link
        odom_to_base = tf_transformations.quaternion_matrix(odom_quat)
        odom_to_base[0:3, 3] = [odom_pos.x, odom_pos.y, odom_pos.z]

        # === Compute odom → world transformation ===
        # First, get base_link → world (inverse of world → base_link)
        base_to_world = tf_transformations.inverse_matrix(world_to_base)
        
        # Now combine: odom → base_link → world
        # Note: The order is important! First apply odom→base, then base→world
        odom_to_world = np.matmul(odom_to_base, base_to_world)
        
        # Extract translation and rotation from the transformation matrix
        trans = tf_transformations.translation_from_matrix(odom_to_world)
        quat = tf_transformations.quaternion_from_matrix(odom_to_world)

        # === Publish TF: odom → world ===
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = self.odom_frame
        tf_msg.child_frame_id = self.world_frame
        tf_msg.transform.translation.x = trans[0]
        tf_msg.transform.translation.y = trans[1]
        tf_msg.transform.translation.z = trans[2]
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(tf_msg)

        # === Publish Ground Truth Odometry (world → base_link) ===
        gt_odom = Odometry()
        gt_odom.header.stamp = now
        gt_odom.header.frame_id = self.world_frame  # 'world'
        gt_odom.child_frame_id = self.base_frame
        gt_odom.pose.pose.position.x = position[0]
        gt_odom.pose.pose.position.y = position[1]
        gt_odom.pose.pose.position.z = position[2]
        gt_odom.pose.pose.orientation.x = gt_quat[0]
        gt_odom.pose.pose.orientation.y = gt_quat[1]
        gt_odom.pose.pose.orientation.z = gt_quat[2]
        gt_odom.pose.pose.orientation.w = gt_quat[3]
        self.odom_gt_pub.publish(gt_odom)
        
        # Track distance traveled
        if self.last_position is not None:
            dx = position[0] - self.last_position[0]
            dy = position[1] - self.last_position[1]
            dz = position[2] - self.last_position[2]
            step_distance = (dx**2 + dy**2 + dz**2) ** 0.5
            self.total_distance += step_distance

        self.last_position = position.copy()
        self.get_logger().info(f"\n Total Distance Traveled: {self.total_distance:.3f} meters \n")

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()