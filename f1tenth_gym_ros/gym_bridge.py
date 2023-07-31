import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gym
import numpy as np
from transforms3d import euler
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class GymBridge(Node):
    def __init__(self):
        super().__init__("gym_bridge")

        self.declare_parameter("ego_namespace")
        self.declare_parameter("ego_odom_topic")
        self.declare_parameter("ego_scan_topic")
        self.declare_parameter("ego_drive_topic")
        self.declare_parameter("scan_distance_to_base_link")
        self.declare_parameter("scan_fov")
        self.declare_parameter("scan_beams")
        self.declare_parameter("map_path")
        self.declare_parameter("map_img_ext")
        self.declare_parameter("num_agent")

        self.num_agents = self.get_parameter("num_agent").value

        # env backend
        self.env = gym.make(
            "f110_gym:f110-v0",
            map=self.get_parameter("map_path").value,
            map_ext=self.get_parameter("map_img_ext").value,
            num_agents=self.num_agents,
        )

        ego_scan_topic = self.get_parameter("ego_scan_topic").value
        ego_drive_topic = self.get_parameter("ego_drive_topic").value
        scan_fov = self.get_parameter("scan_fov").value
        scan_beams = self.get_parameter("scan_beams").value
        self.angle_min = -scan_fov / 2.0
        self.angle_max = scan_fov / 2.0
        self.angle_inc = scan_fov / scan_beams
        self.ego_namespace = self.get_parameter("ego_namespace").value
        ego_odom_topic = "/" + self.get_parameter("ego_odom_topic").value
        self.scan_distance_to_base_link = self.get_parameter(
            "scan_distance_to_base_link"
        ).value

        self.pose_reset_arr = np.zeros((self.num_agents, 3))
        for i in range(len(self.pose_reset_arr)):
            self.pose_reset_arr[i][0] += i
            # self.pose_reset_arr[i][1] += i
            # self.pose_reset_arr[i][2] -= 2.0

        self.obs, _, self.done, _ = self.env.reset(self.pose_reset_arr)

        # sim physical step timer
        self.drive_timer = self.create_timer(0.01, self.drive_timer_callback)
        # topic publishing timer
        self.timer = self.create_timer(0.004, self.timer_callback)

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # topics need to be separate for each car
        self.scan_topics = [f"{ego_scan_topic}{i + 1}" for i in range(self.num_agents)]
        self.drive_topics = [
            f"{ego_drive_topic}{i + 1}" for i in range(self.num_agents)
        ]
        self.odom_topics = [
            f"{self.ego_namespace}{i + 1}{ego_odom_topic}"
            for i in range(self.num_agents)
        ]

        # publishers and subscribers
        self.scan_publishers = []
        self.odom_publishers = []
        self.drive_subscribers = []
        self.drive_msgs = np.zeros(
            (self.num_agents, 2)
        )  # 2 for steering angle and speed
        for i in range(self.num_agents):
            ego_scan_pub = self.create_publisher(LaserScan, self.scan_topics[i], 10)
            ego_odom_pub = self.create_publisher(Odometry, self.odom_topics[i], 10)
            ego_drive_sub = self.create_subscription(
                AckermannDriveStamped, self.drive_topics[i], self.drive_callback, 10
            )
            self.scan_publishers.append(ego_scan_pub)
            self.odom_publishers.append(ego_odom_pub)
            self.drive_subscribers.append(ego_drive_sub)
        self.ego_drive_published = False

        # Initialize InteractiveMarkerServer
        self.server = InteractiveMarkerServer(node=self, namespace="/positions")
        self.clicked_car = 0  # Initialize the clicked car variable

        for i in range(self.num_agents):
            marker = InteractiveMarker()
            marker.header.frame_id = "map"
            marker.name = f"car_{i + 1}_marker"
            marker.description = f"Car {i + 1}"
            # Create a control that displays a box marker
            box_marker = Marker()
            box_marker.type = Marker.CUBE
            box_marker.pose.position.x = -5.0 - i
            box_marker.pose.position.y = -5.0 
            box_marker.pose.position.z = 0.2 
            box_marker.scale.x = 0.5
            box_marker.scale.y = 0.5
            box_marker.scale.z = 0.2
            box_marker.color.r = 0.8
            box_marker.color.g = 0.3
            box_marker.color.b = 0.2
            box_marker.color.a = 1.0

            control = InteractiveMarkerControl()
            control.markers.append(box_marker)
            control.interaction_mode = InteractiveMarkerControl.BUTTON
            control.always_visible = True
            marker.controls.append(control)

            self.server.insert(marker=marker, feedback_callback=self.marker_click_callback)

        self.server.applyChanges()

        self.pose_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.pose_reset_callback, 10
        )

    def drive_callback(self, drive_msg):
        car_number = int(drive_msg.header.frame_id.split("car")[1].split("/")[0]) - 1
        self.drive_msgs[car_number][0] = drive_msg.drive.steering_angle
        self.drive_msgs[car_number][1] = drive_msg.drive.speed
        self.ego_drive_published = True

    def drive_timer_callback(self):
        if self.ego_drive_published:
            self.obs, _, self.done, _ = self.env.step(self.drive_msgs)

    def timer_callback(self):
        for i in range(self.num_agents):
            ego_namespace = self.ego_namespace + str(i + 1)
            ts = self.get_clock().now().to_msg()
            # publish scans
            scan = LaserScan()
            scan.header.stamp = ts
            scan.header.frame_id = ego_namespace + "/laser"
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_inc
            scan.range_min = 0.0
            scan.range_max = 30.0
            scan.ranges = list(self.obs["scans"][i])
            self.scan_publishers[i].publish(scan)

            # publish transforms and odom
            self._publish_odom(ts)
            self._publish_transforms(ts)
            self._publish_laser_transforms(ts)
            self._publish_wheel_transforms(ts)

    def marker_click_callback(self, feedback):
        # Extract the car number from the marker name
        self.clicked_car = int(feedback.marker_name.split("_")[1]) - 1
        self.get_logger().info("Clicked car: " + str(self.clicked_car + 1))

    def pose_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes="rxyz")

        # FIXME: Each time an agent's pose is updated, all the agents stop for a moment
        # Keep all other agents in the same place
        for i in range(self.num_agents):
            self.pose_reset_arr[i][0] = self.obs["poses_x"][i]
            self.pose_reset_arr[i][1] = self.obs["poses_y"][i]
            self.pose_reset_arr[i][2] = self.obs["poses_theta"][i]
            
        # Update chosen agent's pose
        self.pose_reset_arr[self.clicked_car][0] = rx
        self.pose_reset_arr[self.clicked_car][1] = ry
        self.pose_reset_arr[self.clicked_car][2] = rtheta
        
        self.obs, _, self.done, _ = self.env.reset(np.array(self.pose_reset_arr))

    def _publish_odom(self, ts):
        for i in range(self.num_agents):
            ego_namespace = self.ego_namespace + str(i + 1)
            ego_odom = Odometry()
            ego_odom.header.stamp = ts
            ego_odom.header.frame_id = "map"
            ego_odom.child_frame_id = ego_namespace + "/base_link"
            ego_odom.pose.pose.position.x = self.obs["poses_x"][i]
            ego_odom.pose.pose.position.y = self.obs["poses_y"][i]
            ego_quat = euler.euler2quat(0.0, 0.0, self.obs["poses_theta"][i], axes="sxyz")
            ego_odom.pose.pose.orientation.x = ego_quat[1]
            ego_odom.pose.pose.orientation.y = ego_quat[2]
            ego_odom.pose.pose.orientation.z = ego_quat[3]
            ego_odom.pose.pose.orientation.w = ego_quat[0]
            ego_odom.twist.twist.linear.x = self.obs["linear_vels_x"][i] + 1.0
            ego_odom.twist.twist.linear.y = self.obs["linear_vels_y"][i]
            ego_odom.twist.twist.angular.z = self.obs["ang_vels_z"][i]
            self.odom_publishers[i].publish(ego_odom)

    def _publish_transforms(self, ts):
        for i in range(self.num_agents):
            ego_namespace = self.ego_namespace + str(i + 1)
            ego_t = Transform()
            ego_t.translation.x = self.obs["poses_x"][i]
            ego_t.translation.y = self.obs["poses_y"][i]
            ego_t.translation.z = 0.0
            ego_quat = euler.euler2quat(0.0, 0.0, self.obs["poses_theta"][i], axes="sxyz")
            ego_t.rotation.x = ego_quat[1]
            ego_t.rotation.y = ego_quat[2]
            ego_t.rotation.z = ego_quat[3]
            ego_t.rotation.w = ego_quat[0]
            ego_ts = TransformStamped()
            ego_ts.transform = ego_t
            ego_ts.header.stamp = ts
            ego_ts.header.frame_id = "map"
            ego_ts.child_frame_id = ego_namespace + "/base_link"
            self.br.sendTransform(ego_ts)

    def _publish_wheel_transforms(self, ts):
        for i in range(self.num_agents):
            ego_namespace = self.ego_namespace + str(i + 1)
            ego_wheel_ts = TransformStamped()
            ego_wheel_quat = euler.euler2quat(
                0.0, 0.0, self.obs["ang_vels_z"][i], axes="sxyz"
            )
            ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
            ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
            ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
            ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
            ego_wheel_ts.header.stamp = ts
            ego_wheel_ts.header.frame_id = ego_namespace + "/front_left_hinge"
            ego_wheel_ts.child_frame_id = ego_namespace + "/front_left_wheel"
            self.br.sendTransform(ego_wheel_ts)
            ego_wheel_ts.header.frame_id = ego_namespace + "/front_right_hinge"
            ego_wheel_ts.child_frame_id = ego_namespace + "/front_right_wheel"
            self.br.sendTransform(ego_wheel_ts)

    def _publish_laser_transforms(self, ts):
        for i in range(self.num_agents):
            ego_namespace = self.ego_namespace + str(i + 1)
            ego_scan_ts = TransformStamped()
            ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            ego_scan_ts.transform.translation.z = 0.04 + 0.1 + 0.025
            ego_scan_ts.transform.rotation.w = 1.0
            ego_scan_ts.header.stamp = ts
            ego_scan_ts.header.frame_id = ego_namespace + "/base_link"
            ego_scan_ts.child_frame_id = ego_namespace + "/laser"
            self.br.sendTransform(ego_scan_ts)


def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()

    try:
        rclpy.spin(gym_bridge)
    except KeyboardInterrupt:
        print("Interrupt signal detected... Killing node")
    gym_bridge.server.clear()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
