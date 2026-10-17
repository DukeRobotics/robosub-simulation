"""Publish Stonefish measurements on Crush's real driver topics for the robosub-ros2 stack."""

import math

import rclpy
from custom_msgs.msg import ThrusterAllocs
from custom_msgs.srv import SonarSweepRequest
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, FluidPressure, Image, Imu
from std_msgs.msg import Float64, Float64MultiArray
from stonefish_ros2.msg import DVL

from robosub_simulation.allocation import rotor_setpoints
from robosub_simulation.robot import load_robot_config
from robosub_simulation.sensors import COVARIANCE, Depth, Gyro, jpeg, quaternion, vectornav_orientation

RAW = '/simulation/crush/'
# ponytail: constant pack voltage; only the real thrusters.py reads it.
VOLTAGE = 16.0


def orientation(values):
    x, y, z, w = values
    return Quaternion(x=x, y=y, z=z, w=w)


class SimAdapter(Node):
    def __init__(self):
        super().__init__('sim_adapter')
        config = load_robot_config(self.declare_parameter('robot_config', '').value)
        sensors = config['sensors']
        self.mount = quaternion(sensors['imu']['orientation'])
        self.body = None
        self.depth = Depth()
        self.gyro = Gyro(sensors['gyro']['rate'])
        self.imu = self.create_publisher(Imu, '/vectornav/imu', 10)
        self.depth_publisher = self.create_publisher(PoseWithCovarianceStamped, '/sensors/depth', 10)
        self.dvl = self.create_publisher(Odometry, '/sensors/dvl/odom', 10)
        self.gyro_rate = self.create_publisher(TwistWithCovarianceStamped, '/sensors/gyro/angular_velocity/twist', 50)
        self.gyro_yaw = self.create_publisher(PoseWithCovarianceStamped, '/sensors/gyro/angular_position/pose', 50)
        self.voltage = self.create_publisher(Float64, '/sensors/voltage', 1)
        self.create_subscription(Odometry, RAW + 'ground_truth', self.ground_truth_received, 10)
        self.create_subscription(Imu, RAW + 'imu', self.imu_received, 10)
        self.create_subscription(Imu, RAW + 'gyro', self.gyro_received, 50)
        self.create_subscription(FluidPressure, RAW + 'pressure', self.pressure_received, 10)
        self.create_subscription(DVL, RAW + 'dvl', self.dvl_received, 10)
        self.create_timer(1.0, lambda: self.voltage.publish(Float64(data=VOLTAGE)))
        self.create_service(SonarSweepRequest, '/sonar/request', self.sonar_requested)
        # Stands in for offboard_comms thrusters.py and the ESCs. Run with enable_teleop:=false so
        # robosub-ros2 controls is the only source of setpoints; thruster watchdogs stop the rotors
        # when controls is disabled and stops publishing.
        self.thruster_count = len(config['thrusters'])
        self.setpoints = self.create_publisher(Float64MultiArray, RAW + 'thruster_setpoints', 1)
        self.create_subscription(ThrusterAllocs, '/controls/thruster_allocs', self.allocations_received, 1)
        # Cameras exist only when Stonefish renders (not headless).
        for name, camera in config['cameras'].items():
            publisher = self.create_publisher(CompressedImage, camera['topic'], 1)
            self.create_subscription(
                Image,
                f'{RAW}camera/{name}/image_color',
                lambda message, frame=camera['frame'], publisher=publisher: self.camera_received(
                    message, frame, publisher
                ),
                1,
            )

    def allocations_received(self, message):
        # Sim thrusters share the controls config order, so no reordering is needed.
        if len(message.allocs) != self.thruster_count or not all(map(math.isfinite, message.allocs)):
            self.get_logger().warning(f'Ignoring thruster allocations: {list(message.allocs)}', throttle_duration_sec=5)
            return
        self.setpoints.publish(Float64MultiArray(data=rotor_setpoints(message.allocs).tolist()))

    def sonar_requested(self, request, response):
        # ponytail: sees nothing yet; a Stonefish MSIS sweep through sonar.py's pipeline replaces this.
        # Validation follows sonar.py perform_sonar_request.
        response.success = request.start_angle <= request.end_angle and request.distance_of_scan >= 0
        response.message = 'Simulated sonar: no object detected' if response.success else 'Invalid sweep'
        response.is_object = False
        return response

    def camera_received(self, message, frame, publisher):
        output = CompressedImage()
        # usb_camera.py never fills the header; the stamp and frame are harmless extras here.
        output.header.stamp = message.header.stamp
        output.header.frame_id = frame
        output.format = 'jpg'
        output.data = jpeg(message.width, message.height, message.data)
        publisher.publish(output)

    def ground_truth_received(self, message):
        attitude = message.pose.pose.orientation
        self.body = (attitude.x, attitude.y, attitude.z, attitude.w)

    def imu_received(self, message):
        # Upstream stonefish_ros2 builds the IMU quaternion with Bullet's Y-X-Z Euler order, so
        # attitude comes from the exact link orientation and the mount instead.
        if self.body is None:
            return
        output = Imu()
        output.header.stamp = message.header.stamp
        output.header.frame_id = 'imu'
        # use_enu: false passes rates and specific force through in the VN100's FRD body axes.
        output.angular_velocity = message.angular_velocity
        output.linear_acceleration = message.linear_acceleration
        output.orientation = orientation(vectornav_orientation(self.body, self.mount))
        for covariance in (
            output.orientation_covariance,
            output.angular_velocity_covariance,
            output.linear_acceleration_covariance,
        ):
            covariance[0] = covariance[4] = covariance[8] = COVARIANCE
        self.imu.publish(output)

    def gyro_received(self, message):
        rate, yaw = self.gyro.update(message.angular_velocity.z)
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = message.header.stamp
        twist.header.frame_id = 'gyro'
        twist.twist.twist.angular.z = rate
        twist.twist.covariance[35] = COVARIANCE
        self.gyro_rate.publish(twist)
        pose = PoseWithCovarianceStamped()
        pose.header = twist.header
        pose.pose.pose.orientation = orientation(yaw)
        pose.pose.covariance[35] = COVARIANCE
        self.gyro_yaw.publish(pose)

    def pressure_received(self, message):
        z = self.depth.update(message.fluid_pressure)
        if z is None:
            return
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = message.header.stamp
        pose.header.frame_id = 'odom'
        pose.pose.pose.position.z = z
        pose.pose.pose.orientation.w = 1.0
        pose.pose.covariance[14] = COVARIANCE
        self.depth_publisher.publish(pose)

    def dvl_received(self, message):
        # Stonefish reports altitude -1 without bottom lock; the Wayfinder sends NaN and dvl_to_odom drops it.
        if message.altitude < 0:
            return
        odometry = Odometry()
        odometry.header.stamp = message.header.stamp
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'dvl'
        odometry.pose.pose.orientation.w = 1.0
        odometry.twist.twist.linear = message.velocity
        for index in (0, 7, 14):
            odometry.twist.covariance[index] = COVARIANCE
        self.dvl.publish(odometry)


def main():
    rclpy.init()
    node = SimAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
