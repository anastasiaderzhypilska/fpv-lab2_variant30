import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.qos import qos_profile_sensor_data

from .flight_test_node import FlightTestNode
from .flight_state import FlightState


class FlightVariant30(FlightTestNode):
    DX = -6.4
    DY = -0.5
    TOLERANCE = 0.3
    MAX_SPEED = 0.7
    KP = 0.5
    FLIGHT_TIMEOUT = 120.0

    def __init__(self):
        super().__init__()

        self.velocity_pub = self.create_publisher(
            TwistStamped,
            "/ap/cmd_vel",
            qos_profile_sensor_data
        )

        self.start_position = None
        self.target_position = None
        self.navigation_started = None
        self.target_reached = False
        self.last_log_time = 0.0

        self.get_logger().info(
            "Variant 30: DX=-6.4 m, DY=-0.5 m"
        )

    def publish_velocity(self, vx, vy):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = 0.0

        self.velocity_pub.publish(msg)

    def fail(self, message):
        self.get_logger().error(message)
        self.publish_velocity(0.0, 0.0)

        if self.status is not None and self.status.armed:
            self.get_logger().warn(
                "Error detected. Requesting automatic landing."
            )
            self.request_in_progress = False
            self.state = FlightState.LANDING
        else:
            self.state = FlightState.ERROR
    def control_loop(self):
        if self.state == FlightState.HOVERING:
            self.navigate_to_target()
        else:
            super().control_loop()

        if self.state == FlightState.FINISHED:
            self.publish_velocity(0.0, 0.0)
            self.get_logger().info(
                f"Variant 30 completed. "
                f"Target reached: {self.target_reached}"
            )
            self.timer.cancel()

    def navigate_to_target(self):
        if self.status is None or self.odometry is None:
            return

        position = self.odometry.pose.position

        if not self.status.armed:
            if self.hover_started_at is None:
                return
            if time.monotonic() - self.hover_started_at < 5.0:
                return
            self.publish_velocity(0.0, 0.0)
            self.fail("Drone disarmed during navigation")
            return

        if position.z < 1.7:
            return

        if self.start_position is None:
            self.start_position = (
                position.x,
                position.y
            )

            self.target_position = (
                position.x + self.DX,
                position.y + self.DY
            )

            self.navigation_started = time.monotonic()

            self.get_logger().info(
                f"START: x={position.x:.3f}, "
                f"y={position.y:.3f}, z={position.z:.3f}"
            )

            self.get_logger().info(
                f"TARGET: x={self.target_position[0]:.3f}, "
                f"y={self.target_position[1]:.3f}"
            )

        error_x = self.target_position[0] - position.x
        error_y = self.target_position[1] - position.y

        distance = math.hypot(error_x, error_y)

        if distance <= self.TOLERANCE:
            self.publish_velocity(0.0, 0.0)
            self.target_reached = True

            self.get_logger().info(
                f"TARGET REACHED: x={position.x:.3f}, "
                f"y={position.y:.3f}, "
                f"error={distance:.3f} m"
            )

            self.get_logger().info(
                "Stopping horizontal movement. Landing..."
            )

            self.state = FlightState.LANDING
            return

        elapsed = time.monotonic() - self.navigation_started

        if elapsed > self.FLIGHT_TIMEOUT:
            self.publish_velocity(0.0, 0.0)
            self.get_logger().error(
                "Navigation timeout. Starting landing."
            )
            self.state = FlightState.LANDING
            return

        vx = max(
            -self.MAX_SPEED,
            min(self.MAX_SPEED, self.KP * error_x)
        )

        vy = max(
            -self.MAX_SPEED,
            min(self.MAX_SPEED, self.KP * error_y)
        )

        self.publish_velocity(vx, vy)

        if time.monotonic() - self.last_log_time >= 2.0:
            self.get_logger().info(
                f"POSITION: x={position.x:.3f}, "
                f"y={position.y:.3f}, "
                f"distance={distance:.3f} m"
            )
            self.last_log_time = time.monotonic()


def main(args=None):
    rclpy.init(args=args)
    node = FlightVariant30()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.publish_velocity(0.0, 0.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
