#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
)


class VelocityControl(Node):
    def __init__(self):
        super().__init__('velocity_control_node')

        # ---------- QoS ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------- Publishers ----------
        self.pub_traj = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)

        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos)

        # ---------- Velocity Setpoints ----------
        self.vx = 1.0     # m/s
        self.vy = 0.0
        self.vz = 0.0     # upward negative, downward positive

        self.target_yaw = 0.0
        self.counter = 0

        # publish at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Velocity Control Node Started")

    # ================================================================
    # Publish Offboard Mode (enable velocity control)
    # ================================================================
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True      # <<< ENABLE VELOCITY CONTROL
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_offboard_mode.publish(msg)

    # ================================================================
    # Publish VELOCITY SETPOINTS
    # ================================================================
    def publish_velocity_setpoint(self):
        msg = TrajectorySetpoint()

        msg.velocity = [self.vx, self.vy, self.vz]   # <<< VELOCITY CONTROL
        msg.yaw = self.target_yaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        self.pub_traj.publish(msg)

    # ================================================================
    # MAIN TIMER CALLBACK
    # ================================================================
    def timer_callback(self):
        # Always publish offboard mode first
        self.publish_offboard_mode()

        # Publish velocity commands
        self.publish_velocity_setpoint()

        if self.counter % 20 == 0:
            self.get_logger().info(
                f"Publishing velocity [vx={self.vx}, vy={self.vy}, vz={self.vz}]"
            )

        self.counter += 1


# =====================================================================
def main(args=None):
    rclpy.init(args=args)
    node = VelocityControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
