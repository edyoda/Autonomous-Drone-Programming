#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)

class PositionControl(Node):
    def __init__(self):
        super().__init__('position_control_node')

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
        
        # ---------- Variables ----------
        self.counter = 0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -20.0     
        self.target_yaw = 0.0

        # Publish messages at 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Position Control Node Started")


    # ================================================================
    # Publish TRAJECTORY SETPOINT
    # ================================================================
    def publish_position_setpoint(self):
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_traj.publish(msg)



    # ================================================================
    # MAIN TIMER CALLBACK
    # ================================================================
    def timer_callback(self):
        self.publish_position_setpoint()

        if self.counter % 20 == 0:
            self.get_logger().info(
                f"Publishing position [{self.target_x}, {self.target_y}, {self.target_z}]"
            )

        self.counter += 1


# =====================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PositionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

