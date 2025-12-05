#!/usr/bin/env python3
"""
offboard_position_control.py

Minimal PX4 offboard position control (ROS2 Humble, px4_msgs).
Publishes OffboardControlMode + TrajectorySetpoint and sends
commands to set OFFBOARD mode and ARM the vehicle.

Place this file in your ament_python package (e.g. my_position_pkg/my_position_pkg/)
and add an entry_point in setup.py so you can run:
    ros2 run my_position_pkg offboard_position_control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class OffboardPositionControl(Node):
    def __init__(self) -> None:
        super().__init__("offboard_position_control")

        # QoS suitable for PX4 topics - transient_local for latched-like behavior
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.pub_offboard_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.pub_trajectory = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.pub_command = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        # Subscribers (monitoring only)
        self.sub_local_pos = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.vehicle_local_position_cb,
            qos,
        )
        self.sub_status = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_cb, qos
        )

        # State
        self.setpoint_counter = 0
        self.local_pos = VehicleLocalPosition()  # will be updated by subscriber
        self.status = VehicleStatus()  # will be updated by subscriber

        # Desired position (set these as you like)
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -10.0  # negative = up in many PX4 coordinate conventions
        self.target_yaw = 0.0

        # How many initial setpoints to send before requesting OFFBOARD + ARM
        self.initial_setpoints_required = 10

        # Timer publishes at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("PX4 offboard position control node started")

    # ---------- subscribers ----------
    def vehicle_local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.local_pos = msg

    def vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self.status = msg

    # ---------- publish helpers ----------
    def publish_offboard_mode(self) -> None:
        msg = OffboardControlMode()
        # Only position control enabled
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_offboard_mode.publish(msg)

    def publish_position_setpoint(self, x: float, y: float, z: float, yaw: float = 0.0) -> None:
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_trajectory.publish(msg)
        # Light logging so output isn't spammy
        if self.setpoint_counter % 10 == 0:  # log every 10 timer calls
            self.get_logger().info(f"Publishing position setpoint: [{x:.2f}, {y:.2f}, {z:.2f}]")

    def publish_command(self, command: int, *params) -> None:
        """
        Publish a VehicleCommand. params is a sequence of up to 7 params.
        Ensure we send floats to satisfy px4_msgs assertions.
        """
        p = list(params) + [0.0] * (7 - len(params))
        msg = VehicleCommand()
        msg.command = int(command)
        msg.param1 = float(p[0])
        msg.param2 = float(p[1])
        msg.param3 = float(p[2])
        msg.param4 = float(p[3])
        msg.param5 = float(p[4])
        msg.param6 = float(p[5])
        msg.param7 = float(p[6])
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.pub_command.publish(msg)

    def request_offboard_mode(self) -> None:
        # VEHICLE_CMD_DO_SET_MODE: param1 = base_mode, param2 = custom_mode
        # these numbers follow MAVLink/PX4 expectations; verify for your PX4 version.
        self.publish_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Requested OFFBOARD mode")

    def arm(self) -> None:
        # VEHICLE_CMD_COMPONENT_ARM_DISARM: param1 = 1 to arm
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("ARM command sent")

    # ---------- main loop ----------
    def timer_callback(self) -> None:
        # Always publish mode + setpoint to keep PX4 in offboard
        self.publish_offboard_mode()
        self.publish_position_setpoint(self.target_x, self.target_y, self.target_z, self.target_yaw)

        # Send OFFBOARD + ARM once after initial setpoints
        if self.setpoint_counter == self.initial_setpoints_required:
            self.request_offboard_mode()
            self.arm()

        # count up but stop incrementing after a bit (we only want to send the request once)
        if self.setpoint_counter < (self.initial_setpoints_required + 5):
            self.setpoint_counter += 1

        


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OffboardPositionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

