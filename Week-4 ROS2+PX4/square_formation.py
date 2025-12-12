#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand

class SquareMission(Node):
    def __init__(self):
        super().__init__("square_mission_node")

        # ---------------- Publishers ----------------
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.pub_traj = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        # ---------------- Mission Points (Square) ----------------
        self.points = [
            (0.0, 0.0, -50.0),
            (10.0, 0.0, -50.0),
            (10.0, 10.0, -50.0),
            (0.0, 10.0, -50.0),
            (0.0, 0.0, -50.0),
        ]
        self.index = 0
        self.hold_counter = 0

        # Start timer (20 Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)

        # First arming + offboard request will be sent after 10 setpoints
        self.initial_setpoints = 0
        self.get_logger().info("Square mission node started")

    # ---------------- Utility: send mode and arm ----------------
    def send_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.param1 = float(params.get("param1", 0.0))
        msg.param2 = float(params.get("param2", 0.0))
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_cmd.publish(msg)

    def arm(self):
        self.send_vehicle_command(400, param1=1.0)  # VEHICLE_CMD_COMPONENT_ARM_DISARM

    def set_offboard(self):
        self.send_vehicle_command(176, param1=1.0, param2=6.0)  # PX4 mode=6

    # ---------------- Publish offboard mode ----------------
    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_offboard.publish(msg)

    # ---------------- Publish setpoint ----------------
    def publish_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = 0.0
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.pub_traj.publish(msg)

    # ---------------- Main loop ----------------
    def timer_callback(self):
        # Always publish offboard + setpoints
        self.publish_offboard_mode()

        # After 10 setpoints → request OFFBOARD + ARM once
        if self.initial_setpoints < 10:
            self.initial_setpoints += 1
            return
        elif self.initial_setpoints == 10:
            self.set_offboard()
            self.arm()
            self.get_logger().info("OFFBOARD + ARM sent")
            self.initial_setpoints += 1

        # ---------------- Square Movement Logic ----------------
        x, y, z = self.points[self.index]
        self.publish_setpoint(x, y, z)

        # Hold each point for 3 seconds (~60 cycles @ 20 Hz)
        self.hold_counter += 1
        if self.hold_counter > 60:
            self.hold_counter = 0
            self.index = (self.index + 1) % len(self.points)
            next_point = self.points[self.index]
            self.get_logger().info(f"Moving to next point: {next_point}")


def main():
    rclpy.init()
    node = SquareMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
