#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Bottle Controller ROS2 Node
# push: 控制舵机向下接近瓶盖
# rotate: 控制旋转器开/关角度
# grip: 控制夹爪闭合/张开

import os
import sys
import math
import rclpy
from rclpy.node import Node

# 路径：FTServo SDK 与旋转器 SDK
sys.path.insert(0, "/root/Biology-Lab_Project/FTServo_Python")

from scservo_sdk import PortHandler, hls
from RMAxis import Axis_V6, const
from bottle_controller.msg import BottleControl


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


class BottleControllerNode(Node):
    """
    控制带瓶盖试管的末端执行器：
    - push: 舵机 ID=1 向下/回到上方
    - rotate: 旋转器（Modbus RTU，ID=2）
    - grip: 夹爪（Modbus RTU，ID=1）
    """

    # 舵机（下压）参数
    SERVO_ID = 1
    SERVO_SPEED = 80
    SERVO_ACC = 80
    SERVO_TORQUE = 800
    POS_UP = 4000        # push=0
    POS_DOWN = 5000      # push=1

    # 旋转器参数（角度单位与设备一致，demo 中 720 表示两圈）
    CAP_CLOSED_POS = 0.0
    CAP_OPEN_POS = 1080.0
    ROTATE_VEL = 500.0
    ROTATE_ACC = 500.0
    ROTATE_DEC = 500.0
    ROTATE_BAND = 0.1

    # 夹爪参数（0~10，0最开，10最紧）
    GRIP_OPEN = 0.0
    GRIP_CLOSED = 10.0

    # Modbus IDs
    ROTATE_SLAVE_ID = 2
    GRIP_SLAVE_ID = 1

    def __init__(self):
        super().__init__("bottle_controller")

        # 参数（波特率写死）
        self.declare_parameter("servo_port", "/dev/ttyUSB0")
        self.declare_parameter("modbus_port", "/dev/ttyUSB1")

        servo_port = self.get_parameter("servo_port").get_parameter_value().string_value
        servo_baud = 1000000
        modbus_port = self.get_parameter("modbus_port").get_parameter_value().string_value
        modbus_baud = 115200

        # 初始化舵机端口
        self.port_handler = PortHandler(servo_port)
        if not self.port_handler.openPort():
            raise RuntimeError(f"无法打开舵机串口: {servo_port}")
        if not self.port_handler.setBaudRate(servo_baud):
            self.port_handler.closePort()
            raise RuntimeError(f"无法设置舵机波特率: {servo_baud}")
        self.packet_handler = hls(self.port_handler)

        # 初始化旋转器/夹爪
        self.axis_rotate = Axis_V6.create_modbus_rtu(modbus_port, modbus_baud, self.ROTATE_SLAVE_ID)
        assert self.axis_rotate.is_ready()
        self.axis_grip = Axis_V6.create_modbus_rtu(modbus_port, modbus_baud, self.GRIP_SLAVE_ID)
        assert self.axis_grip.is_ready()

        # 订阅
        self.subscription = self.create_subscription(
            BottleControl,
            "/bottle_control",
            self.on_cmd,
            10,
        )

        self.get_logger().info("Bottle Controller 节点已启动，等待 /bottle_control")

    def on_cmd(self, msg: BottleControl):
        push = clamp01(msg.push)
        rotate = clamp01(msg.rotate)
        grip = msg.grip

        self.move_push(push)
        self.move_rotate(rotate)
        self.move_grip(grip)

    def move_push(self, normalized: float):
        if hasattr(self, 'last_push_normalized') and self.last_push_normalized == normalized:
            return
        self.last_push_normalized = normalized
        target = self.POS_UP + (self.POS_DOWN - self.POS_UP) * normalized
        self.get_logger().info(f"push-> {normalized:.2f}, pos={target:.1f}")
        scs_comm_result, scs_error = self.packet_handler.WritePosEx(
            self.SERVO_ID,
            int(target),
            self.SERVO_SPEED,
            self.SERVO_ACC,
            self.SERVO_TORQUE,
        )
        if scs_comm_result != 0:
            self.get_logger().warn(f"push comm: {self.packet_handler.getTxRxResult(scs_comm_result)}")
        if scs_error != 0:
            self.get_logger().error(f"push error: {self.packet_handler.getRxPacketError(scs_error)}")

    def move_rotate(self, normalized: float):
        if hasattr(self, 'last_rotate_normalized') and self.last_rotate_normalized == normalized:
            return
        self.last_rotate_normalized = normalized
        target = self.CAP_CLOSED_POS + (self.CAP_OPEN_POS - self.CAP_CLOSED_POS) * normalized
        self.get_logger().info(f"rotate-> {normalized:.2f}, pos={target:.1f}")
        try:
            self.axis_rotate.move_absolute(
                float(target),
                self.ROTATE_VEL,
                self.ROTATE_ACC,
                self.ROTATE_DEC,
                self.ROTATE_BAND,
            )
        except Exception as e:
            self.get_logger().error(f"rotate move failed: {e}")

    def move_grip(self, closed: bool):
        if hasattr(self, 'last_grip_closed') and self.last_grip_closed == closed:
            return
        self.last_grip_closed = closed
        target = self.GRIP_CLOSED if closed else self.GRIP_OPEN
        self.get_logger().info(f"grip-> {'closed' if closed else 'open'} ({target})")
        try:
            self.axis_grip.move_absolute(float(target), 500, 500, 500, 0.1)
            if closed:
                self.axis_grip.push(5, 20, 500, 0.6, 0.1, 500)
        except Exception as e:
            self.get_logger().error(f"grip move failed: {e}")

    def destroy_node(self):
        self.get_logger().info("关闭舵机串口")
        try:
            if self.port_handler:
                self.port_handler.closePort()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BottleControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

