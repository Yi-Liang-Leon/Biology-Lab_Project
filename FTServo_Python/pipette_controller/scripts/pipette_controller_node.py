#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Pipette Controller ROS2 Node
# Controls ejector (servo ID 1) and plunger (servo ID 2) based on topic commands
#

import rclpy
from rclpy.node import Node
import sys
import os

# 添加 scservo_sdk 路径 (需要添加 scservo_sdk 的父目录)
sys.path.insert(0, "/root/Biology-Lab_Project/FTServo_Python")

from scservo_sdk import PortHandler, hls
from pipette_controller.msg import PipetteControl


class PipetteControllerNode(Node):
    """
    ROS2 节点：控制移液器舵机
    - 舵机1 (ID=1): ejector 弹出器
    - 舵机2 (ID=2): plunger 活塞
    """

    # 舵机参数定义
    EJECTOR_ID = 1
    PLUNGER_ID = 2
    
    # 位置定义
    POS_DEFAULT = 2048          # 默认位置 (bool = False)
    POS_EJECTOR_ACTIVE = 3350   # ejector 激活位置 (bool = True)
    POS_PLUNGER_ACTIVE = 750    # plunger 激活位置 (bool = True)
    
    # 舵机运动参数
    SPEED = 80
    ACC = 80
    TORQUE = 800
    baudrate = 1000000

    def __init__(self):
        super().__init__('pipette_controller')
        
        # 声明参数
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        
        # 获取参数
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        
        # 初始化串口
        self.port_handler = PortHandler(serial_port)
        self.packet_handler = None
        
        # 打开串口
        if self.port_handler.openPort():
            self.get_logger().info(f'成功打开串口: {serial_port}')
        else:
            self.get_logger().error(f'无法打开串口: {serial_port}')
            raise RuntimeError(f'Failed to open port: {serial_port}')
        
        # 设置波特率
        if self.port_handler.setBaudRate(self.baudrate):
            self.get_logger().info(f'成功设置波特率: {self.baudrate}')
        else:
            self.get_logger().error(f'无法设置波特率: {self.baudrate}')
            self.port_handler.closePort()
            raise RuntimeError(f'Failed to set baudrate: {self.baudrate}')
        
        # 初始化协议处理器
        self.packet_handler = hls(self.port_handler)
        
        # 记录当前状态
        self.current_ejector = False
        self.current_plunger = False
        
        # 创建订阅者
        self.subscription = self.create_subscription(
            PipetteControl,
            '/pipette_control',
            self.control_callback,
            10
        )
        
        self.get_logger().info('Pipette Controller 节点已启动，等待控制指令...')
        self.get_logger().info('话题: /pipette_control')
        self.get_logger().info(f'  - ejector (ID={self.EJECTOR_ID}): False->2048, True->3350')
        self.get_logger().info(f'  - plunger (ID={self.PLUNGER_ID}): False->2048, True->750')

    def control_callback(self, msg: PipetteControl):
        """
        处理控制消息回调
        """
        self.get_logger().info(f'收到控制指令: ejector={msg.ejector}, plunger={msg.plunger}')
        
        # 控制 ejector (舵机1)
        if msg.ejector != self.current_ejector:
            self.control_ejector(msg.ejector)
            self.current_ejector = msg.ejector
        
        # 控制 plunger (舵机2)
        if msg.plunger != self.current_plunger:
            self.control_plunger(msg.plunger)
            self.current_plunger = msg.plunger

    def control_ejector(self, active: bool):
        """
        控制 ejector 舵机 (ID=1)
        active=True: 移动到 3350
        active=False: 移动到 2048
        """
        target_pos = self.POS_EJECTOR_ACTIVE if active else self.POS_DEFAULT
        self.get_logger().info(f'Ejector (ID={self.EJECTOR_ID}): 移动到位置 {target_pos}')
        
        scs_comm_result, scs_error = self.packet_handler.WritePosEx(
            self.EJECTOR_ID, 
            target_pos, 
            self.SPEED, 
            self.ACC, 
            self.TORQUE
        )
        
        if scs_comm_result != 0:
            self.get_logger().warn(f'Ejector 通信结果: {self.packet_handler.getTxRxResult(scs_comm_result)}')
        if scs_error != 0:
            self.get_logger().error(f'Ejector 错误: {self.packet_handler.getRxPacketError(scs_error)}')

    def control_plunger(self, active: bool):
        """
        控制 plunger 舵机 (ID=2)
        active=True: 移动到 750
        active=False: 移动到 2048
        """
        target_pos = self.POS_PLUNGER_ACTIVE if active else self.POS_DEFAULT
        self.get_logger().info(f'Plunger (ID={self.PLUNGER_ID}): 移动到位置 {target_pos}')
        
        scs_comm_result, scs_error = self.packet_handler.WritePosEx(
            self.PLUNGER_ID, 
            target_pos, 
            self.SPEED, 
            self.ACC, 
            self.TORQUE
        )
        
        if scs_comm_result != 0:
            self.get_logger().warn(f'Plunger 通信结果: {self.packet_handler.getTxRxResult(scs_comm_result)}')
        if scs_error != 0:
            self.get_logger().error(f'Plunger 错误: {self.packet_handler.getRxPacketError(scs_error)}')

    def destroy_node(self):
        """
        节点销毁时关闭串口
        """
        self.get_logger().info('关闭串口...')
        if self.port_handler:
            self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PipetteControllerNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'节点启动失败: {e}')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
