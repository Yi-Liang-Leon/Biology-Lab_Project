import os
import sys
import time

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "FTServo_Python")))
from scservo_sdk import PortHandler, hls
from RMAxis import Axis_V6

SERVO_PORT = "/dev/ttyUSB0"
SERVO_BAUDRATE = 1000000
SERVO_ID = 1
POS_UP = 4000
POS_DOWN = 5000
SERVO_SPEED = 40
SERVO_ACC = 40
SERVO_TORQUE = 800


def init_servo():
    port_handler = PortHandler(SERVO_PORT)
    if not port_handler.openPort():
        raise RuntimeError(f"无法打开舵机串口 {SERVO_PORT}")
    if not port_handler.setBaudRate(SERVO_BAUDRATE):
        port_handler.closePort()
        raise RuntimeError(f"无法设置波特率 {SERVO_BAUDRATE}")
    return port_handler, hls(port_handler)


def move_servo(packet_handler, position):
    comm_result, error = packet_handler.WritePosEx(
        SERVO_ID,
        position,
        SERVO_SPEED,
        SERVO_ACC,
        SERVO_TORQUE,
    )
    print("舵机通信结果:", packet_handler.getTxRxResult(comm_result))
    if error != 0:
        print("舵机错误:", packet_handler.getRxPacketError(error))


def cleanup_servo(port_handler):
    if port_handler:
        port_handler.closePort()


axis_rtu =Axis_V6.create_modbus_rtu('/dev/ttyUSB1', 115200, 1)
# id=2 旋转电机
axis_rtu2 =Axis_V6.create_modbus_rtu('/dev/ttyUSB1', 115200, 2)

servo_port_handler, servo_packet_handler = init_servo()
try:
    print(axis_rtu.position())

    # axis_rtu.go_home()
    # axis_rtu2.go_home()

    print("舵机抬起")
    move_servo(servo_packet_handler, POS_UP)
    time.sleep(1)

    print("舵机下压到瓶盖")
    move_servo(servo_packet_handler, POS_DOWN)
    time.sleep(1)

    # axis_rtu.move_to(0)

    ##绝对运动 (位置10mm，速度50mm/s，加速度 500mm/s2，减速度 500mm/s2 ，力定位范围0.1N)、
    axis_rtu.move_absolute(10,500, 500, 500, 0.1)

    #推压运动 (往前推进距离5mm，速度 20mm/s，出力15%，加速度 500mm/s2 ，位置范围0.1mm，时间范围500ms)
    axis_rtu.push(5, 20, 500, 0.6, 0.1, 500)

    ##绝对运动 (位置10mm，速度50mm/s，加速度 500mm/s2，减速度 500mm/s2 ，力定位范围0.1N)
    axis_rtu2.move_absolute(-1080,500, 500, 500, 0.1)

    time.sleep(3)

    print("舵机复位抬起")
    move_servo(servo_packet_handler, POS_UP)

    time.sleep(2)
    ##绝对运动 (位置10mm，速度50mm/s，加速度 500mm/s2，减速度 500mm/s2 ，力定位范围0.1N)
    axis_rtu2.move_absolute(0,500, 500, 500, 0.1)
    ##绝对运动 (位置10mm，速度50mm/s，加速度 500mm/s2，减速度 500mm/s2 ，力定位范围0.1N)、
    axis_rtu.move_absolute(0,500, 500, 500, 0.1)
finally:
    cleanup_servo(servo_port_handler)


# axis_rtu.reset_error()
