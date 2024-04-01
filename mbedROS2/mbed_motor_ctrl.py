import rclpy
from rclpy.node import Node

from mbedROS2.SerialSimple import SerialSimple
from mbedROS2.SerialSimple import getMotorSpeed
from mbedROS2.SerialSimple import setMotorSpeed
import sys
import select
import tty
import termios

from std_msgs.msg import Float32MultiArray

def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])

old_settings = termios.tcgetattr(sys.stdin)

def forward(ser:SerialSimple):
    setMotorSpeed(ser, 10.5, 10.5)

def backward(ser:SerialSimple):
    setMotorSpeed(ser, -10.5, -10.5)

def turnLeft(ser:SerialSimple):
    setMotorSpeed(ser, 10.5, -10.5)

def turnRight(ser:SerialSimple):
    setMotorSpeed(ser, -10.5, 10.5)

def stop(ser:SerialSimple):
    setMotorSpeed(ser, 0.0, 0.0)

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('mbedMotorCtrl')
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_vel', 10)
        freq = 0.001
        self.timer = self.create_timer(freq, self.timer_callback)

        self.motor_spd1 = 0.0
        self.motor_spd2 = 0.0

    def timer_callback(self):
        motor_spd = Float32MultiArray()
        motor_spd.data = [self.motor_spd1, self.motor_spd2]
        self.publisher.publish(motor_spd)

def main(args=None):
    rclpy.init(args=args)

    motorPublisher = MotorPublisher()

    ser = SerialSimple(baudrate=115200, port='/dev/ttyACM0')
    ser.init()
    ser.start()

    try:
        tty.setcbreak(sys.stdin.fileno())

        while ser.conn:
            resp = ser.read()
            
            motor_spd1, motor_spd2 = getMotorSpeed(resp)
            motorPublisher.motor_spd1 = motor_spd1
            motorPublisher.motor_spd2 = motor_spd2

            if isData():
                key = sys.stdin.read(1)
                if key == 'w':
                    forward(ser)
                elif key == 's':
                    backward(ser)
                elif key == 'd':
                    turnRight(ser)
                elif key == 'a':
                    turnLeft(ser)
                elif key == 'q':
                    stop(ser)

            rclpy.spin_once(motorPublisher)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(e)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    ser.close()

    motorPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()