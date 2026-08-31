#!/usr/bin/env python3
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Float32MultiArray
from sensor_msgs.msg import JointState
from math import cos, pi
import numpy as np

m1p = PWMOutputDevice(18, frequency=1000)
m1n = DigitalOutputDevice(22)

m2p = PWMOutputDevice(12, frequency=1000)
m2n = DigitalOutputDevice(16)

m3p = PWMOutputDevice(13, frequency=1000)
m3n = DigitalOutputDevice(27)

mp = [m1p, m2p, m3p]
mn = [m1n, m2n, m3n]


def motor_sleep(node, motor=None) -> None:
    if motor is not None and motor!=1 and motor!=2 and motor!=3:
        node.get_logger().error(f"Motor in motor_sleep must be 1, 2, or 3, given: {motor}")
        return
    
    if motor is not None:
        mp[motor-1].value = 0; mn[motor-1].off()
        return

    m1p.value = 0; m1n.off()
    m2p.value = 0; m2n.off() 
    m3p.value = 0; m3n.off()


def brake(duration=1, sleep_after=True) -> None:
    m1p.value = 1.0; m1n.on()    
    m2p.value = 1.0; m2n.on()
    m3p.value = 1.0; m3n.on()
    
    sleep(duration)
    if sleep_after:
        motor_sleep()


def set_duty_cycles(node, duty, duration=None) -> None:
    if len(duty)!=3:
        node.get_logger().error(f"Expected size of duty to be 3, given: {len(duty)}")
        return
    
    for i in range(3):
        if duty[i] > 0:
            mp[i].value = duty[i]
            mn[i].off()
        
        elif duty[i] < 0:
            mp[i].value = 1.0 - abs(duty[i])
            mn[i].on()

        if duration is not None:
            sleep(duration)
            duty[i] = 0

        if duty[i] == 0:
            motor_sleep(node, i+1)


class Motor_controller(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.cmd_sub = self.create_subscription(String, '/motor_cmd', self.motor_command, 10)
        self.state_sub = self.create_subscription(JointState, '/joint_state', self.update_state, 10)
        self.vel_sub = self.create_subscription(Float32, '/wheel_vel', self.update_wheel_vel, 10)
        self.feedback_timer = self.create_timer(0.02, self.feedback)

        self.current_cmd = 'Sleep'
        self.prev_cmd = 'Sleep'

        self.duty = np.array([0.0, 0.0, 0.0])
        self.W_des = np.array([0.0, 0.0, 0.0])
        self.W = 1.0
        self.x = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        self.acceptable_vel_error = 0.1
        self.reset_vel_error = 5
        self.acc_threshold = 60
        self.min_duty_step = 0.001
        self.d_duty = np.array([1.0, 1.0, 1.0])

        self.des_w_pub = self.create_publisher(Float32MultiArray, '/desired_W', 10)
        self.duty_pub = self.create_publisher(Float32MultiArray, '/motor_duty', 10)
        self.dduty_pub = self.create_publisher(Float32MultiArray, '/motor_dduty', 10)
        self.error_pub = self.create_publisher(Float32MultiArray, '/vel_error', 10)
        self.debug_timer = self.create_timer(1, self.debug_helper)
    
    def update_state(self, msg):
        self.x[:,0] = msg.position
        self.x[:,1] = msg.velocity
        self.x[:,2] = msg.effort

    def update_wheel_vel(self, msg):
        self.W = msg.data

    def motor_command(self, msg):
        self.prev_cmd = self.current_cmd
        self.current_cmd = msg.data
        W = self.W

        if self.prev_cmd != self.current_cmd:
            self.d_duty = np.array([1.0, 1.0, 1.0])
            self.duty = np.array([0.0, 0.0, 0.0])
        
        match self.current_cmd:
            case 'Brake':
                brake()
                self.W_des = np.array([0.0, 0.0, 0.0])
            case 'Sleep':
                self.W_des = np.array([0.0, 0.0, 0.0])
            case 'Forward':
                self.W_des = np.array([0.0, -W, W])
            case 'Backward':
                self.W_des = np.array([0.0, W, -W])
            case 'Forward-left':
                self.W_des = np.array([-W, 0.0, W])
            case 'Forward-right':
                self.W_des = np.array([W, -W, 0.0])
            case 'Backward-left':
                self.W_des = np.array([-W, W, 0.0])
            case 'Backward-right':
                self.W_des = np.array([W, 0.0, -W])
            case 'Left':
                self.W_des = np.array([-W*cos(pi/3), W, W])
            case 'Right':
                self.W_des = np.array([W*cos(pi/3), -W, -W])
            case 'Rotate_CCW':
                self.W_des = np.array([-W, -W, -W])
            case 'Rotate_CW':
                self.W_des = np.array([W, W, W])
            case 'Motor 1':
                self.W_des = np.array([W, 0.0, 0.0])
            case 'Motor 2':
                self.W_des = np.array([0.0, W, 0.0])
            case 'Motor 3':
                self.W_des = np.array([0.0, 0.0, W])
            case _:
                self.W_des = np.array([0.0, 0.0, 0.0])

    def feedback(self):
        error = self.W_des - self.x[:,1]
        self.error = error

        for i in range(3):
            if self.W_des[i] == 0:
                self.duty[i] = 0
                continue

            if abs(self.x[i,2]) > self.acc_threshold or abs(error[i]) < self.acceptable_vel_error:
                if abs(self.x[i,2]) > self.acc_threshold:
                    self.get_logger().warning("Waiting for acceleation to settle")
                else:
                    self.get_logger().warning("Velocity error deemed acceptable")
                continue

            if abs(error[i]) > self.reset_vel_error:
                self.d_duty[i] = 0.5

            self.d_duty[i] = max(self.d_duty[i] / 2, self.min_duty_step)
            self.duty[i] += np.sign(error[i]) * self.d_duty[i]
        
        self.duty = np.clip(self.duty, -1.0, 1.0)
        set_duty_cycles(self, self.duty)

    def debug_helper(self):
        des_w_msg = Float32MultiArray()
        duty_msg = Float32MultiArray()
        dduty_msg = Float32MultiArray()
        error_msg = Float32MultiArray()

        des_w_msg.data = self.W_des
        duty_msg.data = self.duty
        dduty_msg.data = self.d_duty
        error_msg.data = self.error

        self.des_w_pub.publish(des_w_msg)
        self.duty_pub.publish(duty_msg)
        self.dduty_pub.publish(dduty_msg)
        self.error_pub.publish(error_msg)

                 
                

def main(args=None):
    rclpy.init(args=args)
    node = Motor_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_sleep()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
