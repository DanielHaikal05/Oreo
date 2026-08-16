#!/usr/bin/env python3
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
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
    
def translate_axis(node, axis=1, duty=1.0, duration=None) -> None:
    if axis!=1 and axis!=2 and axis!=3:
        node.get_logger().error(f"Axis in translate_axis must be 1, 2, or 3, given: {axis}")
        return
        
    p1 = mp[(axis+2)%3]; n1 = mn[(axis+2)%3]
    p2 = mp[(axis+0)%3]; n2 = mn[(axis+0)%3]
    p3 = mp[(axis+1)%3]; n3 = mn[(axis+1)%3] 
    
    duty = max(-1.0, min(1.0, duty))

    if duty > 0:
        p1.value = 0.0
        n1.off()
        p2.value = 1.0 - abs(duty)
        n2.on()
        p3.value = abs(duty)
        n3.off()
    
    elif duty < 0:
        p1.value = 0.0
        n1.off()
        p2.value = abs(duty)
        n2.off()
        p3.value = 1.0 - abs(duty)
        n3.on()
    
    if duration is not None:
        sleep(duration)
        duty = 0
    
    if duty == 0:
        motor_sleep()
        
def rotate(duty=1.0, duration=None) -> None:
    duty = max(-1.0, min(1.0, duty))
    
    if duty > 0:
        m1p.value = 1.0 - abs(duty)
        m1n.on()
        m2p.value = 1.0 - abs(duty)
        m2n.on()        
        m3p.value = 1.0 - abs(duty)
        m3n.on()
    
    elif duty < 0:
        m1p.value = abs(duty)
        m1n.off()
        m2p.value = abs(duty)
        m2n.off()        
        m3p.value = abs(duty)
        m3n.off()
    
    if duration is not None:
        sleep(duration)
        motor_sleep()

    if duty == 0:
        motor_sleep()

def brake(duration=1, sleep_after=True) -> None:
    m1p.value = 1.0; m1n.on()    
    m2p.value = 1.0; m2n.on()
    m3p.value = 1.0; m3n.on()
    
    sleep(duration)
    if sleep_after:
        motor_sleep()

def translate_normal_to_axis(node, axis=1, duty=1.0, duration=None) -> None:
    if axis!=1 and axis!=2 and axis!=3:
        node.get_logger().error(f"Axis in translate_normal_to_axis must be 1, 2, or 3, given: {axis}")
        return
        
    p1 = mp[(axis+2)%3]; n1 = mn[(axis+2)%3]
    p2 = mp[(axis+0)%3]; n2 = mn[(axis+0)%3]
    p3 = mp[(axis+1)%3]; n3 = mn[(axis+1)%3]
    
    duty = max(-1.0, min(1.0, duty))
    
    if duty > 0:
        p1.value = 1.0 - abs(duty)
        n1.on()
        p2.value = abs(duty) * cos(pi/3)
        n2.off()
        p3.value = abs(duty) * cos(pi/3)
        n3.off()

    elif duty < 0:
        p1.value = abs(duty)
        n1.off()
        p2.value = 1.0 - abs(duty) * cos(pi/3)
        n2.on()
        p3.value = 1.0 - abs(duty) * cos(pi/3)
        n3.on()
    
    if duration is not None:
        sleep(duration)
        duty = 0
    
    if duty == 0:
        motor_sleep()

def actuate_motor(node, motor=1, duty=1.0, duration=None) -> None:
    if motor!=1 and motor!=2 and motor!=3:
        node.get_logger().error(f"Motor in actuate_motor must be 1, 2, or 3, given: {motor}")
        return
        
    p1 = mp[(motor+2)%3]; n1 = mn[(motor+2)%3]
    p2 = mp[(motor+0)%3]; n2 = mn[(motor+0)%3]
    p3 = mp[(motor+1)%3]; n3 = mn[(motor+1)%3] 
    
    if duty > 0:
        p1.value = duty
        n1.off()
    
    elif duty < 0:
        p1.value = 1.0 - abs(duty)
        n1.on()

    if duration is not None:
        sleep(duration)
        duty = 0
            
    if duty == 0:
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
        self.current_cmd = 'Sleep'
        self.prev_cmd = 'Sleep'

        self.duty = np.array([0.0, 0.0, 0.0])
        self.W_des = np.array([0.0, 0.0, 0.0])
        self.W = 3.0
        self.x = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        self.acceptable_vel_error = 0.1
        self.reset_vel_error = 2.0
        self.acc_threshold = 0.4
        self.d_duty = np.array([1.0, 1.0, 1.0])
    
    def update_state(self, msg):
        self.x[:,0] = msg.position
        self.x[:,1] = msg.velocity
        self.x[:,2] = msg.effort

        self.feedback()

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
        for i in range(3):
            if self.W_des[i] == 0:
                self.duty[i] = 0
                continue

            error = self.W_des[i] - self.x[i,1]
            if abs(self.x[i,2]) > self.acc_threshold or abs(error) < self.acceptable_vel_error:
                continue

            if abs(error) > self.reset_vel_error:
                self.d_duty[i] = 0.5

            self.d_duty[i] = self.d_duty[i] / 2
            self.duty[i] += np.sign(error) * self.d_duty[i]
        
        self.duty = np.clip(self.duty, -1.0, 1.0)
        set_duty_cycles(self, self.duty)

                 
                

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
