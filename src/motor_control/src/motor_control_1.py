#!/usr/bin/env python3
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from time import sleep, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from math import cos, pi

m1p = PWMOutputDevice(18, frequency=1000)
m1n = DigitalOutputDevice(22)

m2p = PWMOutputDevice(12, frequency=1000)
m2n = DigitalOutputDevice(16)

m3p = PWMOutputDevice(13, frequency=1000)
m3n = DigitalOutputDevice(27)

mp = [m1p, m2p, m3p]
mn = [m1n, m2n, m3n]

V_tr = 1
V_rot = 1

def motor_sleep() -> None:
    m1p.value = 0; m1n.off()
    m2p.value = 0; m2n.off() 
    m3p.value = 0; m3n.off()
    
def translate_axis(node, axis=1, speed=1.0, duration=None) -> None:
    if axis!=1 and axis!=2 and axis!=3:
        node.get_logger().error(f"Axis in translate_axis must be 1, 2, or 3, given: {axis}")
        return
        
    p1 = mp[(axis+2)%3]; n1 = mn[(axis+2)%3]
    p2 = mp[(axis+0)%3]; n2 = mn[(axis+0)%3]
    p3 = mp[(axis+1)%3]; n3 = mn[(axis+1)%3] 
    
    speed = max(-1.0, min(1.0, speed))

    if speed > 0:
        p1.value = 0.0
        n1.off()
        p2.value = 1.0 - abs(speed)
        n2.on()
        p3.value = abs(speed)
        n3.off()
    
    elif speed < 0:
        p1.value = 0.0
        n1.off()
        p2.value = abs(speed)
        n2.off()
        p3.value = 1.0 - abs(speed)
        n3.on()
    
    if duration is not None:
        sleep(duration)
        speed = 0
    
    if speed == 0:
        motor_sleep()
        
def rotate(speed=1.0, duration=None) -> None:
    speed = max(-1.0, min(1.0, speed))
    
    if speed > 0:
        m1p.value = 1.0 - abs(speed)
        m1n.on()
        m2p.value = 1.0 - abs(speed)
        m2n.on()        
        m3p.value = 1.0 - abs(speed)
        m3n.on()
    
    elif speed < 0:
        m1p.value = abs(speed)
        m1n.off()
        m2p.value = abs(speed)
        m2n.off()        
        m3p.value = abs(speed)
        m3n.off()
    
    if duration is not None:
        sleep(duration)
        motor_sleep()

    if speed == 0:
        motor_sleep()

def brake(duration=1, sleep_after=True) -> None:
    m1p.value = 1.0; m1n.on()    
    m2p.value = 1.0; m2n.on()
    m3p.value = 1.0; m3n.on()
    
    sleep(duration)
    if sleep_after:
        motor_sleep()

def translate_normal_to_axis(node, axis=1, speed=1.0, duration=None) -> None:
    if axis!=1 and axis!=2 and axis!=3:
        node.get_logger().error(f"Axis in translate_normal_to_axis must be 1, 2, or 3, given: {axis}")
        return
        
    p1 = mp[(axis+2)%3]; n1 = mn[(axis+2)%3]
    p2 = mp[(axis+0)%3]; n2 = mn[(axis+0)%3]
    p3 = mp[(axis+1)%3]; n3 = mn[(axis+1)%3]
    
    speed = max(-1.0, min(1.0, speed))
    
    if speed > 0:
        p1.value = 1.0 - abs(speed)
        n1.on()
        p2.value = abs(speed) * cos(pi/3)
        n2.off()
        p3.value = abs(speed) * cos(pi/3)
        n3.off()

    elif speed < 0:
        p1.value = abs(speed)
        n1.off()
        p2.value = 1.0 - abs(speed) * cos(pi/3)
        n2.on()
        p3.value = 1.0 - abs(speed) * cos(pi/3)
        n3.on()
    
    if duration is not None:
        sleep(duration)
        speed = 0
    
    if speed == 0:
        motor_sleep()

def actuate_motor(node, motor=1, speed=1.0, duration=None) -> None:
    if motor!=1 and motor!=2 and motor!=3:
        node.get_logger().error(f"Motor in actuate_motor must be 1, 2, or 3, given: {motor}")
        return
        
    p1 = mp[(motor+2)%3]; n1 = mn[(motor+2)%3]
    p2 = mp[(motor+0)%3]; n2 = mn[(motor+0)%3]
    p3 = mp[(motor+1)%3]; n3 = mn[(motor+1)%3] 
    
    if speed > 0:
        p1.value = speed
        n1.off()
        p2.value = 0.0
        n2.off()
        p3.value = 0.0
        n3.off()
    
    elif speed < 0:
        p1.value = 1.0 - abs(speed)
        n1.on()
        p2.value = 0.0
        n2.off()
        p3.value = 0.0
        n3.off()

    if duration is not None:
        sleep(duration)
        speed = 0
            
    if speed == 0:
        motor_sleep()

class Motor_controller(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.sub = self.create_subscription(String, '/motor_cmd', self.motor_voltage, 10)
        self.current_cmd = 'Sleep'
        self.prev_cmd = 'Sleep'
    
    def motor_voltage(self, msg):
        self.prev_cmd = self.current_cmd
        self.current_cmd = msg.data
        
        match self.current_cmd:
            case 'Brake':
                brake()
            case 'Sleep':
                motor_sleep()
            case 'Forward':
                translate_axis(self, axis=1, speed=V_tr)
            case 'Backward':
                translate_axis(self, axis=1, speed=-V_tr)
            case 'Forward-left':
                translate_axis(self, axis=2, speed=V_tr)   
            case 'Forward-right':
                translate_axis(self, axis=3, speed=V_tr) 
            case 'Backward-left':
                translate_axis(self, axis=3, speed=-V_tr)   
            case 'Backward-right':
                translate_axis(self, axis=2, speed=-V_tr)
            case 'Left':
                translate_normal_to_axis(self, axis=1, speed=V_tr)
            case 'Right':
                translate_normal_to_axis(self, axis=1, speed=-V_tr)
            case 'Rotate_CCW':
                rotate(speed=V_rot)
            case 'Rotate_CW':
                rotate(speed=-V_rot)
            case 'Motor 1':
                actuate_motor(self, motor=1, speed=V_tr)
            case 'Motor 2':
                actuate_motor(self, motor=2, speed=V_tr)
            case 'Motor 3':
                actuate_motor(self, motor=3, speed=V_tr)
                

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