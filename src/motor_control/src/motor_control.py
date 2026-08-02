#!/usr/bin/env python3
from gpizero import PWMOutputDevice
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

def motor_sleep() -> none:
    m1p.value = 0; m1n.off()
    m2p.value = 0; m2n.off() 
    m3p.value = 0; m3n.off()
    
def translate_axis(axis=1, speed=1.0, duration=None) -> None:
    if axis!=1 and axis!=2 and axis!=3:
        raise ValueError("Axis in translate_axis must be 1, 2, or 3")
        
    p1 = mp[(axis+2)%3]; n1 = mn[(axis+2)%3]
    p2 = mp[(axis+0)%3]; n2 = mn[(axis+0)%3]
    p3 = mp[(axis+1)%3]; n3 = mn[(axis+1)%3] 
    
    speed = max(-1.0, min(1.0, speed))

    if speed > 0:
        p2.value = abs(speed)
        n2.off()
        p3.value = 1.0 - abs(speed)
        n3.on()
    
    elif speed < 0:
        p2.value = 1.0 - abs(speed)
        n2.on()
        p3.value = abs(speed)
        n3.off()
    
    if duration is not None:
        sleep(duration)
        speed = 0
    
    if speed == 0:
        motor_sleep()
        
def rotate(speed=1.0, duration=None) -> None:
    speed = max(-1.0, min(1.0, speed))
    
    if speed > 0:
        m1p.value = abs(speed)
        m1n.off()
        m2p.value = abs(speed)
        m2n.off()        
        m3p.value = abs(speed)
        m3n.off()
    
    elif speed < 0:
        m1p.value = 1.0 - abs(speed)
        m1n.on()
        m2p.value = 1.0 - abs(speed)
        m2n.on()        
        m3p.value = 1.0 - abs(speed)
        m3n.on()
    
    if duration is not None:
        sleep(duration)
        motor_sleep()

def brake(duration=1, sleep_after=True) -> None:
    m1p.value = 1.0; m1n.on()    
    m2p.value = 1.0; m2n.on()
    m3p.value = 1.0; m3n.on()
    
    sleep(durartion)
    if sleep_after:
        motor_sleep()

def translate_normal_to_axis(axis=1, speed=1.0, duration=None) -> None:
    if axis!=1 and axis!=2 and axis!=3:
        raise ValueError("Axis in translate_axis must be 1, 2, or 3")
        
    p1 = mp[(axis+2)%3]; n1 = mn[(axis+2)%3]
    p2 = mp[(axis+0)%3]; n2 = mn[(axis+0)%3]
    p3 = mp[(axis+1)%3]; n3 = mn[(axis+1)%3]
    
    speed = max(-1.0, min(1.0, speed))
    
    if speed > 0:
        p1.value = speed
        n1.off()
        p2.value = 1.0 - speed * cos(pi/3)
        n2.on()
        p3.value = 1.0 - speed * cos(pi/3)
        n3.on()

    elif speed < 0:
        p1.value = 1.0 - speed
        n1.on()
        p2.value = speed * cos(pi/3)
        n2.off()
        p3.value = speed * cos(pi/3)
        n3.off()
    
    if duration is not None:
        sleep(duration)
        speed = 0
    
    if speed == 0:
        motor_sleep()

class Motor_controller(Node):
    def __init__(self):
        super.__init__(self)
        self.sub = self.create_subscription(String, '/motor_cmd', self.motor_voltage, 10)
        self.current_cmd = 'Sleep'
        self.prev_cmd = 'Sleep'
    
    def motor_voltage(self, msg):
        self.prev_cmd = self.current_cmd
        self.current_cmd = msg.data
        
        switch self.current_cmd:
            case('Brake'):
                brake()
            case('Sleep'):
                motor_sleep()
            case('Forward'):
                translate_axis(axis=1, speed=V_tr)
            case('Backward'):
                translate_axis(axis=1, speed=-V_tr)
            case('Forward-left'):
                translate_axis(axis=2, speed=V_tr)   
            case('Forward-right'):
                translate_axis(axis=3, speed=V_tr) 
            case('Backward-left'):
                translate_axis(axis=3, speed=-V_tr)   
            case('Backard-right'):
                translate_axis(axis=2, speed=-V_tr)
            case('Left'):
                translate_normal_to_axis(axis=1, speed=V_tr)
            case('Right'):
                translate_normal_to_axis(axis=1, speed=-V_tr)
            case('Rotate_CCW'):
                rotate(speed=V_rot)
            case('Rotate_CW'):
                rotate(speed=-V_rot)
