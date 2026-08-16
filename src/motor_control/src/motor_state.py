#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import DigitalInputDevice
from time import time
from sensor_msgs.msg import JointState
from math import pi
import numpy as np

e11 = DigitalInputDevice(23)
e12 = DigitalInputDevice(24)

e21 = DigitalInputDevice(20)
e22 = DigitalInputDevice(21)

e31 = DigitalInputDevice(9)
e32 = DigitalInputDevice(10)


def encoders_status():
    return np.array([[e11.is_active, e12.is_active], [e21.is_active, e22.is_active], [e31.is_active, e32.is_active]])

  
def encoder_direction(seq1, seq2):
    seq1 = seq1.tolist()
    seq2 = seq2.tolist()
    if (seq1, seq2) in [([0,0], [1,0]), ([1,0], [1,1]), ([1,1], [0,1]), ([0,1], [0,0])]:
        return 1
    elif (seq2, seq1) in [([0,0], [1,0]), ([1,0], [1,1]), ([1,1], [0,1]), ([0,1], [0,0])]:
        return -1
    else:
        raise ValueError(f"Encoder sequence ({seq1},{seq2}) invalid")


class Motor_state(Node):
    def __init__(self):
        super().__init__('motor_state')
        self.timeout_timer = self.create_timer(1, self.detect_timeout)
        self.pub = self.create_publisher(JointState, '/joint_state', 10)
        
        self.x_bar = np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                           [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                           [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                           [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]])  # Each dimension is the estimate from one edge
        self.x = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        
        self.direction = np.array([1, 1, 1]) # 1 for CCW, -1 for CW
        self.encoders = encoders_status()

        self.timeout = 2

        self.t_prev = np.array([[None, None, None],
                                [None, None, None]
                                [None, None, None]
                                [None, None, None]])

        self.is_valid = np.array([[False, False, False],
                                  [False, False, False],
                                  [False, False, False],
                                  [False, False, False]])

        for encoder_pin in (e11, e12, e21, e22, e31, e32):
            encoder_pin.when_activated = self.read_encoder
            encoder_pin.when_deactivated = self.read_encoder
    
    def read_encoder(self):
        encoder_reading = encoders_status()
            
        t = time()
        
        for i in range(3):
            if np.all(encoder_reading[i] == self.encoders[i]):
                continue
            
            direction = encoder_direction(self.encoders[i], encoder_reading[i])
            if direction != self.direction[i]:
                self.x[i,1:] = np.array([0,0])
                self.x_bar[:,i,1:] = np.array([[0,0], [0,0], [0,0], [0,0]])
                self.direction[i] = direction
                self.t_prev[:,i] = np.array([[None], [None], [None], [None]])
                self.is_valid[:,i] = np.array([[False], [False], [False], [False]])
                
            self.encoders[i] = encoder_reading[i]

            a = encoder_reading[i][0]; b = encoder_reading[i][1]
            edge_index = 3*a + b - 2*a*b

            if self.t_prev[edge_index][i] is None: # If direction is changed, acc and vel are set to 0, position is unchanged, time is reset
                self.t_prev[edge_index][i] = t
                continue

            dt = t - self.t_prev[edge_index][i]
            self.t_prev[edge_index][i] = t
            qi = self.x_bar[edge_index][i][0]
            qdi = self.x_bar[edge_index][i][1]

            self.x_bar[edge_index][i][0] += self.direction[i] * (2*pi/11)
            self.x_bar[edge_index][i][1] = self.direction[i] * (2*pi/11) * (1/dt)
            self.x_bar[edge_index][i][2] = (self.x_bar[edge_index][i][1] - qdi) / dt
            self.is_valid[edge_index][i] = True

            if edge_index == 0:
                self.x[i,0] = self.x_bar[edge_index][i][0]

            valid = self.is_valid[:,i]
            self.x[1:,:] = np.zeros_like(self.x[1:,:])
            if np.any(valid):
                self.x[i,1:] = np.mean(self.x_bar[valid,i,1:], axis=0)

        self.publish_joint_state()

    def publish_joint_state(self):
        msg = JointState()
        msg.name = ["Joint_1", "Joint_2", "Joint_3"]
        
        msg.position = self.x[:,0]
        msg.velocity = self.x[:,1]
        msg.effort = self.x[:,2]
        
        self.pub.publish(msg)

    def detect_timeout(self):
        t = time()
        for i in range(3):
            if np.all(t > self.t_prev[:,i] + self.timeout) and t > np.all(self.t_prev[:,i] + self.timeout):
                self.x_bar[:,i,1:] = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
                self.x[i,1:] = np.array([0.0, 0.0])
                self.t_prev[:,i] = np.array([None, None, None, None])
 
      
def main(args=None):
    rclpy.init(args=args)
    node = Motor_state()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()