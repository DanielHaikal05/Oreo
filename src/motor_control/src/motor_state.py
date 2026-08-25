#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gpiozero import DigitalInputDevice
from time import monotonic
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
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
        return 0


class Motor_state(Node):
    def __init__(self):
        super().__init__('motor_state')
        self.timeout_timer = self.create_timer(1, self.detect_timeout)
        self.pub = self.create_publisher(JointState, '/joint_state', 10)
        self.edge_pub = self.create_publisher(Float32MultiArray, '/edge_counts', 10)
        
        self.x_bar = np.array([[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                           [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                           [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                           [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]])  # Each dimension is the estimate from one edge
        self.x = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        
        self.direction = np.array([1, 1, 1]) # 1 for CCW, -1 for CW
        self.encoders = encoders_status()

        self.timeout = 2

        self.t_prev = np.array([[None, None, None],
                                [None, None, None],
                                [None, None, None],
                                [None, None, None]], dtype=object)

        self.is_valid = np.array([[False, False, False],
                                  [False, False, False],
                                  [False, False, False],
                                  [False, False, False]])

        e11.when_activated   = lambda: self.read_encoder(0, 0, 1)
        e11.when_deactivated = lambda: self.read_encoder(0, 0, 0)

        e12.when_activated   = lambda: self.read_encoder(0, 1, 1)
        e12.when_deactivated = lambda: self.read_encoder(0, 1, 0)

        e21.when_activated   = lambda: self.read_encoder(1, 0, 1)
        e21.when_deactivated = lambda: self.read_encoder(1, 0, 0)

        e22.when_activated   = lambda: self.read_encoder(1, 1, 1)
        e22.when_deactivated = lambda: self.read_encoder(1, 1, 0)

        e31.when_activated   = lambda: self.read_encoder(2, 0, 1)
        e31.when_deactivated = lambda: self.read_encoder(2, 0, 0)

        e32.when_activated   = lambda: self.read_encoder(2, 1, 1)
        e32.when_deactivated = lambda: self.read_encoder(2, 1, 0)

        self.edge_count = np.zeros((3,2), dtype=int)
    
    
    def read_encoder(self, m, c, s):
        self.edge_count[m][c] +=1
        if self.encoders[m][c] == s:
            self.get_logger().warning("Called read_encoder with no change")
            return
        
        encoder_reading = self.encoders.copy()
        encoder_reading[m][c] = s
        t = monotonic()

        direction = encoder_direction(self.encoders[m], encoder_reading[m])
        if direction == 0:
            self.get_logger().error(f"Invalid transition: {self.encoders[m]} --> {encoder_reading[m]}")
            return
        
        if direction != self.direction[m]:
            self.x[m,1:] = np.array([0,0])
            self.x_bar[:,m,1:] = np.array([[0,0], [0,0], [0,0], [0,0]])
            self.direction[m] = direction
            self.t_prev[:,m] = None
            self.is_valid[:,m] = False  
        self.encoders[m] = encoder_reading[m]

        a = encoder_reading[m][0]; b = encoder_reading[m][1]
        edge_index = 3*a + b - 2*a*b

        if self.t_prev[edge_index][m] is None:
            self.t_prev[edge_index][m] = t
            return

        dt = t - self.t_prev[edge_index][m]
        self.t_prev[edge_index][m] = t
        qdi = self.x_bar[edge_index][m][1]

        self.x_bar[edge_index][m][0] += self.direction[m] * (2*pi/11)
        self.x_bar[edge_index][m][1] = self.direction[m] * (2*pi/11) * (1/dt)
        self.x_bar[edge_index][m][2] = (self.x_bar[edge_index][m][1] - qdi) / dt
        self.is_valid[edge_index][m] = True

        if edge_index == 0:
            self.x[m,0] = self.x_bar[edge_index][m][0]

        valid = self.is_valid[:,m]
        self.x[m,1:] = np.zeros_like(self.x[m,1:])
        if np.any(valid):
            self.x[m,1:] = np.mean(self.x_bar[valid,m,1:], axis=0)

        self.publish_joint_state()


    def publish_joint_state(self):
        msg = JointState()
        msg.name = ["Joint_1", "Joint_2", "Joint_3"]
        
        msg.position = self.x[:,0]
        msg.velocity = self.x[:,1]
        msg.effort = self.x[:,2]
        
        self.pub.publish(msg)

        msg2 = Float32MultiArray()
        msg2.data = self.edge_count.flatten()
        self.edge_pub.publish(msg2)

    def detect_timeout(self):
        t = monotonic()
        for i in range(3):
            if any(a is None for a in self.t_prev[:,i]): 
                continue
            if np.all(t > self.t_prev[:,i] + self.timeout):
                self.x_bar[:,i,1:] = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])
                self.x[i,1:] = np.array([0.0, 0.0])
                self.t_prev[:,i] = np.array([None, None, None, None])
                self.is_valid[:, i] = False
 
      
def main(args=None):
    rclpy.init(args=args)
    node = Motor_state()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()