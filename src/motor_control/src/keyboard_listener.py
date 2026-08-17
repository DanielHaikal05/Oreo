#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import json

class Keyboard_to_cmd(Node):
    def __init__(self):
        super().__init__('Keyboard_to_cmd')
        self.sub = self.create_subscription(String, '/key_press', self.retrieve_key, 10)
        self.pub_cmd = self.create_publisher(String, '/motor_cmd', 10)
        self.pub_vel = self.create_publisher(Float32, 'wheel_vel', 10)
        self.timer = self.create_timer(0.1, self.send_motor_cmd)
        
        self.pressed_keys = {'W': False, 'A': False, 'S': False, 'D': False, 'Space': False, 'Up':False, 'Down':False, 'Right': False, 'Left': False, '1': False, '2': False, '3': False}

        self.vel_step = 0.1
        self.velocity = 3.0
    
    def retrieve_key(self, msg):
        key_data = json.loads(msg.data)
        key = key_data['symbol']
        
        if key not in self.pressed_keys:
            self.get_logger().error(f"Keyboard command '{key}' not recognized")
        else:
            self.pressed_keys[key] = key_data['is_pressed']
    
    def send_motor_cmd(self):
        msg1 = String()
        msg2 = Float32()
        
        if self.pressed_keys['Space']:
            for key in self.pressed_keys:
                if key != 'Space':
                    self.pressed_keys[key] = False
            msg1.data = 'Brake'
        
        elif self.pressed_keys['W'] and not (self.pressed_keys['A'] or self.pressed_keys['D'] or self.pressed_keys['S']):
            msg1.data = 'Forward'
        
        elif self.pressed_keys['A'] and not (self.pressed_keys['D'] or self.pressed_keys['S'] or self.pressed_keys['W']):
            msg1.data = 'Left'
        
        elif self.pressed_keys['D'] and not (self.pressed_keys['A'] or self.pressed_keys['S'] or self.pressed_keys['W']):
            msg1.data = 'Right'
        
        elif self.pressed_keys['W'] and self.pressed_keys['A'] and not self.pressed_keys['D'] and not self.pressed_keys['S']:
            msg1.data = 'Forward-left'
        
        elif self.pressed_keys['W'] and self.pressed_keys['D'] and not self.pressed_keys['A'] and not self.pressed_keys['S']:
            msg1.data = 'Forward-right'
        
        elif self.pressed_keys['S'] and not (self.pressed_keys['A'] or self.pressed_keys['D'] or self.pressed_keys['W']):
            msg1.data = 'Backward'
        
        elif self.pressed_keys['S'] and self.pressed_keys['A'] and not self.pressed_keys['D'] and not self.pressed_keys['W']:
            msg1.data = 'Backward-left'
        
        elif self.pressed_keys['S'] and self.pressed_keys['D'] and not self.pressed_keys['A'] and not self.pressed_keys['W']:
            msg1.data = 'Backward-right'
        
        elif self.pressed_keys['Right'] and not (self.pressed_keys['W'] or self.pressed_keys['A'] or self.pressed_keys['S'] or self.pressed_keys['D'] or self.pressed_keys['Left']):
            msg1.data = 'Rotate_CW'

        elif self.pressed_keys['Left'] and not (self.pressed_keys['W'] or self.pressed_keys['A'] or self.pressed_keys['S'] or self.pressed_keys['D'] or self.pressed_keys['Right']):
            msg1.data = 'Rotate_CCW'
            
        elif self.pressed_keys['1'] and not (self.pressed_keys['2'] or self.pressed_keys['3']):
            msg1.data = 'Motor 1'

        elif self.pressed_keys['2'] and not (self.pressed_keys['1'] or self.pressed_keys['3']):
            msg1.data = 'Motor 2'

        elif self.pressed_keys['3'] and not (self.pressed_keys['1'] or self.pressed_keys['2']):
            msg1.data = 'Motor 3'
                  
        else:
            msg1.data = 'Sleep'

        if self.pressed_keys['Up'] and not self.pressed_keys['Down']:
            self.velocity += self.vel_step

        elif self.pressed_keys['Down'] and not self.pressed_keys['Up']:
            self.velocity -= self.vel_step

        msg2.data = self.velocity
            
        self.pub_cmd.publish(msg1)
        self.pub_vel.publish(msg2)


def main(args=None):
    rclpy.init(args=args)
    node = Keyboard_to_cmd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
