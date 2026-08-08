#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class Keyboard_to_cmd(Node):
    def __init__(self):
        super().__init__('Key_to_cmd')
        self.sub = self.create_subscription(String, '/key_press', self.retrieve_key, 10)
        self.pub = self.create_publisher(String, '/motor_cmd', 10)
        self.timer = self.create_timer(0.1, self.send_motor_cmd)
        
        self.pressed_keys = {'W': False, 'A': False, 'S': False, 'D': False, 'Space': False, 'Right': False, 'Left': False, '1': False, '2': False, '3': False}
    
    def retrieve_key(self, msg):
        key_data = json.loads(msg.data)
        key = key_data['symbol']
        
        if key not in self.pressed_keys:
            self.get_logger().error(f"Keyboard command '{key}' not recognized")
        else:
            self.pressed_keys[key] = key_data['is_pressed']
    
    def send_motor_cmd(self):
        msg = String()
        
        if self.pressed_keys['Space']:
            for key in self.pressed_keys:
                if key != 'Space':
                    self.pressed_keys[key] = False
            msg.data = 'Brake'
        
        elif self.pressed_keys['W'] and not (self.pressed_keys['A'] or self.pressed_keys['D'] or self.pressed_keys['S']):
            msg.data = 'Forward'
        
        elif self.pressed_keys['A'] and not (self.pressed_keys['D'] or self.pressed_keys['S'] or self.pressed_keys['W']):
            msg.data = 'Left'
        
        elif self.pressed_keys['D'] and not (self.pressed_keys['A'] or self.pressed_keys['S'] or self.pressed_keys['W']):
            msg.data = 'Right'
        
        elif self.pressed_keys['W'] and self.pressed_keys['A'] and not self.pressed_keys['D'] and not self.pressed_keys['S']:
            msg.data = 'Forward-left'
        
        elif self.pressed_keys['W'] and self.pressed_keys['D'] and not self.pressed_keys['A'] and not self.pressed_keys['S']:
            msg.data = 'Forward-right'
        
        elif self.pressed_keys['S'] and not (self.pressed_keys['A'] or self.pressed_keys['D'] or self.pressed_keys['W']):
            msg.data = 'Backward'
        
        elif self.pressed_keys['S'] and self.pressed_keys['A'] and not self.pressed_keys['D'] and not self.pressed_keys['W']:
            msg.data = 'Backward-left'
        
        elif self.pressed_keys['S'] and self.pressed_keys['D'] and not self.pressed_keys['A'] and not self.pressed_keys['W']:
            msg.data = 'Backward-right'
        
        elif self.pressed_keys['Right'] and not (self.pressed_keys['W'] or self.pressed_keys['A'] or self.pressed_keys['S'] or self.pressed_keys['D'] or self.pressed_keys['Left']):
            msg.data = 'Rotate_CW'

        elif self.pressed_keys['Left'] and not (self.pressed_keys['W'] or self.pressed_keys['A'] or self.pressed_keys['S'] or self.pressed_keys['D'] or self.pressed_keys['Right']):
            msg.data = 'Rotate_CCW'
            
        elif self.pressed_keys['1'] and not (self.pressed_keys['2'] or self.pressed_keys['3']):
            msg.data = 'Motor 1'

        elif self.pressed_keys['2'] and not (self.pressed_keys['1'] or self.pressed_keys['3']):
            msg.data = 'Motor 2'

        elif self.pressed_keys['3'] and not (self.pressed_keys['1'] or self.pressed_keys['2']):
            msg.data = 'Motor 3'
                  
        else:
            msg.data = 'Sleep'
        
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Keyboard_to_cmd()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
