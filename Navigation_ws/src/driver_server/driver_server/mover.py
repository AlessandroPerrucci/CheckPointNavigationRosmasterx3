#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from Rosmaster_Lib import Rosmaster
robot=Rosmaster(com="/dev/myserial")
class CmdVelSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10  # dimensione della coda
        )
        self.subscription  # evita warning inutilizzati

    def listener_callback(self, msg):
        self.get_logger().info(f'Ricevuto: linear={msg.linear}, angular={msg.angular}')
        if msg.linear.x>0:
            robot.set_motor(30,31,39,36)
        else:
            robot.set_motor(0, 0, 0, 0)

def main(args=None):
    try:
        rclpy.init(args=args)
        node = CmdVelSubscriber()
        rclpy.spin(node)
    finally:
        del robot
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

