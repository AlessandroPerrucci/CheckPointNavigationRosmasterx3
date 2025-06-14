import asyncio
import threading
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Odometry
from movimenti_interfacce.action import MovimentoRobot
from Rosmaster_Lib import Rosmaster

class DriverServer(Node):
    def __init__(self):
        super().__init__('nuova_azione_server')
        self._action_server = ActionServer(
            self,
            MovimentoRobot,
            'movimento_robot',
            self.execute_callback
        )
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.orientamento = 0

        self.bot = Rosmaster(com="/dev/myserial")
        self.bot.create_receive_threading()
        self.bot.set_auto_report_state(True)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.get_logger().info(f'Odom x updated: {self.x}')
        self.get_logger().info(f'Odom y updated: {self.y}')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Esecuzione goal...')
        steps = goal_handle.request.steps

        # Avvia l'operazione asincrona nel ciclo asyncio
        asyncio.run_coroutine_threadsafe(self.handle_goal(steps, goal_handle), asyncio.get_event_loop())

        return goal_handle.get_result()

    async def handle_goal(self, steps, goal_handle):
        await self.go_forward(steps)
        goal_handle.succeed()
        result = MovimentoRobot.Result()
        result.success = True
        return result

    async def go_forward(self, steps):
        distance = steps * 0.05
        if self.orientamento == 0:
            target = self.x - distance
            while self.x > target:
                self.bot.set_motor(30, 30, 40, 35)
                await asyncio.sleep(0.1)
        elif self.orientamento == 90:
            target = self.y - distance
            while self.y > target:
                self.bot.set_motor(30, 30, 40, 35)
                await asyncio.sleep(0.1)
        elif self.orientamento == 180:
            target = self.x + distance
            while self.x < target:
                self.bot.set_motor(30, 30, 40, 35)
                await asyncio.sleep(0.1)
        else:
            target = self.y + distance
            while self.y < target:
                self.bot.set_motor(30, 30, 40, 35)
                await asyncio.sleep(0.1)
        self.bot.set_motor(0, 0, 0, 0)

def ros_spin(node):
    rclpy.spin(node)

def main():
    rclpy.init()
    node = DriverServer()
    spin_thread = threading.Thread(target=ros_spin, args=(node,))
    spin_thread.start()

    try:
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()
