
from rclpy.executors import SingleThreadedExecutor

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav_msgs.msg import Odometry
from movimenti_interfacce.action import MovimentoRobot
from Rosmaster_Lib import Rosmaster
import time

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
        self.subscription

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.get_logger().info('Action server ready.')
        self.bot = Rosmaster(com="/dev/myserial")
        self.bot.create_receive_threading()
        self.bot.set_auto_report_state(True)
        self.rate = self.create_rate(10)
        self.orientamento=0

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        #self.get_logger().info(f'Odom x updated: {self.x}')
        #self.get_logger().info(f'Odom y updated: {self.y}')

    async def calcolodir(self,orientamento_atteso):
        cont=0
        orientamento = self.orientamento
        while not(orientamento == orientamento_atteso):
            orientamento=(orientamento+90)%360
            cont+=1
        if cont > 2:
            await self.go_turn("destra")
        else :
            await self.go_turn("sinistra")

        return

    async def execute_callback(self, goal):
        self.get_logger().info('Esecuzione goal...')
        """
        while not (self.orientamento == goal.request.orientamento_atteso):
            await self.calcolodir(goal.request.orientamento_atteso)
        """
        #success = await self.go_turn('sinistra')
        success = await self.go_forward(steps=goal.request.steps)


        #TODO CREA IF DI SELEZIONE

        result = MovimentoRobot.Result()
        if success:
            goal.succeed()
            result.success = True
        else:
            goal.abort()
            result.success = False

        #result.success = success

        #result.message = "Sono andato avanti di: " + (goal.request.steps * 5) + "cm"
        return result



    async def go_forward(self, steps):
        if self.orientamento == 0:
            dx=0
            distanzagoal=steps*0.05
            s = time.time()
            target = self.x - (steps * 0.05)
           # self.get_logger().info(f"{self.x} ---------------- {target}")
            self.bot.set_motor(30, 30, 40, 35)
            while dx<=distanzagoal:
                f = time.time()
                vx, vy, _ = self.bot.get_motion_data()
                distance_moved = vx * abs(s - f)
                dx += (distance_moved)
                s = time.time()
                print(vx)
            self.bot.set_motor(0, 0, 0, 0)
            print(distance_moved)
        elif self.orientamento == 90:
            target = self.y - (steps * 0.05)
            while self.y > target:
                self.bot.set_motor(30, 30, 40, 35)
                time.sleep(0.1)
            self.bot.set_motor(0, 0, 0, 0)

        elif self.orientamento == 180:
            target = self.x + (steps * 0.05)
            while self.x < target:
                self.bot.set_motor(30, 30, 40, 35)
                time.sleep(0.1)
            self.bot.set_motor(0, 0, 0, 0)

        else:
            target = self.y + (steps * 0.05)
            while self.y < target:
                self.bot.set_motor(30, 30, 40, 35)
                time.sleep(0.1)
            self.bot.set_motor(0, 0, 0, 0)

        self.bot.set_motor(0, 0, 0, 0)
        return True

    async def go_turn(self, direzione):
        imu= self.bot.get_imu_attitude_data()[2]

        if direzione == "sinistra":
            posfin= ((imu+360)%360+ 90)%360
            while abs((((self.bot.get_imu_attitude_data()[2])+360)%360)-posfin)>3:
                self.bot.set_car_motion(0,0,0.6)
            self.bot.set_car_motion(0,0,0)
            self.orientamento=(self.orientamento+90)%360
        elif direzione == "destra":
            posfin = ((imu + 360) % 360 -90)%360
            while abs((((self.bot.get_imu_attitude_data()[2])+360)%360)-posfin)>3:
                self.bot.set_car_motion(0,0,-0.6)
            self.bot.set_car_motion(0,0,0)
            self.orientamento = (self.orientamento - 90) % 360

        return True


def main(args=None):
    rclpy.init(args=args)
    server = DriverServer()

    executor = SingleThreadedExecutor()
    executor.add_node(server)


    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

