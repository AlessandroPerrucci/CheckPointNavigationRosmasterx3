from rclpy.executors import MultiThreadedExecutor

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

        self.x = 0.0
        self.y = 0.0
        self.success = False
        self.moving = False
        self.get_logger().info('Action server ready.')
        self.bot = Rosmaster(com="/dev/myserial")
        self.bot.create_receive_threading()
        self.bot.set_auto_report_state(True)
        self.orientamento = 0



    async def calcolodir(self, orientamento_atteso):
        cont = 0
        orientamento = self.orientamento
        while not (orientamento == orientamento_atteso):
            orientamento = (orientamento + 90) % 360
            cont += 1
        if cont > 2:
            await self.go_turn("destra")
        else:
            await self.go_turn("sinistra")

        return

    async def execute_callback(self, goal):
        self.get_logger().info('Esecuzione goal...')

        while not (self.orientamento == goal.request.orientamento_atteso):
            await self.calcolodir(goal.request.orientamento_atteso)
            time.sleep(1.5)
            print(self.bot.get_imu_attitude_data()[2])
        time.sleep(0.5)
        success = await self.go_forward(steps=goal.request.steps)
        result = MovimentoRobot.Result()
        if success:
            goal.succeed()
            result.success = True
        else:
            goal.abort()
            result.success = False

        # result.success = success

        # result.message = "Sono andato avanti di: " + (goal.request.steps * 5) + "cm"
        return result


    async def go_forward(self, steps):
        #print(self.bot.get_motor_encoder()[0])
        distanza_target = (steps * 0.05)-0.125  # metri
        enc_in=self.bot.get_motor_encoder()[0]


        self.bot.set_motor(30, 31, 39, 36)

        # Usa gli encoder per misurare la distanza
        distanza_percorsa = 0
        while distanza_percorsa < distanza_target:
            DIAMETRO_RUOTA = 0.06  # metri
            PASSI_PER_GIRO = 1205
            CIRCONFERENZA = 3.14159 * DIAMETRO_RUOTA
            METRI_PER_PASSO = CIRCONFERENZA / PASSI_PER_GIRO

            encoder_data = self.bot.get_motor_encoder()[0]
            distanza_percorsa = (encoder_data-enc_in) * METRI_PER_PASSO
            time.sleep(0.01)
            #print(encoder_data-enc_in)
        self.bot.set_motor(0, 0, 0, 0)
        #print(self.bot.get_motor_encoder()[0] - enc_in)
        return True

    async def go_turn(self, direzione):
        if direzione == "sinistra":
            posfin = (self.orientamento + 90) % 360
            while abs((((self.bot.get_imu_attitude_data()[2]) + 360) % 360) - posfin) > 2.5:
                self.bot.set_car_motion(0, 0, 0.6)
            self.bot.set_car_motion(0, 0, 0)
            self.orientamento = (self.orientamento + 90) % 360
        elif direzione == "destra":
            posfin = (self.orientamento - 90) % 360
            while abs((((self.bot.get_imu_attitude_data()[2]) + 360) % 360) - posfin) > 2.5:
                self.bot.set_car_motion(0, 0, -0.6)
            self.bot.set_car_motion(0, 0, 0)
            self.orientamento = (self.orientamento - 90) % 360

        return True


def main(args=None):
    rclpy.init(args=args)
    server = DriverServer()

    executor = MultiThreadedExecutor(num_threads=4)

    try:
        rclpy.spin(server, executor)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
