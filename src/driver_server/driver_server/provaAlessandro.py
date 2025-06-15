import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from movimenti_interfacce.action import MovimentoRobot
from rclpy.action import ActionServer
from Rosmaster_Lib import Rosmaster
import asyncio
import math


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

        self.movement_timer = None
        self.target_position = None
        self.current_goal_handle = None
        self.correction_timer = None
        self.correction_phase = False

        # Parametri per la correzione
        self.position_tolerance = 0.03  # Tolleranza in metri (2cm)
        self.correction_speed = 20  # Velocità ridotta per correzioni
        self.normal_speed = [30, 30, 40, 35]  # Velocità normali

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Esecuzione goal...')

        # Calcolo della direzione
        if self.orientamento != goal_handle.request.orientamento_atteso:
            self.calcolodir(goal_handle.request.orientamento_atteso)

        steps = goal_handle.request.steps
        if steps > 0:
            # Calcolo della posizione target
            if self.orientamento == 0:
                self.target_position = self.x - (steps * 0.05)
            elif self.orientamento == 90:
                self.target_position = self.y - (steps * 0.05)
            elif self.orientamento == 180:
                self.target_position = self.x + (steps * 0.05)
            else:
                self.target_position = self.y + (steps * 0.05)

            self.current_goal_handle = goal_handle
            self.correction_phase = False
            # Avvio del timer per il controllo del movimento
            self.movement_timer = self.create_timer(0.1, self.movement_callback)
            return MovimentoRobot.Result()
        else:
            goal_handle.succeed()
            result = MovimentoRobot.Result()
            result.success = True
            return result

    def movement_callback(self):
        if not self.correction_phase:
            # Fase di movimento normale
            if self.check_target_reached():
                self.bot.set_motor(0, 0, 0, 0)
                self.movement_timer.cancel()
                # Inizia la fase di correzione
                self.start_correction_phase()
            else:
                # Continuazione del movimento normale
                self.bot.set_motor(*self.normal_speed)
        else:
            # Fase di correzione
            self.correction_callback()

    def check_target_reached(self):
        """Controlla se il robot ha raggiunto la posizione target"""
        if self.orientamento == 0:
            return self.x <= self.target_position
        elif self.orientamento == 90:
            return self.y <= self.target_position
        elif self.orientamento == 180:
            return self.x >= self.target_position
        elif self.orientamento == 270:
            return self.y >= self.target_position
        return False

    def start_correction_phase(self):
        """Inizia la fase di correzione della posizione"""
        self.correction_phase = True
        self.get_logger().info('Iniziando correzione posizione...')
        # Usa lo stesso timer ma con callback diversa
        self.movement_timer = self.create_timer(0.05, self.movement_callback)

    def correction_callback(self):
        """Callback per la correzione fine della posizione"""
        current_pos = self.get_current_position()
        error = self.target_position - current_pos

        self.get_logger().info(f'Errore posizione: {error:.4f}m, Tolleranza: {self.position_tolerance}m')

        if abs(error) <= self.position_tolerance:
            # Posizione corretta raggiunta
            self.bot.set_motor(0, 0, 0, 0)
            self.movement_timer.cancel()
            self.correction_phase = False
            self.get_logger().info('Posizione corretta raggiunta!')

            # Successo dell'azione
            result = MovimentoRobot.Result()
            result.success = True
            self.current_goal_handle.succeed()

        elif error > 0:
            # Deve andare avanti (verso il target)
            self.move_forward_correction()
        else:
            # Deve andare indietro (si è mosso troppo)
            self.move_backward_correction()

    def get_current_position(self):
        """Restituisce la posizione corrente lungo l'asse di movimento"""
        if self.orientamento == 0 or self.orientamento == 180:
            return self.x
        else:  # orientamento == 90 or orientamento == 270
            return self.y

    def move_forward_correction(self):
        """Movimento in avanti per correzione"""
        if self.orientamento == 0:
            # Movimento all'indietro (verso x negativo)
            self.bot.set_motor(-self.correction_speed, -self.correction_speed,
                               -self.correction_speed, -self.correction_speed)
        elif self.orientamento == 90:
            # Movimento all'indietro (verso y negativo)
            self.bot.set_motor(-self.correction_speed, -self.correction_speed,
                               -self.correction_speed, -self.correction_speed)
        elif self.orientamento == 180:
            # Movimento in avanti (verso x positivo)
            self.bot.set_motor(self.correction_speed, self.correction_speed,
                               self.correction_speed, self.correction_speed)
        else:  # orientamento == 270
            # Movimento in avanti (verso y positivo)
            self.bot.set_motor(self.correction_speed, self.correction_speed,
                               self.correction_speed, self.correction_speed)

    def move_backward_correction(self):
        """Movimento all'indietro per correzione"""
        if self.orientamento == 0:
            # Movimento in avanti (verso x positivo)
            self.bot.set_motor(self.correction_speed, self.correction_speed,
                               self.correction_speed, self.correction_speed)
        elif self.orientamento == 90:
            # Movimento in avanti (verso y positivo)
            self.bot.set_motor(self.correction_speed, self.correction_speed,
                               self.correction_speed, self.correction_speed)
        elif self.orientamento == 180:
            # Movimento all'indietro (verso x negativo)
            self.bot.set_motor(-self.correction_speed, -self.correction_speed,
                               -self.correction_speed, -self.correction_speed)
        else:  # orientamento == 270
            # Movimento all'indietro (verso y negativo)
            self.bot.set_motor(-self.correction_speed, -self.correction_speed,
                               -self.correction_speed, -self.correction_speed)

    async def calcolodir(self, orientamento_atteso):
        cont = 0
        orientamento = self.orientamento
        while orientamento != orientamento_atteso:
            orientamento = (orientamento + 90) % 360
            cont += 1
        if cont > 2:
            await self.go_turn("destra")
        else:
            await self.go_turn("sinistra")
    async def go_turn(self, direzione):
        imu = self.bot.get_imu_attitude_data()[2]

        if direzione == "sinistra":
            posfin = ((imu + 360) % 360 + 90) % 360
            while abs((((self.bot.get_imu_attitude_data()[2]) + 360) % 360) - posfin) > 3:
                self.bot.set_car_motion(0, 0, 0.6)
            self.bot.set_car_motion(0, 0, 0)
            self.orientamento = (self.orientamento + 90) % 360
        elif direzione == "destra":
            posfin = ((imu + 360) % 360 - 90) % 360
            while abs((((self.bot.get_imu_attitude_data()[2]) + 360) % 360) - posfin) > 3:
                self.bot.set_car_motion(0, 0, -0.6)
            self.bot.set_car_motion(0, 0, 0)
            self.orientamento = (self.orientamento - 90) % 360

        return True


def main(args=None):
    rclpy.init(args=args)
    server = DriverServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        del server.bot
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
