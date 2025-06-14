from sshkeyboard import listen_keyboard
import threading
import time
from Rosmaster_Lib import Rosmaster
import sys

robot = Rosmaster(com = "/dev/myserial")
robot.create_receive_threading()
robot.set_auto_report_state(True,False)
print(robot.get_battery_voltage())
# Stato dei tasti
tasti_premuti = {
    'up': False,
    'down': False,
    'left': False,
    'right': False,
    'a': False,
    'd': False
}

# Timestamp dell'ultima pressione di un tasto
ultimo_input = time.time()

# Funzioni associate a ciascuna freccia
def azione_su():
    #robot.set_car_motion(1,0,0)
    robot.set_motor(30, 31, 39, 36)

def azione_giu():
    robot.set_motor(-30, -31, -39, -36)

def azione_sinistra():
    robot.set_car_motion(0, -0.5, 0)

def azione_destra():
    robot.set_car_motion(0, 0.5, 0)

def azione_spinL():
    robot.set_motor(40,40,-40,-40)

def azione_spinR():
    robot.set_motor(-40,-40,40,40)

def azione_idle():
    robot.set_car_motion(0, 0, 0)

# Funzione chiamata alla pressione di un tasto
def on_press(key):
    global ultimo_input
    ultimo_input = time.time()
    if key == "up":
        tasti_premuti['up'] = True
        azione_su()
    elif key == "down":
        tasti_premuti['down'] = True
        azione_giu()
    elif key == "left":
        tasti_premuti['left'] = True
        azione_sinistra()
    elif key == "right":
        tasti_premuti['right'] = True
        azione_destra()
    elif key == "a":
        tasti_premuti['spinL'] = True
        azione_spinL()
    elif key == "d":
        tasti_premuti['spinR'] = True
        azione_spinR()
    elif key in ["q", "esc"]:
        print("Uscita dal programma.")
        sys.exit(0)

# Funzione chiamata al rilascio di un tasto
def on_release(key):
    if key in tasti_premuti:
        tasti_premuti[key] = False
        robot.set_car_motion(0, 0, 0)


try:
    # Avvia il listener della tastiera
    listen_keyboard(on_press=on_press, on_release=on_release)
except KeyboardInterrupt:
    print("\nInterruzione ricevuta. Fermando il robot...")
finally:
    robot.set_car_motion(0, 0, 0)
    robot.clear_auto_report_data()
    del robot
    print("Robot fermato. Uscita dal programma.")
    sys.exit(0)
