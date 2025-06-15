from sshkeyboard import listen_keyboard
import time
from Rosmaster_Lib import Rosmaster
import sys
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap


class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver_client')
        self.client = self.create_client(SaveMap, '/slam_toolbox/save_map')

        # Aspetta che il service sia disponibile
        timeout_count = 0
        while not self.client.wait_for_service(timeout_sec=1.0):
            timeout_count += 1
            self.get_logger().info('Service save_map non disponibile, aspetto...')
            if timeout_count > 5:  # Limite di tentativi
                self.get_logger().warn('Service non trovato dopo 5 secondi, continuando comunque...')
                break

    def save_map(self, filename='final_map'):
        if not self.client.service_is_ready():
            self.get_logger().error('Service non disponibile')
            return False

        # Crea la richiesta
        request = SaveMap.Request()
        request.name.data = filename

        self.get_logger().info(f'Tentativo di salvataggio mappa: {filename}')

        # Invia la richiesta
        future = self.client.call_async(request)

        # Aspetta la risposta con timeout
        try:
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)

            if future.done():
                result = future.result()
                if result is not None:
                    self.get_logger().info(f'✓ Mappa salvata con successo: {filename}')
                    print(f"   File salvati: {filename}.pgm e {filename}.yaml")
                    return True
                else:
                    self.get_logger().error('Service ha restituito risultato nullo')
                    return False
            else:
                self.get_logger().error('Timeout durante il salvataggio (15 secondi)')
                return False

        except Exception as e:
            self.get_logger().error(f'Errore durante il salvataggio: {str(e)}')
            return False


def save_map_on_exit(max_retries=5, retry_delay=2.0):
    """Funzione per salvare la mappa all'uscita del programma con retry automatico"""

    for attempt in range(max_retries):
        try:
            print(f"Tentativo {attempt + 1}/{max_retries} - Inizializzando ROS2 per salvare la mappa...")

            # Inizializza ROS2 solo se non è già inizializzato
            if not rclpy.ok():
                rclpy.init()

            map_saver = MapSaver()

            # Verifica che il service sia disponibile prima di procedere
            if not map_saver.client.service_is_ready():
                print("Service non pronto, aspetto che SLAM Toolbox sia disponibile...")
                service_ready = False
                for wait_attempt in range(10):  # Aspetta fino a 10 secondi
                    if map_saver.client.wait_for_service(timeout_sec=1.0):
                        service_ready = True
                        print("✓ Service SLAM Toolbox trovato!")
                        break
                    print(f"   Aspettando... ({wait_attempt + 1}/10)")

                if not service_ready:
                    print("✗ Service SLAM Toolbox non disponibile")
                    map_saver.destroy_node()
                    if attempt < max_retries - 1:
                        print(f"   Riprovo tra {retry_delay} secondi...")
                        time.sleep(retry_delay)
                        continue
                    else:
                        print("✗ Tutti i tentativi falliti - Service non disponibile")
                        return False

            # Tenta il salvataggio
            success = map_saver.save_map('robot_exploration_map')

            map_saver.destroy_node()

            if success:
                print("✓ Mappa salvata con successo!")
                return True
            else:
                print(f"✗ Tentativo {attempt + 1} fallito")
                if attempt < max_retries - 1:
                    print(f"   Riprovo tra {retry_delay} secondi...")
                    time.sleep(retry_delay)
                else:
                    print("✗ Tutti i tentativi di salvataggio falliti")
                    return False

        except Exception as e:
            print(f"✗ Errore durante tentativo {attempt + 1}: {e}")
            if attempt < max_retries - 1:
                print(f"   Riprovo tra {retry_delay} secondi...")
                time.sleep(retry_delay)
            else:
                print("✗ Tutti i tentativi falliti a causa di errori")
                return False
        finally:
            # Assicurati che ROS2 sia chiuso correttamente
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except:
                pass

    return False


# Inizializzazione robot
robot = Rosmaster(com="/dev/myserial")
robot.create_receive_threading()
robot.set_auto_report_state(True, False)
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
    robot.set_motor(30, 31, 39, 36)


def azione_giu():
    robot.set_motor(-30, -31, -39, -36)


def azione_sinistra():
    robot.set_car_motion(0, -0.5, 0)


def azione_destra():
    robot.set_car_motion(0, 0.5, 0)


def azione_spinL():
    robot.set_motor(40, 40, -40, -40)


def azione_spinR():
    robot.set_motor(-40, -40, 40, 40)


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

def main(args=None):
    try:
        print("Controllo robot attivo. Usa le frecce per muovere, A/D per ruotare, Q per uscire.")
        # Avvia il listener della tastiera
        listen_keyboard(on_press=on_press, on_release=on_release)
    except KeyboardInterrupt:
        print("\nInterruzione ricevuta. Fermando il robot...")
    finally:
        print("Fermando il robot...")
        robot.set_car_motion(0, 0, 0)
        robot.clear_auto_report_data()
        del robot
        print("Robot fermato.")

        # Salva la mappa prima di uscire
        print("SALVANDO LA MAPPA...")

        map_saved = save_map_on_exit(max_retries=3, retry_delay=3.0)

        if map_saved:
            print("✓ MAPPA SALVATA CORRETTAMENTE!")
        else:
            print("✗ IMPOSSIBILE SALVARE LA MAPPA!")
            print("Verifica che SLAM Toolbox sia in esecuzione:")

        print("Uscita dal programma.")
        sys.exit(0)

if __name__ == '__main__':
    main()