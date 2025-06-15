import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from movimenti_interfacce.action import MovimentoRobot
import cv2
import heapq
import numpy as np
from copy import deepcopy

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def filtrones(pit):
    pitt = deepcopy(pit)  # Copia indipendente
    i = len(pitt) - 2  # Parti dal penultimo elemento

    while i >= 1:  # Scorri all'indietro per evitare conflitti con .remove()
        if (i < len(pitt) - 1 and
            pitt[i][0] == pitt[i-1][0] and
            pitt[i][0] == pitt[i+1][0]):
            pitt.remove(pitt[i])
        elif (i < len(pitt) - 1 and
              pitt[i][1] == pitt[i-1][1] and
              pitt[i][1] == pitt[i+1][1]):
            pitt.remove(pitt[i])
        i -= 1
    azioni= [[0 for _ in range(2)] for _ in range(len(pitt)-1)]

    for i in range(len(pitt)-1):
        if pitt[i+1][0]>pitt[i][0]:
            azioni[i][0]=180 #direzione
        if pitt[i + 1][0] < pitt[i][0]:
            azioni[i][0] = 0  #direzione
        if pitt[i + 1][1] > pitt[i][1]:
            azioni[i][0] = 270  #direzione
        if pitt[i + 1][1] < pitt[i][1]:
            azioni[i][0] = 90  #direzione
        st1=abs(pitt[i+1][0]-pitt[i][0])
        st2=abs(pitt[i+1][1]-pitt[i][1])
        azioni[i][1]=max(st1, st2) #step

    return azioni

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar_dircost(array, start, goal):
    rows, cols = array.shape
    open_set = []

    # Inizializziamo con tutte e 4 le direzioni possibili dal punto di partenza
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Su, Giù, Sinistra, Destra

    # Aggiungiamo il punto di partenza con ogni possibile direzione iniziale
    for dx, dy in neighbors:
        initial_neighbor = (start[0] + dx, start[1] + dy)
        if (0 <= initial_neighbor[0] < rows and
                0 <= initial_neighbor[1] < cols and
                array[initial_neighbor] == 1):
            direction = (dx, dy)
            g_cost = 1  # Costo base per il primo movimento
            f_cost = g_cost + heuristic(initial_neighbor, goal)
            heapq.heappush(open_set, (f_cost, g_cost, initial_neighbor, start, direction))

    # Dizionario per tracciare il percorso: (posizione, direzione) -> (genitore, direzione_genitore)
    came_from = {}
    # Dizionario per i g_score: (posizione, direzione) -> costo
    g_score = {}

    # Inizializziamo i g_score per il punto di partenza
    for dx, dy in neighbors:
        initial_neighbor = (start[0] + dx, start[1] + dy)
        if (0 <= initial_neighbor[0] < rows and
                0 <= initial_neighbor[1] < cols and
                array[initial_neighbor] == 1):
            direction = (dx, dy)
            g_score[(initial_neighbor, direction)] = 1
            came_from[(initial_neighbor, direction)] = (start, None)

    closed_set = set()

    while open_set:
        current_f, current_g, current_pos, parent_pos, current_dir = heapq.heappop(open_set)

        # Usiamo (posizione, direzione) come chiave per il closed_set
        state_key = (current_pos, current_dir)
        if state_key in closed_set:
            continue

        closed_set.add(state_key)

        # Se abbiamo raggiunto il goal, ricostruiamo il percorso
        if current_pos == goal:
            path = []
            current_state = state_key

            # Ricostruiamo il percorso
            while current_state and current_state[0] != start:
                path.append(current_state[0])
                current_state = came_from.get(current_state)

            path.append(start)
            return path[::-1]

        # Esploriamo i vicini
        for dx, dy in neighbors:
            neighbor_pos = (current_pos[0] + dx, current_pos[1] + dy)
            neighbor_dir = (dx, dy)

            if (0 <= neighbor_pos[0] < rows and
                    0 <= neighbor_pos[1] < cols and
                    array[neighbor_pos] == 1):

                # Calcolo del costo: movimento base + penalità per cambio direzione
                move_cost = 1
                penalty = 0
                if neighbor_dir != current_dir:  # Cambio direzione
                    penalty = 1  # Puoi modificare questa penalità

                tentative_g_score = current_g + move_cost + penalty
                neighbor_state = (neighbor_pos, neighbor_dir)

                # Se non abbiamo mai visitato questo stato o abbiamo trovato un percorso migliore
                if (neighbor_state not in g_score or
                        tentative_g_score < g_score[neighbor_state]):
                    g_score[neighbor_state] = tentative_g_score
                    f_score = tentative_g_score + heuristic(neighbor_pos, goal)
                    came_from[neighbor_state] = (current_pos, current_dir)

                    heapq.heappush(open_set, (f_score, tentative_g_score,
                                              neighbor_pos, current_pos, neighbor_dir))

    return None


class MovimentoClient(Node):

    def __init__(self):
        super().__init__('movimento_client')
        self._action_client = ActionClient(self, MovimentoRobot, '/movimento_robot')

    def send_goal_sync(self, steps, orientamento_atteso):
        # Attendi che il server sia disponibile
        self._action_client.wait_for_server()

        # Crea e imposta il goal
        goal_msg = MovimentoRobot.Goal()
        goal_msg.steps = steps
        goal_msg.orientamento_atteso = orientamento_atteso

        self.get_logger().info(f'Invio goal: steps={steps}, orientamento={orientamento_atteso}')

        # Invio goal asincrono ma uso callback per i feedback
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Attendi che il goal venga accettato
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('❌ Goal rifiutato dal server')
            return

        self.get_logger().info('✅ Goal accettato, in attesa del risultato...')

        # Attendi risultato
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result().result

        self.get_logger().info(f'🎯 Risultato: success={result.success}, message="{result.message}"\n')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'📶 Feedback - progresso: {feedback.progress:.2f}')


def main(args=None):
    rclpy.init(args=args)
    client = MovimentoClient()

    try:
        scelta = int(input("inserire 1 se si vuole provare, 0 per automatico"))
        if scelta==1:
            while rclpy.ok():
                try:
                    # Input utente
                    steps = int(input("Inserisci il numero di steps (0 per uscire): "))
                    if steps == 0:
                        break

                    orientamento = int(input("Inserisci l'orientamento atteso (es. 90): "))
                    client.send_goal_sync(steps, orientamento)

                except ValueError:
                    print("⚠ Inserisci solo numeri interi validi.")
        else:
            #print(os.path.abspath("./mappaCorridoio.pgm"))
            pgm_path = '/ros_ws/client_controller/client_controller/mappaLaboratorio.pgm'
            print(os.path.abspath(pgm_path))
            map_img = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            _, bin_map = cv2.threshold(map_img, 205, 255, cv2.THRESH_BINARY)  # 200: threshold tipico
            # bin_map: 0 = occupato, 255 = libero
            # Se serve in formato 0/1:
            bin_map = (bin_map == 255).astype(np.uint8)

            # erosione-->dilatazione (apertura)
            kernel = np.ones((7, 7), np.uint8)  # Crea un kernel di 5x5
            eroded_map = cv2.erode(bin_map, kernel, iterations=1)
            kernel = np.ones((3, 3), np.uint8)
            dilated_map = cv2.dilate(eroded_map, kernel, iterations=1)
            rotated_map = np.rot90(dilated_map, k=-1)
            """originx=29
            originy=174
            checkpointx=10
            checkpointy=37
            start=(originy, originx)
            fin=(checkpointy, checkpointx)
            mosse=filtrones(astar_dircost(rotated_map, start, fin))"""
            #print(mosse)

            cord = [
                [172, 29],
                [30, 8],
                [14, 29],
                [30,8],
                [172,29],

            ]
            for j in range(0, len(cord) - 1):
                start = (cord[j][0], cord[j][1])
                fin = (cord[j + 1][0], cord[j + 1][1])
                mosse = filtrones(astar_dircost(rotated_map, start, fin))
                i = 0
                while rclpy.ok():
                    if i < len(mosse):
                        client.send_goal_sync(mosse[i][1], mosse[i][0])
                        i += 1
                    else:
                        break

    except KeyboardInterrupt:
        print("\n⛔ Interrotto dall'utente.")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
