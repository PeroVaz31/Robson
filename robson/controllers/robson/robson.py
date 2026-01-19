from controller import Keyboard
from robot_interface import RobotInterface
from sensors import LidarSensor
from mapping import OccupancyGrid
from planning import AStarPlanner
import math
import cv2
import numpy as np

# ==================================================
# Inicialização
# ==================================================
robot = RobotInterface()
lidar = LidarSensor(robot, name="Velodyne VLP-16")

# Aumentei um pouco a resolução para o A* ser mais rápido e tolerante
grid = OccupancyGrid(size_m=20.0, resolution=0.1) 
planner = AStarPlanner(grid)

keyboard = Keyboard()
keyboard.enable(robot.timestep)

cv2.namedWindow("MAP", cv2.WINDOW_NORMAL)

# ==================================================
# Estado
# ==================================================
path = None
trajectory = []
following = False

LOOKAHEAD_DISTANCE = 0.6
BASE_SPEED = 4.0 # Aumentei um pouco, 1.0 é muito lento para Pioneer
GOAL_THRESHOLD = 0.4

def find_lookahead(path, x, y, Ld):
    target = None
    # Procura o último ponto do caminho que está a uma distância Ld
    # ou o mais próximo possível fora desse raio
    for px, py in reversed(path):
        dist = math.hypot(px - x, py - y)
        if dist <= Ld:
             # Achamos um ponto dentro do raio, o target deve ser o próximo (que está fora)
             # Mas simplificando: pegamos o ponto logo na borda do raio
             pass
        else:
            target = (px, py)
            # Se achamos um ponto longe o suficiente (de trás pra frente), é ele
            return target
    
    # Fallback: se todos pontos estão perto, vai pro final
    if path:
        return path[-1]
    return None

# ==================================================
# Loop principal
# ==================================================
while robot.step():
    key = keyboard.getKey()

    # ---------- Encerrar ----------
    if key == ord('Q'):
        break

    # ---------- Planejar (Tecla P) ----------
    if key == ord('P'):
        start = grid.world_to_grid(robot.x, robot.y)
        # Define um goal fixo para teste ou aleatório se preferir
        goal_world = (2.0, 0.0) 
        goal = grid.world_to_grid(*goal_world)

        grid_path = planner.plan(start, goal)
        
        if grid_path:
            path = [grid.grid_to_world(x, y) for x, y in grid_path]
            following = False
            print(f"Caminho encontrado: {len(path)} passos")
        else:
            print("Caminho não encontrado!")

    # ---------- Seguir (Tecla F) ----------
    if key == ord('F') and path:
        following = True
        print("Iniciando navegação...")

    # ==================================================
    # 1. PEGAR ESTADO
    # ==================================================
    robot_pose = robot.get_pose()
    x, y, theta = robot_pose

    pos = robot.gps.getValues()
    #rint(f"GPS raw -> x:{pos[0]:.3f}  y:{pos[1]:.3f}  z:{pos[2]:.3f}")

    
    # Armazena trajetória para debug visual
    trajectory.append((x, y))

    # ==================================================
    # 2. MAPEAMENTO
    # ==================================================
    points = lidar.get_2d_points()
    grid.update_from_points(points, robot_pose)

    # ==================================================
    # 3. CONTROLE (PURE PURSUIT)
    # ==================================================
    if following and path:
        gx, gy = path[-1]
        dist_to_goal = math.hypot(gx - x, gy - y)
        
        if dist_to_goal < GOAL_THRESHOLD:
            robot.set_speed(0.0, 0.0)
            following = False
            print("CHEGOU NO OBJETIVO!")
            path = None # Limpa caminho
        else:
            look = find_lookahead(path, x, y, LOOKAHEAD_DISTANCE)
            if look:
                lx, ly = look
                
                # Transformação para frame do robô
                dx = lx - x
                dy = ly - y

                # Rotação da matriz inversa
                # Se theta estiver correto (Bussola corrigida), isso funciona
                x_r = math.cos(theta) * dx + math.sin(theta) * dy
                y_r = -math.sin(theta) * dx + math.cos(theta) * dy

                # Curvatura
                # y_r é o erro lateral. Se for negativo, ponto está à direita (ou esq dependendo do eixo)
                curvature = 2 * y_r / (LOOKAHEAD_DISTANCE ** 2)
                
                v = BASE_SPEED
                # Reduz velocidade em curvas fechadas
                v = v / (1 + abs(curvature)*0.5)
                
                omega = v * curvature

                # Cinemática Pioneer
                L = 0.33 # eixo
                R = 0.0975 # roda
                
                vl = (v - omega * L / 2) / R
                vr = (v + omega * L / 2) / R

                robot.set_speed(vl, vr)
            else:
                 # Sem lookahead válido
                 robot.set_speed(0.0, 0.0)

    else:
        # Modo manual ou parado
        # Adicione controle manual aqui se quiser (setas)
        robot.set_speed(0.0, 0.0)

    # ==================================================
    # 4. VISUALIZAÇÃO
    # ==================================================
    img = grid.to_cv_image(scale=3) # Scale menor para performance

    # Desenha o robô
    rx, ry = grid.world_to_grid(x, y)
    cv2.circle(img, (rx * 3, ry * 3), 4, (0, 255, 255), -1)
    
    # Desenha direção do robô (uma linha curta)
    end_x = int(rx * 3 + 10 * math.cos(-theta)) # -theta para visualização as vezes
    end_y = int(ry * 3 + 10 * math.sin(-theta))
    # Nota: No OpenCV Y cresce para baixo, no mundo cresce para cima/Norte. 
    # Essa inversão visual é normal.

    # Desenha caminho
    if path:
        pts = []
        for px, py in path:
            gx, gy = grid.world_to_grid(px, py)
            pts.append([gx * 3, gy * 3])
        cv2.polylines(img, [np.array(pts)], False, (0, 0, 255), 1)
        
        # Desenha lookahead point
        if following and 'lx' in locals():
            lgx, lgy = grid.world_to_grid(lx, ly)
            cv2.circle(img, (lgx*3, lgy*3), 3, (255, 0, 255), -1)

    cv2.imshow("MAP", img)
    cv2.waitKey(1)

cv2.destroyAllWindows()