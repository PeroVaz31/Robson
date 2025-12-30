from controller import Keyboard
from robot_interface import RobotInterface
from motion import MotionController
from sensors import LidarSensor
from mapping import OccupancyGrid
from planning import AStarPlanner
import math

# ----------------------------
# Inicialização
# ----------------------------
robot = RobotInterface()
motion = MotionController(robot)
lidar = LidarSensor(robot, name="Velodyne VLP-16", layer_index=7)

display = robot.robot.getDevice("debug_display")
grid = OccupancyGrid(size_m=10.0, resolution=0.05)
planner = AStarPlanner(grid)

keyboard = Keyboard()
keyboard.enable(robot.timestep)

print("Mapeamento + Planejamento + Execução (Compass)")
print("P = planejar | F = seguir | Q = sair")

# ----------------------------
# Estado do robô (posição simples)
# ----------------------------
robot_x = 0.0
robot_y = 0.0

# ----------------------------
# Planejamento
# ----------------------------
path = None
path_index = 0
following = False

def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

# ----------------------------
# Loop principal
# ----------------------------
while robot.step():
    key = keyboard.getKey()

    # -------- Encerrar --------
    if key == ord('Q'):
        print("Encerrando...")
        break

    # -------- Planejar --------
    if key == ord('P'):
        start = (grid.center, grid.center)
        goal_world = (2.0, 1.0)   # goal fixo
        goal = grid.world_to_grid(*goal_world)

        path = planner.plan(start, goal)
        if path:
            path_index = 0
            following = False
            print(f"Caminho planejado com {len(path)} células")
        else:
            print("Falha no planejamento")

    # -------- Seguir --------
    if key == ord('F') and path:
        following = True
        print("Seguindo caminho")

    # -------- Mapeamento contínuo --------
    ranges = lidar.get_2d_layer_ranges()
    points = lidar.ranges_to_points_xy(ranges)
    grid.update_from_points(points)
    grid.draw(display)

    # -------- Execução do caminho --------
    if following and path_index < len(path):
        gx, gy = path[path_index]
        tx, ty = grid.grid_to_world(gx, gy)

        dx = tx - robot_x
        dy = ty - robot_y
        dist = math.hypot(dx, dy)

        # ORIENTAÇÃO REAL
        robot_theta = robot.get_yaw()
        desired_theta = math.atan2(dy, dx)
        angle_error = normalize_angle(desired_theta - robot_theta)

        # Controle proporcional
        Kp_ang = 1.5
        Kp_lin = 0.5

        omega = Kp_ang * angle_error
        v = Kp_lin * dist

        # Saturação
        omega = max(min(omega, 1.0), -1.0)
        v = max(min(v, 0.6), 0.0)

        # Muito desalinhado → gira no lugar
        if abs(angle_error) > 0.6:
            if omega > 0:
                motion.rotate_left(abs(omega))
            else:
                motion.rotate_right(abs(omega))
        else:
            # Movimento suave
            left = v - omega
            right = v + omega
            robot.set_speed(left, right)

            # Atualização grosseira da posição
            robot_x += 0.03 * math.cos(robot_theta)
            robot_y += 0.03 * math.sin(robot_theta)

        # Chegou no waypoint
        if dist < 0.15:
            path_index += 1

    else:
        motion.forward(0.3)

# ----------------------------
# Salvar mapa final
# ----------------------------
w = display.getWidth()
h = display.getHeight()

image = display.imageCopy(0, 0, w, h)
display.imageSave(image, "final_map.png")
display.imageDelete(image)

print("Mapa salvo em final_map.png")
print("Controller finalizado")
