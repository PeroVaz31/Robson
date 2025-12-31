from controller import Keyboard
from robot_interface import RobotInterface
from motion import MotionController
from sensors import LidarSensor
from mapping import OccupancyGrid
from planning import AStarPlanner
import math

# ============================
# Inicialização
# ============================
robot = RobotInterface()
motion = MotionController(robot)
lidar = LidarSensor(robot, name="Velodyne VLP-16", layer_index=7)

display = robot.robot.getDevice("debug_display")
grid = OccupancyGrid(size_m=10.0, resolution=0.05)
planner = AStarPlanner(grid)

keyboard = Keyboard()
keyboard.enable(robot.timestep)

print("Mapeamento + A* + ODOMETRIA (CORREÇÃO CINEMÁTICA)")
print("P = planejar | F = seguir | Q = sair")

# ============================
# Planejamento
# ============================
path = None
path_index = 0
following = False

# ============================
# Parâmetros físicos (Pioneer 3-DX)
# ============================
WHEEL_RADIUS = 0.0975   # m
WHEEL_BASE = 0.33       # m
MAX_WHEEL_SPEED = 12.3  # rad/s

# ============================
# Parâmetros de controle
# ============================
Kp_ang = 3.0
Kp_lin = 1.5

MAX_LIN = 1.5   # m/s
MAX_ANG = 3.0   # rad/s

ANGLE_ALIGN_THRESHOLD = 0.3
WAYPOINT_THRESHOLD = 0.25

def normalize_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a

# ============================
# Loop principal
# ============================
while robot.step():
    key = keyboard.getKey()

    if key == ord('Q'):
        break

    if key == ord('P'):
        start = grid.world_to_grid(robot.x, robot.y)
        goal_world = (2.0, 1.0)
        goal = grid.world_to_grid(*goal_world)

        path = planner.plan(start, goal)
        if path:
            path_index = 0
            following = False
            print(f"Caminho com {len(path)} pontos")

    if key == ord('F') and path:
        following = True
        print("Seguindo caminho")

    # ----------------------------
    # Mapeamento contínuo
    # ----------------------------
    ranges = lidar.get_2d_layer_ranges()
    points = lidar.ranges_to_points_xy(ranges)
    grid.update_from_points(points)
    grid.draw(display)

    # ----------------------------
    # Seguimento com CINEMÁTICA CORRETA
    # ----------------------------
    if following and path_index < len(path):
        gx, gy = path[path_index]
        tx, ty = grid.grid_to_world(gx, gy)

        x, y, theta = robot.get_pose()

        dx = tx - x
        dy = ty - y
        dist = math.hypot(dx, dy)

        desired_theta = math.atan2(dy, dx)
        angle_error = normalize_angle(desired_theta - theta)

        # Controle P
        v = Kp_lin * dist
        omega = Kp_ang * angle_error

        # Saturação de referência
        v = max(min(v, MAX_LIN), 0.0)
        omega = max(min(omega, MAX_ANG), -MAX_ANG)

        # PRIORIDADE ANGULAR
        if abs(angle_error) > ANGLE_ALIGN_THRESHOLD:
            v = 0.0  # gira no lugar

        # ----------------------------
        # CINEMÁTICA INVERSA CORRETA
        # ----------------------------
        v_l = (v - omega * WHEEL_BASE / 2.0) / WHEEL_RADIUS
        v_r = (v + omega * WHEEL_BASE / 2.0) / WHEEL_RADIUS

        # Saturação real dos motores
        v_l = max(min(v_l, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED)
        v_r = max(min(v_r, MAX_WHEEL_SPEED), -MAX_WHEEL_SPEED)

        robot.set_speed(v_l, v_r)

        if dist < WAYPOINT_THRESHOLD:
            path_index += 1

    else:
        motion.forward(0.5)

# ============================
# Salvar mapa final
# ============================
w = display.getWidth()
h = display.getHeight()
image = display.imageCopy(0, 0, w, h)
display.imageSave(image, "final_map.png")
display.imageDelete(image)
