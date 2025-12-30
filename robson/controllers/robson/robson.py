from controller import Keyboard
from robot_interface import RobotInterface
from motion import MotionController
from sensors import LidarSensor
from mapping import OccupancyGrid

# ----------------------------
# Inicialização
# ----------------------------
robot = RobotInterface()
motion = MotionController(robot)
lidar = LidarSensor(robot, name="Velodyne VLP-16", layer_index=7)

display = robot.robot.getDevice("debug_display")
grid = OccupancyGrid(size_m=10.0, resolution=0.05)

keyboard = Keyboard()
keyboard.enable(robot.timestep)

print("Mapeamento iniciado")
print("Pressione Q para parar e salvar o mapa")

# ----------------------------
# Loop principal
# ----------------------------
running = True

while robot.step() and running:
    # Leitura do teclado
    key = keyboard.getKey()
    if key == ord('Q'):
        print("Tecla Q pressionada. Finalizando...")
        break

    # LiDAR → pontos
    ranges = lidar.get_2d_layer_ranges()
    points = lidar.ranges_to_points_xy(ranges)

    # Atualiza mapa
    grid.update_from_points(points)
    grid.draw(display)

    # Movimento simples
    motion.forward(0.3)

# ----------------------------
# Salvamento FINAL do mapa
# ----------------------------
w = display.getWidth()
h = display.getHeight()

image = display.imageCopy(0, 0, w, h)
display.imageSave(image, "final_map.png")
display.imageDelete(image)

print("Mapa salvo em final_map.png")
print("Controller finalizado")
