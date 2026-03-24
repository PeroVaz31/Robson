from controller import Robot
import pygame
import numpy as np
import matplotlib.pyplot as plt

# ==============================
# Inicialização do robô
# ==============================

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# ==============================
# Motores
# ==============================

left_motor = robot.getDevice("left wheel")
right_motor = robot.getDevice("right wheel")

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

MAX_SPEED = 4.0

# ==============================
# LiDAR
# ==============================

lidar = robot.getDevice("Hokuyo URG-04LX")  # CONFERE O NOME
lidar.enable(timestep)

# ==============================
# Joystick
# ==============================

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("Nenhum controle detectado")
    joystick = None
else:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print("Controle detectado:", joystick.get_name())

# ==============================
# Visualização
# ==============================

plt.ion()
fig, ax = plt.subplots()

# ==============================
# Loop principal
# ==============================

while robot.step(timestep) != -1:

    # ==========================
    # CONTROLE
    # ==========================

    if joystick is not None:
        pygame.event.pump()

        forward = -joystick.get_axis(1)
        turn = -joystick.get_axis(2)

        left_speed = (forward - turn) * MAX_SPEED
        right_speed = (forward + turn) * MAX_SPEED

        # normalização
        max_val = max(abs(left_speed), abs(right_speed))
        if max_val > MAX_SPEED:
            left_speed = left_speed / max_val * MAX_SPEED
            right_speed = right_speed / max_val * MAX_SPEED

        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

    # ==========================
    # LIDAR
    # ==========================

    ranges = lidar.getRangeImage()

    fov = lidar.getFov()
    num_points = len(ranges)

    angles = np.linspace(-fov/2, fov/2, num_points)

    x = []
    y = []

    for i in range(num_points):
        r = ranges[i]

        if r == float('inf'):
            continue

        angle = angles[i]

        x.append(r * np.cos(angle))
        y.append(-r * np.sin(angle))

    # ==========================
    # VISUALIZAÇÃO
    # ==========================

    ax.clear()
    ax.scatter(y, x, s=5)

    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_title("LiDAR 2D")

    plt.pause(0.001)