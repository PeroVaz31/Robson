import math
from operator import pos
from controller import Robot

class RobotInterface:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # ===============================
        # Motores (Pioneer 3-DX)
        # ===============================
        self.left_motor = self.robot.getDevice("left wheel")
        self.right_motor = self.robot.getDevice("right wheel")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # ===============================
        # GPS (posição GLOBAL)
        # ===============================
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)

        # ===============================
        # Compass (orientação GLOBAL)
        # ===============================
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)

        # ===============================
        # Estado
        # ===============================
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def step(self):
        return self.robot.step(self.timestep) != -1

    def get_pose(self):
        # 1. Pega a Posição (GPS) - ESSENCIAL
        pos = self.gps.getValues()
        self.x = pos[0]
        self.y = pos[1]

        # 2. Pega a Rotação (Compass)
        north = self.compass.getValues()
        rad = math.atan2(north[2], north[0])
        
        # --- A CORREÇÃO CERTA ---
        # Mantemos 'rad' positivo (para a rotação acompanhar o robô corretamente)
        # E subtraímos 1.5708 (PI/2) para alinhar o Norte do Webots com o X do mapa.

        self.theta = -rad 
        
        # Normalização entre -PI e PI
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # DEBUG: Se ainda der pau, descomente a linha abaixo para ler o ângulo no console
        # print(f"Theta: {self.theta:.2f}")

        return self.x, self.y, self.theta

    def set_speed(self, vl, vr):
        MAX_VAL = 12.3
        vl = max(min(vl, MAX_VAL), -MAX_VAL)
        vr = max(min(vr, MAX_VAL), -MAX_VAL)
        
        self.left_motor.setVelocity(vl)
        self.right_motor.setVelocity(vr)