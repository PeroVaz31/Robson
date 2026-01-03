import math
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
        # GPS
        pos = self.gps.getValues()
        # Webots Coordinate System:
        # X -> Leste/Oeste
        # Y -> Cima/Baixo (Ignorar)
        # Z -> Norte/Sul
        self.x = pos[0]
        self.y = pos[2] # Usamos Z do Webots como Y do plano 2D

        # Compass
        north = self.compass.getValues()
        
        # O norte magnético no Webots é o eixo Z (geralmente [0, 0, 1])
        # O Norte do plano cartesiano é X (geralmente [1, 0, 0])
        # north[0] = X, north[2] = Z
        rad = math.atan2(north[0], north[2])
        
        # ===== CORREÇÃO DO DESLIZAMENTO =====
        # Antes estava: bearing = (rad - 1.5708)
        # Se o mapa "corre" para a esquerda enquanto o robô vai para direita,
        # significa que estamos 180 graus invertidos.
        # Adicionamos math.pi para girar o mundo mental do robô corretamente.
        bearing = (rad - 1.5708 + math.pi)
        
        # Mantemos a inversão de sinal se a rotação estava correta (não girava o mapa)
        self.theta = -bearing 
        
        # Normalização simples entre -PI e PI para evitar números gigantes
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        return self.x, self.y, self.theta

    def set_speed(self, vl, vr):
        MAX_VAL = 12.3
        vl = max(min(vl, MAX_VAL), -MAX_VAL)
        vr = max(min(vr, MAX_VAL), -MAX_VAL)
        
        self.left_motor.setVelocity(vl)
        self.right_motor.setVelocity(vr)