from controller import Robot
import math

class RobotInterface:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        self.dt = self.timestep / 1000.0

        # Motores (Pioneer 3-DX)
        self.left_motor = self.robot.getDevice("left wheel")
        self.right_motor = self.robot.getDevice("right wheel")

        if self.left_motor is None or self.right_motor is None:
            raise RuntimeError("Motores não encontrados.")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Compass
        self.compass = self.robot.getDevice("compass")
        if self.compass:
            self.compass.enable(self.timestep)

        # Parâmetros físicos do Pioneer
        self.wheel_radius = 0.0975      # metros
        self.wheel_base = 0.33          # distância entre rodas (m)

        # Estado (odometria)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v_l = 0.0
        self.v_r = 0.0

    # ----------------------------
    # Movimento
    # ----------------------------
    def set_speed(self, left, right):
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

        # velocidades lineares das rodas
        self.v_l = left * self.wheel_radius
        self.v_r = right * self.wheel_radius

    # ----------------------------
    # Atualização da odometria
    # ----------------------------
    def update_odometry(self):
        v = (self.v_r + self.v_l) / 2.0
        omega = (self.v_r - self.v_l) / self.wheel_base

        self.theta += omega * self.dt
        self.theta = self.normalize_angle(self.theta)

        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def step(self):
        if self.robot.step(self.timestep) == -1:
            return False
        self.update_odometry()
        return True

    # ----------------------------
    # Pose atual
    # ----------------------------
    def get_pose(self):
        return self.x, self.y, self.theta
