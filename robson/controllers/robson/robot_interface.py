from controller import Robot
import math

class RobotInterface:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # ----------------------------
        # Motores (Pioneer 3-DX)
        # ----------------------------
        self.left_motor = self.robot.getDevice("left wheel")
        self.right_motor = self.robot.getDevice("right wheel")

        if self.left_motor is None or self.right_motor is None:
            raise RuntimeError("Motores não encontrados. Verifique os nomes no Webots.")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # ----------------------------
        # Compass (orientação real)
        # ----------------------------
        self.compass = self.robot.getDevice("compass")
        if self.compass:
            self.compass.enable(self.timestep)

    # ----------------------------
    # Movimento
    # ----------------------------
    def set_speed(self, left, right):
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def step(self):
        return self.robot.step(self.timestep) != -1

    # ----------------------------
    # Orientação real (yaw)
    # ----------------------------
    def get_yaw(self):
        """
        Retorna yaw (rad) usando Compass
        """
        if not self.compass:
            return 0.0

        values = self.compass.getValues()
        # Convenção Webots: atan2(x, z)
        yaw = math.atan2(values[0], values[2])
        return yaw
