from controller import Robot


class RobotInterface:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Motores (Pioneer 3SX)
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def step(self):
        return self.robot.step(self.timestep) != -1

    def set_speed(self, left, right):
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def stop(self):
        self.set_speed(0.0, 0.0)
