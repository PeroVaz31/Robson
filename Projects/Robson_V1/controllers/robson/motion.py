class MotionController:
    def __init__(self, robot_interface, max_speed=5.0):
        self.robot = robot_interface
        self.max_speed = max_speed

    def stop(self):
        self.robot.set_speed(0.0, 0.0)

    def forward(self, speed_ratio=0.5):
        speed = self.max_speed * speed_ratio
        self.robot.set_speed(speed, speed)

    def rotate_left(self, speed_ratio=0.3):
        speed = self.max_speed * speed_ratio
        self.robot.set_speed(-speed, speed)

    def rotate_right(self, speed_ratio=0.3):
        speed = self.max_speed * speed_ratio
        self.robot.set_speed(speed, -speed)
