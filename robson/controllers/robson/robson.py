from robot_interface import RobotInterface

robot = RobotInterface()

print("Controller organizado iniciado")

while robot.step():
    robot.set_speed(2.0, 2.0)
