from robot_interface import RobotInterface

robot = RobotInterface()
display = robot.robot.getDevice("debug_display")

w = display.getWidth()
h = display.getHeight()

frame = 0

print("Teste de Display (API correta)")

while robot.step():
    # desenha algo
    display.setColor(0x000000)
    display.fillRectangle(0, 0, w, h)

    cx, cy = w // 2, h // 2
    display.setColor(0x00FF00)
    display.drawLine(cx - 50, cy, cx + 50, cy)
    display.drawLine(cx, cy - 50, cx, cy + 50)

    display.setColor(0xFFFFFF)
    display.drawText("DISPLAY OK", 20, 20)

    # salva UMA vez
    if frame == 20:
        image = display.imageCopy(0, 0, w, h)
        display.imageSave(image, "debug_display_test.png")
        display.imageDelete(image)
        print("Imagem salva: debug_display_test.png")

    frame += 1
