from robot_interface import RobotInterface

robot = RobotInterface()
display = robot.robot.getDevice("debug_display")

w = display.getWidth()
h = display.getHeight()

frame = 0

print("Teste de Display (salvando imagem)")

while robot.step():
    # fundo preto
    display.setColor(0x000000)
    display.fillRectangle(0, 0, w, h)

    # cruz no centro
    cx, cy = w // 2, h // 2
    display.setColor(0x00FF00)
    display.drawLine(cx - 50, cy, cx + 50, cy)
    display.drawLine(cx, cy - 50, cx, cy + 50)

    display.setColor(0xFFFFFF)
    display.drawText("DISPLAY OK", 20, 20)

    # salva UMA imagem
    if frame == 20:
        display.imageSave("debug_display_test.png")
        print("Imagem salva: debug_display_test.png")

    frame += 1
