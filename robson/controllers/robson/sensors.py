import math

class LidarSensor:
    def __init__(self, robot_interface, name="Velodyne VLP-16", layer_index=7):
        self.robot = robot_interface.robot
        self.timestep = robot_interface.timestep

        self.lidar = self.robot.getDevice(name)
        if self.lidar is None:
            raise RuntimeError(f"LiDAR '{name}' não encontrado.")

        self.lidar.enable(self.timestep)

        # Configurações
        self.layer_index = layer_index
        self.num_layers = self.lidar.getNumberOfLayers()
        self.h_res = self.lidar.getHorizontalResolution()
        self.fov = self.lidar.getFov()  # rad

        # Display para debug (opcional)
        self.display = self.robot.getDevice("debug_display")

    def get_2d_layer_ranges(self):
        ranges = self.lidar.getRangeImage()
        if not ranges:
            return None

        start = self.layer_index * self.h_res
        end = start + self.h_res
        return ranges[start:end]

    def ranges_to_points_xy(self, ranges):
        """Converte ranges 2D em pontos (x,y) no frame do robô."""
        points = []
        if not ranges:
            return points

        angle_min = -self.fov / 2.0
        angle_inc = self.fov / (len(ranges) - 1)

        for i, r in enumerate(ranges):
            if r <= 0.01 or math.isinf(r):
                continue
            theta = angle_min + i * angle_inc
            x = r * math.cos(theta)
            y = r * math.sin(theta)
            points.append((x, y))
        return points

    def debug_draw_points(self, points, scale=15):
        """Desenha pontos no Display para validação visual."""
        if not self.display:
            return

        w = self.display.getWidth()
        h = self.display.getHeight()
        cx, cy = w // 2, h // 2

        self.display.setColor(0x000000)
        self.display.fillRectangle(0, 0, w, h)

        self.display.setColor(0x00FF00)
        for x, y in points:
            px = int(cx + x * scale)
            py = int(cy - y * scale)
            if 0 <= px < w and 0 <= py < h:
                self.display.drawPixel(px, py)

        # Eixo frontal (vermelho)
        self.display.setColor(0xFF0000)
        self.display.drawLine(cx, cy, cx + 20, cy)
