class OccupancyGrid:
    """
    Occupancy Grid 2D com ray tracing (Bresenham)
    -1 = desconhecido
     0 = livre
     1 = ocupado
    """

    def __init__(self, size_m=10.0, resolution=0.05):
        self.size_m = size_m
        self.resolution = resolution
        self.cells = int(size_m / resolution)
        self.center = self.cells // 2

        self.grid = [[-1 for _ in range(self.cells)]
                     for _ in range(self.cells)]

    # ----------------------------
    # Conversão de coordenadas
    # ----------------------------
    def world_to_grid(self, x, y):
        gx = int(x / self.resolution) + self.center
        gy = int(y / self.resolution) + self.center
        return gx, gy

    # ----------------------------
    # Bresenham
    # ----------------------------
    def bresenham(self, x0, y0, x1, y1):
        cells = []

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

        return cells

    # ----------------------------
    # Atualização por raio
    # ----------------------------
    def update_ray(self, robot_cell, hit_cell):
        rx, ry = robot_cell
        hx, hy = hit_cell

        cells = self.bresenham(rx, ry, hx, hy)

        # Espaço livre (tudo menos o último)
        for cx, cy in cells[:-1]:
            if 0 <= cx < self.cells and 0 <= cy < self.cells:
                self.grid[cx][cy] = 0

        # Obstáculo
        cx, cy = cells[-1]
        if 0 <= cx < self.cells and 0 <= cy < self.cells:
            self.grid[cx][cy] = 1

    # ----------------------------
    # Atualização do mapa
    # ----------------------------
    def update_from_points(self, points):
        robot_cell = (self.center, self.center)

        for x, y in points:
            gx, gy = self.world_to_grid(x, y)
            if 0 <= gx < self.cells and 0 <= gy < self.cells:
                self.update_ray(robot_cell, (gx, gy))

    # ----------------------------
    # Desenho no Display
    # ----------------------------
    def draw(self, display):
        w = display.getWidth()
        h = display.getHeight()
        scale = w / self.cells

        for i in range(self.cells):
            for j in range(self.cells):
                value = self.grid[i][j]

                if value == -1:
                    color = 0x222222  # desconhecido
                elif value == 0:
                    color = 0x777777  # livre
                else:
                    color = 0x00FF00  # ocupado

                display.setColor(color)
                display.fillRectangle(
                    int(i * scale),
                    int(j * scale),
                    int(scale),
                    int(scale)
                )

    # ----------------------------
    # Conversão grid → mundo
    # ----------------------------
    def grid_to_world(self, gx, gy):
        x = (gx - self.center) * self.resolution
        y = (gy - self.center) * self.resolution
        return x, y
