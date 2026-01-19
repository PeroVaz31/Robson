import numpy as np
import cv2
import math


class OccupancyGrid:
    """
    Occupancy Grid GLOBAL
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

    # ==================================================
    # Conversões de coordenadas
    # ==================================================
    def world_to_grid(self, x, y):
        gx = int(round(x / self.resolution)) + self.center
        gy = int(round(y / self.resolution)) + self.center
        return gx, gy


    def grid_to_world(self, gx, gy):
        x = (gx - self.center) * self.resolution
        y = (gy - self.center) * self.resolution
        return x, y

    # ==================================================
    # Bresenham (ray tracing)
    # ==================================================
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

    # ==================================================
    # Atualização de um raio
    # ==================================================
    def update_ray(self, robot_cell, hit_cell):
        rx, ry = robot_cell
        hx, hy = hit_cell

        cells = self.bresenham(rx, ry, hx, hy)

        # Espaço livre
        for cx, cy in cells[:-1]:
            if 0 <= cx < self.cells and 0 <= cy < self.cells:
                self.grid[cx][cy] = 0

        # Obstáculo
        cx, cy = cells[-1]
        if 0 <= cx < self.cells and 0 <= cy < self.cells:
            self.grid[cx][cy] = 1

    # ==================================================
    # ATUALIZAÇÃO GLOBAL DO MAPA (aqui deve estar a merda!!!!)
    # (LiDAR no frame do robô → mapa no mundo)
    # ==================================================
    def update_from_points(self, points, robot_pose):
        """
        points: lista de pontos (x, y) no FRAME DO ROBÔ
        robot_pose: (x, y, theta) no FRAME GLOBAL
        """
        rx, ry, theta = robot_pose

        # Proteção contra NaN na pose
        if math.isnan(rx) or math.isnan(ry) or math.isnan(theta):
            return

        robot_cell = self.world_to_grid(rx, ry)

        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        for lx, ly in points:
            # Proteção contra lixo do LiDAR
            if math.isnan(lx) or math.isnan(ly):
                continue
            if math.isinf(lx) or math.isinf(ly):
                continue

            # Transformação robô → mundo
            wx = cos_t * lx - sin_t * ly + rx
            wy = sin_t * lx + cos_t * ly + ry

            if math.isnan(wx) or math.isnan(wy):
                continue
            if math.isinf(wx) or math.isinf(wy):
                continue

            gx, gy = self.world_to_grid(wx, wy)

            if 0 <= gx < self.cells and 0 <= gy < self.cells:
                self.update_ray(robot_cell, (gx, gy))

    # ==================================================
    # Visualização OpenCV
    # ==================================================
    def to_cv_image(self, scale=4):
        img = np.zeros((self.cells, self.cells, 3), dtype=np.uint8)

        for i in range(self.cells):
            for j in range(self.cells):
                v = self.grid[i][j]
                iy = self.cells - 1 - j  # INVERTE O Y AQUI

                if v == -1:
                    img[iy, i] = (40, 40, 40)        # desconhecido
                elif v == 0:
                    img[iy, i] = (180, 180, 180)    # livre
                else:
                    img[iy, i] = (0, 255, 0)        # ocupado

        if scale != 1:
            img = cv2.resize(
                img,
                None,
                fx=scale,
                fy=scale,
                interpolation=cv2.INTER_NEAREST
            )

        return img
