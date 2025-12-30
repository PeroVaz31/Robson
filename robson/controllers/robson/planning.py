import heapq
import math

class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.cells = grid.cells

    def heuristic(self, a, b):
        # Distância Euclidiana
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def neighbors(self, node):
        x, y = node
        neigh = []

        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),
                       (-1,-1),(-1,1),(1,-1),(1,1)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.cells and 0 <= ny < self.cells:
                if self.grid.grid[nx][ny] != 1:  # não ocupado
                    neigh.append((nx, ny))
        return neigh

    def plan(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.neighbors(current):
                tentative_g = g_score[current] + self.heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    heapq.heappush(open_set, (f, neighbor))

        return None  # sem caminho

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
