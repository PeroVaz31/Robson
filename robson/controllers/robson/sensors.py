import math

class LidarSensor:
    """
    Interface para LiDAR do Webots.
    Retorna pontos (x, y) no FRAME DO ROBÔ.
    Pontos inválidos (NaN / inf) são descartados.
    """

    def __init__(self, robot_interface, name="Velodyne VLP-16"):
        self.robot_interface = robot_interface
        self.robot = robot_interface.robot
        self.timestep = robot_interface.timestep

        self.lidar = self.robot.getDevice(name)
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()

    # ==================================================
    # Pontos 2D válidos no frame do robô
    # ==================================================
    def get_2d_points(self):
        """
        Projeta o LiDAR 3D em um plano 2D.
        Filtra o próprio robô e camadas inúteis.
        """
        points = []
        cloud = self.lidar.getPointCloud()

        if not cloud:
            return points

        # ===== CONFIGURAÇÕES DE FILTRO =====
        # O Pioneer tem aprox 44cm de comprimento e 38cm de largura.
        # Precisamos ignorar tudo dentro de um raio de segurança.
        MIN_DIST_SQ = 0.40 ** 2  # 40cm de raio cego (ao quadrado para otimizar)
        
        # Filtro de altura (Z relativo ao sensor)
        # O VLP-16 tem camadas de -15 a +15 graus.
        # Z=0 é o horizonte. 
        # Vamos pegar uma fatia fina ao redor do horizonte para evitar o chão e o teto.
        Z_MIN = -0.15 
        Z_MAX = 0.05
        # ===================================

        for p in cloud:
            x, y, z = p.x, p.y, p.z

            # 1. Filtro de Altura (Ignora chão e teto)
            if z < Z_MIN or z > Z_MAX:
                continue

            # 2. Filtro de "Self-Collision" (Ignora o próprio robô)
            # Calcula distância ao quadrado no plano XY
            dist_sq = x*x + y*y
            
            if dist_sq < MIN_DIST_SQ:
                continue

            # 3. Filtro de Validade Numérica
            if math.isnan(x) or math.isnan(y) or math.isinf(x) or math.isinf(y):
                continue

            # Se passou em tudo, adiciona na lista
            # No Webots PointCloud: x=frente, y=esquerda
            points.append((x, y))

        return points