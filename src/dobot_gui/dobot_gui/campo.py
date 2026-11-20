"""Campo de trabalho calculado a partir dos pontos medidos no laboratório.

Os pontos (aba Campo de Trabalho) dão o AMBIENTE; o resto sai do robô:

  chão      a altura do tampo, pelos pontos da base (posição do flange);
  teto      a altura dos pontos do teto (a MAIS BAIXA deles: o lado seguro);
  em volta  os contornos medidos são os LIMITES SEGUROS: embaixo o contorno dos
            pontos da base, em cima o dos pontos do teto. Entre os dois, em cada
            direção em volta do robô, a parede vai de um ao outro em linha reta.
            Entre dois pontos vizinhos de um contorno, o contorno é reto;
  alcance   até onde o braço chega na horizontal em cada altura (ALCANCE_MM):
            só corta o campo onde for ainda menor que os contornos.

Um ponto (x, y, z) está DENTRO se: chão <= z <= teto, e a distância dele ao eixo
do robô é no máximo a parede naquela direção e altura e o alcance naquela altura.
Nada passa dos contornos medidos.

Tudo é na posição do FLANGE, sem ferramenta (como os pontos foram medidos), em
milímetros, no referencial do modelo do robô (o mesmo do Robô 3D).
"""
import math

import numpy as np

# Alcance horizontal máximo do flange (mm) em cada altura (mm), calculado pelo
# modelo do CR10A (cr10_robot.xacro do gêmeo): J1 fixa (o alcance é igual em
# toda volta), J2 de 2 em 2°, J3 de 2 em 2° (limite ±163,9°), J4 de 15 em 15°,
# J5 em 0/90/180/270°; 2,8 milhões de posições, o maior raio em cada faixa de
# 10 mm. Amostrado: pode ficar uns milímetros abaixo do alcance real (lado seguro).
ALCANCE_MM = (
    (0, 1342), (20, 1343), (40, 1344), (60, 1348), (80, 1350), (100, 1351), (120, 1352),
    (140, 1353), (160, 1352), (180, 1352), (200, 1354), (220, 1353), (240, 1352),
    (260, 1351), (280, 1349), (300, 1347), (320, 1345), (340, 1345), (360, 1342),
    (380, 1339), (400, 1336), (420, 1332), (440, 1327), (460, 1323), (480, 1320),
    (500, 1316), (520, 1311), (540, 1306), (560, 1300), (580, 1293), (600, 1286),
    (620, 1279), (640, 1272), (660, 1266), (680, 1259), (700, 1252), (720, 1244),
    (740, 1235), (760, 1225), (780, 1216), (800, 1205), (820, 1195), (840, 1184),
    (860, 1172), (880, 1161), (900, 1148), (920, 1135), (940, 1123), (960, 1109),
    (980, 1095), (1000, 1081), (1020, 1066), (1040, 1050), (1060, 1032), (1080, 1013),
    (1100, 995), (1120, 977), (1140, 960), (1160, 940), (1180, 914), (1200, 895),
    (1220, 874), (1240, 844), (1260, 822), (1280, 795), (1300, 767), (1320, 733),
    (1340, 708), (1360, 671), (1380, 634), (1400, 594), (1420, 552), (1440, 509),
    (1460, 451), (1480, 391),
)
_ALC_Z = np.array([z for z, _ in ALCANCE_MM], float)
_ALC_R = np.array([r for _, r in ALCANCE_MM], float)


def alcance_mm(z_mm):
    """Alcance horizontal do flange na altura z (mm); 0 acima do topo da tabela."""
    if z_mm > _ALC_Z[-1]:
        return 0.0
    return float(np.interp(z_mm, _ALC_Z, _ALC_R))


class Campo:
    """Campo de trabalho montado com as posições do flange (mm) nos pontos medidos."""

    def __init__(self, base_mm, teto_mm):
        base = np.asarray(base_mm, float)
        teto = np.asarray(teto_mm, float)
        if len(base) < 3 or len(teto) < 1:
            raise ValueError('Precisa de pelo menos 3 pontos da base e 1 do teto.')
        self.chao_z = float(base[:, 2].mean())
        self.chao_variacao = float(base[:, 2].max() - base[:, 2].min())
        self.teto_z = float(teto[:, 2].min())
        # Contornos em volta do eixo do robô, em ordem de ângulo.
        self.contorno = _em_volta(base)
        self.contorno_teto = _em_volta(teto) if len(teto) >= 3 else None
        self.base = base
        self.teto = teto

    def raio_contorno(self, ang):
        """Distância (mm) do eixo do robô até o contorno da BASE na direção `ang`."""
        return _raio_poligono(self.contorno, ang)

    def raio_contorno_teto(self, ang):
        """Distância (mm) do eixo do robô até o contorno do TETO na direção `ang`.
        Com menos de 3 pontos do teto não há contorno: vale o da base."""
        if self.contorno_teto is None:
            return self.raio_contorno(ang)
        return _raio_poligono(self.contorno_teto, ang)

    def raio_parede(self, ang, z):
        """Parede na direção `ang` e altura z: do contorno da base (no chão) ao do
        teto (no teto), em linha reta."""
        f = (z - self.chao_z) / (self.teto_z - self.chao_z) if self.teto_z > self.chao_z else 0.0
        f = min(max(f, 0.0), 1.0)
        return (1 - f) * self.raio_contorno(ang) + f * self.raio_contorno_teto(ang)

    def raio(self, ang, z):
        """Até onde o campo vai na direção `ang` (rad), na altura z (mm)."""
        if z < self.chao_z - 1e-6 or z > self.teto_z + 1e-6:
            return 0.0
        return min(self.raio_parede(ang, z), alcance_mm(z))

    def dentro(self, xyz_mm, folga_mm=0.0):
        """True se o ponto (mm) está dentro do campo, com `folga_mm` de sobra."""
        x, y, z = xyz_mm
        if z < self.chao_z + folga_mm or z > self.teto_z - folga_mm:
            return False
        return math.hypot(x, y) <= self.raio(math.atan2(y, x), z) - folga_mm

    def malha(self, n_ang=96, n_z=28):
        """Grade da superfície do campo: (n_z, n_ang, 3) em mm, de baixo (chão) para
        cima (teto), dando a volta no robô."""
        angs = np.linspace(-math.pi, math.pi, n_ang, endpoint=False)
        zs = np.linspace(self.chao_z, self.teto_z, n_z)
        g = np.empty((n_z, n_ang, 3))
        for i, z in enumerate(zs):
            r = np.array([self.raio(a, z) for a in angs])
            g[i, :, 0] = r * np.cos(angs)
            g[i, :, 1] = r * np.sin(angs)
            g[i, :, 2] = z
        return g


def _em_volta(pts):
    """Pontos (N,3) em ordem de ângulo em volta do eixo do robô, só x e y."""
    return pts[np.argsort(np.arctan2(pts[:, 1], pts[:, 0])), :2]


def _raio_poligono(p, ang):
    """Distância do eixo do robô até onde o raio na direção `ang` (rad) cruza o
    polígono `p` (N,2). Os pontos vêm em ordem de ângulo em volta do eixo, então o
    raio cruza o contorno uma vez só; se cruzar mais (pontos no mesmo ângulo), vale
    o cruzamento mais perto (o lado seguro)."""
    d = np.array([math.cos(ang), math.sin(ang)])
    n, melhor = len(p), None
    for i in range(n):
        a, b = p[i], p[(i + 1) % n]
        e = b - a
        det = d[0] * (-e[1]) - d[1] * (-e[0])
        if abs(det) < 1e-12:
            continue
        # a + s·e = t·d  →  resolve (t, s)
        t = (a[0] * (-e[1]) - a[1] * (-e[0])) / det
        s = (d[0] * a[1] - d[1] * a[0]) / det
        if t > 0 and -1e-9 <= s <= 1 + 1e-9:
            melhor = t if melhor is None else min(melhor, t)
    return 0.0 if melhor is None else float(melhor)
