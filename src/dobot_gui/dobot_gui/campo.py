"""Campo de trabalho calculado a partir dos pontos medidos no laboratório.

Os pontos foram medidos em PLANOS (alturas) em volta do robô inteiro:

  planos      os pontos são juntados pela altura (até TOLERANCIA_PLANO_MM de
              diferença = mesmo plano). Em cada plano, os pontos em volta do robô,
              em ordem de ângulo, formam o CONTORNO dele: o limite seguro naquela
              altura. Entre dois pontos vizinhos o contorno é reto;
  referência  os pontos chamados "mesa" ou "começo …" só dão a altura certa do
              plano (foram medidos perto do robô): entram na altura, não no
              contorno (senão o campo afundaria onde eles foram gravados);
  entre       entre dois planos vizinhos, em cada direção em volta do robô, a
              parede vai em linha reta do contorno de baixo ao de cima;
  chão/topo   o plano mais baixo é o chão e o mais alto é o topo: nada abaixo do
              primeiro nem acima do último;
  alcance     até onde o braço chega na horizontal em cada altura (ALCANCE_MM):
              só corta o campo onde for ainda menor que os contornos.

Um ponto (x, y, z) está DENTRO se: chão <= z <= topo, e a distância dele ao eixo
do robô é no máximo a parede naquela direção e altura e o alcance naquela altura.

Tudo é na posição do FLANGE, sem ferramenta (como os pontos foram medidos), em
milímetros, no referencial do modelo do robô (o mesmo do Robô 3D).
"""
import math
import unicodedata

import numpy as np

# Pontos com até esta diferença de altura são do mesmo plano.
TOLERANCIA_PLANO_MM = 15.0
# Arredondamento da altura: um ponto até isto acima do topo ou abaixo do chão
# ainda conta como no chão/topo (os pontos nivelados ficam a milésimos de mm).
FOLGA_ALTURA_MM = 0.5
# Um plano precisa de pelo menos isto de pontos em volta para ter contorno.
MINIMO_CONTORNO = 3

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
# Alcance horizontal máximo do flange (mm) em cada altura (mm) COM A PONTA PARA
# BAIXO (o eixo do flange apontando para o chão, como os pontos foram medidos).
# Calculado com cinemática inversa no modelo do CR10A (o mesmo do Robô 3D; junta 3
# até ±163,9°, as outras até ±359°): em cada altura, de 20 em 20 mm, o braço foi
# levado cada vez mais longe do eixo até não chegar mais (afinado até 1 mm), e
# cada posição conferida com a cinemática direta. Acima de 1240 mm não chega.
ALCANCE_PONTA_BAIXO_MM = (
    (0, 1312), (20, 1313), (40, 1314), (60, 1314), (80, 1314), (100, 1314), (120, 1313),
    (140, 1312), (160, 1310), (180, 1309), (200, 1307), (220, 1304), (240, 1302),
    (260, 1298), (280, 1295), (300, 1291), (320, 1287), (340, 1283), (360, 1278),
    (380, 1272), (400, 1267), (420, 1261), (440, 1254), (460, 1248), (480, 1240),
    (500, 1233), (520, 1225), (540, 1216), (560, 1208), (580, 1198), (600, 1188),
    (620, 1178), (640, 1167), (660, 1156), (680, 1145), (700, 1132), (720, 1119),
    (740, 1106), (760, 1092), (780, 1077), (800, 1062), (820, 1046), (840, 1029),
    (860, 1012), (880, 994), (900, 974), (920, 954), (940, 933), (960, 911), (980, 888),
    (1000, 863), (1020, 837), (1040, 809), (1060, 780), (1080, 748), (1100, 714),
    (1120, 677), (1140, 637), (1160, 592), (1180, 541), (1200, 482), (1220, 407),
    (1240, 290),
)
_ALC_Z = np.array([z for z, _ in ALCANCE_MM], float)
_ALC_R = np.array([r for _, r in ALCANCE_MM], float)


_ALB_Z = np.array([z for z, _ in ALCANCE_PONTA_BAIXO_MM], float)
_ALB_R = np.array([r for _, r in ALCANCE_PONTA_BAIXO_MM], float)


def alcance_ponta_baixo_mm(z_mm):
    """Alcance horizontal do flange com a ponta para baixo na altura z (mm); 0
    acima do topo da tabela."""
    if z_mm > _ALB_Z[-1] or z_mm < _ALB_Z[0]:
        return 0.0
    return float(np.interp(z_mm, _ALB_Z, _ALB_R))


def alcance_mm(z_mm):
    """Alcance horizontal do flange na altura z (mm); 0 acima do topo da tabela."""
    if z_mm > _ALC_Z[-1]:
        return 0.0
    return float(np.interp(z_mm, _ALC_Z, _ALC_R))


def eh_referencia(nome):
    """Ponto de referência de altura: chamado "mesa" ou "começo …" (com ou sem acento)."""
    n = unicodedata.normalize('NFKD', (nome or '').strip().lower())
    n = ''.join(c for c in n if not unicodedata.combining(c))
    return n == 'mesa' or n.startswith('comeco')


class Plano:
    """Um plano medido: altura (mm), contorno em volta do eixo (N,2) e os pontos."""

    def __init__(self, z, contorno, pontos, referencias):
        self.z = z
        self.contorno = contorno
        self.pontos = pontos
        self.referencias = referencias

    def raio(self, ang):
        return _raio_poligono(self.contorno, ang)


class Campo:
    """Campo de trabalho montado com as posições do flange (mm) nos pontos medidos.

    `contorno_mm`: pontos da borda de cada plano; `referencias_mm`: pontos que só
    dão a altura do plano (podem faltar)."""

    def __init__(self, contorno_mm, referencias_mm=()):
        self.planos, self.ignorados = agrupar_planos(contorno_mm, referencias_mm)
        if len(self.planos) < 2:
            raise ValueError('Precisa de pelo menos 2 planos com 3 pontos em volta cada.')
        self.chao_z = self.planos[0].z
        self.teto_z = self.planos[-1].z
        self._zs = np.array([p.z for p in self.planos])
        self.pontos = np.vstack([p.pontos for p in self.planos])
        self.referencias = np.vstack([p.referencias for p in self.planos]).reshape(-1, 3)

    def raio_parede(self, ang, z):
        """Parede na direção `ang` (rad) e altura z: em linha reta entre o contorno
        do plano de baixo e o do plano de cima."""
        i = int(np.clip(np.searchsorted(self._zs, z) - 1, 0, len(self.planos) - 2))
        a, b = self.planos[i], self.planos[i + 1]
        f = min(max((z - a.z) / (b.z - a.z), 0.0), 1.0) if b.z > a.z else 0.0
        return (1 - f) * a.raio(ang) + f * b.raio(ang)

    def raio(self, ang, z):
        """Até onde o campo vai na direção `ang` (rad), na altura z (mm)."""
        if z < self.chao_z - FOLGA_ALTURA_MM or z > self.teto_z + FOLGA_ALTURA_MM:
            return 0.0
        z = min(max(z, self.chao_z), self.teto_z)
        return min(self.raio_parede(ang, z), alcance_mm(z))

    def dentro(self, xyz_mm, folga_mm=0.0):
        """True se o ponto (mm) está dentro do campo, com `folga_mm` de sobra."""
        x, y, z = xyz_mm
        if z < self.chao_z - FOLGA_ALTURA_MM + folga_mm or \
                z > self.teto_z + FOLGA_ALTURA_MM - folga_mm:
            return False
        return math.hypot(x, y) <= self.raio(math.atan2(y, x), z) - folga_mm

    def malha(self, n_ang=96, por_vao=3):
        """Grade da superfície: (n_z, n_ang, 3) em mm, de baixo para cima, dando a
        volta no robô. Tem uma fila exatamente em cada plano medido (os índices
        ficam em `self.filas_planos`) e `por_vao` filas entre dois planos."""
        angs = np.linspace(-math.pi, math.pi, n_ang, endpoint=False)
        zs, self.filas_planos = [], []
        for i, p in enumerate(self.planos):
            self.filas_planos.append(len(zs))
            zs.append(p.z)
            if i + 1 < len(self.planos):
                zs += list(np.linspace(p.z, self.planos[i + 1].z, por_vao + 2)[1:-1])
        g = np.empty((len(zs), n_ang, 3))
        for i, z in enumerate(zs):
            r = np.array([self.raio(a, z) for a in angs])
            g[i, :, 0] = r * np.cos(angs)
            g[i, :, 1] = r * np.sin(angs)
            g[i, :, 2] = z
        return g


def agrupar_planos(contorno_mm, referencias_mm=()):
    """Junta os pontos em planos pela altura. Devolve ([Plano], quantos pontos
    ficaram de fora por estarem num plano com menos de MINIMO_CONTORNO na borda)."""
    pts = np.asarray(contorno_mm, float).reshape(-1, 3)
    refs = np.asarray(referencias_mm, float).reshape(-1, 3)
    todos = [(p, False) for p in pts] + [(p, True) for p in refs]
    todos.sort(key=lambda x: x[0][2])
    grupos = []
    for p, ref in todos:
        if grupos and p[2] - grupos[-1][0][0][2] <= TOLERANCIA_PLANO_MM:
            grupos[-1].append((p, ref))
        else:
            grupos.append([(p, ref)])
    planos, ignorados = [], 0
    for g in grupos:
        borda = np.array([p for p, ref in g if not ref]).reshape(-1, 3)
        if len(borda) < MINIMO_CONTORNO:
            ignorados += len(g)
            continue
        rf = np.array([p for p, ref in g if ref]).reshape(-1, 3)
        # Altura do plano: a da referência, se houver; senão a média da borda.
        z = float(rf[:, 2].mean()) if len(rf) else float(borda[:, 2].mean())
        planos.append(Plano(z, _em_volta(borda), borda, rf))
    return planos, ignorados


class CampoDaBase:
    """Campo calculado SÓ com os pontos da base (o plano mais baixo) e a
    cinemática do robô, sem os planos de cima:

      chão      a altura da base (a da referência "mesa", se houver);
      em volta  paredes imaginárias verticais subindo do contorno da base;
      alcance   até onde o flange chega com a ponta para baixo em cada altura
                (ALCANCE_PONTA_BAIXO_MM, cinemática inversa + direta);
      topo      a altura em que o braço, com a ponta para baixo, não sobe mais;
      medido    com `limite` (o Campo dos pontos medidos), nunca passa dele: em
                cada direção e altura vale o menor dos dois, e o topo não passa
                do plano medido mais alto.

    Mesmo formato do Campo para o desenho (malha, filas_planos, pontos, referencias)."""

    N_ANEIS = 7      # anéis desenhados (chão, alturas intermediárias e topo)

    def __init__(self, contorno_mm, referencias_mm=(), limite=None):
        planos, _ign = agrupar_planos(contorno_mm, referencias_mm)
        if not planos:
            raise ValueError('Precisa de pelo menos 3 pontos da base.')
        self.base = planos[0]
        self.limite = limite
        self.chao_z = self.base.z
        self.teto_alcance_z = float(_ALB_Z[_ALB_R > 0].max())
        self.teto_z = self.teto_alcance_z
        if limite is not None:
            self.teto_z = min(self.teto_z, limite.teto_z)
        self.pontos = self.base.pontos
        self.referencias = self.base.referencias

    def raio(self, ang, z):
        """Até onde o campo vai na direção `ang` (rad), na altura z (mm)."""
        if z < self.chao_z - FOLGA_ALTURA_MM or z > self.teto_z + FOLGA_ALTURA_MM:
            return 0.0
        z = min(max(z, self.chao_z), self.teto_z)
        r = min(self.base.raio(ang), alcance_ponta_baixo_mm(z))
        if self.limite is not None:
            r = min(r, self.limite.raio(ang, z))
        return r

    def dentro(self, xyz_mm, folga_mm=0.0):
        x, y, z = xyz_mm
        if z < self.chao_z - FOLGA_ALTURA_MM + folga_mm or \
                z > self.teto_z + FOLGA_ALTURA_MM - folga_mm:
            return False
        return math.hypot(x, y) <= self.raio(math.atan2(y, x), z) - folga_mm

    def malha(self, n_ang=72, n_z=26):
        """Grade da superfície (n_z, n_ang, 3) em mm. As filas ficam mais juntas
        perto do topo, onde o alcance diminui depressa."""
        angs = np.linspace(-math.pi, math.pi, n_ang, endpoint=False)
        t = np.linspace(0.0, 1.0, n_z)
        zs = self.chao_z + (self.teto_z - self.chao_z) * (1 - (1 - t) ** 1.6)
        g = np.empty((n_z, n_ang, 3))
        for i, z in enumerate(zs):
            r = np.array([self.raio(a, z) for a in angs])
            g[i, :, 0] = r * np.cos(angs)
            g[i, :, 1] = r * np.sin(angs)
            g[i, :, 2] = z
        self.filas_planos = sorted({int(round(k)) for k in np.linspace(0, n_z - 1, self.N_ANEIS)})
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
