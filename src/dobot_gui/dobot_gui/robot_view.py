"""Mini robô: viewport 3D do CR10 em PyQt5, movido pelos ângulos das juntas.

Opcionalmente desenha um segundo robô, translúcido (o "alvo"), que pode ser
editado com o mouse: clicar numa peça dele escolhe a junta que a move, e arrastar
para os lados gira essa junta. Mexer no alvo só muda o desenho e avisa quem
estiver ouvindo (`ao_mudar_alvo`); quem decide mover o robô é a página.

Usa o modelo (URDF + malhas STL) e a matemática de câmera/sombreamento do
projeto cr10twin (Martins-Lucaas, Apache-2.0), adaptados de Tk para Qt.
"""
import bisect
import math
import os
import time
from collections import deque

import numpy as np
from PyQt5.QtCore import QPointF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import QSizePolicy, QWidget

from .campo import Campo
from .robot3d_render import Camera, scene_triangles, shade_and_sort
from .robot3d_scene import UrdfSceneError, parse_urdf

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# Convenção do projeto cr10twin (kinematics.urdf_to_dobot): só a junta 1 tem
# o sinal invertido entre o modelo e os ângulos que o robô Dobot reporta.
DOBOT_TO_URDF_SIGN = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]

TRIANGLE_BUDGET = 7000
# Movimento liso: enquanto o robô se mexe, o desenho usa um modelo mais leve
# (~1/2 dos triângulos, quase igual nesse tamanho) e a posição é interpolada
# entre as leituras, mostrada com um pequeno atraso para sempre haver uma leitura
# depois do instante desenhado. Parado, volta ao modelo detalhado.
TRIANGLE_BUDGET_LEVE = 2500
QUADRO_MS = 16                # até ~60 quadros por segundo enquanto se move
ATRASO_S = 0.06               # o desenho fica este tempo atrás da última leitura
PARADO_S = 0.3                # sem mudar por este tempo: desenha o modelo detalhado
CR10_REACH_M = 1.375
# Campo de trabalho (aba Campo de Trabalho): faces translúcidas, arestas e pontos.
CAMPO_FACE = QColor(56, 189, 248, 38)
CAMPO_CHAO = QColor(56, 189, 248, 60)
CAMPO_ARESTA = QColor(125, 211, 252, 200)
CAMPO_PONTO_BASE = QColor(250, 204, 21)
CAMPO_PONTO_TETO = QColor(244, 114, 182)
GROUND_Z = -0.03  # o modelo tem a base 3 cm acima do chão (world_joint)

# Alvo: arrastar 1 px gira a junta escolhida tanto (Shift: ajuste fino).
GRAUS_POR_PX = 0.5
GRAUS_POR_PX_FINO = 0.1
ALVO_ALFA = 120                  # 0..255: quão opaco é o robô-alvo
# Cor do alvo conforme a verificação do caminho até ele.
CORES_ALVO = {'ok': (44, 160, 44), 'ruim': (214, 39, 40), 'conferindo': (240, 170, 30),
              'ponto': (59, 130, 246)}   # azul: ponto do Campo de Trabalho (não é verificação)
# Peça do modelo -> junta (0..5) que a move.
JUNTA_DA_PECA = {'Link1': 0, 'Link2': 1, 'Link3': 2, 'Link4': 3, 'Link5': 4, 'Link6': 5,
                 'sensor_s02': 5, 'ft_flange_link': 5, 'load_cell_link': 5,
                 'coupler_tool_link': 5, 'tool_tip_link': 5}
# Peças da célula de carga no modelo do gêmeo (cr10_robot_com_celula.xacro).
PECAS_CELULA = ('ft_flange_link', 'load_cell_link', 'coupler_tool_link', 'tool_tip_link')
JUNTAS_CELULA = ('ft_flange_attach', 'load_cell_attach', 'coupler_tool_attach',
                 'tool_tip_attach')
_URDF_CELULA = None

BACKGROUND = QColor('#0f172a')
GRID_COLOR = QColor('#1e293b')
REACH_COLOR = QColor('#334155')
TEXT_COLOR = QColor('#94a3b8')


def _data_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory('dobot_gui'), 'robot3d')
        if os.path.isdir(path):
            return path
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(__file__)), 'robot3d')


def _urdf_sensor(medidas_mm):
    """Invólucro do sensor inercial como uma caixa presa no centro do flange."""
    x, y, z = (v / 1000.0 for v in medidas_mm)
    return f'''
  <link name="sensor_s02">
    <visual>
      <geometry><box size="{x} {y} {z}"/></geometry>
      <material name="sensor_cinza"><color rgba="0.40 0.40 0.43 1"/></material>
    </visual>
  </link>
  <joint name="joint_sensor_s02" type="fixed">
    <origin xyz="0 0 {z / 2}" rpy="0 0 0"/>
    <parent link="Link6"/>
    <child link="sensor_s02"/>
  </joint>
'''


def _urdf_celula():
    """A célula de carga com a ponteira, tirada do modelo do gêmeo (as mesmas
    malhas do Gazebo), só com as partes visíveis. '' se não der para ler."""
    global _URDF_CELULA
    if _URDF_CELULA is not None:
        return _URDF_CELULA
    try:
        import xml.etree.ElementTree as ET

        import xacro
        from ament_index_python.packages import get_package_share_directory
        caminho = os.path.join(get_package_share_directory('dobot_twin'), 'urdf',
                               'cr10_robot_com_celula.xacro')
        raiz = ET.fromstring(xacro.process_file(caminho).toxml())
        partes = []
        for mat in raiz.findall('material'):
            partes.append(ET.tostring(mat, encoding='unicode'))
        for link in raiz.findall('link'):
            if link.get('name') in PECAS_CELULA:
                for tag in ('collision', 'inertial'):
                    for el in link.findall(tag):
                        link.remove(el)
                partes.append(ET.tostring(link, encoding='unicode'))
        for junta in raiz.findall('joint'):
            if junta.get('name') in JUNTAS_CELULA:
                partes.append(ET.tostring(junta, encoding='unicode'))
        _URDF_CELULA = '\n'.join(partes)
    except Exception:
        _URDF_CELULA = ''
    return _URDF_CELULA


def load_robot_scene(budget=TRIANGLE_BUDGET, ferramenta=None):
    """Lê o URDF do CR10 e carrega as malhas, com o que está no flange:
    'celula' (a célula de carga com a ponteira, do modelo do gêmeo), as medidas
    (mm) de uma caixa, como a do sensor inercial S02, ou None (nada).
    Levanta UrdfSceneError se falhar."""
    data = _data_dir()
    with open(os.path.join(data, 'cr10.urdf'), 'r', encoding='utf-8') as f:
        text = f.read()
    text = text.replace('filename="meshes/', f'filename="{data}/meshes/')
    extra = ''
    if ferramenta == 'celula':
        extra = _urdf_celula()
    elif ferramenta:
        extra = _urdf_sensor(ferramenta)
    if extra:
        fim = text.rindex('</robot>')
        text = text[:fim] + extra + text[fim:]
    return _cena_com_cache(text, budget)


# Simplificar as malhas leva ~1 s a cada abertura (mais, com o Gazebo subindo
# junto). O resultado depende só do URDF, do orçamento de triângulos e das
# malhas, então fica guardado em ~/.cache e é só lido nas próximas vezes.
PASTA_CACHE_3D = os.path.expanduser('~/.cache/iris/robo3d')
VERSAO_CACHE_3D = 1


def _cena_com_cache(text, budget):
    import hashlib
    import pickle
    import re
    h = hashlib.sha1(f'{VERSAO_CACHE_3D}|{budget}|'.encode() + text.encode())
    for caminho in sorted(set(re.findall(r'filename="(?:file://)?([^"]+)"', text))):
        try:
            st = os.stat(caminho)
            h.update(f'{caminho}|{st.st_size}|{st.st_mtime_ns}'.encode())
        except OSError:
            h.update(f'{caminho}|?'.encode())
    arquivo = os.path.join(PASTA_CACHE_3D, h.hexdigest() + '.pkl')
    try:
        with open(arquivo, 'rb') as f:
            return pickle.load(f)
    except Exception:
        pass
    cena = parse_urdf(text, triangle_budget=budget)
    try:
        os.makedirs(PASTA_CACHE_3D, exist_ok=True)
        tmp = arquivo + f'.{os.getpid()}.tmp'
        with open(tmp, 'wb') as f:
            pickle.dump(cena, f, protocol=pickle.HIGHEST_PROTOCOL)
        os.replace(tmp, arquivo)
    except Exception:
        pass
    return cena


def dobot_deg_to_joint_values(q_deg):
    """Ângulos do Dobot (graus) → valores de junta do URDF (rad)."""
    return {name: math.radians(sign * float(v))
            for name, sign, v in zip(JOINT_NAMES, DOBOT_TO_URDF_SIGN, q_deg)}


def _cores_rgba(rgb, alfa=255):
    """Cores (N,3) uint8 (+ alfa) -> um inteiro ARGB por triângulo."""
    rgb = rgb.astype(np.int64)
    return (np.asarray(alfa, dtype=np.int64) << 24) | \
        (rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]


class RobotView(QWidget):
    """Desenha o CR10 em 3D. Mouse: esquerdo gira, direito/meio move, roda dá zoom.
    Com o alvo editável: esquerdo sobre o alvo escolhe a junta e arrastar a gira."""

    ao_mudar_alvo = pyqtSignal(object)        # lista de 6 ângulos (graus)

    def __init__(self, parent=None, ferramenta=None):
        super().__init__(parent)
        self.setMinimumHeight(320)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(False)

        self.camera = Camera(dist=2.1)
        self.camera.target = np.array([0.0, 0.0, 0.45])
        self.q_deg = [0.0] * 6
        self._world_tris = None
        self._colors = None
        self._error = None
        self._last_mouse = None
        self._drag_button = None
        self._caption = ''
        # Alvo (robô translúcido): None = não desenha.
        self.alvo_deg = None
        self._alvo_estado = 'conferindo'
        self._alvo_editavel = False
        self._alvo_tris = None
        self._junta_escolhida = None      # junta sendo arrastada (0..5)
        self._limites = (-180.0, 180.0)
        self._desenho = None              # (uv, dono) do último quadro, para o clique
        # Leituras (instante, ângulos) para a interpolação e o modelo em uso.
        self._amostras = deque(maxlen=64)
        self._leve = False
        self._ultima_mudanca = 0.0
        self._timer = QTimer(self)
        self._timer.setInterval(QUADRO_MS)
        self._timer.timeout.connect(self._quadro)

        self._cores_qt = {}               # cor (ARGB em um inteiro) -> QColor
        # Campo de trabalho: (campo.Campo, grade (n_z, n_ang, 3) em metros) ou None.
        self._campo = None
        self.scene = None
        self._cena_leve = None
        self._cena = None                 # a que está desenhada agora (detalhada ou leve)
        self.definir_sensor(ferramenta)

    def definir_sensor(self, ferramenta):
        """Recarrega o modelo com o que está no flange: 'celula', as medidas (mm)
        de uma caixa (sensor S02) ou None (nada)."""
        try:
            self.scene = load_robot_scene(ferramenta=ferramenta)
            self._cena_leve = load_robot_scene(budget=TRIANGLE_BUDGET_LEVE, ferramenta=ferramenta)
            self._error = None
        except (UrdfSceneError, OSError) as e:
            self.scene = self._cena_leve = None
            self._error = f'Não foi possível carregar o modelo 3D: {e}'
        self._rebuild()
        self.update()

    # --- dados ---

    def seguir(self, q_deg):
        """Nova leitura do robô (graus, convenção Dobot) para o movimento liso:
        o desenho anda até ela interpolando, ~60 vezes por segundo."""
        self._amostras.append((time.monotonic(), [float(v) for v in q_deg]))
        if self.isVisible() and not self._timer.isActive():
            self._timer.start()

    def _posicao_em(self, t):
        """Ângulos interpolados entre as duas leituras em volta do instante t."""
        a = self._amostras
        tempos = [x[0] for x in a]
        i = bisect.bisect_right(tempos, t)
        if i == 0:
            return a[0][1]
        if i >= len(a):
            return a[-1][1]
        (t0, q0), (t1, q1) = a[i - 1], a[i]
        f = (t - t0) / (t1 - t0) if t1 > t0 else 1.0
        return [x0 + (x1 - x0) * f for x0, x1 in zip(q0, q1)]

    def _quadro(self):
        if not self.isVisible() or not self._amostras:
            self._timer.stop()
            return
        agora = time.monotonic()
        q = self._posicao_em(agora - ATRASO_S)
        if max(abs(a - b) for a, b in zip(q, self.q_deg)) > 0.01:
            self.q_deg = q
            self._leve = True
            self._ultima_mudanca = agora
            self._rebuild()
            self.update()
        elif agora - self._ultima_mudanca > PARADO_S:
            if self._leve:                 # parou: um quadro com o modelo detalhado
                self._leve = False
                self._rebuild()
                self.update()
            self._timer.stop()

    def showEvent(self, evento):
        super().showEvent(evento)
        if self._amostras:
            self._timer.start()

    def set_joints_deg(self, q_deg):
        """Novos ângulos (graus, convenção Dobot). Só redesenha se mudou."""
        q = [float(v) for v in q_deg]
        if max(abs(a - b) for a, b in zip(q, self.q_deg)) < 0.05:
            return
        self.q_deg = q
        self._rebuild()
        self.update()

    def definir_alvo(self, q_deg, editavel=False, limites=None):
        """Mostra (lista de 6 ângulos) ou esconde (None) o robô-alvo."""
        self.alvo_deg = None if q_deg is None else [float(v) for v in q_deg]
        self._alvo_editavel = bool(editavel) and q_deg is not None
        if limites is not None:
            self._limites = (float(limites[0]), float(limites[1]))
        if q_deg is None:
            self._junta_escolhida = None
        self._rebuild()
        self.update()

    def definir_estado_alvo(self, estado):
        """'ok' (verde), 'ruim' (vermelho), 'conferindo' (amarelo) ou 'ponto' (azul)."""
        if estado != self._alvo_estado:
            self._alvo_estado = estado
            self._rebuild()
            self.update()

    def definir_campo(self, base_deg, teto_deg):
        """Desenha o campo de trabalho CALCULADO (campo.Campo) a partir dos pontos
        medidos (listas de 6 ângulos, convenção Dobot): chão e contorno pelos
        pontos da base, teto pelos do teto e o alcance do braço em cada altura.
        Menos de 3 pontos da base ou nenhum do teto apaga o desenho. Devolve o
        Campo (ou None)."""
        self._campo = None
        if self.scene is not None and len(base_deg) >= 3 and len(teto_deg) >= 1:
            def flanges(lista):
                return [self.scene.link_transforms(dobot_deg_to_joint_values(q))
                        ['Link6'][:3, 3] * 1000.0 for q in lista]
            campo = Campo(flanges(base_deg), flanges(teto_deg))
            g = campo.malha(n_ang=72, n_z=20) / 1000.0
            self._campo = (campo, g)
        self.update()
        return self._campo[0] if self._campo else None

    def _draw_campo(self, painter, w, h, parte):
        """parte='faces' (atrás do robô) ou 'linhas' (por cima, finas)."""
        if self._campo is None:
            return
        cam = self.camera
        campo, g = self._campo
        nz, na, _ = g.shape
        uv, d = cam.project(g.reshape(-1, 3), w, h)
        uv, d = uv.reshape(nz, na, 2), d.reshape(nz, na)
        painter.setRenderHint(QPainter.Antialiasing, True)
        if parte == 'faces':
            painter.setPen(Qt.NoPen)
            polis = []
            for i in range(nz - 1):
                for j in range(na):
                    k = (j + 1) % na
                    dd = (d[i, j], d[i, k], d[i + 1, k], d[i + 1, j])
                    if min(dd) > cam.near:
                        polis.append((sum(dd) / 4, (uv[i, j], uv[i, k], uv[i + 1, k],
                                                    uv[i + 1, j]), CAMPO_FACE))
            for fila, cor in ((0, CAMPO_CHAO), (nz - 1, CAMPO_CHAO)):   # chão e teto
                if np.all(d[fila] > cam.near):
                    polis.append((float(d[fila].mean()), tuple(uv[fila]), cor))
            for _d, pts, cor in sorted(polis, key=lambda x: -x[0]):    # longe primeiro
                painter.setBrush(cor)
                painter.drawPolygon(*[QPointF(float(p[0]), float(p[1])) for p in pts])
            return
        # Linhas: anéis em algumas alturas e meridianos a cada 30°.
        painter.setPen(QPen(CAMPO_ARESTA, 1.2))
        for i in sorted({0, nz // 3, 2 * nz // 3, nz - 1}):
            for j in range(na):
                k = (j + 1) % na
                if d[i, j] > cam.near and d[i, k] > cam.near:
                    painter.drawLine(QPointF(*uv[i, j]), QPointF(*uv[i, k]))
        for j in range(0, na, max(1, na // 12)):
            for i in range(nz - 1):
                if d[i, j] > cam.near and d[i + 1, j] > cam.near:
                    painter.drawLine(QPointF(*uv[i, j]), QPointF(*uv[i + 1, j]))
        # Os pontos medidos.
        painter.setPen(Qt.NoPen)
        for pts, cor in ((campo.base, CAMPO_PONTO_BASE), (campo.teto, CAMPO_PONTO_TETO)):
            puv, pd = cam.project(np.asarray(pts) / 1000.0, w, h)
            painter.setBrush(cor)
            for p, dd in zip(puv.tolist(), pd.tolist()):
                if dd > cam.near:
                    painter.drawEllipse(QPointF(*p), 3.5, 3.5)

    def set_caption(self, text):
        if text != self._caption:
            self._caption = text
            self.update()

    def _rebuild(self):
        if self.scene is None:
            return
        cena = self._cena_leve if self._leve and self._cena_leve is not None else self.scene
        self._cena = cena
        transforms = cena.link_transforms(dobot_deg_to_joint_values(self.q_deg))
        self._world_tris, self._colors = scene_triangles(cena, transforms)
        self._alvo_tris = self._alvo_cores = self._alvo_igual = None
        if self.alvo_deg is not None:
            t_alvo = cena.link_transforms(dobot_deg_to_joint_values(self.alvo_deg))
            self._alvo_tris, cores = scene_triangles(cena, t_alvo)
            # Peças do alvo no mesmo lugar que as do robô: não são desenhadas (os
            # dois desenhos se misturariam), mas continuam valendo para o clique.
            igual = np.array([np.allclose(transforms.get(link, np.eye(4)),
                                          t_alvo.get(link, np.eye(4)), atol=1e-5)
                              for link in cena.part_links])
            self._alvo_igual = igual[cena.flat_part]
            tinta = np.array(CORES_ALVO.get(self._alvo_estado, CORES_ALVO['conferindo']), float)
            self._alvo_cores = 0.35 * cores + 0.65 * tinta

    # --- desenho ---

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), BACKGROUND)
        w, h = self.width(), self.height()

        if self._error:
            painter.setPen(TEXT_COLOR)
            painter.drawText(self.rect().adjusted(16, 16, -16, -16),
                             Qt.AlignCenter | Qt.TextWordWrap, self._error)
            return

        self._draw_ground(painter, w, h)
        self._draw_campo(painter, w, h, 'faces')
        self._draw_robot(painter, w, h)
        self._draw_campo(painter, w, h, 'linhas')
        self._draw_hud(painter, w, h)

    def _draw_ground(self, painter, w, h):
        cam = self.camera
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.setPen(QPen(GRID_COLOR, 1))
        span, step = 1.5, 0.25
        ticks = np.arange(-span, span + 1e-9, step)
        # Todas as linhas do chão numa projeção só (uma por linha pesava no quadro).
        n = len(ticks)
        ext = np.empty((2 * n, 2, 3))
        ext[:n, 0] = np.c_[np.full(n, -span), ticks, np.full(n, GROUND_Z)]
        ext[:n, 1] = np.c_[np.full(n, span), ticks, np.full(n, GROUND_Z)]
        ext[n:, 0] = np.c_[ticks, np.full(n, -span), np.full(n, GROUND_Z)]
        ext[n:, 1] = np.c_[ticks, np.full(n, span), np.full(n, GROUND_Z)]
        uv, depth = cam.project(ext.reshape(-1, 3), w, h)
        uv, depth = uv.reshape(-1, 2, 2), depth.reshape(-1, 2)
        for (a, b), (da, db) in zip(uv.tolist(), depth.tolist()):
            if da > cam.near and db > cam.near:
                painter.drawLine(QPointF(*a), QPointF(*b))

        # círculo de alcance nominal do CR10 (1375 mm)
        angles = np.linspace(0, 2 * math.pi, 96)
        ring = np.stack([CR10_REACH_M * np.cos(angles),
                         CR10_REACH_M * np.sin(angles),
                         np.full_like(angles, GROUND_Z)], axis=1)
        uv, depth = cam.project(ring, w, h)
        painter.setPen(QPen(REACH_COLOR, 1.2, Qt.DashLine))
        for i in range(len(uv) - 1):
            if depth[i] > cam.near and depth[i + 1] > cam.near:
                painter.drawLine(QPointF(*uv[i]), QPointF(*uv[i + 1]))

    def _draw_robot(self, painter, w, h):
        self._desenho = None
        if self._world_tris is None or self._world_tris.shape[0] == 0:
            return
        painter.setRenderHint(QPainter.Antialiasing, False)
        painter.setPen(Qt.NoPen)
        uv, rgb = shade_and_sort(self._world_tris, self._colors, self.camera, w, h)
        self._pintar(painter, uv, _cores_rgba(rgb))
        if self._alvo_tris is None:
            return
        # O alvo vem numa camada por cima (translúcido, como em programas de CAD):
        # ordenado junto com o robô, os dois desenhos se misturariam onde encostam.
        uv, rgb, idx = shade_and_sort(self._alvo_tris, self._alvo_cores, self.camera, w, h,
                                      com_indices=True)
        pecas = self._cena.flat_part[idx]
        # Dono de cada triângulo: a junta que move a peça (-1 = base, não mexe).
        junta_da_peca = np.array([JUNTA_DA_PECA.get(n, -1) for n in self._cena.part_links],
                                 dtype=np.int64)
        dono = junta_da_peca[pecas]
        self._desenho = (uv, dono, np.ones(len(idx), dtype=bool))
        visivel = ~self._alvo_igual[idx]
        alfa = np.where((dono == self._junta_escolhida) & (dono >= 0), 255, ALVO_ALFA)
        self._pintar(painter, uv[visivel], _cores_rgba(rgb[visivel], alfa[visivel]))

    def _pintar(self, painter, uv, chaves):
        """Pinta os triângulos já ordenados. A conversão para números do Python
        é feita de uma vez e as cores ficam guardadas (o laço é o que custa)."""
        cache = self._cores_qt
        pincel, poligono, ponto = painter.setBrush, painter.drawPolygon, QPointF
        for (x0, y0, x1, y1, x2, y2), k in zip(uv.reshape(-1, 6).tolist(), chaves.tolist()):
            cor = cache.get(k)
            if cor is None:
                cor = cache[k] = QColor((k >> 16) & 255, (k >> 8) & 255, k & 255, k >> 24)
            pincel(cor)
            poligono(ponto(x0, y0), ponto(x1, y1), ponto(x2, y2))

    def _junta_no_ponto(self, x, y):
        """Junta (0..5) da peça do alvo mais à frente no ponto da tela, ou None."""
        if self._desenho is None:
            return None
        uv, dono, alvo = self._desenho
        sel = np.flatnonzero(alvo & (dono >= 0))
        if not len(sel):
            return None
        a, b, c = uv[sel, 0], uv[sel, 1], uv[sel, 2]
        p = np.array([x, y], dtype=float)

        def lado(p1, p2, p3):
            return ((p1[..., 0] - p3[..., 0]) * (p2[..., 1] - p3[..., 1])
                    - (p2[..., 0] - p3[..., 0]) * (p1[..., 1] - p3[..., 1]))
        d1, d2, d3 = lado(p, a, b), lado(p, b, c), lado(p, c, a)
        dentro = ~(((d1 < 0) | (d2 < 0) | (d3 < 0)) & ((d1 > 0) | (d2 > 0) | (d3 > 0)))
        if not np.any(dentro):
            return None
        return int(dono[sel[np.flatnonzero(dentro)[-1]]])   # o último desenhado = o da frente

    def _draw_hud(self, painter, w, h):
        painter.setRenderHint(QPainter.Antialiasing, True)
        font = QFont(painter.font())
        font.setPointSize(9)
        painter.setFont(font)
        painter.setPen(TEXT_COLOR)
        if self._caption:
            painter.drawText(14, 22, self._caption)
        if self._alvo_editavel and self._junta_escolhida is not None:
            j = self._junta_escolhida
            painter.drawText(14, 42, f'J{j + 1} do alvo: {self.alvo_deg[j]:.1f}°  '
                                     '(arraste para os lados; Shift = ajuste fino)')
        dica = 'Arraste: girar   ·   Roda: zoom   ·   Botão direito: mover'
        if self._alvo_editavel:
            dica = 'Arraste uma peça do alvo: gira a junta dela   ·   ' + dica
        painter.drawText(14, h - 12, dica)

    # --- mouse ---

    def mousePressEvent(self, event):
        self._last_mouse = event.pos()
        self._drag_button = event.button()
        self._junta_escolhida = None
        if self._alvo_editavel and event.button() == Qt.LeftButton:
            self._junta_escolhida = self._junta_no_ponto(event.x(), event.y())
            self.update()

    def mouseReleaseEvent(self, _event):
        self._drag_button = None
        if self._junta_escolhida is not None:
            self._junta_escolhida = None
            self.update()

    def mouseMoveEvent(self, event):
        if self._last_mouse is None or self._drag_button is None:
            return
        dx = event.x() - self._last_mouse.x()
        dy = event.y() - self._last_mouse.y()
        self._last_mouse = event.pos()
        if self._junta_escolhida is not None:
            # Gira só o ALVO (nada vai para o robô daqui).
            fino = bool(event.modifiers() & Qt.ShiftModifier)
            j = self._junta_escolhida
            novo = self.alvo_deg[j] + dx * (GRAUS_POR_PX_FINO if fino else GRAUS_POR_PX)
            novo = round(min(self._limites[1], max(self._limites[0], novo)), 1)
            if novo != self.alvo_deg[j]:
                self.alvo_deg[j] = novo
                self._rebuild()
                self.ao_mudar_alvo.emit(list(self.alvo_deg))
        elif self._drag_button == Qt.LeftButton:
            self.camera.orbit(-dx * 0.008, dy * 0.008)
        else:
            self.camera.pan(dx, dy, self.height())
        self.update()

    def wheelEvent(self, event):
        self.camera.zoom(0.9 if event.angleDelta().y() > 0 else 1.1)
        self.update()
