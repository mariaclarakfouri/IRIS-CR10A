"""Coletas: o robô no sensor (palpação), com o explorador do cr10twin.

É a aba "Palpation" da palpation_gui do cr10twin (Lucas Martins), com os mesmos
modos, parâmetros, limites, validações e mensagem de início; o que muda é o
visual e o caminho até o robô:

  * quem planeja e controla a coleta é o `tactile_explorer` (pacote
    dobot_coletas, cópia sem mudanças). Ele move o gêmeo e o robô real segue
    o gêmeo pelo servidor do IRIS, com a verificação de segurança dele
    em cada ponto (ver dobot_control/coleta.py);
  * antes de começar, o robô vai à home da coleta por um movimento normal
    (MovJ, conferido pela segurança), como o pré-home da palpation_gui. Só
    então a coleta é armada no servidor, que confere se o robô real e o gêmeo
    estão na mesma posição;
  * Parar (do painel) e EMERGÊNCIA congelam o explorador e param o robô;
    "Encerrar" pede ao explorador para terminar e voltar à home.

Os dados ficam em ~/iris_registros/coletas (registrador do cr10twin).
"""
import glob
import json
import os
import threading
import time
from collections import deque

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QButtonGroup, QCheckBox, QDoubleSpinBox, QGridLayout, QHBoxLayout, QLabel,
    QPushButton, QSizePolicy, QSpinBox, QVBoxLayout, QWidget,
)

from ..constantes import HOME_JOINTS
from ..grafico_ao_vivo import GraficoAoVivo
from ..tema import COR
from ..widgets import Cartao, rotulo
from .base import Modulo

try:
    from builtin_interfaces.msg import Time as TempoRos
    from geometry_msgs.msg import Point
    from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                           QoSReliabilityPolicy, qos_profile_sensor_data)
    from rcl_interfaces.msg import Log
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, Empty, Float32, String
    from std_srvs.srv import SetBool
    from touch_pack_msgs.msg import PalpationStart, PalpationStatus

    from dobot_coletas.constants import (
        CONTACT_ON_N, FORCE_ABORT_LIMIT_N, FORCE_SETPOINT_MAX_N, FT_NOMINAL_RATE_HZ,
        HOLD_TOL_N, MATRIX_MAX_POINTS, MATRIX_SAFE_Z_MM_DEFAULT, MATRIX_SAFE_Z_MM_MAX,
        MATRIX_SAFE_Z_MM_MIN, MATRIX_SPAN_MAX_MM, MATRIX_TRANSIT_MMS_DEFAULT,
        MATRIX_TRANSIT_MMS_MAX, MATRIX_TRANSIT_MMS_MIN, PROBE_ALIGN_FORCE_N_DEFAULT,
        PROBE_ALIGN_POINTS_DEFAULT, PROBE_ALIGN_POINTS_MAX, PROBE_ALIGN_POINTS_MIN,
        PROBE_ALIGN_RADIUS_MM_DEFAULT, PROBE_ALIGN_RADIUS_MM_MAX, PROBE_ALIGN_RADIUS_MM_MIN,
        PROBE_ALIGN_RETRACT_MM_DEFAULT, PROBE_ALIGN_RETRACT_MM_MAX,
        PROBE_ALIGN_RETRACT_MM_MIN, PROBE_ALIGN_TILT_HARD_MAX_DEG,
        PROBE_ALIGN_TILT_MAX_DEG_DEFAULT, STAIRCASE_MODES, STEP_MAX_LEVELS, FMOD_MODES,
        hold_tol_n, new_run_id, staircase_levels,
    )
    COLETAS_OK, ERRO_COLETAS = True, ''
except ImportError as e:            # pacotes das coletas não compilados
    COLETAS_OK, ERRO_COLETAS = False, str(e)

# --- Limites da palpation_gui do cr10twin (os mesmos números) ---------------
FORCE_SP_DEFAULT = 2.0
SPEED_MIN, SPEED_MAX, SPEED_DEFAULT = 1.0, 30.0, 10.0              # mm/s
APPROACH_MIN, APPROACH_MAX, APPROACH_DEFAULT = 1.0, 30.0, 8.0      # mm/s
DEPTH_MIN, DEPTH_MAX, DEPTH_DEFAULT = 0.0, 120.0, 5.0              # mm
REPEAT_MIN, REPEAT_MAX, REPEAT_DEFAULT = 1, 50, 1
SLIDE_DIST_MIN, SLIDE_DIST_MAX, SLIDE_DIST_DEFAULT = 1.0, 300.0, 50.0
FMOD_HZ_MIN, FMOD_HZ_MAX = 0.1, 10.0
FMOD_CYCLES_MIN, FMOD_CYCLES_MAX, FMOD_CYCLES_DEFAULT = 1, 200, 20
FMOD_CTRL_DT_S = 0.030
FMOD_DT_MIN_S = 0.004
FMOD_MIN_PTS_PER_CYCLE = 5
SERVOJ_T_MIN_S = 0.020
SERVOJ_PERIODO_S = 0.030              # o mesmo do launch (dobot_bringup)
MATRIX_STEP_MIN, MATRIX_STEP_MAX, MATRIX_STEP_DEFAULT = 0.5, 50.0, 5.0
MATRIX_N_MIN, MATRIX_N_MAX, MATRIX_N_DEFAULT = 1, 20, 5
MATRIX_SIZE_MIN, MATRIX_SIZE_DEFAULT = MATRIX_STEP_MIN, 20.0
SPEED_FACTOR_PALPACAO = 10.0          # o cr10twin força SpeedFactor(10) na palpação
START_REPUBLISH_LOCKOUT_S = 2.0
FASES_ENCERRADAS = ('IDLE', 'DONE', 'ABORTED', 'FROZEN')
ESPERA_EXPLORADOR_S = 15.0
G_MS2 = 9.80665
LOG_AVISO = 30                        # nível WARN do /rosout
JANELA_GRAFICO_S = 30.0
PASTA_COLETAS = os.path.expanduser('~/iris_registros/coletas')
LARGURA_ROTULO = 190                  # mesma coluna de rótulos em todos os cartões
LARGURA_SELETOR = 330

MODOS = (('TOUCH', 'Toque'), ('SLIDE', 'Deslizamento'), ('MANUAL', 'Manual'),
         ('MATRIX_MAP', 'Mapa em grade'))
DESCRICAO_MODO = {
    'TOUCH': 'Desce até a força alvo, segura e volta à home. Repete o número de toques pedido.',
    'SLIDE': 'Desce até a força alvo, segura, desliza de lado com a altura travada e volta.',
    'MANUAL': 'Desce até a força alvo e segura até você encerrar. A força alvo pode ser '
              'mudada durante a coleta; com a escada, percorre os patamares sozinho.',
    'MATRIX_MAP': 'Deixe a ponteira acima do primeiro ponto (sem passar pela home). O primeiro '
                  'contato vira a origem e cada ponto da grade é tocado com a mesma força.',
}
NOME_FASE = {
    'IDLE': 'parado', 'HOME': 'indo à home', 'CALIBRATING': 'medindo a superfície',
    'DESCENDING': 'descendo', 'HOLD': 'segurando a força', 'MODULATING': 'onda de força',
    'SLIDING': 'deslizando', 'TRANSIT': 'indo ao próximo ponto', 'RETRACT': 'subindo',
    'DONE': 'concluída', 'ABORTED': 'abortada', 'FROZEN': 'congelada',
}


# --- Funções copiadas da palpation_gui (textos traduzidos) ------------------

def fmod_wave_dt(hz, servoj_period_s=FMOD_CTRL_DT_S):
    want = 1.0 / max(hz * FMOD_MIN_PTS_PER_CYCLE, 1e-9)
    return min(max(want, FMOD_DT_MIN_S, SERVOJ_T_MIN_S, servoj_period_s), FMOD_CTRL_DT_S)


def fmod_max_freq_hz(servoj_period_s=FMOD_CTRL_DT_S):
    return 1.0 / max(servoj_period_s * FMOD_MIN_PTS_PER_CYCLE, 1e-9)


def _serpentina(ix0, ix1, iy0, iy1):
    out = []
    for k, iy in enumerate(range(iy0, iy1 + 1)):
        cols = range(ix0, ix1 + 1) if k % 2 == 0 else range(ix1, ix0 - 1, -1)
        out.extend((ix, iy) for ix in cols)
    return out


def _cantos(n_cols, n_rows):
    xs, ys = sorted({0, n_cols - 1}), sorted({0, n_rows - 1})
    if len(xs) == 1 and len(ys) == 1:
        return [(0, 0)]
    if len(xs) == 1:
        return [(0, ys[0]), (0, ys[1])]
    if len(ys) == 1:
        return [(xs[0], 0), (xs[1], 0)]
    return [(xs[0], ys[0]), (xs[1], ys[0]), (xs[1], ys[1]), (xs[0], ys[1])]


def _ordem_indices(n_cols, n_rows, caminho):
    full = _serpentina(0, n_cols - 1, 0, n_rows - 1)
    if caminho != 'CORNERS':
        return full
    cantos = _cantos(n_cols, n_rows)
    vistos = set(cantos)
    return cantos + [p for p in full if p not in vistos]


def nos_da_grade(p):
    """Nós da grade (mm, relativos à origem) na ordem de visita, ou ([], erro).
    Mesma conta de gui_matrix._matrix_grid_nodes do cr10twin."""
    quadrada = p['matrix_shape'] == 'SQUARE'
    por_tamanho = p['matrix_sizing'] == 'SIZE'
    n_cols = max(MATRIX_N_MIN, min(MATRIX_N_MAX, int(p['matrix_cols'])))
    n_rows = max(MATRIX_N_MIN, min(MATRIX_N_MAX, int(p['matrix_rows'])))
    if quadrada:
        n_rows = n_cols
    if por_tamanho:
        largura, altura = p['matrix_width'], p['matrix_height']
        if quadrada:
            altura = largura
        largura = max(MATRIX_SIZE_MIN, min(MATRIX_SPAN_MAX_MM, largura))
        altura = max(MATRIX_SIZE_MIN, min(MATRIX_SPAN_MAX_MM, altura))
        step_x = largura / (n_cols - 1) if n_cols > 1 else 0.0
        step_y = altura / (n_rows - 1) if n_rows > 1 else 0.0
        finos = [f'passo em {ax} de {s:.2f} mm' for ax, s, n in
                 (('X', step_x, n_cols), ('Y', step_y, n_rows)) if n > 1 and s < MATRIX_STEP_MIN]
        if finos:
            return [], (f'{" e ".join(finos)}, abaixo do mínimo de {MATRIX_STEP_MIN:.1f} mm: '
                        'aumente o tamanho ou diminua colunas/linhas')
    else:
        step_x, step_y = p['matrix_step_x'], p['matrix_step_y']
        if quadrada:
            step_y = step_x
        step_x = max(MATRIX_STEP_MIN, min(MATRIX_STEP_MAX, step_x))
        step_y = max(MATRIX_STEP_MIN, min(MATRIX_STEP_MAX, step_y))
    total = n_cols * n_rows
    if total < 2:
        return [], 'uma grade 1×1 não tem pontos além da origem'
    if total > MATRIX_MAX_POINTS:
        return [], f'{total} pontos passam do máximo de {MATRIX_MAX_POINTS}'
    span = max(step_x * (n_cols - 1), step_y * (n_rows - 1))
    if span > MATRIX_SPAN_MAX_MM:
        return [], f'a grade ocupa {span:.0f} mm, acima de {MATRIX_SPAN_MAX_MM:.0f} mm'
    ordem = _ordem_indices(n_cols, n_rows, p['matrix_path'])
    return [(ix * step_x, iy * step_y) for ix, iy in ordem], ''


# --- Peças visuais ----------------------------------------------------------

def _numero(minimo, maximo, valor, passo, casas=1, sufixo=''):
    spin = QDoubleSpinBox()
    spin.setRange(minimo, maximo)
    spin.setDecimals(casas)
    spin.setSingleStep(passo)
    spin.setValue(valor)
    if sufixo:
        spin.setSuffix(f' {sufixo}')
    spin.setFixedWidth(118)
    spin.setKeyboardTracking(False)
    return spin


def _inteiro(minimo, maximo, valor, sufixo=''):
    spin = QSpinBox()
    spin.setRange(minimo, maximo)
    spin.setValue(valor)
    if sufixo:
        spin.setSuffix(f' {sufixo}')
    spin.setFixedWidth(118)
    spin.setKeyboardTracking(False)
    return spin


class Seletor(QWidget):
    """Botões lado a lado, um marcado por vez."""
    mudou = pyqtSignal(str)

    def __init__(self, opcoes, valor=None):
        super().__init__()
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        self._grupo = QButtonGroup(self)
        self._botoes = {}
        for chave, texto in opcoes:
            b = QPushButton(texto)
            b.setObjectName('segmento')
            b.setCheckable(True)
            b.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            b.clicked.connect(lambda _c, k=chave: self._clicado(k))
            self._grupo.addButton(b)
            self._botoes[chave] = b
            lay.addWidget(b)
        self.definir(valor if valor in self._botoes else opcoes[0][0])

    def _clicado(self, chave):
        self._valor = chave
        self.mudou.emit(chave)

    def definir(self, chave):
        if chave in self._botoes:
            self._valor = chave
            self._botoes[chave].setChecked(True)

    def valor(self):
        return self._valor


class Formulario:
    """Grade rótulo | campo, com a explicação como dica do mouse."""

    def __init__(self, layout_pai):
        self.grade = QGridLayout()
        self.grade.setHorizontalSpacing(14)
        self.grade.setVerticalSpacing(8)
        self.grade.setColumnStretch(2, 1)
        self.grade.setColumnMinimumWidth(0, LARGURA_ROTULO)
        layout_pai.addLayout(self.grade)
        self._linha = 0
        self.linhas = {}

    def campo(self, chave, texto, widget, dica='', extra=None):
        r = rotulo(texto, 'cartaoTexto')
        if dica:
            r.setToolTip(dica)
            widget.setToolTip(dica)
        self.grade.addWidget(r, self._linha, 0)
        self.grade.addWidget(widget, self._linha, 1)
        self.linhas[chave] = (r, widget) + ((extra,) if extra is not None else ())
        if extra is not None:
            self.grade.addWidget(extra, self._linha, 2)
        self._linha += 1
        return widget

    def largo(self, widget):
        self.grade.addWidget(widget, self._linha, 0, 1, 3)
        self._linha += 1
        return widget

    def mostrar(self, chave, visivel):
        for w in self.linhas[chave]:
            w.setVisible(visivel)


def _nota(texto=''):
    n = rotulo(texto, 'cartaoTexto')
    n.setWordWrap(True)
    n.setStyleSheet(f'color: {COR["texto_fraco"]};')
    return n


class PreviaGrade(QWidget):
    """Desenho da grade: origem, ordem de visita e o ponto em execução."""

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(190)
        self._nos = []
        self._ativo = 0

    def definir(self, nos, ativo=0):
        self._nos, self._ativo = list(nos), int(ativo)
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        r = QRectF(self.rect()).adjusted(18, 14, -18, -14)
        p.fillRect(self.rect(), QColor(COR['linha_alterna']))
        if not self._nos:
            p.setPen(QColor(COR['texto_fraco']))
            p.drawText(self.rect(), Qt.AlignCenter, 'grade inválida')
            return
        xs = [n[0] for n in self._nos]
        ys = [n[1] for n in self._nos]
        w, h = max(max(xs) - min(xs), 1e-6), max(max(ys) - min(ys), 1e-6)
        esc = min(r.width() / w, r.height() / h)
        x0 = r.left() + (r.width() - w * esc) / 2 - min(xs) * esc
        y0 = r.bottom() - (r.height() - h * esc) / 2 + min(ys) * esc

        def px(n):
            return QPointF(x0 + n[0] * esc, y0 - n[1] * esc)

        p.setPen(QPen(QColor(COR['borda_forte']), 1, Qt.DashLine))
        for a, b in zip(self._nos, self._nos[1:]):
            p.drawLine(px(a), px(b))
        for i, n in enumerate(self._nos):
            if i == 0:
                cor = QColor(COR['primaria'])
            elif self._ativo and i == self._ativo:
                cor = QColor(COR['sucesso'])
            elif self._ativo and i < self._ativo:
                cor = QColor(COR['texto_fraco'])
            else:
                cor = QColor(COR['texto_medio'])
            p.setPen(Qt.NoPen)
            p.setBrush(cor)
            raio = 5 if i == 0 or i == self._ativo else 3
            p.drawEllipse(px(n), raio, raio)
        p.setPen(QColor(COR['primaria']))
        o = px(self._nos[0])
        p.drawText(QPointF(o.x() + 7, o.y() - 6), 'origem')


# --- A página ---------------------------------------------------------------

class Coletas(Modulo):
    NOME = 'Coletas'
    CHAVE_HOME = 'coletas'
    TEXTO_HOME = ('Ponteira para baixo, acima da amostra e fora da margem de segurança: antes de cada '
                  'coleta o robô vai até ela. No Mapa em grade a home não é usada.')
    ICONE = 'record'
    DESCRICAO = ('O robô no sensor: toque, deslizamento, força manual e mapa em grade, '
                 'com o explorador e o registrador do cr10twin.')

    status_recebido = pyqtSignal(object)
    estado_coleta = pyqtSignal(str)

    def construir(self):
        self._fase = 'IDLE'
        self._fase_t = time.monotonic()
        self._ciclo = (0, 0)
        self._ponto = (0, 0)
        self._pausado = False
        self._home_conhecida = False
        self._estado_srv = {}
        self._forca = None              # (monotonic, N)
        self._alvo_cache = FORCE_SP_DEFAULT   # força alvo lida pela thread do ROS
        self._forca_buf = deque(maxlen=4000)
        self._forca_lock = threading.Lock()
        self._tarado = False
        self._tara_resposta = None
        self._tara_evento = threading.Event()
        self._tara_nova = None
        self._relogio = None            # último /clock (tempo do Gazebo)
        self._aviso_explorador = ''
        self._inicio_publicado = 0.0
        self._rodando = False

        if not COLETAS_OK:
            c = Cartao('Coletas indisponíveis',
                       'Os pacotes das coletas (dobot_coletas e touch_pack_msgs) não estão '
                       f'compilados neste computador ({ERRO_COLETAS}). Rode colcon build na '
                       'pasta do projeto e abra de novo.')
            self.layout_modulo.addWidget(c)
            self.layout_modulo.addStretch(1)
            self.iniciar_btn = None
            return

        colunas = QHBoxLayout()
        colunas.setSpacing(16)
        esquerda, direita = QVBoxLayout(), QVBoxLayout()
        esquerda.setSpacing(16)
        direita.setSpacing(16)
        colunas.addLayout(esquerda, 3)
        colunas.addLayout(direita, 2)
        self.layout_modulo.addLayout(colunas)

        self._montar_parametros(esquerda)
        self._montar_execucao(direita)
        esquerda.addStretch(1)
        direita.addStretch(1)

        self._ao_mudar_forca(self.forca.value())
        self._atualizar_onda()
        self._ligar_ros()
        self.status_recebido.connect(self._ao_status)
        self.estado_coleta.connect(self._ao_estado_coleta)
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._atualizar_painel)
        self._timer.start(100)
        self._timer_forca = QTimer(self)
        self._timer_forca.setSingleShot(True)
        self._timer_forca.setInterval(150)
        self._timer_forca.timeout.connect(self._publicar_forca_ao_vivo)
        self._ao_mudar_modo(self.modo.valor())

    # ================================================================ montagem

    def _montar_parametros(self, lay):
        cm = Cartao('Modo da coleta')
        self.modo = Seletor(MODOS, 'TOUCH')
        self.modo.mudou.connect(self._ao_mudar_modo)
        cm.corpo.addWidget(self.modo)
        self.descricao_modo = _nota()
        cm.corpo.addWidget(self.descricao_modo)
        lay.addWidget(cm)

        # Força e repetições
        cf = Cartao('Força')
        f = Formulario(cf.corpo)
        self._form_forca = f
        self.forca = f.campo('forca', 'Força alvo', _numero(
            CONTACT_ON_N, FORCE_SETPOINT_MAX_N, FORCE_SP_DEFAULT, 0.1, 1, 'N'),
            'Compressão mantida na descida, no HOLD e no deslizamento, em passos de 0,1 N '
            f'({CONTACT_ON_N:g} a {FORCE_SETPOINT_MAX_N:g} N). A coleta é cancelada acima de '
            f'{FORCE_ABORT_LIMIT_N:g} N. No modo Manual, mudar este valor durante a coleta muda '
            'o alvo na hora.', extra=_nota())
        self.forca_g = f.linhas['forca'][2]
        self.forca.valueChanged.connect(self._ao_mudar_forca)
        self.repeticoes = f.campo('repeticoes', 'Repetições', _inteiro(
            REPEAT_MIN, REPEAT_MAX, REPEAT_DEFAULT),
            'Quantos ciclos completos (descida, HOLD, [deslizamento], subida) seguidos. '
            'No Toque, é o número de toques.')
        self.janela_hold = f.campo('tempo', 'Duração do HOLD', _numero(
            0.2, 60.0, 5.0, 0.5, 1, 's'),
            'Quanto tempo o robô fica parado na amostra segurando a força alvo, depois de chegar '
            'nela. É a janela de medição (estatísticas do summary.json). Se a força sair da '
            'faixa, ele corrige sem reiniciar a contagem. Em cada toque, ou em cada ponto da '
            'grade. No Manual, o robô segura até você encerrar.')
        lay.addWidget(cf)

        # Deslizamento
        self.cartao_desliza = Cartao('Deslizamento')
        f = Formulario(self.cartao_desliza.corpo)
        self.velocidade = f.campo('vel', 'Velocidade', _numero(
            SPEED_MIN, SPEED_MAX, SPEED_DEFAULT, 1.0, 1, 'mm/s'),
            'Velocidade do deslizamento. Valores do artigo de referência: 5, 10 e 15 mm/s.')
        self.distancia = f.campo('dist', 'Distância', _numero(
            SLIDE_DIST_MIN, SLIDE_DIST_MAX, SLIDE_DIST_DEFAULT, 5.0, 1, 'mm'),
            'Comprimento do deslizamento. Máximo de segurança: 300 mm.')
        self.direcao = f.campo('dir', 'Direção', Seletor(
            [('+X', '+X'), ('-X', '−X'), ('+Y', '+Y'), ('-Y', '−Y')], '+Y'),
            'Direção do deslizamento, nos eixos do mundo do gêmeo.')
        self.direcao.setFixedWidth(LARGURA_SELETOR)
        lay.addWidget(self.cartao_desliza)

        # Força modulada (Toque)
        self.cartao_onda = Cartao('Força modulada')
        f = Formulario(self.cartao_onda.corpo)
        self.onda = f.campo('forma', 'Forma', Seletor(
            [('OFF', 'Desligada'), ('SINE', 'Seno'), ('COSINE', 'Cosseno')], 'OFF'),
            'Desligada: o HOLD mantém a força alvo. Seno/Cosseno: depois de estabilizar na média '
            'entre mínimo e máximo, a força segue a onda (por posição, dx = dF/K, com K medido '
            'na descida). A onda pedida vai na coluna setpoint_n do CSV.')
        self.onda.setFixedWidth(LARGURA_SELETOR)
        self.onda.mudou.connect(lambda _v: self._atualizar_onda())
        self._form_onda = f
        self.onda_min = f.campo('omin', 'Força mínima', _numero(
            CONTACT_ON_N, FORCE_SETPOINT_MAX_N, 2.0, 0.1, 1, 'N'), 'Força no vale da onda.')
        self.onda_max = f.campo('omax', 'Força máxima', _numero(
            CONTACT_ON_N, FORCE_SETPOINT_MAX_N, 3.0, 0.1, 1, 'N'),
            'Força no pico da onda. Amplitude (máx − mín)/2, até 5 N.')
        self.onda_hz = f.campo('ohz', 'Frequência', _numero(
            FMOD_HZ_MIN, FMOD_HZ_MAX, 2.0, 0.1, 1, 'Hz'),
            'Com o ServoJ a 30 ms, o teto é de '
            f'{fmod_max_freq_hz(SERVOJ_PERIODO_S):.2f} Hz (5 pontos por ciclo); acima disso o '
            'explorador recusa a onda.')
        self.onda_ciclos = f.campo('ociclos', 'Ciclos', _inteiro(
            FMOD_CYCLES_MIN, FMOD_CYCLES_MAX, FMOD_CYCLES_DEFAULT),
            'Períodos por toque. A onda dura ciclos ÷ frequência segundos.')
        self.previa_onda = f.largo(_nota())
        lay.addWidget(self.cartao_onda)

        # Escada (Manual e Mapa)
        self.cartao_escada = Cartao('Escada de força')
        f = Formulario(self.cartao_escada.corpo)
        self.escada_passo = f.campo('epasso', 'Degrau', _numero(0.0, 5.0, 0.0, 0.1, 1, 'N'),
                                    'Altura de cada degrau. 0 desliga a escada.')
        self.escada_inicio = f.campo('einicio', 'Primeiro patamar', _numero(
            CONTACT_ON_N, FORCE_SETPOINT_MAX_N, 0.5, 0.1, 1, 'N'), 'Força do primeiro patamar.')
        self.escada_pico = f.campo('epico', 'Patamar de pico', _numero(
            CONTACT_ON_N, FORCE_SETPOINT_MAX_N, 3.0, 0.1, 1, 'N'),
            'Patamar mais alto, sempre alcançado exatamente. Depois a coleta desce pelos '
            'mesmos patamares (a ida e volta mostra histerese e relaxação).')
        self.escada_tempo = f.campo('etempo', 'Tempo em cada patamar', _numero(
            0.0, 120.0, 5.0, 0.5, 1, 's'),
            'Janela de medição de cada patamar, depois de estabilizar.')
        self.previa_escada = f.largo(_nota())
        lay.addWidget(self.cartao_escada)

        # Grade (Mapa)
        self.cartao_grade = Cartao('Grade')
        f = Formulario(self.cartao_grade.corpo)
        self.grade_formato = f.campo('gformato', 'Formato', Seletor(
            [('SQUARE', 'Quadrada'), ('RECT', 'Retangular')], 'SQUARE'))
        self.grade_formato.setFixedWidth(LARGURA_SELETOR)
        self.grade_medida = f.campo('gmedida', 'Medir por', Seletor(
            [('STEP', 'Passo'), ('SIZE', 'Tamanho')], 'STEP'),
            'Passo: distância entre pontos. Tamanho: o lado da peça (o passo sai da conta).')
        self.grade_medida.setFixedWidth(LARGURA_SELETOR)
        self.grade_ordem = f.campo('gordem', 'Ordem', Seletor(
            [('CORNERS', 'Cantos primeiro'), ('SERPENTINE', 'Serpentina')], 'CORNERS'),
            'Cantos primeiro: os 4 extremos são tocados antes, como conferência de posição.')
        self.grade_ordem.setFixedWidth(LARGURA_SELETOR)
        self.grade_cols = f.campo('gcols', 'Colunas', _inteiro(MATRIX_N_MIN, MATRIX_N_MAX,
                                                               MATRIX_N_DEFAULT))
        self.grade_linhas = f.campo('glinhas', 'Linhas', _inteiro(MATRIX_N_MIN, MATRIX_N_MAX,
                                                                  MATRIX_N_DEFAULT))
        self.grade_px = f.campo('gpx', 'Passo em X', _numero(
            MATRIX_STEP_MIN, MATRIX_STEP_MAX, MATRIX_STEP_DEFAULT, 0.5, 1, 'mm'))
        self.grade_py = f.campo('gpy', 'Passo em Y', _numero(
            MATRIX_STEP_MIN, MATRIX_STEP_MAX, MATRIX_STEP_DEFAULT, 0.5, 1, 'mm'))
        self.grade_larg = f.campo('glarg', 'Largura', _numero(
            MATRIX_SIZE_MIN, MATRIX_SPAN_MAX_MM, MATRIX_SIZE_DEFAULT, 1.0, 1, 'mm'))
        self.grade_alt = f.campo('galt', 'Altura', _numero(
            MATRIX_SIZE_MIN, MATRIX_SPAN_MAX_MM, MATRIX_SIZE_DEFAULT, 1.0, 1, 'mm'))
        self.grade_folga = f.campo('gfolga', 'Folga entre pontos', _numero(
            MATRIX_SAFE_Z_MM_MIN, MATRIX_SAFE_Z_MM_MAX, MATRIX_SAFE_Z_MM_DEFAULT, 1.0, 1, 'mm'),
            'Quanto a ponteira sobe para ir de um ponto ao outro (medido na normal da peça).')
        self.grade_transito = f.campo('gtrans', 'Velocidade entre pontos', _numero(
            MATRIX_TRANSIT_MMS_MIN, MATRIX_TRANSIT_MMS_MAX, MATRIX_TRANSIT_MMS_DEFAULT, 1.0,
            1, 'mm/s'))
        self.previa_grade = f.largo(PreviaGrade())
        self.resumo_grade = f.largo(_nota())
        self._form_grade = f
        for s in (self.grade_formato, self.grade_medida, self.grade_ordem):
            s.mudou.connect(lambda _v: self._atualizar_grade())
        lay.addWidget(self.cartao_grade)

        # Avançado
        self.cartao_avancado = Cartao('Ajustes avançados')
        self.btn_avancado = QPushButton('Mostrar')
        self.btn_avancado.setObjectName('discreto')
        self.btn_avancado.setCheckable(True)
        self.btn_avancado.setFixedWidth(110)
        self.btn_avancado.toggled.connect(self._mostrar_avancado)
        self.cartao_avancado.corpo.addWidget(self.btn_avancado)
        self._caixa_avancado = QWidget()
        av = QVBoxLayout(self._caixa_avancado)
        av.setContentsMargins(0, 4, 0, 0)
        av.setSpacing(10)
        f = Formulario(av)
        self.profundidade = f.campo('prof', 'Curso máximo da descida', _numero(
            DEPTH_MIN, DEPTH_MAX, DEPTH_DEFAULT, 0.5, 1, 'mm'),
            'Quanto a ponteira pode descer no máximo. A descida para antes, ao chegar na força alvo.')
        self.vel_descida = f.campo('vdesc', 'Velocidade da descida', _numero(
            APPROACH_MIN, APPROACH_MAX, APPROACH_DEFAULT, 1.0, 1, 'mm/s'),
            'Descida no ar. No primeiro contato o braço PARA, alivia o pico de inércia e fecha na '
            'força em micropassos de 10 a 20 µm.')
        self.tol_hold = f.campo('tol', 'HOLD: tolerância', _numero(
            round(HOLD_TOL_N, 3), 2.0, round(hold_tol_n(FORCE_SP_DEFAULT), 3), 0.01, 3, 'N'),
            'Meia largura da faixa em volta da força alvo em que ela conta como estável. O piso '
            f'({HOLD_TOL_N:.3f} N) é o ruído da célula filtrada.')
        self.limite_hold = f.campo('lim', 'HOLD: tempo para chegar na força', _numero(
            2.0, 60.0, 8.0, 1.0, 1, 's'),
            'Quanto tempo ele espera a força entrar na faixa. Passou disso, segue com aviso e '
            'faz a medição assim mesmo (o tempo segurando a força não é cortado).')
        self.inclinacao = f.campo('incl', 'Inclinação da superfície', _numero(
            -10.0, 10.0, 0.0, 0.1, 1, '°'),
            'Inclinação da peça na direção do deslizamento (+ = sobe). O deslizamento segue essa '
            'rampa. 0 para uma peça nivelada.')
        self.home_conhecida = f.largo(QCheckBox('Home conhecida (descida em 2 estágios)'))
        self.home_conhecida.setToolTip(
            'Marca sozinha quando o explorador já aprendeu onde fica a superfície nesta home: a '
            'descida vai rápido até perto do contato e só rasteja o fim. DESMARQUE ao trocar a '
            'amostra, o calço ou a fixação: o contato aprendido é esquecido e a próxima descida '
            'rasteja do início (essa descida de aprendizado não é gravada).')
        self.home_conhecida.clicked.connect(self._ao_clicar_home_conhecida)

        self.calibrar = f.largo(QCheckBox('Medir a superfície e alinhar o ângulo de ataque'))
        self.calibrar.setToolTip(
            'Antes da coleta, alguns toques leves em volta do ponto medem o plano da peça e a '
            'ponteira passa a descer na normal medida. Custa algumas descidas a mais e aborta '
            'se o plano não for confiável ou inclinado demais.')
        self._caixa_calibrar = QWidget()
        fc = Formulario(QVBoxLayout(self._caixa_calibrar))
        fc.grade.setContentsMargins(22, 0, 0, 0)
        self.cal_pontos = fc.campo('cp', 'Toques', _inteiro(
            PROBE_ALIGN_POINTS_MIN, PROBE_ALIGN_POINTS_MAX, PROBE_ALIGN_POINTS_DEFAULT),
            'Toques no anel (mais um no centro). 4 ou mais usam mínimos quadrados.')
        self.cal_raio = fc.campo('cr', 'Raio', _numero(
            PROBE_ALIGN_RADIUS_MM_MIN, PROBE_ALIGN_RADIUS_MM_MAX, PROBE_ALIGN_RADIUS_MM_DEFAULT,
            1.0, 1, 'mm'), 'Raio do anel de toques; fique dentro da região plana da peça.')
        self.cal_forca = fc.campo('cf', 'Força dos toques', _numero(
            CONTACT_ON_N, FORCE_SETPOINT_MAX_N, PROBE_ALIGN_FORCE_N_DEFAULT, 0.1, 1, 'N'),
            'Leve, para não marcar a peça. Nunca passa da força alvo.')
        self.cal_recuo = fc.campo('crec', 'Recuo antes de girar', _numero(
            PROBE_ALIGN_RETRACT_MM_MIN, PROBE_ALIGN_RETRACT_MM_MAX,
            PROBE_ALIGN_RETRACT_MM_DEFAULT, 1.0, 1, 'mm'),
            'Subida em linha reta antes de o punho girar (a ponta descreve um arco de ~68 mm).')
        self.cal_incl = fc.campo('cinc', 'Inclinação máxima', _numero(
            1.0, PROBE_ALIGN_TILT_HARD_MAX_DEG, PROBE_ALIGN_TILT_MAX_DEG_DEFAULT, 1.0, 1, '°'),
            'Acima disso a coleta aborta: inclinação assim é problema de fixação.')
        f.largo(self._caixa_calibrar)

        self.angulo_manual = f.largo(QCheckBox('Ângulo de ataque escolhido (contato oblíquo)'))
        self.angulo_manual.setToolTip(
            'Só Toque e Manual. Um toque vertical acha o alvo e a ponteira volta a ele pelo '
            'ângulo pedido.')
        self._caixa_manual = QWidget()
        fm = Formulario(QVBoxLayout(self._caixa_manual))
        fm.grade.setContentsMargins(22, 0, 0, 0)
        self.man_incl = fm.campo('mi', 'Inclinação', _numero(
            0.0, PROBE_ALIGN_TILT_HARD_MAX_DEG, 0.0, 1.0, 1, '°'), 'Desvio da vertical.')
        self.man_az = fm.campo('ma', 'Rumo', _numero(-180.0, 180.0, 0.0, 5.0, 1, '°'),
                               'Rumo da ferramenta no plano do mundo: 0° = +X, 90° = +Y.')
        self.man_forca = fm.campo('mf', 'Força do toque', _numero(
            CONTACT_ON_N, FORCE_SETPOINT_MAX_N, PROBE_ALIGN_FORCE_N_DEFAULT, 0.1, 1, 'N'))
        self.man_folga = fm.campo('mfo', 'Folga até a peça', _numero(
            PROBE_ALIGN_RETRACT_MM_MIN, PROBE_ALIGN_RETRACT_MM_MAX,
            PROBE_ALIGN_RETRACT_MM_DEFAULT, 1.0, 1, 'mm'))
        f.largo(self._caixa_manual)
        self.calibrar.toggled.connect(lambda v: self._alternar_ataque(self.calibrar, v))
        self.angulo_manual.toggled.connect(lambda v: self._alternar_ataque(self.angulo_manual, v))
        self._caixa_calibrar.hide()
        self._caixa_manual.hide()
        self.cartao_avancado.corpo.addWidget(self._caixa_avancado)
        self._caixa_avancado.hide()
        lay.addWidget(self.cartao_avancado)

        for w in (self.onda_min, self.onda_max, self.onda_hz, self.onda_ciclos,
                  self.escada_passo, self.escada_inicio, self.escada_pico, self.escada_tempo):
            w.valueChanged.connect(lambda _v: self._atualizar_previas())
        for w in (self.grade_cols, self.grade_linhas, self.grade_px, self.grade_py,
                  self.grade_larg, self.grade_alt):
            w.valueChanged.connect(lambda _v: self._atualizar_grade())

    def _montar_execucao(self, lay):

        # Executar
        ce = Cartao('Executar')
        self.iniciar_btn = QPushButton('Iniciar coleta')
        self.iniciar_btn.clicked.connect(self._iniciar)
        ce.corpo.addWidget(self.iniciar_btn)
        linha = QHBoxLayout()
        self.pausar_btn = QPushButton('Pausar')
        self.pausar_btn.setObjectName('secundario')
        self.pausar_btn.clicked.connect(self._pausar)
        self.encerrar_btn = QPushButton('Encerrar e voltar à home')
        self.encerrar_btn.setObjectName('secundario')
        self.encerrar_btn.setToolTip('O explorador termina a coleta e leva o robô à home. '
                                     'Para parar no lugar na hora, use Parar.')
        self.encerrar_btn.clicked.connect(self._encerrar)
        linha.addWidget(self.pausar_btn)
        linha.addWidget(self.encerrar_btn)
        ce.corpo.addLayout(linha)
        self.mensagem = _nota('')
        ce.corpo.addWidget(self.mensagem)
        lay.addWidget(ce)

        # Leitura ao vivo
        cl = Cartao('Ao vivo')
        g = QGridLayout()
        g.setHorizontalSpacing(14)
        g.setVerticalSpacing(6)
        self.forca_valor = QLabel('—')
        self.forca_valor.setStyleSheet('font-size: 26px; font-weight: 700;')
        self.forca_estado = rotulo('', 'cartaoTexto')
        g.addWidget(rotulo('Força na ponteira', 'cartaoTexto'), 0, 0)
        g.addWidget(self.forca_valor, 1, 0, 1, 2)
        g.addWidget(self.forca_estado, 2, 0, 1, 2)
        self.lbl_alvo = rotulo('—', 'cartaoTexto')
        self.lbl_fase = rotulo('—', 'cartaoTexto')
        self.lbl_tempo = rotulo('—', 'cartaoTexto')
        self.lbl_ponto = rotulo('—', 'cartaoTexto')
        for i, (nome, w) in enumerate((('Força alvo', self.lbl_alvo), ('Fase', self.lbl_fase),
                                       ('Tempo na fase', self.lbl_tempo),
                                       ('Ciclo / ponto', self.lbl_ponto))):
            g.addWidget(rotulo(nome, 'cartaoTexto'), 3 + i, 0)
            g.addWidget(w, 3 + i, 1)
        g.setColumnStretch(1, 1)
        cl.corpo.addLayout(g)
        # Tara aqui mesmo: zera a força que a coleta usa (/load_cell/tare, o leitor do
        # cr10twin) e, junto, a leitura da aba Célula de Carga.
        linha_tara = QHBoxLayout()
        self.tara_btn = QPushButton('Fazer tara')
        self.tara_btn.setObjectName('secundario')
        self.tara_btn.setToolTip('Zera a força com a ponteira NO AR, sem encostar em nada. Usa a '
                                 'média de quase 1 s e recusa se a célula estiver se mexendo. '
                                 'Zera também a leitura da aba Célula de Carga.')
        self.tara_btn.clicked.connect(self._fazer_tara)
        self.tara_txt = rotulo('', 'cartaoTexto')
        self.tara_txt.setWordWrap(True)
        linha_tara.addWidget(self.tara_btn)
        linha_tara.addWidget(self.tara_txt, 1)
        cl.corpo.addLayout(linha_tara)
        self.grafico = GraficoAoVivo('Força (N)', ['Medida', 'Alvo'], 'N', altura_px=200,
                                     faixa_min=1.0)
        cl.corpo.addWidget(self.grafico)
        lay.addWidget(cl)

        cd = Cartao('Dados')
        cd.corpo.addWidget(_nota(
            f'Só no robô real (no gêmeo nada é gravado). Cada coleta vai para {PASTA_COLETAS}'
            '/<sensor>/<modo>/<data_hora>: samples.csv e params.json (registrador do cr10twin) '
            'e grafico_robo.png (força, trajetória da ponteira e fases). O sensor tátil tem '
            'gráfico e gravação próprios, na aba Tátil. A primeira descida numa home nova é de '
            'aprendizado e não é gravada.'))
        lay.addWidget(cd)

    # ===================================================================== ROS

    def _ligar_ros(self):
        node = self.janela.ros_node
        self._node = node
        comando = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self._pub_inicio = node.create_publisher(PalpationStart, '/palpation/start', comando)
        self._pub_parar = node.create_publisher(String, '/palpation/stop', 10)
        self._pub_pausa = node.create_publisher(Bool, '/palpation/pause', 10)
        self._pub_forca = node.create_publisher(Float32, '/palpation/set_force', 10)
        self._pub_esquecer = node.create_publisher(Empty, '/palpation/forget_contact', 10)
        self._pub_tara = node.create_publisher(Empty, '/load_cell/tare', 10)
        self._cli_coleta = node.create_client(SetBool, 'coleta')
        node.create_subscription(PalpationStatus, '/palpation/status',
                                 lambda m: self.status_recebido.emit(m), 10)
        node.create_subscription(Float32, '/load_cell/force_net', self._on_forca,
                                 qos_profile_sensor_data)
        node.create_subscription(Bool, '/load_cell/tared',
                                 lambda m: setattr(self, '_tarado', bool(m.data) or self._tarado),
                                 10)
        node.create_subscription(String, '/load_cell/tare_result', self._on_tara, 10)
        node.create_subscription(String, '/dobot/coleta_estado',
                                 lambda m: self.estado_coleta.emit(m.data), 10)
        node.create_subscription(Clock, '/clock', self._on_relogio, qos_profile_sensor_data)
        # Último aviso/erro do explorador: é o motivo quando uma coleta aborta.
        node.create_subscription(Log, '/rosout', self._on_log, 50)

    def _on_forca(self, msg):
        t = time.monotonic()
        with self._forca_lock:
            self._forca = (t, float(msg.data))
            self._forca_buf.append((t, float(msg.data), self._alvo_cache))

    def _on_tara(self, msg):
        self._tara_nova = str(msg.data)        # mostrada pela thread do Qt
        self._tara_resposta = str(msg.data)
        if self._tara_resposta.startswith('ok'):
            self._tarado = True
        self._tara_evento.set()

    def _on_log(self, msg):
        # Log.WARN vem como byte no Humble (b'\x1e'): compara com o número.
        try:
            if msg.name == 'tactile_explorer' and int(msg.level) >= LOG_AVISO:
                self._aviso_explorador = str(msg.msg)
        except Exception:
            pass

    def _on_relogio(self, msg):
        self._relogio = msg.clock

    def _ao_status(self, m):
        if m.phase != self._fase:
            self._fase_t = time.monotonic()
        self._fase = str(m.phase)
        self._ciclo = (int(m.cycle), int(m.cycles_total))
        self._ponto = (int(m.wp_index), int(m.wp_total))
        self._pausado = bool(m.paused)
        if bool(m.home_known) != self._home_conhecida:
            self._home_conhecida = bool(m.home_known)
            self.home_conhecida.setChecked(self._home_conhecida)
        if self.modo.valor() == 'MATRIX_MAP':
            nos, _err = nos_da_grade(self._params_grade())
            self.previa_grade.definir(nos, self._ponto[0])

    def _ao_estado_coleta(self, texto):
        try:
            self._estado_srv = json.loads(texto)
        except ValueError:
            return

    def _chamar_coleta(self, ligar, espera_s=8.0):
        """Arma/desarma a coleta no servidor (chamado fora da thread do Qt)."""
        if not self._cli_coleta.wait_for_service(timeout_sec=2.0):
            return False, 'O servidor do robô não respondeu (serviço /coleta).'
        req = SetBool.Request()
        req.data = bool(ligar)
        pronto = threading.Event()
        fut = self._cli_coleta.call_async(req)
        fut.add_done_callback(lambda _f: pronto.set())
        if not pronto.wait(espera_s):
            return False, 'O servidor do robô não respondeu a tempo.'
        r = fut.result()
        return bool(r.success), str(r.message)

    # =============================================================== interface

    def widgets_de_movimento(self):
        return [w for w in (self.iniciar_btn,) if w is not None]

    def _mostrar_avancado(self, visivel):
        self._caixa_avancado.setVisible(visivel)
        self.btn_avancado.setText('Esconder' if visivel else 'Mostrar')

    def _alternar_ataque(self, qual, ligado):
        # Os dois modos de ataque são excludentes (como no cr10twin).
        if ligado:
            outro = self.angulo_manual if qual is self.calibrar else self.calibrar
            outro.setChecked(False)
        self._caixa_calibrar.setVisible(self.calibrar.isChecked())
        self._caixa_manual.setVisible(self.angulo_manual.isChecked())

    def _ao_mudar_modo(self, modo):
        self.descricao_modo.setText(DESCRICAO_MODO[modo])
        self.cartao_desliza.setVisible(modo == 'SLIDE')
        self.cartao_onda.setVisible(modo in FMOD_MODES)
        self.cartao_escada.setVisible(modo in STAIRCASE_MODES)
        self.cartao_grade.setVisible(modo == 'MATRIX_MAP')
        self._form_forca.mostrar('repeticoes', modo not in ('MANUAL', 'MATRIX_MAP'))
        self._form_forca.mostrar('tempo', modo != 'MANUAL')
        self._form_forca.linhas['repeticoes'][0].setText(
            'Número de toques' if modo == 'TOUCH' else 'Repetições')
        self.iniciar_btn.setText({'TOUCH': 'Iniciar toque', 'MANUAL': 'Iniciar coleta manual',
                                  'MATRIX_MAP': 'Iniciar mapa em grade'}.get(
                                      modo, 'Iniciar deslizamento'))
        self._atualizar_previas()
        self._atualizar_grade()

    def _ao_mudar_forca(self, v):
        self._alvo_cache = float(v)
        self.forca_g.setText(f'≈ {v / G_MS2 * 1000:.0f} g')
        self._atualizar_previas()
        if self.modo.valor() == 'MANUAL' and self._fase not in FASES_ENCERRADAS:
            self._timer_forca.start()        # espera 150 ms sem mudança (como o cr10twin)

    def _publicar_forca_ao_vivo(self):
        if self.modo.valor() != 'MANUAL' or self._fase in FASES_ENCERRADAS:
            return
        self._pub_forca.publish(Float32(data=float(self.forca.value())))
        self._msg(f'Nova força alvo: {self.forca.value():.1f} N.')

    def _ao_clicar_home_conhecida(self, marcado):
        if marcado and not self._home_conhecida:
            self.home_conhecida.setChecked(False)
            self._msg('A home fica "conhecida" sozinha, ao aprender o contato numa descida.')
            return
        if not marcado:
            self._pub_esquecer.publish(Empty())
            self._home_conhecida = False
            self._msg('Contato aprendido desta home esquecido: a próxima descida rasteja do '
                      'início e aprende de novo.')

    def _atualizar_onda(self):
        ligada = self.onda.valor() != 'OFF'
        for chave in ('omin', 'omax', 'ohz', 'ociclos'):
            self._form_onda.mostrar(chave, ligada)
        self._atualizar_previas()

    def _atualizar_previas(self):
        if not hasattr(self, 'previa_escada'):
            return
        modo = self.modo.valor()
        # Escada (mesma staircase_levels do explorador)
        passo, ini = self.escada_passo.value(), self.escada_inicio.value()
        pico, tempo = self.escada_pico.value(), self.escada_tempo.value()
        if passo <= 0.0 or pico <= ini:
            self.previa_escada.setText('Escada desligada: ' + (
                'cada ponto da grade segura uma força só.' if modo == 'MATRIX_MAP' else
                'o Manual segura uma força só, ajustável durante a coleta.'))
        else:
            lv = staircase_levels(ini, passo, pico)
            if not lv:
                self.previa_escada.setText(f'Degrau pequeno demais: mais de {STEP_MAX_LEVELS} '
                                           'patamares. Aumente o degrau.')
            else:
                seq = ' → '.join(f'{v:g}' for v in lv[:(len(lv) + 1) // 2])
                n = 1
                if modo == 'MATRIX_MAP':
                    n = max(len(nos_da_grade(self._params_grade())[0]) - 1, 1)
                total = len(lv) * tempo * n
                self.previa_escada.setText(
                    f'{len(lv)} patamares: {seq} N, e depois de volta pelos mesmos. '
                    + (f'Em cada um dos {n} pontos: pelo menos {total / 60:.0f} min de medição.'
                       if modo == 'MATRIX_MAP' else
                       f'Pelo menos {total / 60:.1f} min de medição, mais a estabilização.'))
        # Onda
        forma = self.onda.valor()
        self.previa_onda.setVisible(forma != 'OFF')
        if forma == 'OFF':
            return
        fmin, fmax = sorted((self.onda_min.value(), self.onda_max.value()))
        hz = self.onda_hz.value()
        amp = (fmax - fmin) / 2
        if amp <= 1e-3:
            self.previa_onda.setText('Mínimo e máximo iguais: sem onda, fica a força alvo.')
            return
        dt = fmod_wave_dt(hz, SERVOJ_PERIODO_S)
        pts = 1.0 / max(hz * dt, 1e-9)
        if pts < FMOD_MIN_PTS_PER_CYCLE:
            # Único aviso que fica: com isso o explorador recusa a onda.
            self.previa_onda.setText(
                f'Frequência alta demais: o explorador vai RECUSAR a onda (máximo de '
                f'{fmod_max_freq_hz(SERVOJ_PERIODO_S):.2f} Hz).')
            return
        self.previa_onda.setText(f'A célula de carga mede cerca de {FT_NOMINAL_RATE_HZ / hz:.0f} '
                                 f'amostras por ciclo da onda ({FT_NOMINAL_RATE_HZ:.0f} por segundo).')

    def _params_grade(self):
        return {'matrix_shape': self.grade_formato.valor(),
                'matrix_sizing': self.grade_medida.valor(),
                'matrix_path': self.grade_ordem.valor(),
                'matrix_cols': self.grade_cols.value(), 'matrix_rows': self.grade_linhas.value(),
                'matrix_step_x': self.grade_px.value(), 'matrix_step_y': self.grade_py.value(),
                'matrix_width': self.grade_larg.value(), 'matrix_height': self.grade_alt.value()}

    def _atualizar_grade(self):
        if not hasattr(self, '_form_grade'):
            return
        quad = self.grade_formato.valor() == 'SQUARE'
        tam = self.grade_medida.valor() == 'SIZE'
        f = self._form_grade
        f.mostrar('glinhas', not quad)
        f.mostrar('gpx', not tam)
        f.mostrar('gpy', not tam and not quad)
        f.mostrar('glarg', tam)
        f.mostrar('galt', tam and not quad)
        nos, err = nos_da_grade(self._params_grade())
        self.previa_grade.definir(nos, self._ponto[0])
        self.resumo_grade.setText(err.capitalize() + '.' if err else
                                  f'Origem + {len(nos) - 1} pontos.')
        self._atualizar_previas()

    def _msg(self, texto, tipo=''):
        cor = {'erro': COR['perigo'], 'aviso': COR['aviso_texto'],
               'ok': COR['sucesso']}.get(tipo, COR['texto_fraco'])
        self.mensagem.setStyleSheet(f'color: {cor};')
        self.mensagem.setText(texto)

    @property
    def home(self):
        """A Home das Coletas (cartão no alto da aba), ou None se não foi gravada."""
        return self.janela.home_de('coletas')

    def ao_ler_juntas(self, juntas, robo_parado):
        self._juntas = list(juntas)

    def _atualizar_painel(self):
        agora = time.monotonic()
        with self._forca_lock:
            forca = self._forca
            buf = list(self._forca_buf)
        recente = forca is not None and agora - forca[0] < 3.0
        if recente:
            f = forca[1]
            self.forca_valor.setText(f'{f:+.2f} N   {f / G_MS2 * 1000:+.0f} g')
            if not self._tarado:
                txt, cor = 'tara ainda não feita', COR['aviso_texto']
            elif f > FORCE_ABORT_LIMIT_N * 0.9:
                txt, cor = f'perto do limite ({FORCE_ABORT_LIMIT_N:.0f} N)', COR['perigo']
            elif f > CONTACT_ON_N:
                txt, cor = 'em contato', COR['sucesso']
            else:
                txt, cor = 'sem contato', COR['texto_fraco']
            self.forca_estado.setText(txt)
            self.forca_estado.setStyleSheet(f'color: {cor};')
        else:
            self.forca_valor.setText('—')
            self.forca_estado.setText('sem leitura da célula (/load_cell/force_net)')
            self.forca_estado.setStyleSheet(f'color: {COR["texto_fraco"]};')
        self.lbl_alvo.setText(f'{self.forca.value():.1f} N  (≈ {self.forca.value() / G_MS2 * 1000:.0f} g)')
        fase = self._fase
        txt = f'{NOME_FASE.get(fase, fase)} ({fase})'
        if self._pausado:
            txt += ' · PAUSADA'
        self.lbl_fase.setText(txt)
        ativa = fase not in FASES_ENCERRADAS
        self.lbl_tempo.setText(f'{agora - self._fase_t:.1f} s' if ativa else '—')
        c, ct = self._ciclo
        p, pt = self._ponto
        partes = []
        if ct > 0 and ativa:
            partes.append(f'ciclo {c} de {ct}')
        if pt > 0:
            partes.append(f'ponto {p} de {pt}')
        self.lbl_ponto.setText(' · '.join(partes) or '—')
        self.pausar_btn.setEnabled(ativa)
        self.pausar_btn.setText('Continuar' if self._pausado else 'Pausar')
        self.encerrar_btn.setEnabled(ativa)
        self.tara_btn.setEnabled(not ativa and not self._rodando)
        resp, self._tara_nova = self._tara_nova, None
        if resp is not None:
            self._mostrar_tara(resp)
        if buf:
            arr = np.array(buf)
            arr = arr[arr[:, 0] > agora - JANELA_GRAFICO_S]
            if arr.size:
                self.grafico.definir(arr[:, 0], arr[:, 1:].T, agora, JANELA_GRAFICO_S)

    # ================================================================ comandos

    def _pausar(self):
        if self._fase in FASES_ENCERRADAS:
            return
        novo = not self._pausado
        self._pub_pausa.publish(Bool(data=novo))
        self._pausado = novo
        self._msg('Coleta pausada: posição mantida.' if novo else 'Coleta retomada.',
                  'aviso' if novo else 'ok')

    def _fazer_tara(self):
        if self._fase not in FASES_ENCERRADAS or self._rodando:
            return
        self.tara_txt.setText('Fazendo a tara...')
        self.tara_txt.setStyleSheet(f'color: {COR["texto_fraco"]};')
        self._pub_tara.publish(Empty())
        self.janela.ros_node.pedir_tara()          # a leitura da aba Célula de Carga também
        QTimer.singleShot(4000, self._tara_sem_resposta)

    def _tara_sem_resposta(self):
        if self.tara_txt.text() == 'Fazendo a tara...':
            self.tara_txt.setText('A célula não respondeu à tara (o leitor está rodando?).')
            self.tara_txt.setStyleSheet(f'color: {COR["perigo"]};')

    def _mostrar_tara(self, resp):
        partes = resp.split(';')
        if partes[0] == 'ok':
            txt, cor = (f'Tara feita às {time.strftime("%H:%M:%S")}.', COR['sucesso'])
        elif len(partes) > 1 and partes[1] == 'drifting':
            txt, cor = ('Tara recusada: a célula estava se mexendo. Deixe a ponteira parada no '
                        'ar e tente de novo.', COR['perigo'])
        elif len(partes) > 1 and partes[1] == 'no_data':
            txt, cor = 'Tara recusada: sem leitura da célula.', COR['perigo']
        else:
            txt, cor = f'Tara recusada ({resp}).', COR['perigo']
        self.tara_txt.setText(txt)
        self.tara_txt.setStyleSheet(f'color: {cor};')

    def _encerrar(self):
        self._pub_parar.publish(String(data='stop'))
        self._msg('Encerrando: o explorador termina e volta à home.', 'aviso')

    def _payload(self):
        """Valores da tela, validados como no _on_start da palpation_gui.
        Devolve (payload, None) ou (None, motivo)."""
        modo = self.modo.valor()
        forca = self.forca.value()
        fmin, fmax = sorted((self.onda_min.value(), self.onda_max.value()))
        forma = self.onda.valor() if modo in FMOD_MODES else 'OFF'
        avisos = []
        if forma != 'OFF' and (fmax - fmin) / 2 <= 1e-3:
            avisos.append('Onda sem amplitude: coleta com força constante.')
            forma = 'OFF'
        passo = self.escada_passo.value() if modo in STAIRCASE_MODES else 0.0
        ini, pico = self.escada_inicio.value(), self.escada_pico.value()
        if passo > 0.0:
            if pico <= ini:
                avisos.append('Escada precisa de pico maior que o primeiro patamar: força única.')
                passo = 0.0
            elif not staircase_levels(ini, passo, pico):
                return None, f'A escada passaria de {STEP_MAX_LEVELS} patamares: aumente o degrau.'
        manual = self.angulo_manual.isChecked()
        if manual and modo not in ('TOUCH', 'MANUAL'):
            return None, 'O ângulo de ataque escolhido só vale nos modos Toque e Manual.'
        profundidade = self.profundidade.value()
        nos = []
        folga, transito = self.grade_folga.value(), self.grade_transito.value()
        if modo == 'MATRIX_MAP':
            nos, err = nos_da_grade(self._params_grade())
            if err or len(nos) < 2:
                return None, f'Grade inválida: {err}'
            if profundidade < folga * 1.5:
                profundidade = min(DEPTH_MAX, folga * 1.5)
                self.profundidade.setValue(profundidade)
                avisos.append(f'Curso da descida elevado para {profundidade:.1f} mm '
                              '(1,5 × a folga entre pontos).')
        home = self.home or self.janela.ultimas_juntas() or [0.0] * 6   # Mapa: não é usada
        home_urdf = [-home[0]] + [float(v) for v in home[1:]]
        p = {
            'speed_mms': self.velocidade.value(), 'depth_mm': profundidade, 'force_n': forca,
            'slide_dist_mm': self.distancia.value(),
            'approach_speed_mms': self.vel_descida.value(),
            'slide_dir': self.direcao.valor(),
            'repeats': self.repeticoes.value(),
            'speed_factor_pct': SPEED_FACTOR_PALPACAO,
            'hold_tol_n': self.tol_hold.value(), 'hold_stable_s': self.janela_hold.value(),
            'hold_timeout_s': self.limite_hold.value(),
            'slide_slope_deg': self.inclinacao.value(), 'mode': modo,
            'force_mod_shape': forma, 'force_mod_hz': self.onda_hz.value(),
            'force_mod_min_n': fmin, 'force_mod_max_n': fmax,
            'force_mod_cycles': self.onda_ciclos.value(),
            'step_size_n': passo, 'step_start_n': ini, 'step_max_n': pico,
            'step_dwell_s': self.escada_tempo.value(),
            'probe_align': 'MANUAL' if manual else ('ON' if self.calibrar.isChecked() else 'OFF'),
            'probe_align_manual_tilt_deg': self.man_incl.value() if manual else 0.0,
            'probe_align_manual_azimuth_deg': self.man_az.value() if manual else 0.0,
            'probe_align_points': self.cal_pontos.value(),
            'probe_align_radius_mm': self.cal_raio.value(),
            'probe_align_force_n': self.man_forca.value() if manual else self.cal_forca.value(),
            'probe_align_retract_mm': (self.man_folga.value() if manual
                                       else self.cal_recuo.value()),
            'probe_align_tilt_max_deg': self.cal_incl.value(),
            'home_deg': home_urdf,
            'matrix_waypoints_mm': nos[1:], 'matrix_safe_z_mm': folga,
            'matrix_transit_mms': transito, 'matrix_shape': self.grade_formato.valor(),
        }
        return (p, avisos), None

    def _iniciar(self):
        if self._rodando or time.monotonic() - self._inicio_publicado < START_REPUBLISH_LOCKOUT_S:
            self._msg('Início já enviado: esperando o explorador.', 'aviso')
            return
        if self._fase not in FASES_ENCERRADAS:
            self._msg(f'O explorador ainda está em {self._fase}: encerre antes.', 'aviso')
            return
        if self._relogio is None:
            self._msg('O gêmeo virtual não está rodando (sem /clock): a coleta move o gêmeo e o '
                      'robô segue. Abra com o gêmeo.', 'erro')
            return
        if self.modo.valor() != 'MATRIX_MAP' and self.home is None:
            self._msg('Grave a Home das Coletas (cartão no alto da aba) antes de iniciar.', 'erro')
            return
        res, erro = self._payload()
        if erro:
            self._msg(erro, 'erro')
            return
        payload, avisos = res
        self._msg(' '.join(avisos), 'aviso' if avisos else '')
        self._rodando = True
        nome = dict(MODOS)[payload['mode']]
        self.janela.iniciar_acao(f'Coleta: {nome}', f'Coletas: iniciando {nome} '
                                 f'(força {payload["force_n"]:.1f} N).')
        threading.Thread(target=self._executar, args=(payload,), daemon=True).start()

    def _executar(self, p):
        j = self.janela
        try:
            ok, msg = self._executar_passos(p)
        except Exception as e:                 # nunca deixa o painel preso
            ok, msg = False, f'Erro na coleta: {e}'
        self._rodando = False
        j.terminar_acao(ok, msg)

    def _executar_passos(self, p):
        j = self.janela
        # 1) Pré-home: o robô vai à home por um movimento normal (conferido pela
        #    segurança). O Mapa em grade parte de onde a ponteira está.
        if p['mode'] != 'MATRIX_MAP':
            j.progresso.emit({'detalhe': 'Levando o robô à home da coleta...'})
            ok, msg = j.mover_e_esperar(list(self.home))
            if not ok:
                return False, f'Coleta não iniciada: o robô não chegou à home ({msg}).'
        if j.parado():
            return False, 'Coleta parada antes de começar.'
        # 2) Tara (só se ainda não foi feita), com a ponteira no ar.
        if not self._tarado:
            j.progresso.emit({'detalhe': 'Fazendo a tara da célula...'})
            self._tara_evento.clear()
            self._pub_tara.publish(Empty())
            if not self._tara_evento.wait(4.0) or not str(self._tara_resposta).startswith('ok'):
                return False, ('Coleta não iniciada: a tara da célula falhou '
                               f'({self._tara_resposta or "sem resposta"}).')
        # 3) Arma no servidor: ele confere que o robô real e o gêmeo estão juntos.
        j.progresso.emit({'detalhe': 'Conferindo o robô e o gêmeo...'})
        fim = time.monotonic() + 6.0
        while True:
            ok, msg = self._chamar_coleta(True)
            if ok or 'diferentes' not in msg or time.monotonic() > fim or j.parado():
                break
            j.esperar(0.5)                     # o gêmeo ainda está alcançando o robô
        if not ok:
            return False, f'Coleta não iniciada: {msg}'
        # 4) Comando de início, carimbado com o relógio do gêmeo (o do explorador).
        msg = PalpationStart()
        msg.speed_mms = float(p['speed_mms'])
        msg.depth_mm = float(p['depth_mm'])
        msg.force_n = float(p['force_n'])
        msg.slide_dist_mm = float(p['slide_dist_mm'])
        msg.approach_speed_mms = float(p['approach_speed_mms'])
        msg.slide_dir = str(p['slide_dir'])
        msg.repeats = int(p['repeats'])
        msg.speed_factor_pct = float(p['speed_factor_pct'])
        msg.home_deg = [float(v) for v in p['home_deg']]
        msg.hold_tol_n = float(p['hold_tol_n'])
        msg.hold_stable_s = float(p['hold_stable_s'])
        msg.hold_timeout_s = float(p['hold_timeout_s'])
        msg.slide_slope_deg = float(p['slide_slope_deg'])
        msg.mode = str(p['mode'])
        msg.force_mod_shape = str(p['force_mod_shape'])
        msg.force_mod_hz = float(p['force_mod_hz'])
        msg.force_mod_min_n = float(p['force_mod_min_n'])
        msg.force_mod_max_n = float(p['force_mod_max_n'])
        msg.force_mod_cycles = int(p['force_mod_cycles'])
        msg.step_size_n = float(p['step_size_n'])
        msg.step_start_n = float(p['step_start_n'])
        msg.step_max_n = float(p['step_max_n'])
        msg.step_dwell_s = float(p['step_dwell_s'])
        msg.probe_align = str(p['probe_align'])
        msg.probe_align_points = int(p['probe_align_points'])
        msg.probe_align_radius_mm = float(p['probe_align_radius_mm'])
        msg.probe_align_force_n = float(p['probe_align_force_n'])
        msg.probe_align_retract_mm = float(p['probe_align_retract_mm'])
        msg.probe_align_tilt_max_deg = float(p['probe_align_tilt_max_deg'])
        msg.probe_align_manual_tilt_deg = float(p['probe_align_manual_tilt_deg'])
        msg.probe_align_manual_azimuth_deg = float(p['probe_align_manual_azimuth_deg'])
        msg.waypoints = [Point(x=float(x) / 1000.0, y=float(y) / 1000.0, z=0.0)
                         for x, y in p['matrix_waypoints_mm']]
        msg.safe_z_mm = float(p['matrix_safe_z_mm'])
        msg.transit_speed_mms = float(p['matrix_transit_mms'])
        msg.grid_shape = str(p['matrix_shape'])
        msg.run_id = new_run_id()
        relogio = self._relogio
        msg.stamp = TempoRos(sec=relogio.sec, nanosec=relogio.nanosec)
        self._aviso_explorador = ''
        self._pub_inicio.publish(msg)
        self._inicio_publicado = time.monotonic()
        j.log(f'Coletas: início publicado ({msg.mode}, run {msg.run_id}).')

        # 5) Acompanha até o fim.
        t0 = time.monotonic()
        viu_ativa = False
        while True:
            if j.parado():
                return False, 'Coleta parada (o explorador foi congelado e o robô parou).'
            fase = self._fase
            if fase not in FASES_ENCERRADAS:
                viu_ativa = True
                c, ct = self._ciclo
                pt, ptt = self._ponto
                extra = (f' · ciclo {c}/{ct}' if ct > 1 else '') + \
                        (f' · ponto {pt}/{ptt}' if ptt else '')
                j.progresso.emit({'detalhe': f'{NOME_FASE.get(fase, fase)}{extra}'})
            elif viu_ativa:
                break
            elif time.monotonic() - t0 > ESPERA_EXPLORADOR_S:
                self._chamar_coleta(False)
                return False, ('O explorador não começou a coleta (veja as linhas do '
                               'tactile_explorer no terminal).')
            j.esperar(0.2)
        motivo = self._estado_srv.get('mensagem', '')
        if self._node.modo == 'real':          # no gêmeo nada é gravado
            threading.Thread(target=self._salvar_grafico_robo, args=(msg, fase),
                             daemon=True).start()
        if fase == 'DONE':
            return True, f'Coleta concluída (run {msg.run_id}).'
        if fase == 'FROZEN':
            return False, f'Coleta congelada. {motivo}'
        aviso = self._aviso_explorador
        return False, ('Coleta encerrada antes do fim (ABORTED). '
                       + (f'Explorador: {aviso}' if aviso else
                          'Pelo botão Encerrar, ou veja as linhas do tactile_explorer no terminal.'))

    def _salvar_grafico_robo(self, msg, fase):
        """Espera o registrador fechar o samples.csv da coleta e salva o gráfico
        do robô ao lado (grafico_robo.png). Coleta de aprendizado não tem dados."""
        from ..grafico_coleta import gerar_grafico_robo
        fim = time.monotonic() + 30.0
        pasta, tamanho, estavel = None, -1, 0
        while time.monotonic() < fim:
            achadas = glob.glob(os.path.join(PASTA_COLETAS, '*', msg.mode, msg.run_id))
            if achadas:
                pasta = achadas[0]
                csv_path = os.path.join(pasta, 'samples.csv')
                t = os.path.getsize(csv_path) if os.path.isfile(csv_path) else -1
                estavel = estavel + 1 if t == tamanho and t > 0 else 0
                tamanho = t
                if estavel >= 4:               # 2 s sem crescer: o run fechou
                    break
            time.sleep(0.5)
        if pasta is None:
            return
        nome = dict(MODOS).get(msg.mode, msg.mode)
        titulo = (f'{nome} · força alvo {msg.force_n:.1f} N · run {msg.run_id}'
                  + ('' if fase == 'DONE' else f' · terminou em {fase}'))
        try:
            png = gerar_grafico_robo(pasta, titulo)
        except Exception as e:
            self.janela.log(f'Coletas: não consegui desenhar o gráfico do robô ({e}).', 'ERRO')
            return
        if png:
            self.janela.log(f'Coletas: gráfico do robô salvo em {png}.')

    # ============================================================ configuração

    _CAMPOS = ('forca', 'repeticoes', 'velocidade', 'distancia', 'onda_min', 'onda_max',
               'onda_hz', 'onda_ciclos', 'escada_passo', 'escada_inicio', 'escada_pico',
               'escada_tempo', 'grade_cols', 'grade_linhas', 'grade_px', 'grade_py',
               'grade_larg', 'grade_alt', 'grade_folga', 'grade_transito', 'profundidade',
               'vel_descida', 'tol_hold', 'janela_hold', 'limite_hold', 'inclinacao',
               'cal_pontos', 'cal_raio', 'cal_forca', 'cal_recuo', 'cal_incl', 'man_incl',
               'man_az', 'man_forca', 'man_folga')
    _SELETORES = ('modo', 'direcao', 'onda', 'grade_formato', 'grade_medida', 'grade_ordem')

    def salvar_config(self, dados):
        if not COLETAS_OK:
            return
        d = {c: getattr(self, c).value() for c in self._CAMPOS}
        d.update({s: getattr(self, s).valor() for s in self._SELETORES})
        d['calibrar'] = self.calibrar.isChecked()
        d['angulo_manual'] = self.angulo_manual.isChecked()
        dados['coletas'] = d

    def carregar_config(self, dados):
        if not COLETAS_OK:
            return
        d = dados.get('coletas') or {}
        for c in self._CAMPOS:
            if c in d:
                try:
                    getattr(self, c).setValue(d[c])
                except (TypeError, ValueError):
                    pass
        for s in self._SELETORES:
            if s in d:
                getattr(self, s).definir(d[s])
        self._atualizar_onda()
        self.calibrar.setChecked(bool(d.get('calibrar', False)))
        self.angulo_manual.setChecked(bool(d.get('angulo_manual', False)))
        # Configuração antiga: a home ficava aqui; passa a ser a Home das Coletas.
        home = d.get('home')
        if (isinstance(home, list) and len(home) == 6
                and self.janela.home_de('coletas') is None
                and any(abs(float(a) - b) > 0.05 for a, b in zip(home, HOME_JOINTS))):
            self.janela.homes['coletas'] = [float(v) for v in home]
        self._ao_mudar_modo(self.modo.valor())
