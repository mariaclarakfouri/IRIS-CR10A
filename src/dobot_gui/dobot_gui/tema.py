"""Tema visual da plataforma: cores, folha de estilo (QSS) e ícones.

Toda cor usada na interface vem de COR. Para mudar o visual, mude aqui.
"""
import os

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QIcon, QPainter, QPen, QPixmap, QPolygonF

COR = {
    # Paleta do visual de referência (ui.py): grafite, superfícies claras e um
    # único acento teal. Vermelho = emergência/erro; âmbar = aviso.
    'escuro_fundo': '#161b22',
    'escuro_painel': '#202731',
    'escuro_borda': '#3a434f',
    'escuro_texto': '#e2e6eb',
    'escuro_texto_fraco': '#aeb7c2',
    'claro_fundo': '#edf0f3',
    'cartao': '#ffffff',
    'borda': '#d3d9df',
    'borda_forte': '#b4bdc7',
    'borda_hover': '#8f9aa6',
    'linha_suave': '#eef1f4',
    'linha_alterna': '#f7f9fa',
    'trilho': '#e3e8ec',
    'texto': '#1b2128',
    'texto_medio': '#3f4852',
    'texto_fraco': '#5b6270',
    # ações e estados
    'primaria': '#2f6b73',
    'primaria_hover': '#255a61',
    'primaria_press': '#1d4d53',
    'primaria_suave': '#e3eeef',
    'sucesso': '#1e7a4f',
    'aviso': '#f59e0b',
    'aviso_suave': '#fff1cc',
    'aviso_texto': '#7a4a00',
    'parar': '#d86a16',
    'parar_hover': '#b9580f',
    'perigo': '#c0303f',
    'perigo_hover': '#a8283a',
    'perigo_suave': '#fbecef',
    'desabilitado_fundo': '#f4f6f8',
    'desabilitado_texto': '#98a1ac',
    # faixa de modo (única indicação da fonte dos dados)
    # Simulação em âmbar (cor de aviso): não dá para confundir o gêmeo com o robô real.
    'faixa_sim': '#f59e0b',
    'faixa_sim_texto': '#1f2937',
    'faixa_real': '#0f4c5c',
}

FONTE_SANS = ("'IBM Plex Sans', 'Source Sans 3', 'Source Sans Pro', 'Inter', "
              "'Segoe UI', 'Noto Sans', 'Ubuntu', 'DejaVu Sans', sans-serif")
FONTE_MONO = ("'IBM Plex Mono', 'JetBrains Mono', 'DejaVu Sans Mono', "
              "'Ubuntu Mono', 'Noto Sans Mono', monospace")

try:
    from ament_index_python.packages import get_package_share_directory
    _ICONS_DIR = get_package_share_directory('dobot_gui') + '/icons'
except Exception:  # noqa: BLE001
    _ICONS_DIR = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'icons')


_QSS = """
QWidget {
    color: %(texto)s;
    font-family: %(fonte_sans)s;
    font-size: 13px;
}
QLabel, QSlider, QCheckBox { background: transparent; }
QToolTip {
    background-color: %(escuro_fundo)s; color: #ffffff;
    border: none; padding: 5px 7px; font-size: 12px;
}

/* ===== Cabeçalho: identidade, estado e segurança ===== */
QFrame#cabecalho { background-color: %(escuro_fundo)s; }
QLabel#appTitulo { color: #ffffff; font-size: 16px; font-weight: 700; }
QLabel#appSubtitulo { color: #c9cfd8; font-size: 12px; }
QPushButton#headerBotao, QPushButton#headerConectar, QPushButton#headerAlarme {
    background-color: transparent; color: #ffffff;
    border: 1px solid #5e6978; border-radius: 3px;
    padding: 6px 12px; font-size: 13px; font-weight: 600; min-height: 20px;
}
QPushButton#headerConectar { min-width: 96px; }
QPushButton#headerBotao:hover, QPushButton#headerConectar:hover,
QPushButton#headerAlarme:hover { background-color: #262e38; border-color: #8895a6; }
QPushButton#headerBotao:pressed, QPushButton#headerConectar:pressed,
QPushButton#headerAlarme:pressed { background-color: #202731; }
QPushButton#headerBotao:disabled, QPushButton#headerConectar:disabled,
QPushButton#headerAlarme:disabled {
    background-color: transparent; color: #7d8794; border-color: #3a434f;
}
QPushButton#headerConectar[connected="true"],
QPushButton#headerConectar[connected="true"]:disabled {
    background-color: %(primaria)s; color: #ffffff; border-color: %(primaria)s;
}
QPushButton#emergencia {
    background-color: %(perigo)s; color: #ffffff; border: 1px solid #e8707c;
    border-radius: 3px; padding: 8px 16px; font-size: 13px; font-weight: 800;
    letter-spacing: 1px; min-height: 22px;
}
QPushButton#emergencia:hover { background-color: %(perigo_hover)s; }
QPushButton#emergencia:pressed { background-color: #8f2330; }

/* ===== Chips de estado ===== */
QLabel#chip {
    border-radius: 2px; padding: 2px 8px; font-size: 11px; font-weight: 700;
}

/* ===== Faixa de modo: única indicação da fonte dos dados ===== */
QLabel#faixaModo { padding: 0 24px; font-size: 12px; font-weight: 700; }
QLabel#faixaModo[modo="sim"] { background-color: %(faixa_sim)s; color: %(faixa_sim_texto)s; }
QLabel#faixaModo[modo="real"] { background-color: %(faixa_real)s; color: #ffffff; }

/* ===== Barra do robô (destino, velocidade, home) ===== */
QFrame#barraRobo { background-color: %(cartao)s; border-bottom: 1px solid %(borda)s; }
QLabel#robotIp {
    font-family: %(fonte_mono)s; font-size: 12px; font-weight: 600; color: %(texto)s;
}
QLabel#ladoRotulo { color: %(texto_fraco)s; font-size: 12px; font-weight: 600; }
QLabel#ladoValor {
    color: %(texto)s; font-family: %(fonte_mono)s; font-size: 13px; font-weight: 700;
}
QLabel#ladoDica { color: %(texto_fraco)s; font-size: 12px; }

/* ===== Navegação superior ===== */
QFrame#faixaNav { background-color: %(cartao)s; border-bottom: 1px solid %(borda)s; }
QPushButton#navItem {
    background-color: transparent; color: %(texto_fraco)s; border: none;
    border-bottom: 3px solid transparent; border-radius: 0px;
    padding: 0 16px; font-size: 13px; font-weight: 600;
}
QPushButton#navItem:hover { background-color: transparent; color: %(texto)s; }
QPushButton#navItem:pressed { background-color: transparent; }
QPushButton#navItem:checked {
    background-color: transparent; color: %(texto)s;
    border-bottom: 3px solid %(primaria)s;
}

/* ===== Conteúdo ===== */
QWidget#conteudo, QScrollArea#rolagem, QWidget#pagina { background-color: %(claro_fundo)s; }
QScrollArea#rolagem { border: none; }
QLabel#paginaTitulo { font-size: 21px; font-weight: 600; color: %(texto)s; }
QLabel#paginaDescricao { font-size: 13px; color: %(texto_fraco)s; }

QFrame#cartao {
    background-color: %(cartao)s; border: 1px solid %(borda)s; border-radius: 0px;
}
QLabel#cartaoTitulo { font-size: 14px; font-weight: 600; color: %(texto)s; }
QLabel#cartaoTexto { font-size: 12px; color: %(texto_fraco)s; }
QLabel#rotulo { font-size: 13px; font-weight: 600; color: %(texto_medio)s; }
QLabel#valorMono {
    font-family: %(fonte_mono)s; font-size: 13px; color: %(texto)s;
}

/* Alerta em linha (mensagens de estado) */
QLabel#alerta {
    border-radius: 0px; padding: 8px 12px; font-size: 12px;
    background-color: %(primaria_suave)s; color: %(primaria_press)s;
    border: 1px solid #b9d3d6; border-left: 4px solid %(primaria)s;
}
QLabel#alerta[tipo="erro"] {
    background-color: %(perigo_suave)s; color: #8f2330;
    border: 1px solid #e3a6ad; border-left: 4px solid %(perigo)s;
}
QLabel#alerta[tipo="aviso"] {
    background-color: %(aviso_suave)s; color: %(aviso_texto)s;
    border: 1px solid #e6d39a; border-left: 4px solid %(aviso)s;
}
QLabel#alerta[tipo="ok"] {
    background-color: #e8f3ed; color: #155c3b;
    border: 1px solid #b7d8c6; border-left: 4px solid %(sucesso)s;
}

/* Painel de atividade (ensaio em andamento) */
QFrame#atividade {
    background-color: %(cartao)s; border: 1px solid %(borda)s;
    border-left: 4px solid %(primaria)s; border-radius: 0px;
}
QLabel#atividadeTitulo { font-size: 14px; font-weight: 600; color: %(texto)s; }
QLabel#atividadeDetalhe { font-size: 12px; color: %(texto_fraco)s; }
QLabel#atividadeTempo {
    font-family: %(fonte_mono)s; font-size: 13px; font-weight: 700; color: %(primaria)s;
}
QProgressBar {
    background-color: %(trilho)s; border: none; border-radius: 0px;
    min-height: 6px; max-height: 6px;
}
QProgressBar::chunk { background-color: %(primaria)s; border-radius: 0px; }

/* ===== Botões: primário = preenchido; secundário = com borda ===== */
QPushButton {
    background-color: %(primaria)s; color: #ffffff; border: 1px solid %(primaria)s;
    border-radius: 3px; padding: 6px 12px; font-size: 13px; font-weight: 600;
    min-height: 20px;
}
QPushButton:hover { background-color: %(primaria_hover)s; border-color: %(primaria_hover)s; }
QPushButton:pressed { background-color: %(primaria_press)s; border-color: %(primaria_press)s; }
QPushButton:disabled {
    background-color: %(desabilitado_fundo)s; color: %(desabilitado_texto)s;
    border-color: %(borda)s;
}
QPushButton#secundario {
    background-color: %(cartao)s; color: %(texto)s; border: 1px solid %(borda_forte)s;
    font-weight: 500;
}
QPushButton#secundario:hover {
    background-color: %(linha_alterna)s; border-color: %(borda_hover)s;
}
QPushButton#secundario:pressed { background-color: %(primaria_suave)s; }
QPushButton#secundario:disabled {
    background-color: %(desabilitado_fundo)s; color: %(desabilitado_texto)s;
    border-color: %(borda)s;
}
QPushButton#discreto {
    background-color: transparent; color: %(texto_medio)s; border: 1px solid %(borda_forte)s;
    border-radius: 3px; padding: 6px 12px; font-weight: 500;
}
QPushButton#discreto:hover { background-color: %(claro_fundo)s; border-color: %(borda_hover)s; }
QPushButton#parar {
    background-color: %(parar)s; color: #ffffff; border: 1px solid %(parar)s;
    padding: 8px 16px; font-weight: 700;
}
QPushButton#parar:hover { background-color: %(parar_hover)s; border-color: %(parar_hover)s; }
QPushButton#parar:disabled {
    background-color: %(desabilitado_fundo)s; color: %(desabilitado_texto)s;
    border-color: %(borda)s;
}
QPushButton#alternar:checked { background-color: %(perigo)s; border-color: %(perigo)s; }
/* Seletor de opções lado a lado */
QPushButton#segmento {
    background-color: %(cartao)s; color: %(texto)s; border: 1px solid %(borda_forte)s;
    border-radius: 0px; padding: 6px 12px; font-weight: 500;
}
QPushButton#segmento:hover { background-color: %(linha_alterna)s; }
QPushButton#segmento:checked {
    background-color: %(primaria)s; color: #ffffff; border-color: %(primaria)s;
}
QPushButton#segmento:disabled { color: %(desabilitado_texto)s; }

/* ===== Sub-abas dentro de um módulo ===== */
QTabWidget#subAbas::pane { border: none; top: 0px; }
QTabWidget#subAbas QTabBar::tab {
    background-color: transparent; color: %(texto_fraco)s; border: none;
    border-bottom: 3px solid transparent; padding: 9px 16px 7px 16px; margin-right: 0px;
    font-size: 13px; font-weight: 600;
}
QTabWidget#subAbas QTabBar::tab:hover { color: %(texto)s; }
QTabWidget#subAbas QTabBar::tab:selected {
    color: %(texto)s; border-bottom: 3px solid %(primaria)s;
}

/* ===== Tabelas ===== */
QTableWidget {
    background-color: #ffffff; alternate-background-color: %(linha_alterna)s;
    border: 1px solid %(borda)s; border-radius: 0px; gridline-color: transparent;
    outline: 0; font-size: 12px;
    selection-background-color: %(primaria_suave)s; selection-color: %(texto)s;
}
QTableWidget::item { padding: 0 8px; border-bottom: 1px solid %(linha_suave)s; }
QTableWidget::item:selected { background-color: %(primaria_suave)s; color: %(texto)s; }
QHeaderView::section {
    background-color: %(cartao)s; color: %(texto_fraco)s; border: none;
    border-bottom: 1px solid %(borda_forte)s; padding: 6px 8px;
    font-weight: 600; font-size: 12px;
}
QTableCornerButton::section { background-color: %(cartao)s; border: none; }
QLabel#statValor {
    font-family: %(fonte_mono)s; font-size: 18px; font-weight: 600; color: %(texto)s;
}
QLabel#statRotulo { font-size: 11px; color: %(texto_fraco)s; font-weight: 600; }

/* ===== Rodapé ===== */
QFrame#rodape { background-color: %(cartao)s; border-top: 1px solid %(borda)s; }
QLabel#rodapeTexto { font-size: 12px; color: %(texto_fraco)s; }
QLabel#rodapeMono {
    font-family: %(fonte_mono)s; font-size: 12px; color: %(texto_medio)s;
}
QFrame#rodapeSeparador { background-color: %(borda)s; }

/* ===== Campos ===== */
QSlider::groove:horizontal { height: 4px; background: %(borda)s; border-radius: 2px; }
QSlider::sub-page:horizontal { background: %(primaria)s; border-radius: 2px; }
QSlider::add-page:horizontal { background: %(borda)s; border-radius: 2px; }
QSlider::handle:horizontal {
    background: #ffffff; border: 2px solid %(primaria)s; width: 12px; height: 12px;
    margin: -6px 0; border-radius: 8px;
}
QSlider::sub-page:horizontal:disabled { background: %(borda_forte)s; }
QSlider::handle:horizontal:disabled { border-color: %(borda_forte)s; }

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background-color: #ffffff; color: %(texto)s; border: 1px solid %(borda_forte)s;
    border-radius: 3px; padding: 5px 9px; min-height: 20px; font-size: 13px;
    selection-background-color: %(primaria)s; selection-color: #ffffff;
}
QSpinBox, QDoubleSpinBox, QComboBox { padding: 5px 22px 5px 9px; min-width: 64px; }
QLineEdit:hover, QSpinBox:hover, QDoubleSpinBox:hover, QComboBox:hover {
    border-color: %(borda_hover)s;
}
QLineEdit:focus { border: 2px solid %(primaria)s; padding: 4px 8px; }
QSpinBox:focus, QDoubleSpinBox:focus, QComboBox:focus {
    border: 2px solid %(primaria)s; padding: 4px 21px 4px 8px;
}
QLineEdit:disabled, QSpinBox:disabled, QDoubleSpinBox:disabled, QComboBox:disabled {
    color: %(desabilitado_texto)s; background-color: %(desabilitado_fundo)s;
    border-color: %(borda)s;
}
QSpinBox::up-button, QSpinBox::down-button,
QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {
    subcontrol-origin: border; width: 18px; border-left: 1px solid %(borda_forte)s;
    background-color: %(claro_fundo)s;
}
QSpinBox::up-button, QDoubleSpinBox::up-button {
    subcontrol-position: top right; border-top-right-radius: 3px;
}
QSpinBox::down-button, QDoubleSpinBox::down-button {
    subcontrol-position: bottom right; border-bottom-right-radius: 3px;
}
QSpinBox::up-button:hover, QSpinBox::down-button:hover,
QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover {
    background-color: %(borda)s;
}
QSpinBox::up-arrow, QDoubleSpinBox::up-arrow { image: url(%%UP_ARROW%%); width: 8px; height: 8px; }
QSpinBox::down-arrow, QDoubleSpinBox::down-arrow { image: url(%%DOWN_ARROW%%); width: 8px; height: 8px; }
QComboBox::drop-down { border: none; width: 22px; }
QComboBox::down-arrow { image: url(%%DOWN_ARROW%%); width: 8px; height: 8px; }
QComboBox QAbstractItemView {
    background-color: #ffffff; color: %(texto)s; border: 1px solid %(borda_forte)s;
    selection-background-color: %(primaria_suave)s; selection-color: %(texto)s; outline: 0;
}

QCheckBox { spacing: 8px; }
QCheckBox::indicator {
    width: 14px; height: 14px; border: 1px solid %(borda_hover)s;
    border-radius: 2px; background: #ffffff;
}
QCheckBox::indicator:checked {
    background: %(primaria)s; border-color: %(primaria)s; image: url(%%CHECK%%);
}
QCheckBox::indicator:disabled { background: %(desabilitado_fundo)s; border-color: %(borda)s; }

QPlainTextEdit#logView {
    background-color: #ffffff; border: 1px solid %(borda)s; border-radius: 0px;
    padding: 8px; font-family: %(fonte_mono)s;
    font-size: 12px; color: %(texto)s;
}

/* ===== Caixas de diálogo ===== */
QMessageBox { background-color: %(cartao)s; }
QMessageBox QLabel { color: %(texto)s; font-size: 13px; }
QMessageBox QPushButton { min-width: 90px; }

/* ===== Barras de rolagem ===== */
QScrollBar:vertical { background: transparent; width: 8px; margin: 0; }
QScrollBar::handle:vertical { background: #c3cad1; border-radius: 4px; min-height: 24px; }
QScrollBar::handle:vertical:hover { background: %(borda_hover)s; }
QScrollBar:horizontal { background: transparent; height: 8px; margin: 0; }
QScrollBar::handle:horizontal { background: #c3cad1; border-radius: 4px; min-width: 24px; }
QScrollBar::handle:horizontal:hover { background: %(borda_hover)s; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0px; width: 0px; }
QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }
"""

_TOKENS = dict(COR, fonte_sans=FONTE_SANS, fonte_mono=FONTE_MONO)

# Na formatação com %, "%%UP_ARROW%%" vira "%UP_ARROW%".
STYLE_SHEET = (
    (_QSS % _TOKENS)
    .replace('%UP_ARROW%', os.path.join(_ICONS_DIR, 'up_arrow.png'))
    .replace('%DOWN_ARROW%', os.path.join(_ICONS_DIR, 'down_arrow.png'))
    .replace('%CHECK%', os.path.join(_ICONS_DIR, 'check.png'))
)


def make_icon(kind, color='#ffffff', size=16):
    """Ícones de traço fino desenhados em código (sem depender de arquivos)."""
    scale = 2
    pixmap = QPixmap(size * scale, size * scale)
    pixmap.setDevicePixelRatio(scale)
    pixmap.fill(Qt.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.Antialiasing)
    k = size / 16.0
    painter.scale(k, k)
    pen = QPen(QColor(color))
    pen.setWidthF(1.6)
    pen.setCapStyle(Qt.RoundCap)
    pen.setJoinStyle(Qt.RoundJoin)
    painter.setPen(pen)
    painter.setBrush(Qt.NoBrush)

    def poly(*points):
        return QPolygonF([QPointF(x, y) for x, y in points])

    def linha(x1, y1, x2, y2):
        painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

    if kind == 'power':
        painter.drawArc(QRectF(3, 3.5, 10, 10), 120 * 16, 300 * 16)
        linha(8, 1.5, 8, 7)
    elif kind == 'x':
        linha(4, 4, 12, 12)
        linha(12, 4, 4, 12)
    elif kind == 'home':
        painter.drawPolyline(poly((2, 8), (8, 2.5), (14, 8)))
        painter.drawPolyline(poly((4, 7), (4, 13.5), (12, 13.5), (12, 7)))
    elif kind == 'warning':
        painter.drawPolygon(poly((8, 2), (14.5, 13.5), (1.5, 13.5)))
        linha(8, 6.5, 8, 9.5)
        painter.drawPoint(QPointF(8, 11.5))
    elif kind == 'save':
        painter.drawRoundedRect(QRectF(2.5, 2.5, 11, 11), 1.5, 1.5)
        painter.drawPolyline(poly((5.5, 2.5), (5.5, 6), (10.5, 6), (10.5, 2.5)))
        painter.drawRect(QRectF(5, 9, 6, 4.5))
    elif kind == 'play':
        painter.setBrush(QColor(color))
        painter.drawPolygon(poly((5, 3), (13, 8), (5, 13)))
    elif kind == 'stop':
        painter.setBrush(QColor(color))
        painter.drawRoundedRect(QRectF(4, 4, 8, 8), 1.2, 1.2)
    elif kind == 'octagon':
        painter.drawPolygon(poly((5, 1.5), (11, 1.5), (14.5, 5), (14.5, 11),
                                 (11, 14.5), (5, 14.5), (1.5, 11), (1.5, 5)))
        painter.setBrush(QColor(color))
        painter.drawRect(QRectF(6, 6, 4, 4))
    elif kind == 'sliders':   # Controle de juntas
        for y, x in ((4, 10), (8, 5), (12, 9)):
            linha(2, y, 14, y)
            painter.setBrush(QColor(color))
            painter.drawEllipse(QPointF(x, y), 1.8, 1.8)
            painter.setBrush(Qt.NoBrush)
    elif kind == 'rotate':    # Rotação inercial
        painter.drawArc(QRectF(2.5, 2.5, 11, 11), 40 * 16, 290 * 16)
        painter.drawPolyline(poly((10.5, 1.5), (12.8, 4.2), (9.6, 5.4)))
    elif kind == 'cube':      # Robô 3D
        painter.drawPolygon(poly((8, 1.8), (14, 5), (8, 8.2), (2, 5)))
        painter.drawPolyline(poly((2, 5), (2, 11), (8, 14.2), (14, 11), (14, 5)))
        linha(8, 8.2, 8, 14.2)
    elif kind == 'list':      # Registro
        for y in (4, 8, 12):
            painter.drawPoint(QPointF(2.8, y))
            linha(5.5, y, 13.5, y)
    elif kind == 'chart':     # Análise de Juntas
        painter.drawPolyline(poly((2, 2.5), (2, 13.5), (14, 13.5)))
        painter.drawPolyline(poly((4, 10.5), (7, 7), (9.5, 9), (13.5, 4)))
    elif kind == 'nivel':     # Alinhamento da mesa: nível de bolha
        painter.drawRoundedRect(QRectF(1.5, 5, 13, 6), 3, 3)
        linha(6, 5, 6, 11)
        linha(10, 5, 10, 11)
        painter.setBrush(QColor(color))
        painter.drawEllipse(QPointF(8, 8), 1.4, 1.4)
    elif kind == 'grid':      # Sensores táteis
        painter.drawRoundedRect(QRectF(2, 2, 12, 12), 1.5, 1.5)
        for x in (6, 10):
            linha(x, 2, x, 14)
            linha(2, x, 14, x)
    elif kind == 'record':
        painter.setBrush(QColor(color))
        painter.drawEllipse(QPointF(8, 8), 4.5, 4.5)
    elif kind == 'pause':
        painter.setBrush(QColor(color))
        painter.drawRect(QRectF(4, 3.5, 2.6, 9))
        painter.drawRect(QRectF(9.4, 3.5, 2.6, 9))
    elif kind == 'shield':
        painter.drawPolygon(poly((8, 1.8), (13.5, 4), (13, 9), (8, 14.2), (3, 9), (2.5, 4)))
        painter.drawPolyline(poly((5.5, 8), (7.4, 10), (10.8, 6.2)))
    elif kind == 'corte':     # Campo do Experimento: cantos de recorte
        painter.drawPolyline(poly((4.5, 1.5), (4.5, 11.5), (14.5, 11.5)))
        painter.drawPolyline(poly((1.5, 4.5), (11.5, 4.5), (11.5, 14.5)))
    elif kind == 'tool':      # Ferramentas: chave de boca
        linha(3, 13, 9.2, 6.8)
        painter.drawArc(QRectF(7.5, 2, 7, 7), -135 * 16, 270 * 16)
    elif kind == 'robot':     # ícone do aplicativo: braço articulado
        pen.setWidthF(2.0)
        painter.setPen(pen)
        linha(3, 14, 13, 14)
        painter.drawRoundedRect(QRectF(4.5, 11, 5, 3), 1, 1)
        linha(7, 11, 5, 6)
        linha(5, 6, 11, 3.5)
        painter.setBrush(QColor(color))
        for x, y in ((7, 11), (5, 6), (11, 3.5)):
            painter.drawEllipse(QPointF(x, y), 1.4, 1.4)
        linha(11, 3.5, 13.5, 6)
    painter.end()
    return QIcon(pixmap)


def app_icon():
    """Ícone da janela: braço robótico branco sobre um quadrado azul."""
    size = 64
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.transparent)
    painter = QPainter(pixmap)
    painter.setRenderHint(QPainter.Antialiasing)
    painter.setBrush(QColor(COR['primaria']))
    painter.setPen(Qt.NoPen)
    painter.drawRoundedRect(QRectF(2, 2, 60, 60), 14, 14)
    painter.drawPixmap(10, 10, make_icon('robot', '#ffffff', 44).pixmap(44, 44))
    painter.end()
    return QIcon(pixmap)