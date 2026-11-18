"""Peças visuais reutilizáveis da interface (usadas pela base e pelos módulos)."""
import time

from PyQt5.QtCore import QSize, Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QFrame,
    QHBoxLayout,
    QLabel,
    QProgressBar,
    QPushButton,
    QSlider,
    QSpinBox,
    QVBoxLayout,
)

from .tema import COR, make_icon


class JointSpinBox(QSpinBox):
    """QSpinBox que avisa quando o valor muda pelas setinhas (stepBy)."""

    stepped = pyqtSignal()

    def stepBy(self, steps):
        super().stepBy(steps)
        self.stepped.emit()


def sincronizar(slider, spin):
    """Liga uma barra a um campo numérico: mexer em um atualiza o outro."""
    def _spin_de_slider(valor):
        spin.blockSignals(True)
        spin.setValue(valor)
        spin.blockSignals(False)

    def _slider_de_spin(valor):
        slider.blockSignals(True)
        slider.setValue(valor)
        slider.blockSignals(False)

    slider.valueChanged.connect(_spin_de_slider)
    spin.valueChanged.connect(_slider_de_spin)


def barra_e_campo(minimo, maximo, valor, sufixo='', spin_cls=QSpinBox):
    """Cria (barra, campo) já sincronizados."""
    slider = QSlider(Qt.Horizontal)
    slider.setRange(minimo, maximo)
    slider.setValue(valor)
    spin = spin_cls()
    spin.setRange(minimo, maximo)
    spin.setValue(valor)
    if sufixo:
        spin.setSuffix(sufixo)
    sincronizar(slider, spin)
    return slider, spin


def rotulo(texto, nome='rotulo'):
    lbl = QLabel(texto)
    lbl.setObjectName(nome)
    return lbl


def com_icone(botao, tipo, cor='#ffffff', tamanho=16):
    botao.setIcon(make_icon(tipo, cor))
    botao.setIconSize(QSize(tamanho, tamanho))
    return botao


class Cartao(QFrame):
    """Bloco branco com título e texto opcionais; o conteúdo vai em `corpo`."""

    def __init__(self, titulo=None, texto=None, parent=None):
        super().__init__(parent)
        self.setObjectName('cartao')
        lay = QVBoxLayout(self)
        lay.setContentsMargins(20, 16, 20, 16)
        lay.setSpacing(10)
        if titulo:
            lay.addWidget(rotulo(titulo, 'cartaoTitulo'))
        if texto:
            t = rotulo(texto, 'cartaoTexto')
            t.setWordWrap(True)
            lay.addWidget(t)
        self.corpo = lay


class Chip(QLabel):
    """Etiqueta arredondada de estado (ex.: REAL, SIMULAÇÃO, Conectado)."""

    # Espaço interno do selo: padding de 8 px de cada lado no tema (+ folga no
    # tamanho pedido, porque o Qt só considera que o texto cabe com sobra).
    _PADDING = 16
    _FOLGA = 24

    def __init__(self, texto='', parent=None):
        super().__init__(texto, parent)
        self.setObjectName('chip')
        self.setFixedHeight(20)
        self.setAlignment(Qt.AlignCenter)
        self._completo = texto

    def definir(self, texto, fundo, cor_texto='#ffffff'):
        self._completo = texto
        self.setToolTip(texto)
        self.setStyleSheet(f'background-color: {fundo}; color: {cor_texto};')
        self._encaixar()
        self.updateGeometry()

    # Em janela estreita o selo encurta com "…" (o texto inteiro fica na dica),
    # em vez de cortar o texto ou espremer os botões ao lado.
    def sizeHint(self):
        return QSize(self.fontMetrics().horizontalAdvance(self._completo) + self._FOLGA, 20)

    def minimumSizeHint(self):
        return QSize(self.fontMetrics().horizontalAdvance('Ab…') + self._FOLGA, 20)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._encaixar()

    def _encaixar(self):
        cabe = max(0, self.width() - self._PADDING)
        self.setText(self.fontMetrics().elidedText(self._completo, Qt.ElideRight, cabe)
                     if self.width() > 0 else self._completo)


class Alerta(QLabel):
    """Mensagem em linha no topo do conteúdo. Tipos: info, ok, aviso, erro."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('alerta')
        self.setWordWrap(True)
        self.hide()

    def mostrar(self, texto, tipo='info'):
        self.setText(texto)
        self.setProperty('tipo', tipo)
        self.style().unpolish(self)
        self.style().polish(self)
        self.show()

    def esconder(self):
        self.setText('')
        self.hide()


class PainelAtividade(QFrame):
    """Aparece enquanto o robô executa algo: título, detalhe, progresso,
    relógio e o botão Parar movimento."""

    parar_clicado = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('atividade')
        lay = QHBoxLayout(self)
        lay.setContentsMargins(16, 12, 12, 12)
        lay.setSpacing(16)

        textos = QVBoxLayout()
        textos.setSpacing(8)
        topo = QHBoxLayout()
        self.titulo = rotulo('', 'atividadeTitulo')
        self.tempo = rotulo('', 'atividadeTempo')
        topo.addWidget(self.titulo)
        topo.addStretch()
        topo.addWidget(self.tempo)
        textos.addLayout(topo)
        self.barra = QProgressBar()
        self.barra.setRange(0, 1000)
        self.barra.setTextVisible(False)
        self.barra.setFixedHeight(6)
        textos.addWidget(self.barra)
        self.detalhe = rotulo('', 'atividadeDetalhe')
        self.detalhe.setWordWrap(True)
        textos.addWidget(self.detalhe)
        lay.addLayout(textos, 1)

        self.parar_btn = com_icone(QPushButton('Parar movimento'), 'stop')
        self.parar_btn.setObjectName('parar')
        self.parar_btn.setToolTip(
            'Para o robô e cancela a sequência, sem emergência: continua '
            'conectado e pronto para o próximo comando.')
        self.parar_btn.clicked.connect(self.parar_clicado.emit)
        lay.addWidget(self.parar_btn, 0, Qt.AlignVCenter)

        self._inicio = None
        self._duracao = None
        self._relogio = QTimer(self)
        self._relogio.setInterval(500)
        self._relogio.timeout.connect(self._atualizar_relogio)
        self.hide()

    # --- controle ---

    def iniciar(self, titulo):
        self.titulo.setText(titulo)
        self.detalhe.setText('')
        self.tempo.setText('')
        self.barra.setRange(0, 0)          # "ocupado" até haver progresso
        self.parar_btn.setEnabled(True)
        self._inicio = time.monotonic()
        self._duracao = None
        self._relogio.start()
        self.show()

    def atualizar(self, dados):
        """dados: dict com 'detalhe', 'fracao' (0–1 ou None), 'titulo',
        'cronometro' = (inicio_monotonic, duracao_s ou None)."""
        if dados.get('titulo'):
            self.titulo.setText(dados['titulo'])
        if 'detalhe' in dados:
            self.detalhe.setText(dados['detalhe'])
        if 'cronometro' in dados and dados['cronometro']:
            self._inicio, self._duracao = dados['cronometro']
        fracao = dados.get('fracao')
        if fracao is not None:
            self.barra.setRange(0, 1000)
            self.barra.setValue(int(max(0.0, min(1.0, fracao)) * 1000))
        self._atualizar_relogio()

    def terminar(self):
        self._relogio.stop()
        self.hide()

    @staticmethod
    def formatar(segundos):
        segundos = int(max(0, segundos))
        h, resto = divmod(segundos, 3600)
        m, s = divmod(resto, 60)
        return f'{h}:{m:02d}:{s:02d}' if h else f'{m:02d}:{s:02d}'

    def _atualizar_relogio(self):
        if self._inicio is None:
            return
        decorrido = time.monotonic() - self._inicio
        if self._duracao:
            self.tempo.setText(f'{self.formatar(decorrido)} / {self.formatar(self._duracao)}')
            self.barra.setRange(0, 1000)
            self.barra.setValue(int(min(1.0, decorrido / self._duracao) * 1000))
        else:
            self.tempo.setText(self.formatar(decorrido))


class PontoEstado(QLabel):
    """Bolinha colorida de estado."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(9, 9)
        self.definir(COR['desabilitado_texto'])

    def definir(self, cor):
        self.setStyleSheet(f'background-color: {cor}; border-radius: 4px;')


def separador_vertical():
    linha = QFrame()
    linha.setObjectName('rodapeSeparador')
    linha.setFixedWidth(1)
    linha.setFixedHeight(14)
    return linha