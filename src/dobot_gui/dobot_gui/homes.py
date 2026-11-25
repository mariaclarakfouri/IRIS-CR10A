"""A home do robô.

A posição (graus, convenção do Dobot) fica em `janela.homes[chave]`, salva nas
configurações da interface, e começa na posição de repouso do robô.

O cartão `CartaoHome` fica no Controle Juntas: as 6 juntas, "Usar posição atual"
(só preenche os campos), "Gravar home" (pede confirmação e salva), "Ir para a
home" (pede confirmação e move) e "Drag mode". O "Ir para Home" da barra
de cima vai para esta mesma home.
"""
from PyQt5.QtWidgets import (QDoubleSpinBox, QHBoxLayout, QPushButton,
                             QVBoxLayout, QWidget)

from .constantes import JOINT_MAX, JOINT_MIN, JOINT_NAMES
from .tema import COR
from .widgets import Cartao, com_icone, rotulo

# Chave e nome da home (uma só).
HOMES = (
    ('controle_juntas', 'Home'),
)
NOME_HOME = dict(HOMES)
# A home "padrão" da plataforma (janela.home_joints): a do Controle Juntas, que
# começa na posição de repouso conhecida do robô (constantes.HOME_JOINTS).
HOME_PADRAO = 'controle_juntas'


DICA_ARRASTAR = ('Liga o drag mode do robô (o mesmo do botão no braço): você guia o braço '
                 'com a mão. Para sair, aperte o botão no robô ou clique em "Desabilitar drag".')
DICA_ARRASTAR_SIM = 'Só no robô real: o gêmeo virtual não pode ser guiado com a mão.'
TEXTO_DRAG = 'Drag mode'
TEXTO_SAIR_DRAG = 'Desabilitar drag'


def texto_juntas(juntas):
    return '   '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, juntas))


class CartaoHome(Cartao):
    """Cartão da home de uma aba. Normalmente só mostra a home e o "Ir para a
    home"; os campos para trocá-la ficam escondidos atrás de "Alterar home",
    para nenhum clique ou rolada do mouse sem querer mudar a home."""

    def __init__(self, janela, chave, texto=None):
        super().__init__(NOME_HOME[chave], texto)
        self.janela = janela
        self.chave = chave
        self._movimento_ok = False

        # Resumo (sempre visível): valores gravados, Ir para a home, Alterar home.
        resumo = QHBoxLayout()
        resumo.setSpacing(10)
        self.valores = rotulo('', 'valorMono')
        self.ir_btn = com_icone(QPushButton('Ir para a home'), 'home')
        self.ir_btn.clicked.connect(lambda: self.janela.ir_para_home(self.chave))
        self.mao_btn = QPushButton(TEXTO_DRAG)
        self.mao_btn.setObjectName('secundario')
        self.mao_btn.clicked.connect(self.janela.alternar_drag)
        self.alterar_btn = QPushButton('Alterar home')
        self.alterar_btn.setObjectName('discreto')
        self.alterar_btn.clicked.connect(self._abrir_editor)
        resumo.addWidget(self.valores, 1)
        resumo.addWidget(self.ir_btn)
        resumo.addWidget(self.mao_btn)
        resumo.addWidget(self.alterar_btn)
        self.corpo.addLayout(resumo)

        # Editor (escondido até clicar em Alterar home).
        self.editor = QWidget()
        ed = QVBoxLayout(self.editor)
        ed.setContentsMargins(0, 6, 0, 0)
        ed.setSpacing(10)
        campos = QHBoxLayout()
        campos.setSpacing(8)
        self.spins = []
        for nome in JOINT_NAMES:
            campos.addWidget(rotulo(nome, 'cartaoTexto'))
            s = QDoubleSpinBox()
            s.setRange(JOINT_MIN, JOINT_MAX)
            s.setDecimals(1)
            s.setSingleStep(1.0)
            s.setSuffix(' °')
            s.setFixedWidth(92)
            s.setKeyboardTracking(False)
            s.valueChanged.connect(self._atualizar_estado)
            campos.addWidget(s)
            campos.addSpacing(6)
            self.spins.append(s)
        campos.addStretch(1)
        ed.addLayout(campos)
        botoes = QHBoxLayout()
        botoes.setSpacing(10)
        self.atual_btn = QPushButton('Usar posição atual')
        self.atual_btn.setObjectName('discreto')
        self.atual_btn.setToolTip('Preenche os campos com a posição atual do robô (não grava).')
        self.atual_btn.clicked.connect(self._usar_atual)
        self.gravar_btn = com_icone(QPushButton('Gravar home'), 'save', COR['texto'])
        self.gravar_btn.setObjectName('secundario')
        self.gravar_btn.clicked.connect(self._gravar)
        self.cancelar_btn = QPushButton('Cancelar')
        self.cancelar_btn.setObjectName('discreto')
        self.cancelar_btn.clicked.connect(self._fechar_editor)
        self.estado = rotulo('', 'cartaoTexto')
        botoes.addWidget(self.atual_btn)
        botoes.addWidget(self.gravar_btn)
        botoes.addWidget(self.cancelar_btn)
        botoes.addSpacing(8)
        botoes.addWidget(self.estado, 1)
        ed.addLayout(botoes)
        self.corpo.addWidget(self.editor)
        self.editor.hide()
        self.recarregar()

    # --- estado ---

    def recarregar(self):
        """Mostra a home gravada no resumo e nos campos (sem home: a posição
        atual nos campos, para servir de ponto de partida)."""
        home = self.janela.home_de(self.chave)
        if home is None:
            self.valores.setText('não gravada')
            self.valores.setStyleSheet(f'color: {COR["aviso_texto"]};')
        else:
            self.valores.setText(texto_juntas(home))
            self.valores.setStyleSheet('')
        valores = home if home is not None else (self.janela.ultimas_juntas() or [0.0] * 6)
        for s, v in zip(self.spins, valores):
            s.blockSignals(True)
            s.setValue(float(v))
            s.blockSignals(False)
        self._atualizar_estado()

    def _campos(self):
        return [round(s.value(), 1) for s in self.spins]

    def _atualizar_estado(self, *_):
        home = self.janela.home_de(self.chave)
        if home is None:
            txt, cor = 'Ainda não gravada.', COR['aviso_texto']
        elif any(abs(a - b) > 0.05 for a, b in zip(self._campos(), home)):
            txt, cor = 'Valores alterados, ainda não gravados.', COR['aviso_texto']
        else:
            txt, cor = 'Igual à home gravada.', COR['texto_fraco']
        self.estado.setText(txt)
        self.estado.setStyleSheet(f'color: {cor};')
        self.ir_btn.setEnabled(self._movimento_ok and home is not None)
        atualizar = getattr(self.janela, 'atualizar_botoes_drag', None)
        if callable(atualizar):
            atualizar()            # o botão de drag segue o estado da janela

    def definir_movimento_habilitado(self, ok):
        self._movimento_ok = bool(ok)
        self._atualizar_estado()

    def _abrir_editor(self):
        self.recarregar()
        self.editor.show()
        self.alterar_btn.hide()

    def _fechar_editor(self):
        self.recarregar()                 # descarta o que não foi gravado
        self.editor.hide()
        self.alterar_btn.show()

    # --- botões ---

    def _usar_atual(self):
        atual = self.janela.ultimas_juntas()
        if atual is None:
            self.janela.alerta.mostrar('Sem a posição do robô ainda: conecte primeiro.', 'aviso')
            return
        for s, v in zip(self.spins, atual):
            s.setValue(round(float(v), 1))

    def _gravar(self):
        novos = self._campos()
        if not self.janela.confirmar(
                'Gravar home',
                f'Gravar como {NOME_HOME[self.chave]}?\n{texto_juntas(novos)}',
                'Gravar'):
            return
        self.janela.gravar_home(self.chave, novos)
        self._fechar_editor()

