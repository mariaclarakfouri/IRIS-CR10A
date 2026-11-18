"""Uma home por aba.

Cada aba que tem home guarda a sua posição (graus, convenção do Dobot) em
`janela.homes[chave]`, salva nas configurações da interface. A do Controle
Juntas começa na posição de repouso do robô; as outras começam sem home gravada:
o "Ir para a home" fica travado até você gravar uma.

O cartão `CartaoHome` é o mesmo em todas as abas: as 6 juntas, "Usar posição
atual" (só preenche os campos), "Gravar home" (pede confirmação e salva) e "Ir
para a home" (pede confirmação e move, com a mesma verificação de segurança de
todo movimento). A barra de cima da interface lista todas as homes.
"""
from PyQt5.QtWidgets import (QDoubleSpinBox, QGridLayout, QHBoxLayout, QPushButton,
                             QVBoxLayout, QWidget)

from .constantes import JOINT_MAX, JOINT_MIN, JOINT_NAMES
from .tema import COR
from .widgets import Cartao, com_icone, rotulo

# Chave e nome de cada home. A ordem é a do menu da barra de cima e a da lista
# do Controle Juntas.
HOMES = (
    ('controle_juntas', 'Home do Controle Juntas'),
)
NOME_HOME = dict(HOMES)
# A home "padrão" da plataforma (janela.home_joints): a do Controle Juntas, que
# começa na posição de repouso conhecida do robô (constantes.HOME_JOINTS).
HOME_PADRAO = 'controle_juntas'
# Uma linha por aba na lista do Controle Juntas e no menu da barra de cima.
LISTA_ABAS = tuple((nome, chave) for chave, nome in HOMES)


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
        self.alterar_btn = QPushButton('Alterar home')
        self.alterar_btn.setObjectName('discreto')
        self.alterar_btn.clicked.connect(self._abrir_editor)
        resumo.addWidget(self.valores, 1)
        resumo.addWidget(self.ir_btn)
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


class CartaoTodasHomes(Cartao):
    """Todas as homes numa lista, cada uma com o seu "Ir para a home" (aba
    Controle Juntas). Para mudar uma home, use o cartão da aba dela."""

    chave = None                         # atualiza quando qualquer home é gravada

    def __init__(self, janela):
        super().__init__('Homes de todas as abas',
                         'Para mudar uma home, use o cartão no alto da aba dela.')
        self.janela = janela
        self._movimento_ok = False
        grade = QGridLayout()
        grade.setHorizontalSpacing(18)
        grade.setVerticalSpacing(8)
        grade.setColumnStretch(1, 1)
        self.linhas = {}
        for i, (nome, chave) in enumerate(LISTA_ABAS):
            r_nome = rotulo(nome, 'cartaoTexto')
            r_nome.setStyleSheet('font-weight: 600;')
            valores = rotulo('', 'valorMono')
            botao = com_icone(QPushButton('Ir para a home'), 'home')
            botao.setObjectName('secundario')
            botao.clicked.connect(lambda _c, k=chave: self.janela.ir_para_home(k))
            grade.addWidget(r_nome, i, 0)
            grade.addWidget(valores, i, 1)
            grade.addWidget(botao, i, 2)
            self.linhas[nome] = (chave, valores, botao)
        self.corpo.addLayout(grade)
        self.recarregar()

    def recarregar(self):
        for chave, valores, botao in self.linhas.values():
            home = self.janela.home_de(chave)
            if home is None:
                valores.setText('não gravada')
                valores.setStyleSheet(f'color: {COR["texto_fraco"]};')
            else:
                valores.setText(texto_juntas(home))
                valores.setStyleSheet('')
            botao.setEnabled(self._movimento_ok and home is not None)

    def definir_movimento_habilitado(self, ok):
        self._movimento_ok = bool(ok)
        self.recarregar()
