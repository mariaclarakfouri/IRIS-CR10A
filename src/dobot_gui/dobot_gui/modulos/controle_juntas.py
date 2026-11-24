"""Módulo Controle Juntas: move cada junta em tempo real (a home desta aba fica no
cartão de home, ver homes.py)."""
import threading
import time

from PyQt5.QtCore import QRegExp, QTimer, pyqtSignal
from PyQt5.QtGui import QRegExpValidator
from PyQt5.QtWidgets import QGridLayout, QHBoxLayout, QLineEdit, QPushButton

from ..constantes import HOME_JOINTS, JOINT_MAX, JOINT_MIN, JOINT_NAMES
from ..widgets import Cartao, JointSpinBox, barra_e_campo, com_icone, rotulo
from .base import Modulo
from .campo_trabalho import VELOCIDADE_MAX_PONTO

# "Ir para uma posição": limites de cada junta no robô (graus). A J3 vai até
# ±160°; as outras dão a volta (±360°), então valem ângulos como 310° ou -270°.
LIMITES_POSICAO = [(-360.0, 360.0), (-360.0, 360.0), (-160.0, 160.0),
                   (-360.0, 360.0), (-360.0, 360.0), (-360.0, 360.0)]
MOVIMENTO_GRANDE_GRAUS = 90


class ControleJuntas(Modulo):
    NOME = 'Controle Juntas'
    CHAVE_HOME = 'controle_juntas'
    ICONE = 'sliders'
    DESCRICAO = 'Movimento manual de cada junta, em tempo real.'

    jog_result = pyqtSignal(bool, str)

    def construir(self):
        self.jog_result.connect(self._on_jog_result)

        juntas = Cartao(
            'Juntas',
            'Cada junta vai de -180° a 180°. O robô se move quando você clica na bolinha '
            'da barra e arrasta, clica nas setinhas ou digita um valor e aperta Enter '
            '(sem Enter, o valor digitado é descartado). A rodinha do mouse e cliques '
            'fora da bolinha não mexem em nada. Depois de '
            'conectar, os valores mostrados são lidos direto do robô.')
        grade = QGridLayout()
        grade.setHorizontalSpacing(14)
        grade.setVerticalSpacing(12)
        grade.setColumnStretch(1, 1)

        self.joint_sliders = []
        self.joint_spinboxes = []
        for i, nome in enumerate(JOINT_NAMES):
            slider, spin = barra_e_campo(JOINT_MIN, JOINT_MAX, HOME_JOINTS[i], '°',
                                         spin_cls=JointSpinBox)
            slider.setEnabled(False)
            spin.setEnabled(False)
            # Arrastar: manda só quando a barra dá uma parada (ou ao soltar), pra o
            # robô ir direto e liso até o grau final em vez de parar e arrancar a
            # cada grau intermediário.
            slider.valueChanged.connect(lambda _v: self._jog_debounce.start())
            slider.sliderReleased.connect(self._request_jog_now)
            spin.confirmado.connect(self._request_jog)       # só no Enter
            spin.stepped.connect(self._request_jog)
            grade.addWidget(rotulo(nome), i, 0)
            grade.addWidget(slider, i, 1)
            grade.addWidget(spin, i, 2)
            self.joint_sliders.append(slider)
            self.joint_spinboxes.append(spin)
        juntas.corpo.addLayout(grade)
        self.layout_modulo.addWidget(juntas)
        self.layout_modulo.addWidget(self._cartao_posicao())

        self.layout_modulo.addStretch()

        self._jog_debounce = QTimer(self)
        self._jog_debounce.setSingleShot(True)
        self._jog_debounce.setInterval(75)
        self._jog_debounce.timeout.connect(self._request_jog)
        self._jog_busy = False
        self._jog_pending = False
        self._last_jog_time = 0.0

    def _cartao_posicao(self):
        c = Cartao(
            'Ir para uma posição',
            'Escreva o ângulo de cada junta (em graus) e clique em "Ir para esta posição". '
            'Nada se move enquanto você escreve: o robô só vai depois de você confirmar. '
            'J3 vai até ±160°; as outras até ±360°. Como todo movimento, o caminho é '
            'conferido pela segurança do campo de trabalho.')
        grade = QGridLayout()
        grade.setHorizontalSpacing(14)
        grade.setVerticalSpacing(8)
        numero = QRegExpValidator(QRegExp(r'^-?\d{0,3}([.,]\d{0,3})?$'))
        self.posicao_edits = []
        for i, nome in enumerate(JOINT_NAMES):
            edit = QLineEdit()
            edit.setValidator(numero)
            lo, hi = LIMITES_POSICAO[i]
            edit.setPlaceholderText(f'{lo:.0f} a {hi:.0f}')
            edit.setMaximumWidth(140)
            grade.addWidget(rotulo(nome), i // 3, (i % 3) * 2)
            grade.addWidget(edit, i // 3, (i % 3) * 2 + 1)
            self.posicao_edits.append(edit)
        grade.setColumnStretch(6, 1)
        c.corpo.addLayout(grade)
        botoes = QHBoxLayout()
        botoes.setSpacing(10)
        atual = QPushButton('Usar a posição atual')
        atual.setObjectName('discreto')
        atual.setToolTip('Preenche com onde o robô está agora (depois é só mudar o que quiser).')
        atual.clicked.connect(self._preencher_posicao_atual)
        self.posicao_btn = com_icone(QPushButton('Ir para esta posição'), 'play')
        self.posicao_btn.setEnabled(False)
        self.posicao_btn.clicked.connect(self._ir_para_posicao)
        botoes.addWidget(atual)
        botoes.addStretch(1)
        botoes.addWidget(self.posicao_btn)
        c.corpo.addLayout(botoes)
        return c

    def _preencher_posicao_atual(self):
        q = self.janela.ultimas_juntas()
        if not q:
            self.janela.alerta.mostrar('Ainda não há leitura do robô: conecte primeiro.', 'aviso')
            return
        for edit, v in zip(self.posicao_edits, q):
            edit.setText(f'{v:.1f}')

    def _ler_posicao(self):
        """(lista de 6 graus, None) ou (None, o que está errado)."""
        alvo = []
        for i, (edit, nome) in enumerate(zip(self.posicao_edits, JOINT_NAMES)):
            texto = edit.text().strip().replace(',', '.')
            if texto in ('', '-', '.', '-.'):
                return None, f'Falta o ângulo da {nome}.'
            try:
                v = float(texto)
            except ValueError:
                return None, f'O ângulo da {nome} não é um número.'
            lo, hi = LIMITES_POSICAO[i]
            if not lo <= v <= hi:
                return None, f'{nome} vai de {lo:.0f}° a {hi:.0f}° (você escreveu {v:g}°).'
            alvo.append(v)
        return alvo, None

    def _ir_para_posicao(self):
        j = self.janela
        if j._action_running:
            return
        alvo, erro = self._ler_posicao()
        if erro:
            j.alerta.mostrar(erro, 'aviso')
            return
        vel = j.speed_slider.value()
        if not j.sim_mode and vel > VELOCIDADE_MAX_PONTO:
            j.alerta.mostrar(f'No robô real, só com velocidade até {VELOCIDADE_MAX_PONTO}% '
                             f'(está em {vel}%). Baixe a velocidade e tente de novo.', 'aviso')
            return
        robo = 'gêmeo' if j.sim_mode else 'robô'
        de = j.ultimas_juntas() or alvo
        maior = max(abs(a - b) for a, b in zip(alvo, de))
        aviso = (f'\n\nATENÇÃO: movimento grande ({maior:.0f}° numa junta): confira se não há '
                 'ninguém nem nada perto do robô.' if maior > MOVIMENTO_GRANDE_GRAUS else '')
        texto = (f'Mover o {robo} para a posição escrita.\n\n'
                 'De:  ' + ', '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, de))
                 + '\nPara: ' + ', '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, alvo))
                 + f'\n\nVelocidade: {vel}%' + aviso
                 + '\n\nO caminho é conferido pela segurança do campo de trabalho antes de '
                 'sair. Parar e EMERGÊNCIA continuam valendo durante o movimento.')
        if not j.confirmar('Ir para uma posição', texto, f'Mover o {robo}'):
            return
        if j._action_running:
            return
        j.iniciar_acao('Indo para a posição escrita', 'Ir para uma posição: '
                       + ', '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, alvo)))
        threading.Thread(target=self._executar_posicao, args=(alvo,), daemon=True).start()

    def _executar_posicao(self, alvo):
        j = self.janela
        ok, msg = False, 'Movimento não concluído.'
        try:
            ok, msg = j.mover_e_esperar(alvo)
        finally:
            j.terminar_acao(ok, 'Robô na posição escrita.' if ok else msg)

    # --- interface com a base ---

    def widgets_de_movimento(self):
        return self.joint_sliders + self.joint_spinboxes + [self.posicao_btn]

    def posicionar_cartao_home(self, cartao):
        # Logo abaixo das juntas, antes do espaço que sobra no fim da página.
        self.layout_modulo.insertWidget(self.layout_modulo.count() - 1, cartao)

    def carregar_config(self, dados):
        joints = dados.get('joints')
        if isinstance(joints, list) and len(joints) == len(self.joint_spinboxes):
            for spin, value in zip(self.joint_spinboxes, joints):
                spin.setValue(value)

    def salvar_config(self, dados):
        dados['joints'] = [spin.value() for spin in self.joint_spinboxes]

    def ao_ler_juntas(self, juntas, robo_parado):
        # Não sobrescreve enquanto os controles estão desabilitados (movimento
        # automático em andamento), enquanto o usuário arrasta uma barra ou logo
        # depois de mandar um movimento manual. Enquanto o robô ainda anda, a
        # barra mostra o destino pedido; parado, mostra a posição real.
        if not self.joint_sliders[0].isEnabled():
            return
        if any(slider.isSliderDown() for slider in self.joint_sliders):
            return
        if self._jog_busy or self._jog_pending or time.time() - self._last_jog_time < 1.0:
            return
        if not robo_parado:
            return
        for spin, value in zip(self.joint_spinboxes, juntas):
            if spin.hasFocus() and spin.editando():
                continue                  # não apaga o que você está digitando
            spin.setValue(round(value))

    # --- movimento manual ---

    def _request_jog_now(self):
        self._jog_debounce.stop()
        self._request_jog()

    def _request_jog(self):
        """Movimento manual em tempo real: no máximo 1 comando em andamento e
        sempre com o valor mais recente (o robô não recebe uma fila de comandos)."""
        self._last_jog_time = time.time()
        self._jog_pending = True
        if not self._jog_busy:
            self._send_jog()

    def _send_jog(self):
        if not self._jog_pending:
            return
        self._jog_pending = False
        self._jog_busy = True
        joints = [spin.value() for spin in self.joint_spinboxes]
        threading.Thread(
            target=self.janela.ros_node.call_jog,
            args=(joints, lambda ok, msg: self.jog_result.emit(ok, msg)),
            daemon=True,
        ).start()

    def _on_jog_result(self, ok, msg):
        self._jog_busy = False
        if not ok:
            self._jog_pending = False
            if self.janela.avisar_inseguro(msg):
                return
            self.janela.log(f'Erro no movimento manual: {msg}', 'ERRO')
            if 'não conectado' in msg:
                self.janela.marcar_desconectado()
                self.janela.alerta.mostrar(
                    'O backend diz que o robô não está conectado (tem outro '
                    'dobot_action_server rodando?). Clique em Conectar de novo.', 'erro')
            else:
                self.janela.alerta.mostrar(f'Erro no movimento: {msg}', 'erro')
            return
        self.janela.alerta.mostrar('Movimento enviado ao robô.', 'info')
        if self._jog_pending:
            QTimer.singleShot(80, self._send_jog)
