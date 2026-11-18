"""Módulo Controle Juntas: move cada junta em tempo real (a home desta aba fica no
cartão de home, ver homes.py)."""
import threading
import time

from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import QGridLayout

from ..constantes import HOME_JOINTS, JOINT_MAX, JOINT_MIN, JOINT_NAMES
from ..widgets import Cartao, JointSpinBox, barra_e_campo, rotulo
from .base import Modulo


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
            'Cada junta vai de -180° a 180°. O robô se move enquanto você arrasta a '
            'barra, usa as setinhas ou digita um valor e aperta Enter. Depois de '
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
            spin.editingFinished.connect(self._request_jog)
            spin.stepped.connect(self._request_jog)
            grade.addWidget(rotulo(nome), i, 0)
            grade.addWidget(slider, i, 1)
            grade.addWidget(spin, i, 2)
            self.joint_sliders.append(slider)
            self.joint_spinboxes.append(spin)
        juntas.corpo.addLayout(grade)
        self.layout_modulo.addWidget(juntas)

        self.layout_modulo.addStretch()

        self._jog_debounce = QTimer(self)
        self._jog_debounce.setSingleShot(True)
        self._jog_debounce.setInterval(75)
        self._jog_debounce.timeout.connect(self._request_jog)
        self._jog_busy = False
        self._jog_pending = False
        self._last_jog_time = 0.0

    # --- interface com a base ---

    def widgets_de_movimento(self):
        return self.joint_sliders + self.joint_spinboxes

    def posicionar_cartao_home(self, cartao):
        # Logo abaixo das juntas, antes do espaço que sobra no fim da página.
        # (Com mais abas, a lista homes.CartaoTodasHomes volta para cá.)
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
