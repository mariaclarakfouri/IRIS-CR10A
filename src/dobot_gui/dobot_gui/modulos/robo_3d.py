"""Módulo Robô 3D: modelo do CR10A espelhando a posição real do robô e, com
"Comandar o robô pelo 3D", um robô-alvo translúcido para mover o robô até ele.

Segurança (de propósito, em camadas):
  * mexer no alvo (arrastando no 3D ou digitando) NUNCA manda nada ao robô: só
    muda o desenho;
  * a cada mudança, o caminho do robô até o alvo passa pela verificação de
    segurança do servidor (a mesma de todo movimento) e o alvo fica verde,
    vermelho (com o motivo) ou amarelo (conferindo);
  * "Mover o robô até o alvo" só fica ativo com o alvo verde, verificado a partir
    da posição em que o robô está agora, com o robô parado e liberado pela janela
    e a velocidade até VELOCIDADE_MAX_3D;
  * antes de mover, pede confirmação (de onde, para onde, velocidade); o servidor
    confere tudo de novo antes de mandar o MovJ; Parar e EMERGÊNCIA continuam
    valendo durante o movimento.
"""
import threading
import time

from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QPushButton,
)

from ..constantes import JOINT_MAX, JOINT_MIN, JOINT_NAMES, SENSOR_S02_MM
from ..robot_view import RobotView
from ..tema import COR
from ..widgets import Cartao, rotulo
from .base import Modulo

# Velocidade máxima aceita para mover pelo 3D (%), abaixo do limite geral: o
# alvo é escolhido no olho, então o movimento vai devagar.
VELOCIDADE_MAX_3D = 30
# Espera depois da última mudança do alvo antes de conferir o caminho (ms).
ESPERA_VERIFICAR_MS = 350
FLUXO_MUDO_S = 1.0           # sem o tópico de estado por mais que isso, o desenho segue a leitura da janela
# Se o robô andar mais que isto (graus) depois da verificação, ela é refeita.
TOLERANCIA_POSICAO_GRAUS = 0.5
# Acima disto (graus, na junta que mais muda), a confirmação avisa.
MOVIMENTO_GRANDE_GRAUS = 90.0


class Robo3D(Modulo):
    NOME = 'Robô 3D'
    ICONE = 'cube'
    DESCRICAO = ('Modelo 3D do CR10A espelhando a posição real do robô; com "Comandar o '
                 'robô pelo 3D", move o robô até um alvo escolhido no próprio 3D.')

    caminho_verificado = pyqtSignal(bool, str, object)
    movimento_terminou = pyqtSignal()
    estado_recebido = pyqtSignal(object)   # ângulos do tópico de estado (vem da thread do ROS)

    def construir(self):
        self._real = None                 # última posição lida do robô (graus)
        self._verificado = None           # (alvo, posição do robô, seguro, mensagem)
        self._mov_habilitado = False
        self._movendo = False
        self.caminho_verificado.connect(self._ao_verificar)
        self.movimento_terminou.connect(self._ao_terminar_movimento)
        # O desenho segue o tópico de estado (~30 por segundo), não a leitura da janela
        # (a cada 0,4 s): é isso que deixa o movimento liso. A leitura da janela só move
        # o desenho se o tópico ficar mudo.
        self._ultimo_fluxo = 0.0
        self.estado_recebido.connect(self._ao_receber_estado)
        assinar = getattr(self.janela.ros_node, 'assinar_estado', None)
        if callable(assinar):
            assinar(lambda msg: self.estado_recebido.emit(list(msg.angulo_graus)))
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(ESPERA_VERIFICAR_MS)
        self._timer.timeout.connect(self._verificar)

        linha = QHBoxLayout()
        linha.setSpacing(16)

        cartao = Cartao(texto='Arraste para girar, use a roda para aproximar e o botão '
                              'direito para mover a vista.')
        # O que desenhar no flange: a ferramenta com que a plataforma foi aberta
        # (iris = célula, com-sensor = sensor inercial S02, sem-nada = nada).
        ponta = getattr(self.janela.ros_node, 'ponta', 'celula')
        self._ferramenta = {'celula': 'celula', 'sensor': SENSOR_S02_MM}.get(ponta)
        self.mostrar_sensor = QCheckBox(
            'Mostrar a célula de carga FA7155 (com a ponteira) no flange' if ponta == 'celula'
            else 'Mostrar o sensor inercial S02 no flange')
        self.mostrar_sensor.setChecked(True)
        self.mostrar_sensor.toggled.connect(self._alternar_sensor)
        self.mostrar_sensor.setVisible(self._ferramenta is not None)
        cartao.corpo.addWidget(self.mostrar_sensor)
        self.robot_view = RobotView(ferramenta=self._ferramenta)
        self.robot_view.setMinimumHeight(460)
        self.robot_view.ao_mudar_alvo.connect(self._alvo_mudou_no_3d)
        cartao.corpo.addWidget(self.robot_view, 1)
        linha.addWidget(cartao, 1)
        linha.addWidget(self._montar_comando())
        self.layout_modulo.addLayout(linha, 1)
        velocidade = getattr(self.janela, 'speed_slider', None)
        if velocidade is not None and hasattr(velocidade, 'valueChanged'):
            velocidade.valueChanged.connect(lambda _v: self._atualizar_painel())
        self._atualizar_painel()

    def _robo(self):
        return 'gêmeo' if getattr(self.janela, 'sim_mode', False) is True else 'robô'

    def _montar_comando(self):
        c = Cartao('Comandar pelo 3D')
        c.setFixedWidth(300)
        self.comandar = QCheckBox('Comandar o robô pelo 3D')
        self.comandar.setToolTip('Mostra um robô-alvo translúcido. Mexer nele não move nada: '
                                 'o robô só se mexe em "Mover o robô até o alvo".')
        self.comandar.toggled.connect(self._alternar_comando)
        c.corpo.addWidget(self.comandar)

        grade = QGridLayout()
        grade.setHorizontalSpacing(8)
        grade.setVerticalSpacing(4)
        self.spins = []
        for i, nome in enumerate(JOINT_NAMES):
            grade.addWidget(rotulo(nome), i, 0)
            s = QDoubleSpinBox()
            s.setRange(JOINT_MIN, JOINT_MAX)
            s.setDecimals(1)
            s.setSingleStep(1.0)
            s.setSuffix(' °')
            s.setKeyboardTracking(False)
            s.valueChanged.connect(self._alvo_mudou_nos_campos)
            grade.addWidget(s, i, 1)
            self.spins.append(s)
        c.corpo.addLayout(grade)

        atalhos = QHBoxLayout()
        atalhos.setSpacing(6)
        self.copiar_btn = QPushButton('Posição atual')
        self.copiar_btn.setObjectName('discreto')
        self.copiar_btn.setToolTip('Põe o alvo onde o robô está agora.')
        self.copiar_btn.clicked.connect(self._alvo_na_posicao_atual)
        atalhos.addWidget(self.copiar_btn)
        self.home_alvo_btn = QPushButton('Home')
        self.home_alvo_btn.setObjectName('discreto')
        self.home_alvo_btn.setToolTip('Põe o alvo na posição home.')
        self.home_alvo_btn.setToolTip('Põe o alvo na home padrão (a do Controle Juntas).')
        self.home_alvo_btn.clicked.connect(lambda: self._definir_alvo(self.janela.home_joints))
        atalhos.addWidget(self.home_alvo_btn)
        c.corpo.addLayout(atalhos)

        self.status = rotulo('', 'cartaoTexto')
        self.status.setWordWrap(True)
        c.corpo.addWidget(self.status)
        self.mover_btn = QPushButton('Mover o robô até o alvo')
        self.mover_btn.setEnabled(False)
        self.mover_btn.clicked.connect(self._confirmar_e_mover)
        c.corpo.addWidget(self.mover_btn)
        self.dica = rotulo('', 'cartaoTexto')
        self.dica.setWordWrap(True)
        c.corpo.addWidget(self.dica)
        c.corpo.addStretch()
        return c

    # --- liga / desliga ---

    def _alternar_comando(self, ligado):
        if ligado:
            inicio = self._real if self._real is not None else self.janela.home_joints
            self._definir_alvo(inicio)
        else:
            self._timer.stop()
            self._verificado = None
            self.robot_view.definir_alvo(None)
        self._atualizar_painel()

    def _alvo(self):
        return [round(s.value(), 1) for s in self.spins]

    def _definir_alvo(self, juntas):
        for s, v in zip(self.spins, juntas):
            s.blockSignals(True)
            s.setValue(float(v))
            s.blockSignals(False)
        self._alvo_mudou()

    def _alvo_na_posicao_atual(self):
        if self._real is None:
            self.janela.alerta.mostrar(
                'Posição atual ainda desconhecida. Conecte ao robô primeiro.', 'aviso')
            return
        self._definir_alvo(self._real)

    def _alvo_mudou_no_3d(self, juntas):
        for s, v in zip(self.spins, juntas):
            s.blockSignals(True)
            s.setValue(float(v))
            s.blockSignals(False)
        self._alvo_mudou(redesenhar=False)

    def _alvo_mudou_nos_campos(self, _valor=None):
        self._alvo_mudou()

    def _alvo_mudou(self, redesenhar=True):
        """Alvo novo: nada é enviado ao robô; só redesenha e confere de novo."""
        if not self.comandar.isChecked():
            return
        if redesenhar:
            self.robot_view.definir_alvo(self._alvo(), editavel=True,
                                         limites=(JOINT_MIN, JOINT_MAX))
        self._verificado = None
        self.robot_view.definir_estado_alvo('conferindo')
        self._timer.start()
        self._atualizar_painel()

    # --- verificação ---

    def _verificar(self):
        if not self.comandar.isChecked() or self._movendo:
            return
        alvo, real = self._alvo(), (list(self._real) if self._real is not None else None)
        self.janela.ros_node.call_verificar_caminho(
            alvo, lambda ok, msg: self.caminho_verificado.emit(ok, msg, (alvo, real)))

    def _ao_verificar(self, seguro, msg, contexto):
        alvo, real = contexto
        if not self.comandar.isChecked() or alvo != self._alvo():
            return                     # o alvo mudou enquanto conferia
        self._verificado = (alvo, real, bool(seguro), msg)
        self.robot_view.definir_estado_alvo('ok' if seguro else 'ruim')
        self._atualizar_painel()

    def _verificacao_vale(self):
        """True se a última verificação é deste alvo, a partir de onde o robô está."""
        v = self._verificado
        if v is None or v[0] != self._alvo() or v[1] is None or self._real is None:
            return False
        return max(abs(a - b) for a, b in zip(v[1], self._real)) <= TOLERANCIA_POSICAO_GRAUS

    # --- painel ---

    def definir_movimento_habilitado(self, habilitado):
        # Liberação da janela (conectado, sem ação em andamento, sem emergência).
        self._mov_habilitado = habilitado
        super().definir_movimento_habilitado(habilitado)
        self._atualizar_painel()

    def widgets_de_movimento(self):
        return [self.mover_btn]

    def _motivo_bloqueio(self):
        """Por que o botão de mover está desligado ('' = pode mover)."""
        if not self.comandar.isChecked():
            return 'Ligue "Comandar o robô pelo 3D".'
        if self._movendo:
            return f'O {self._robo()} está indo até o alvo.'
        if not self._mov_habilitado:
            return f'O {self._robo()} não está pronto (conecte, ou espere a ação em andamento).'
        vel = self.janela.speed_slider.value()
        if vel > VELOCIDADE_MAX_3D:
            return (f'Velocidade em {vel}%: pelo 3D, o limite é {VELOCIDADE_MAX_3D}%. Baixe a '
                    'velocidade na barra do robô.')
        if self._real is not None and self._alvo() == [round(v, 1) for v in self._real]:
            return f'O alvo é a posição em que o {self._robo()} já está.'
        if not self._verificacao_vale():
            return 'Conferindo o caminho até o alvo…'
        return ''                      # se o caminho foi recusado, o motivo está no status

    def _atualizar_painel(self):
        ligado = self.comandar.isChecked()
        for w in self.spins + [self.copiar_btn, self.home_alvo_btn]:
            w.setEnabled(ligado and not self._movendo)
        v = self._verificado
        if not ligado:
            self.status.setStyleSheet('')
            self.status.setText('Desligado: o 3D só mostra onde o robô está.')
        elif v is not None and self._verificacao_vale():
            self.status.setStyleSheet(f'color: {COR["sucesso"] if v[2] else COR["perigo"]};')
            self.status.setText(('✔ ' if v[2] else '✖ ') + (v[3] or ('caminho seguro' if v[2]
                                                                    else 'caminho recusado')))
        else:
            self.status.setStyleSheet(f'color: {COR["aviso_texto"]};')
            self.status.setText('Conferindo o caminho até o alvo…')
        motivo = self._motivo_bloqueio()
        pode = (motivo == '' and v is not None and v[2] and self._verificacao_vale())
        self.mover_btn.setEnabled(pode)
        self.dica.setText(motivo if ligado else
                          'Ligado, aparece um robô-alvo translúcido: clique numa peça dele e '
                          'arraste para os lados para girar a junta, ou digite os ângulos. '
                          'Mexer no alvo não move nada.')

    # --- mover ---

    def _confirmar_e_mover(self):
        j = self.janela
        if self._motivo_bloqueio() or not self._verificacao_vale() or not self._verificado[2]:
            self._atualizar_painel()
            return
        alvo, de = self._alvo(), self._real
        maior = max(abs(a - b) for a, b in zip(alvo, de))
        aviso = (f'\n\nATENÇÃO: movimento grande ({maior:.0f}° numa junta). Confira se não há '
                 'ninguém nem nada perto do robô.' if maior > MOVIMENTO_GRANDE_GRAUS else '')
        texto = ('De:  ' + ', '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, de))
                 + '\nPara: ' + ', '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, alvo))
                 + f'\n\nVelocidade: {j.speed_slider.value()}%' + aviso
                 + '\n\nO caminho é conferido de novo antes de sair do lugar. Parar e '
                   'EMERGÊNCIA continuam valendo durante o movimento.')
        if not j.confirmar('Mover pelo 3D', texto, f'Mover o {self._robo()}'):
            return
        if j._action_running or not self._mov_habilitado:
            return
        self._movendo = True
        self._atualizar_painel()
        j.iniciar_acao('Mover pelo 3D', 'Mover pelo 3D: destino '
                       + ', '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, alvo)))
        threading.Thread(target=self._executar, args=(alvo,), daemon=True).start()

    def _executar(self, alvo):
        j = self.janela
        ok, msg = False, 'Movimento não concluído.'
        try:
            ok, msg = j.mover_e_esperar(alvo)
        finally:
            j.terminar_acao(ok, 'Movimento pelo 3D concluído.' if ok else msg)
            self.movimento_terminou.emit()

    def _ao_terminar_movimento(self):
        # Na thread da janela: a posição mudou, então confere o alvo de novo.
        self._movendo = False
        self._alvo_mudou(redesenhar=False)

    # --- leitura ---

    def _ao_receber_estado(self, q):
        if len(q) != 6:
            return
        self._ultimo_fluxo = time.monotonic()
        self.robot_view.seguir(q)

    def ao_ler_juntas(self, juntas, robo_parado):
        anterior = self._real
        self._real = list(juntas)
        if time.monotonic() - self._ultimo_fluxo > FLUXO_MUDO_S:
            self.robot_view.set_joints_deg(juntas)
        if not self.comandar.isChecked():
            # Desligado: os campos mostram onde o robô está (não ficam em 0,0°).
            for s, v in zip(self.spins, juntas):
                if abs(s.value() - float(v)) >= 0.05:
                    s.blockSignals(True)
                    s.setValue(float(v))
                    s.blockSignals(False)
            return
        if self._movendo:
            return
        # O robô andou (outra página, Home...): a verificação antiga não vale mais.
        if self._verificado is not None and not self._verificacao_vale():
            self._alvo_mudou(redesenhar=False)
        elif anterior is None:
            self._alvo_mudou(redesenhar=False)
        else:
            self._atualizar_painel()

    def _alternar_sensor(self, ligado):
        self.robot_view.definir_sensor(self._ferramenta if ligado else None)

    def carregar_config(self, dados):
        self.mostrar_sensor.setChecked(bool(dados.get('mostrar_sensor_3d', True)))

    def salvar_config(self, dados):
        dados['mostrar_sensor_3d'] = self.mostrar_sensor.isChecked()
