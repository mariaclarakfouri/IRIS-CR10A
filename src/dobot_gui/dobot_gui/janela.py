"""Janela principal da plataforma: a base comum a todos os módulos.

Cuida de: conexão com o robô, velocidade, posição home, leitura periódica da
posição, parar movimento, emergência, mensagens, painel de progresso,
registro e configurações salvas. Os módulos (pasta `modulos/`) usam essa base
para mover o robô — veja modulos/base.py.
"""
import datetime
import json
import os
import re
import threading
import time

from PyQt5.QtCore import QSize, Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QButtonGroup,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMenu,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSlider,
    QStackedWidget,
    QVBoxLayout,
    QWidget,
)

from .constantes import (
    APP_NOME,
    APP_SUBTITULO,
    APP_VERSION,
    GET_JOINTS_TIMEOUT_SECONDS,
    HOME_JOINTS,
    HOME_TOLERANCE_DEG,
    JOINT_NAMES,
    LOG_HISTORY_LINES,
    LOG_PATH,
    POLL_FAILS_BEFORE_LOST,
    POLL_TIMEOUT_SECONDS,
    ROBOT_IP,
    ROBOT_STATES,
    SETTINGS_PATH,
    VELOCIDADE_INICIAL,
    VELOCIDADE_MAXIMA_PERMITIDA,
)
from .homes import HOME_PADRAO, LISTA_ABAS, NOME_HOME, CartaoHome, texto_juntas
from .modulos import MODULO_REGISTRO, MODULOS
from .tema import COR, app_icon, make_icon
from .widgets import (
    Alerta,
    Chip,
    PainelAtividade,
    PontoEstado,
    com_icone,
    rotulo,
    separador_vertical,
)


class JanelaPrincipal(QWidget):
    connect_result = pyqtSignal(bool, str)
    speed_result = pyqtSignal(bool, str)
    joints_polled = pyqtSignal(bool, str, object)
    emergency_stop_result = pyqtSignal(bool, str)
    stop_motion_result = pyqtSignal(bool, str)
    disconnect_result = pyqtSignal(bool, str)
    clear_alarm_result = pyqtSignal(bool, str)
    # Vindos das threads de fundo dos ensaios (a interface só pode ser mexida
    # na thread dela, então tudo passa por sinais).
    progresso = pyqtSignal(object)        # dict, ver PainelAtividade.atualizar
    acao_resultado = pyqtSignal(bool, str)
    log_message = pyqtSignal(str)

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        # Uma home por aba (ver homes.py). A padrão é a do Controle Juntas.
        self.homes = {HOME_PADRAO: list(HOME_JOINTS)}
        self._last_polled = None
        modo = getattr(ros_node, 'modo', 'real')
        robot_ip = getattr(ros_node, 'robot_ip', ROBOT_IP)
        self.sim_mode = isinstance(modo, str) and modo == 'sim'
        self.robot_ip = robot_ip if isinstance(robot_ip, str) and robot_ip else ROBOT_IP

        self.setWindowTitle(f'{APP_NOME} — SIMULAÇÃO (gêmeo virtual)' if self.sim_mode
                            else APP_NOME)
        self.setWindowIcon(app_icon())
        self.resize(1200, 820)
        self.setMinimumSize(980, 640)

        for sinal, slot in (
                (self.connect_result, self._on_connect_result),
                (self.speed_result, self._on_speed_result),
                (self.joints_polled, self._on_joints_polled),
                (self.emergency_stop_result, self._on_emergency_stop_result),
                (self.stop_motion_result, self._on_stop_motion_result),
                (self.disconnect_result, self._on_disconnect_result),
                (self.clear_alarm_result, self._on_clear_alarm_result),
                (self.progresso, self._on_progresso),
                (self.acao_resultado, self._on_acao_resultado),
                (self.log_message, self.log)):
            sinal.connect(slot)

        # Pedido de "Parar movimento": interrompe também as ESPERAS entre os
        # movimentos de uma sequência, não só o movimento em andamento.
        self._parar_evento = threading.Event()
        # A emergência também interrompe as esperas, mas NÃO é uma parada
        # normal: o robô fica desconectado e o aviso tem de dizer isso.
        self._emergencia_acionada = False
        # Texto do passo atual de uma sequência; o feedback do servidor
        # ("faltam X°") é mostrado DEPOIS dele, em vez de apagá-lo.
        self._progresso_prefixo = None
        self._action_running = False
        self._robot_state = 'disconnected'

        self._montar()
        self._iniciar_temporizadores()

        self._load_log_history()
        self.log('Interface iniciada.')
        self._set_robot_state('disconnected')
        self._update_pose(None)
        self._carregar_config()

    # ==================================================================
    # Montagem da janela
    # ==================================================================

    def _montar(self):
        raiz = QVBoxLayout(self)
        raiz.setContentsMargins(0, 0, 0, 0)
        raiz.setSpacing(0)
        raiz.addWidget(self._montar_cabecalho())
        raiz.addWidget(self._montar_faixa_modo())

        self.modulos = [cls(self) for cls in MODULOS]
        self.registro = MODULO_REGISTRO(self)
        self._cartoes_home = []
        for m in self.modulos:
            if m.CHAVE_HOME:
                card = CartaoHome(self, m.CHAVE_HOME, m.TEXTO_HOME)
                m.cartao_home = card
                self._cartoes_home.append(card)
                m.posicionar_cartao_home(card)
            for extra in m.cartoes_extras_home():
                self._cartoes_home.append(extra)
        raiz.addWidget(self._montar_barra_robo())
        raiz.addWidget(self._montar_navegacao())
        raiz.addWidget(self._montar_conteudo(), 1)
        raiz.addWidget(self._montar_rodape())

    def _montar_cabecalho(self):
        """Faixa grafite: identidade, estado e segurança."""
        cab = QFrame()
        cab.setObjectName('cabecalho')
        cab.setFixedHeight(56)
        self._cabecalho = cab
        lay = QHBoxLayout(cab)
        lay.setContentsMargins(24, 0, 24, 0)
        lay.setSpacing(12)

        icone = QLabel()
        icone.setPixmap(app_icon().pixmap(32, 32))
        lay.addWidget(icone)
        titulos = QVBoxLayout()
        titulos.setSpacing(1)
        titulos.addWidget(rotulo(APP_NOME, 'appTitulo'))
        self._subtitulo = rotulo(APP_SUBTITULO, 'appSubtitulo')
        titulos.addWidget(self._subtitulo)
        lay.addLayout(titulos)
        lay.addSpacing(8)

        self.chip_estado = Chip()
        lay.addWidget(self.chip_estado, 0, Qt.AlignVCenter)
        lay.addStretch()

        self.connect_btn = com_icone(QPushButton('Conectar'), 'power')
        self.connect_btn.setObjectName('headerConectar')
        self.connect_btn.setProperty('connected', False)
        self.connect_btn.clicked.connect(self.on_connect_clicked)
        self.disconnect_btn = com_icone(QPushButton('Desconectar'), 'x', COR['escuro_texto'])
        self.disconnect_btn.setObjectName('headerBotao')
        self.disconnect_btn.setEnabled(False)
        self.disconnect_btn.clicked.connect(self.on_disconnect_clicked)
        self.clear_alarm_btn = com_icone(QPushButton('Limpar alarme'), 'warning', '#fbbf24')
        self.clear_alarm_btn.setObjectName('headerAlarme')
        self.clear_alarm_btn.clicked.connect(self.on_clear_alarm_clicked)
        for b in (self.connect_btn, self.disconnect_btn, self.clear_alarm_btn):
            lay.addWidget(b)
        lay.addSpacing(16)       # distância deliberada entre os controles e a emergência
        self.stop_btn = com_icone(QPushButton('EMERGÊNCIA'), 'octagon')
        self.stop_btn.setObjectName('emergencia')
        self.stop_btn.setToolTip('Para o robô imediatamente e o desabilita. Para '
                                 'voltar: Limpar alarme e Conectar.')
        self.stop_btn.clicked.connect(self.on_emergency_stop_clicked)
        # A emergência nunca encolhe, qualquer que seja a largura da janela.
        self.stop_btn.ensurePolished()
        self.stop_btn.setMinimumWidth(self.stop_btn.sizeHint().width())
        lay.addWidget(self.stop_btn)
        return cab

    def _montar_faixa_modo(self):
        """Faixa estrutural sob o cabeçalho: diz UMA vez a fonte dos comandos."""
        faixa = QLabel('MODO SIMULAÇÃO — os comandos movem só o GÊMEO VIRTUAL no '
                       'Gazebo. O robô real não é tocado.' if self.sim_mode
                       else 'ROBÔ REAL — os comandos movem o robô físico.')
        faixa.setObjectName('faixaModo')
        faixa.setProperty('modo', 'sim' if self.sim_mode else 'real')
        faixa.setFixedHeight(28)
        faixa.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        self.faixa_modo = faixa
        return faixa

    def _montar_barra_robo(self):
        """Destino da conexão e controles do robô sempre visíveis
        (velocidade e Home)."""
        barra = QFrame()
        barra.setObjectName('barraRobo')
        barra.setFixedHeight(48)
        lay = QHBoxLayout(barra)
        lay.setContentsMargins(24, 8, 24, 8)
        lay.setSpacing(12)

        lay.addWidget(rotulo('Gêmeo virtual' if self.sim_mode else f'IP {self.robot_ip}',
                             'robotIp'))
        lay.addSpacing(8)
        lay.addWidget(separador_vertical())
        lay.addSpacing(8)

        lay.addWidget(rotulo('Velocidade', 'ladoRotulo'))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, VELOCIDADE_MAXIMA_PERMITIDA)
        self.speed_slider.setValue(VELOCIDADE_INICIAL)
        self.speed_slider.setFixedWidth(160)
        self.speed_slider.setEnabled(False)
        self.speed_slider.valueChanged.connect(self.on_speed_slider_moved)
        lay.addWidget(self.speed_slider)
        self.speed_value_label = rotulo(f'{VELOCIDADE_INICIAL}%', 'ladoValor')
        self.speed_value_label.setMinimumWidth(44)
        self.speed_value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        lay.addWidget(self.speed_value_label)
        lay.addWidget(rotulo(f'Limite de segurança: {VELOCIDADE_MAXIMA_PERMITIDA}%', 'ladoDica'))

        lay.addSpacing(8)
        lay.addWidget(separador_vertical())
        lay.addSpacing(8)
        # Um menu com a home de cada aba.
        self.home_btn = com_icone(QPushButton('Ir para Home'), 'home', COR['texto'])
        self.home_btn.setObjectName('secundario')
        self.home_btn.setEnabled(False)
        self._menu_homes = QMenu(self.home_btn)
        self._menu_homes.aboutToShow.connect(self._montar_menu_homes)
        self.home_btn.setMenu(self._menu_homes)
        self.home_btn.setMinimumWidth(self.home_btn.sizeHint().width() + 24)   # espaço da setinha
        lay.addWidget(self.home_btn)
        lay.addStretch()
        return barra

    def _montar_navegacao(self):
        """Navegação horizontal no topo: módulos, separador e Registro."""
        faixa = QFrame()
        faixa.setObjectName('faixaNav')
        faixa.setFixedHeight(44)
        lay = QHBoxLayout(faixa)
        lay.setContentsMargins(16, 0, 16, 0)
        lay.setSpacing(0)

        self._nav = QButtonGroup(self)
        self._nav.setExclusive(True)
        self._paginas = []

        def item(modulo):
            b = QPushButton(f'  {modulo.NOME}')
            b.setObjectName('navItem')
            b.setCheckable(True)
            b.setCursor(Qt.PointingHandCursor)
            b.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Expanding)
            b.setIcon(make_icon(modulo.ICONE, COR['texto_fraco']))
            b.setIconSize(QSize(16, 16))
            idx = len(self._paginas)
            self._paginas.append(modulo)
            b.clicked.connect(lambda _c, i=idx: self.mostrar_pagina(i))
            self._nav.addButton(b, idx)
            lay.addWidget(b)

        for m in self.modulos:
            item(m)
        lay.addSpacing(8)
        lay.addWidget(separador_vertical())
        lay.addSpacing(8)
        item(self.registro)
        lay.addStretch()
        return faixa

    def _montar_conteudo(self):
        conteudo = QWidget()
        conteudo.setObjectName('conteudo')
        lay = QVBoxLayout(conteudo)
        lay.setContentsMargins(24, 16, 24, 16)
        lay.setSpacing(12)

        self.alerta = Alerta()
        lay.addWidget(self.alerta)
        self.atividade = PainelAtividade()
        self.atividade.parar_clicado.connect(self.on_stop_motion_clicked)
        lay.addWidget(self.atividade)

        self.paginas = QStackedWidget()
        for modulo in self._paginas:
            pagina = QWidget()
            pagina.setObjectName('pagina')
            pl = QVBoxLayout(pagina)
            pl.setContentsMargins(0, 0, 8, 0)
            pl.setSpacing(4)
            pl.addWidget(rotulo(modulo.NOME, 'paginaTitulo'))
            desc = rotulo(modulo.DESCRICAO, 'paginaDescricao')
            desc.setWordWrap(True)
            pl.addWidget(desc)
            pl.addSpacing(12)
            pl.addWidget(modulo, 1)
            rolagem = QScrollArea()
            rolagem.setObjectName('rolagem')
            rolagem.setWidgetResizable(True)
            rolagem.setFrameShape(QFrame.NoFrame)
            rolagem.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
            rolagem.setWidget(pagina)
            self.paginas.addWidget(rolagem)
        lay.addWidget(self.paginas, 1)
        self.mostrar_pagina(0)
        return conteudo

    def _montar_rodape(self):
        rod = QFrame()
        rod.setObjectName('rodape')
        lay = QHBoxLayout(rod)
        lay.setContentsMargins(24, 6, 24, 6)
        lay.setSpacing(12)
        self.ponto_estado = PontoEstado()
        self.texto_estado = rotulo('', 'rodapeTexto')
        lay.addWidget(self.ponto_estado)
        lay.addWidget(self.texto_estado)
        lay.addWidget(separador_vertical())
        self.rodape_velocidade = rotulo('', 'rodapeTexto')
        lay.addWidget(self.rodape_velocidade)
        lay.addWidget(separador_vertical())
        escudo = QLabel()
        escudo.setPixmap(make_icon('shield', COR['sucesso']).pixmap(14, 14))
        lay.addWidget(escudo)
        seguranca = rotulo('Segurança ativa', 'rodapeTexto')
        celula = getattr(self.ros_node, 'ferramenta', '') == 'celula_fa7155'
        seguranca.setToolTip('Verificação de segurança ligada, com '
                             + ('a célula de carga FA7155 e a ponteira' if celula
                                else 'o sensor S02') + ' no modelo.')
        lay.addWidget(seguranca)
        lay.addWidget(separador_vertical())
        self.pose_label = rotulo('', 'rodapeMono')
        lay.addWidget(self.pose_label)
        self._reservar_largura_pose()
        lay.addStretch()
        self.clock_label = rotulo('', 'rodapeTexto')
        lay.addWidget(self.clock_label)
        sep_versao = separador_vertical()
        lay.addWidget(sep_versao)
        versao = rotulo(f'v{APP_VERSION}', 'rodapeTexto')
        lay.addWidget(versao)
        # Em janela estreita, relógio e versão saem antes de qualquer outro item.
        self._rodape = rod
        self._rodape_opcionais = (self.clock_label, sep_versao, versao)
        self._rodape_largura_toda = None
        self._atualizar_rodape_velocidade()
        return rod

    def _iniciar_temporizadores(self):
        self._speed_debounce = QTimer(self)
        self._speed_debounce.setSingleShot(True)
        self._speed_debounce.setInterval(150)
        self._speed_debounce.timeout.connect(self._send_pending_speed)
        self._pending_speed = None
        # O robô guarda a última velocidade que recebeu (do tablet, de outra
        # sessão...), que pode ser bem maior que a mostrada aqui. Por isso, ao
        # conectar, a velocidade do controle é enviada e os movimentos só são
        # liberados depois que o robô confirma.
        self._speed_confirmed = False

        self._joints_poll_timer = QTimer(self)
        self._joints_poll_timer.setInterval(400)
        self._joints_poll_timer.timeout.connect(self._poll_joints)
        self._last_polled = None
        self._joints_poll_busy = False
        self._poll_started = 0.0
        self._poll_fail_count = 0

        self._clock_timer = QTimer(self)
        self._clock_timer.setInterval(1000)
        self._clock_timer.timeout.connect(self._tick_clock)
        self._clock_timer.start()
        self._tick_clock()

        self._tooltip_timer = QTimer(self)
        self._tooltip_timer.setInterval(300)
        self._tooltip_timer.timeout.connect(self._refresh_tooltips)
        self._tooltip_timer.start()

    def mostrar_pagina(self, indice):
        self.paginas.setCurrentIndex(indice)
        botao = self._nav.button(indice)
        if botao is not None:
            botao.setChecked(True)
        for b in self._nav.buttons():
            b.setIcon(make_icon(self._paginas[self._nav.id(b)].ICONE,
                                COR['texto'] if b.isChecked() else COR['texto_fraco']))

    def modulo(self, nome):
        for m in self._paginas:
            if m.NOME == nome:
                return m
        return None

    # ==================================================================
    # Configurações
    # ==================================================================

    def _carregar_config(self):
        try:
            with open(SETTINGS_PATH, 'r') as f:
                dados = json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return
        homes = dados.get('homes')
        if isinstance(homes, dict):
            for chave, juntas in homes.items():
                if chave in NOME_HOME and isinstance(juntas, list) and len(juntas) == 6:
                    self.homes[chave] = [float(v) for v in juntas]
        else:
            # Configuração antiga, com uma home só: vira a padrão.
            home = dados.get('home_joints')
            if isinstance(home, list) and len(home) == len(HOME_JOINTS):
                self.homes[HOME_PADRAO] = home
        for m in self._paginas:
            m.carregar_config(dados)
        for card in self._cartoes_home:
            card.recarregar()

    def salvar_config(self):
        dados = {'homes': {k: list(v) for k, v in self.homes.items()},
                 'home_joints': list(self.home_joints)}
        for m in self._paginas:
            m.salvar_config(dados)
        try:
            os.makedirs(os.path.dirname(SETTINGS_PATH), exist_ok=True)
            with open(SETTINGS_PATH, 'w') as f:
                json.dump(dados, f)
        except OSError:
            pass

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._ajustar_cabecalho()
        self._ajustar_rodape()

    def _ajustar_cabecalho(self):
        # Em janela estreita, o subtítulo sai em vez de ficar cortado. A largura
        # necessária muda com o selo de estado, por isso é medida de novo a cada vez.
        if not hasattr(self, '_subtitulo'):
            return
        self._subtitulo.setVisible(True)
        precisa = self._cabecalho.minimumSizeHint().width()
        self._subtitulo.setVisible(self.width() >= precisa)

    def _ajustar_rodape(self):
        if self._rodape_largura_toda is None:
            # Largura de que o rodapé precisa com tudo visível (a posição já tem
            # a largura do pior caso reservada, então o valor não muda depois).
            self._rodape_largura_toda = self._rodape.minimumSizeHint().width()
        cabe_tudo = self.width() >= self._rodape_largura_toda
        for w in self._rodape_opcionais:
            w.setVisible(cabe_tudo)

    def closeEvent(self, event):
        self.log('Interface encerrada.')
        self.salvar_config()
        super().closeEvent(event)

    # ==================================================================
    # Estado, mensagens e registro
    # ==================================================================

    def _set_robot_state(self, state):
        self._robot_state = state
        texto, cor, _dica = ROBOT_STATES[state]
        self.chip_estado.definir(texto, cor, '#0b1220' if state in ('alarm',) else '#ffffff')
        self.ponto_estado.definir(cor)
        self.texto_estado.setText(texto)
        self._ajustar_cabecalho()
        if state in ('disconnected', 'alarm', 'emergency'):
            self._update_pose(None)
        self._refresh_tooltips()

    def _update_pose(self, joints):
        if not joints:
            self.pose_label.setText('Posição  —')
            return
        self.pose_label.setText('   '.join(
            f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, joints)))

    def _reservar_largura_pose(self):
        """A posição das 6 juntas nunca é cortada nem encosta no relógio: reserva a
        largura do texto mais comprido possível (ex.: "J1 -179.9°" em cada junta)."""
        self.pose_label.ensurePolished()
        maior = '   '.join(f'{n} -179.9°' for n in JOINT_NAMES)
        self.pose_label.setMinimumWidth(self.pose_label.fontMetrics().horizontalAdvance(maior) + 4)

    def _atualizar_rodape_velocidade(self):
        self.rodape_velocidade.setText(f'Velocidade {self.speed_slider.value()}%')

    def _controles_de_movimento(self):
        ws = [self.speed_slider, self.home_btn]
        for m in self._paginas:
            ws += m.widgets_de_movimento()
        return ws

    def _refresh_tooltips(self):
        """Nos controles bloqueados, explica por que estão bloqueados."""
        dica = ROBOT_STATES[self._robot_state][2]
        for w in self._controles_de_movimento():
            w.setToolTip(dica if not w.isEnabled() else '')

    def _set_joint_controls_enabled(self, enabled):
        self.home_btn.setEnabled(enabled)
        for card in self._cartoes_home:
            card.definir_movimento_habilitado(enabled)
        for m in self._paginas:
            m.definir_movimento_habilitado(enabled)

    def confirmar(self, titulo, texto, texto_confirmar='Confirmar'):
        box = QMessageBox(self)
        box.setWindowTitle(titulo)
        box.setText(texto)
        box.setIcon(QMessageBox.Question)
        sim = box.addButton(texto_confirmar, QMessageBox.AcceptRole)
        nao = box.addButton('Cancelar', QMessageBox.RejectRole)
        nao.setObjectName('discreto')
        box.exec_()
        return box.clickedButton() is sim

    def log(self, texto, level='INFO'):
        """Anota no arquivo e no Registro. Só chamar da thread da interface
        (das threads de fundo, use log_de_fundo)."""
        agora = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        linha = f'{agora} [{level}] {texto}'
        try:
            os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
            with open(LOG_PATH, 'a', encoding='utf-8') as f:
                f.write(linha + '\n')
        except OSError:
            pass
        self.registro.adicionar(linha, level)

    def log_de_fundo(self, texto):
        self.log_message.emit(texto)

    def _load_log_history(self):
        try:
            with open(LOG_PATH, 'r', encoding='utf-8') as f:
                linhas = f.read().splitlines()[-LOG_HISTORY_LINES:]
        except OSError:
            return
        for linha in linhas:
            achou = re.search(r'\[(INFO|AVISO|ERRO)\]', linha)
            self.registro.adicionar(linha, achou.group(1) if achou else 'INFO', apagada=True)
        if linhas:
            self.registro.adicionar('──────── nova sessão ────────', 'INFO', apagada=True)

    def _tick_clock(self):
        self.clock_label.setText(datetime.datetime.now().strftime('%d/%m/%Y  %H:%M:%S'))

    # ==================================================================
    # Conexão
    # ==================================================================

    def _repintar_conectar(self):
        self.connect_btn.style().unpolish(self.connect_btn)
        self.connect_btn.style().polish(self.connect_btn)

    def on_connect_clicked(self):
        self.connect_btn.setEnabled(False)
        self.alerta.mostrar('Conectando...')
        threading.Thread(
            target=self.ros_node.call_connect,
            args=(lambda ok, msg: self.connect_result.emit(ok, msg),),
            daemon=True,
        ).start()

    def _on_connect_result(self, ok, msg):
        if ok:
            self.connect_btn.setText('Conectado')
            self.connect_btn.setProperty('connected', True)
            self.connect_btn.setEnabled(False)
            self.disconnect_btn.setEnabled(True)
            self.speed_slider.setEnabled(True)
            # Movimentos ficam travados até o robô confirmar a velocidade
            # (liberados em _on_speed_result).
            self._speed_confirmed = False
            self._set_joint_controls_enabled(False)
            self._joints_poll_timer.start()
            self._poll_joints()
            self._set_robot_state('connected')
            self._poll_fail_count = 0
            alvo = 'gêmeo virtual (SIMULAÇÃO)' if self.sim_mode else f'robô ({self.robot_ip})'
            self.log(f'Conectado ao {alvo}. {msg}')
            speed = self.speed_slider.value()
            self.alerta.mostrar(f'Ajustando a velocidade do robô para {speed}%...')
            self._pending_speed = speed
            self._send_pending_speed()
        else:
            self.alerta.mostrar(f'Falha ao conectar: {msg}', 'erro')
            self.connect_btn.setEnabled(True)
            self.log(f'Falha ao conectar: {msg}', 'ERRO')
            if self._robot_state not in ('alarm', 'emergency'):
                self._set_robot_state('disconnected')
        self._repintar_conectar()

    def on_clear_alarm_clicked(self):
        self.clear_alarm_btn.setEnabled(False)
        self.alerta.mostrar('Limpando alarme...')
        threading.Thread(
            target=self.ros_node.call_clear_alarm,
            args=(lambda ok, msg: self.clear_alarm_result.emit(ok, msg),),
            daemon=True,
        ).start()

    def _on_clear_alarm_result(self, ok, msg):
        self.clear_alarm_btn.setEnabled(True)
        if ok:
            self.alerta.mostrar('Alarme limpo. Clique em Conectar para reabilitar o robô.',
                                'aviso')
            if self._robot_state in ('disconnected', 'emergency', 'alarm'):
                self._set_robot_state('alarm')
            self.log(f'Alarme limpo. {msg}')
        else:
            self.log(f'Falha ao limpar alarme: {msg}', 'ERRO')
            self.alerta.mostrar(f'Falha ao limpar alarme: {msg}', 'erro')

    def on_disconnect_clicked(self):
        self.disconnect_btn.setEnabled(False)
        self.alerta.mostrar('Desconectando...')
        threading.Thread(
            target=self.ros_node.call_disconnect,
            args=(lambda ok, msg: self.disconnect_result.emit(ok, msg),),
            daemon=True,
        ).start()

    def _on_disconnect_result(self, ok, msg):
        if ok:
            self._joints_poll_timer.stop()
            if 'falhou' in msg:
                self.alerta.mostrar('Desconectado, mas o robô pode não ter sido '
                                    'desabilitado.', 'aviso')
            else:
                self.alerta.mostrar('Desconectado.')
            self.connect_btn.setText('Conectar')
            self.connect_btn.setProperty('connected', False)
            self.connect_btn.setEnabled(True)
            self.disconnect_btn.setEnabled(False)
            self._set_joint_controls_enabled(False)
            self.speed_slider.setEnabled(False)
            self._set_robot_state('disconnected')
            self.log(f'Desconectado. {msg}')
        else:
            self.log(f'Falha ao desconectar: {msg}', 'ERRO')
            self.alerta.mostrar(f'Falha ao desconectar: {msg}', 'erro')
            self.disconnect_btn.setEnabled(True)
        self._repintar_conectar()

    def marcar_desconectado(self):
        self._joints_poll_timer.stop()
        self._poll_fail_count = 0
        self._set_robot_state('disconnected')
        self.connect_btn.setText('Conectar')
        self.connect_btn.setProperty('connected', False)
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self._set_joint_controls_enabled(False)
        self.speed_slider.setEnabled(False)
        self._repintar_conectar()

    # --- conexão perdida ---

    def _register_poll_failure(self, reason):
        self._poll_fail_count += 1
        if (self._poll_fail_count >= POLL_FAILS_BEFORE_LOST
                and self._robot_state in ('connected', 'moving')):
            self._set_robot_state('lost')
            self._set_joint_controls_enabled(False)
            self.speed_slider.setEnabled(False)
            self.log(f'Sem resposta do robô ({reason}). Verifique o cabo e a rede.', 'ERRO')
            self.alerta.mostrar('Sem resposta do robô. Verifique o cabo e a rede; se não '
                                'voltar, desconecte e conecte de novo.', 'erro')

    def _register_poll_success(self):
        self._poll_fail_count = 0
        if self._robot_state == 'lost':
            self._set_robot_state('connected')
            self._set_joint_controls_enabled(self._speed_confirmed)
            self.speed_slider.setEnabled(True)
            self.log('Comunicação com o robô restabelecida.', 'AVISO')
            self.alerta.mostrar('Comunicação com o robô restabelecida.', 'ok')

    # ==================================================================
    # Velocidade
    # ==================================================================

    def on_speed_slider_moved(self, value):
        self.speed_value_label.setText(f'{value}%')
        self._atualizar_rodape_velocidade()
        self._pending_speed = value
        self._speed_debounce.start()

    def _send_pending_speed(self):
        if self._pending_speed is None:
            return
        value = self._pending_speed
        threading.Thread(
            target=self.ros_node.call_set_speed,
            args=(value, lambda ok, msg: self.speed_result.emit(ok, msg)),
            daemon=True,
        ).start()

    def _on_speed_result(self, ok, msg):
        if not ok:
            self.log(f'Erro ao ajustar velocidade: {msg}', 'ERRO')
            texto = f'Erro ao ajustar velocidade: {msg}'
            if not self._speed_confirmed:
                texto += ('\nMovimentos bloqueados até a velocidade ser aceita. '
                          'Mexa no controle de velocidade para tentar de novo.')
            self.alerta.mostrar(texto, 'erro')
            return
        if not self._speed_confirmed:
            self._speed_confirmed = True
            self.log(f'Velocidade do robô confirmada: {self.speed_slider.value()}%.')
            if self._robot_state in ('connected', 'moving') and not self._action_running:
                self._set_joint_controls_enabled(True)
                self.alerta.esconder()

    # ==================================================================
    # Leitura periódica da posição
    # ==================================================================

    def _poll_joints(self):
        if self._joints_poll_busy:
            # Sem resposta há tempo demais = robô/backend não está respondendo
            # (durante Home/ensaios a leitura espera na fila, então não conta).
            if (self._action_running
                    or time.time() - self._poll_started <= POLL_TIMEOUT_SECONDS):
                return
            self._joints_poll_busy = False
            self._register_poll_failure('sem resposta')
        self._joints_poll_busy = True
        self._poll_started = time.time()
        threading.Thread(
            target=self.ros_node.call_get_joints,
            args=(lambda ok, msg, joints: self.joints_polled.emit(ok, msg, joints),),
            daemon=True,
        ).start()

    def _on_joints_polled(self, ok, msg, joints):
        self._joints_poll_busy = False
        if not ok or joints is None:
            if 'não conectado' in (msg or ''):
                self.log('O backend informou que o robô não está conectado.', 'ERRO')
                self.marcar_desconectado()
            else:
                self._register_poll_failure(msg or 'falha na leitura')
            return
        self._register_poll_success()

        previous = self._last_polled
        self._last_polled = list(joints)
        self._update_pose(joints)
        moving = previous is not None and any(
            abs(a - b) > 0.3 for a, b in zip(previous, joints))
        if self._robot_state in ('connected', 'moving') and not self._action_running:
            self._set_robot_state('moving' if moving else 'connected')
        robo_parado = previous is not None and not moving
        for m in self._paginas:
            m.ao_ler_juntas(joints, robo_parado)

    # ==================================================================
    # Ações (Home, ensaios) — base usada pelos módulos
    # ==================================================================

    def iniciar_acao(self, titulo, texto_log):
        """Começa uma ação que move o robô: trava os controles e mostra o
        painel de atividade com o botão Parar."""
        self._parar_evento.clear()
        self._emergencia_acionada = False
        self._progresso_prefixo = None
        self._action_running = True
        self.log(texto_log)
        self._set_robot_state('moving')
        self._set_joint_controls_enabled(False)
        self.speed_slider.setEnabled(False)
        self.alerta.esconder()
        self.atividade.iniciar(titulo)

    def terminar_acao(self, ok, msg):
        """Chamado pela thread da ação quando ela termina (qualquer thread)."""
        self.acao_resultado.emit(ok, msg)

    def _on_acao_resultado(self, ok, msg):
        self._action_running = False
        self.atividade.terminar()
        if ok:
            self.log(msg)
            self.alerta.mostrar(msg, 'ok')
        elif self._parada_normal():
            self.log(f'Ação parada pelo usuário ({msg}).')
            self.alerta.mostrar('Movimento parado. Robô conectado e pronto.', 'aviso')
        else:
            self.log(f'Erro: {msg}', 'ERRO')
            self.alerta.mostrar(f'Erro: {msg}', 'erro')
        if self._robot_state in ('emergency', 'disconnected', 'alarm'):
            return  # emergência/desconexão no meio da ação: continua bloqueado
        self._set_joint_controls_enabled(True)
        self.speed_slider.setEnabled(True)
        self._set_robot_state('connected')

    def _on_progresso(self, dados):
        self.atividade.atualizar(dados)

    def definir_prefixo(self, texto):
        self._progresso_prefixo = texto

    def parado(self):
        return self._parar_evento.is_set()

    def _parada_normal(self):
        """True se a ação terminou por causa do botão Parar (não emergência)."""
        return self._parar_evento.is_set() and not self._emergencia_acionada

    def esperar(self, segundos):
        """time.sleep que acorda na hora se pedirem para parar."""
        self._parar_evento.wait(segundos)

    def mover_e_esperar(self, joints, ao_status=None):
        """Chama a action move_joint e bloqueia (na thread de fundo) até o
        resultado. Todo movimento passa pela verificação de segurança do
        servidor. `ao_status(texto)` recebe cada aviso de andamento do
        servidor ("Verificando segurança do caminho...", "Movendo...")."""
        if self.parado():
            return False, 'Movimento parado pelo usuário.'
        done_event = threading.Event()
        outcome = {}

        def on_feedback(status):
            prefixo = self._progresso_prefixo
            self.progresso.emit({'detalhe': f'{prefixo} · {status}' if prefixo else status})
            if ao_status is not None:
                ao_status(status)

        def on_done(ok, msg):
            outcome['ok'] = ok
            outcome['msg'] = msg
            done_event.set()

        self.ros_node.call_move_joint(list(joints), on_feedback, on_done)
        done_event.wait()
        return outcome['ok'], outcome['msg']

    def seguir_trajetoria_e_esperar(self, pontos, periodo_s, ao_avancar=None):
        """Percurso contínuo (movimento em 8): o robô passa pelos pontos sem
        parar. Bloqueia (na thread de fundo) até o resultado. O servidor confere
        velocidade e segurança do percurso inteiro antes de mexer o robô.
        `ao_avancar(indice)` é chamado a cada aviso de andamento."""
        if self.parado():
            return False, 'Movimento parado pelo usuário.'
        done_event = threading.Event()
        outcome = {}

        def on_feedback(status, indice):
            prefixo = self._progresso_prefixo
            self.progresso.emit({'detalhe': f'{prefixo} · {status}' if prefixo else status})
            if ao_avancar is not None:
                ao_avancar(indice)

        def on_done(ok, msg):
            outcome['ok'] = ok
            outcome['msg'] = msg
            done_event.set()

        self.ros_node.call_seguir_trajetoria(pontos, periodo_s, on_feedback, on_done)
        done_event.wait()
        return outcome['ok'], outcome['msg']

    def mover_ponta_e_esperar(self, deslocamento_mm, velocidade_mms, parar_no_contato=False,
                              limiar_n=1.0, tocar_mesa=False, altura_mm=None):
        """Ponteira em linha reta, mantendo a orientação (medição da mesa).
        Bloqueia (na thread de fundo) até o resultado. Devolve (ok, msg,
        resultado), com resultado = dict(contato, percorrido_mm, juntas,
        ponta_mm, forca_n) ou None. Com `altura_mm`, sobe/desce na vertical
        até essa altura da ponteira (deslocamento_mm é ignorado)."""
        if self.parado():
            return False, 'Movimento parado pelo usuário.', None
        done_event = threading.Event()
        outcome = {}

        def on_feedback(status, percorrido, forca):
            prefixo = self._progresso_prefixo
            texto = f'{status} {percorrido:.1f} mm · {forca:.2f} N'
            self.progresso.emit({'detalhe': f'{prefixo} · {texto}' if prefixo else texto})

        def on_done(ok, msg, resultado):
            outcome.update(ok=ok, msg=msg, resultado=resultado)
            done_event.set()

        self.ros_node.call_mover_ponta(deslocamento_mm, velocidade_mms, parar_no_contato,
                                       limiar_n, tocar_mesa, on_feedback, on_done, altura_mm)
        done_event.wait()
        return outcome['ok'], outcome['msg'], outcome['resultado']

    def ler_juntas_bloqueando(self, timeout=GET_JOINTS_TIMEOUT_SECONDS):
        """Lê a posição real das 6 juntas, bloqueando (na thread de fundo).
        Devolve (ok, mensagem, juntas)."""
        done_event = threading.Event()
        outcome = {'ok': False, 'msg': 'sem resposta do robô', 'joints': None}

        def on_done(ok, msg, joints):
            outcome.update(ok=ok, msg=msg, joints=joints)
            done_event.set()

        self.ros_node.call_get_joints(on_done)
        done_event.wait(timeout)
        return outcome['ok'], outcome['msg'], outcome['joints']

    def ir_para_home_e_conferir(self):
        """Manda o robô para o home e CONFERE, lendo a posição real, que todas
        as 6 juntas chegaram. Os ensaios só começam se isto devolver True.

        A action move_joint já espera o robô chegar, mas se não conseguir ler a
        posição ela cai numa espera de tempo fixo e responde "ok" sem ter
        conferido nada; esta checagem fecha essa brecha."""
        self.progresso.emit({'detalhe': 'Indo para home...'})
        ok, msg = self.mover_e_esperar(list(self.home_joints))
        if not ok:
            return False, f'Falha ao ir para home: {msg}'

        self.progresso.emit({'detalhe': 'Conferindo se o robô está no home...'})
        ok, msg, joints = self.ler_juntas_bloqueando()
        if not ok or not joints or len(joints) != len(self.home_joints):
            return False, ('Não deu para confirmar que o robô está no home '
                           f'(falha ao ler a posição: {msg}).')
        fora = [
            f'J{i + 1} está em {atual:.1f}° (home: {alvo}°)'
            for i, (atual, alvo) in enumerate(zip(joints, self.home_joints))
            if abs((atual - alvo + 180.0) % 360.0 - 180.0) > HOME_TOLERANCE_DEG
        ]
        if fora:
            return False, 'O robô não está no home. ' + '; '.join(fora)
        self.log_de_fundo('Home conferido: ' + ', '.join(
            f'J{i + 1} {v:.1f}°' for i, v in enumerate(joints)))
        return True, ''

    # --- Home ---

    @property
    def home_joints(self):
        """A home padrão da plataforma = a do Controle Juntas."""
        return self.homes[HOME_PADRAO]

    @home_joints.setter
    def home_joints(self, valores):
        self.homes[HOME_PADRAO] = list(valores)

    def home_de(self, chave):
        """Home gravada de uma aba (lista de 6 graus) ou None."""
        h = self.homes.get(chave)
        return list(h) if h is not None else None

    def ultimas_juntas(self):
        return list(self._last_polled) if self._last_polled is not None else None

    def gravar_home(self, chave, juntas):
        self.homes[chave] = [float(v) for v in juntas]
        self.salvar_config()
        texto = f'{NOME_HOME[chave]} gravada: {texto_juntas(juntas)}'
        self.log(texto)
        self.alerta.mostrar(texto, 'ok')
        for card in self._cartoes_home:
            if card.chave in (chave, None):
                card.recarregar()
        for m in self._paginas:
            m.home_mudou(chave)

    def _montar_menu_homes(self):
        self._menu_homes.clear()
        for nome, chave in LISTA_ABAS:
            home = self.homes.get(chave)
            acao = self._menu_homes.addAction(nome if home is not None
                                              else f'{nome} (não gravada)')
            acao.setEnabled(home is not None)
            if home is not None:
                acao.setToolTip(texto_juntas(home))
                acao.triggered.connect(lambda _c, k=chave: self.ir_para_home(k))
        self._menu_homes.setToolTipsVisible(True)

    def ir_para_home(self, chave):
        home = self.home_de(chave)
        if home is None:
            self.alerta.mostrar(f'{NOME_HOME[chave]} ainda não foi gravada.', 'aviso')
            return
        if not self.confirmar('Ir para a home',
                              f'O robô vai se mover para a {NOME_HOME[chave]}:\n'
                              f'{texto_juntas(home)}', 'Ir para a home'):
            return
        self.iniciar_acao(f'Indo para a {NOME_HOME[chave]}',
                          f'Ir para a {NOME_HOME[chave]}: {texto_juntas(home)}')
        self.atividade.atualizar({'detalhe': 'Indo para a home...'})

        def correr():
            ok, msg = self.mover_e_esperar(home)
            self.terminar_acao(ok, f'Robô na {NOME_HOME[chave]}.' if ok else msg)

        threading.Thread(target=correr, daemon=True).start()

    # ==================================================================
    # Parar movimento e emergência
    # ==================================================================

    def on_stop_motion_clicked(self):
        self._parar_evento.set()
        self.atividade.parar_btn.setEnabled(False)
        self.log('Parar movimento acionado.', 'AVISO')
        self.atividade.atualizar({'detalhe': 'Parando o movimento...'})
        threading.Thread(
            target=self.ros_node.call_stop_motion,
            args=(lambda ok, msg: self.stop_motion_result.emit(ok, msg),),
            daemon=True,
        ).start()

    def _on_stop_motion_result(self, ok, msg):
        if ok:
            self.log(f'Robô parado (continua conectado). {msg}')
        else:
            self.log(f'Falha ao parar o movimento: {msg}. Se o robô ainda se '
                     'move, use o botão de EMERGÊNCIA.', 'ERRO')
            self.alerta.mostrar('Falha ao parar o movimento — se o robô ainda se move, '
                                'use o botão de EMERGÊNCIA.', 'erro')

    def on_emergency_stop_clicked(self):
        # Dispara imediatamente, sem checar nenhum estado — o STOP tem que
        # funcionar não importa o que mais esteja acontecendo na interface.
        self._emergencia_acionada = True
        self._parar_evento.set()
        self.atividade.parar_btn.setEnabled(False)
        self.log('BOTÃO DE EMERGÊNCIA acionado.', 'AVISO')
        self.alerta.mostrar('Enviando PARADA DE EMERGÊNCIA...', 'erro')
        threading.Thread(
            target=self.ros_node.call_emergency_stop,
            args=(lambda ok, msg: self.emergency_stop_result.emit(ok, msg),),
            daemon=True,
        ).start()

    def _on_emergency_stop_result(self, ok, msg):
        if ok:
            self.log(f'Parada de emergência enviada ao robô. Resposta: {msg}', 'AVISO')
            self.alerta.mostrar('PARADA DE EMERGÊNCIA acionada. Limpe o alarme e '
                                'reconecte para voltar a operar.', 'erro')
        else:
            self.log(f'FALHA ao enviar a parada de emergência: {msg}', 'ERRO')
            self.alerta.mostrar(f'FALHA ao enviar PARADA DE EMERGÊNCIA: {msg}', 'erro')

        self._joints_poll_timer.stop()
        self._set_robot_state('emergency')
        # Trava tudo que move o robô. Só libera de novo depois de reconectar
        # (o backend limpa o erro/reabilita o robô nesse momento).
        self._set_joint_controls_enabled(False)
        self.speed_slider.setEnabled(False)
        self.connect_btn.setText('Conectar')
        self.connect_btn.setProperty('connected', False)
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self._repintar_conectar()