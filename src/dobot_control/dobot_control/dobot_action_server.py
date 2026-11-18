import collections
import math
import os
import signal
import socket
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Empty
from std_srvs.srv import SetBool, Trigger

from dobot_interfaces.action import MoveJoint, MoverPonta, SeguirTrajetoria
from dobot_interfaces.srv import GetJoints, JogJoint, SetSpeed, VerificarCaminho

from .coleta import ColetaMixin

TEMPO_MOVIMENTO = 8  # segundos de espera fixa após cada MovJ
VELOCIDADE_MAXIMA_PERMITIDA = 50  # limite de segurança, em % da velocidade máxima do robô
SLEEP_STEP = 0.2  # granularidade da espera durante o movimento, pra poder abortar rápido
TOLERANCIA_GRAUS = 0.5  # o robô "chegou" quando todas as juntas estão a menos disso do destino
TEMPO_MAXIMO_MOVIMENTO = 180  # segurança: desiste de esperar o robô chegar depois disso (s)
COMMAND_TIMEOUT = 5.0  # segundos esperando a resposta do robô antes de considerar que ele parou de responder
TEMPO_PARADO_SEM_CHEGAR = 3.0  # se o robô fica parado esse tempo longe do destino, desiste (s)

# --- Percurso contínuo (ServoJ), usado pelo movimento em 8 ---
# O ServoJ NÃO respeita o SpeedFactor: o limite de velocidade é conferido aqui,
# ponto a ponto, contra VELOCIDADE_MAXIMA_PERMITIDA da velocidade máxima de
# cada junta (valores típicos da ficha do CR10, os mesmos do gêmeo).
VELOCIDADE_MAX_JUNTA_GRAUS_S = (120.0, 120.0, 150.0, 180.0, 180.0, 180.0)
ACELERACAO_MAX_GRAUS_S2 = 400.0  # barra saltos: percurso tem de ser suave
LIMITE_JUNTA_GRAUS = (180.0, 180.0, 160.0, 180.0, 180.0, 180.0)
PERIODO_SERVOJ_S = (0.02, 0.2)   # guia TCP/IP V4.5.1: t mínimo de 0,02 s
SERVOJ_AHEADTIME = 50
SERVOJ_GAIN = 500
MAX_PONTOS_TRAJETORIA = 40000
TOLERANCIA_INICIO_GRAUS = 1.0    # o 1º ponto tem de ser a posição atual
TOLERANCIA_FIM_GRAUS = 2.0
# --- Mover a ponteira em linha reta (medição da mesa) ---
PERIODO_PONTA_S = 0.05
VELOCIDADE_PONTA_MAX_MMS = 20.0          # movimento livre (subir, ir para o lado)
VELOCIDADE_CONTATO_MAX_MMS = 5.0         # descendo para encostar
LIMIAR_CONTATO_N = (0.2, 20.0)           # faixa aceita para "encostou"
FORCA_MAXIMA_N = 40.0                    # passou disto (em relação ao começo): para
FORCA_VELHA_S = 0.25                     # sem amostra da célula há mais que isso: para
JANELA_FORCA_BASE_S = 0.3
INCLINACAO_PONTEIRA_MAX_GRAUS = 20.0     # para encostar: a ponteira aponta para baixo


class DobotActionServer(ColetaMixin, Node):

    def __init__(self):
        super().__init__('dobot_action_server')

        self.declare_parameter('robot_ip', '192.168.5.2')
        self.declare_parameter('dashboard_port', 29999)
        # 'real' = robô de verdade pela rede. 'sim' = gêmeo virtual no Gazebo
        # (pacote dobot_twin): os mesmos comandos vão para um dashboard
        # simulado, e o robô real não é tocado.
        self.declare_parameter('modo', 'real')
        self.modo = str(self.get_parameter('modo').value).strip().lower()
        self._sim_dashboard = None
        if self.modo == 'sim':
            from dobot_twin.sim_dashboard import SimDashboard
            self._sim_dashboard = SimDashboard(
                self, callback_group=MutuallyExclusiveCallbackGroup())
            self.get_logger().warn(
                'MODO SIMULAÇÃO: os comandos vão para o gêmeo virtual, NÃO para o robô real.')
        elif self.modo != 'real':
            raise ValueError(f"modo deve ser 'real' ou 'sim', não {self.modo!r}")

        # Segurança: verificação geométrica de todo movimento (seguranca.py) e
        # proteções do controlador enviadas ao conectar. Se a configuração não
        # carregar, NENHUM movimento é aceito (falha para o lado seguro).
        self.declare_parameter('arquivo_seguranca', os.path.join(
            get_package_share_directory('dobot_control'), 'config', 'seguranca.yaml'))
        # Ferramenta na ponta: '' = a de geometria.ferramenta (sensor S02);
        # um nome = perfil da seção `ferramentas` do seguranca.yaml.
        self.ferramenta = str(self.declare_parameter('ferramenta', '').value).strip()
        self.seguranca = None
        self.cinematica = None
        self.profundidade_contato = 0.006
        self.config_seguranca = None
        self.erro_seguranca = None
        self._carregar_seguranca()
        # Último destino mandado ao robô: um MovJ novo entra na FILA do Dobot
        # e começa de onde o anterior termina, então os dois trechos são
        # verificados.
        self._ultimo_alvo = None
        # Velocidade pedida pela interface (%). É reenviada ao robô antes de cada
        # MovJ: o SpeedFactor é global e pode ser trocado por fora (tablet,
        # DobotStudio, religar o robô) sem o servidor ficar sabendo.
        self._velocidade_pct = None

        self.dashboard = None
        self.emergency_stop_flag = False
        # Modo arrastar (StartDrag): o braço é guiado com a mão. Enquanto ligado,
        # nenhum movimento por comando é aceito.
        self._arrastando = False
        # "Parar movimento" (sem emergência): cada pedido incrementa este
        # contador. Um movimento guarda o valor de quando começou; se mudou,
        # ele foi parado. Contador em vez de flag para um movimento novo nunca
        # "apagar" um pedido de parada que chegou no meio do caminho.
        self._parada_geracao = 0
        # O robô só aceita UMA conexão na porta do dashboard, então todo mundo
        # (movimento, velocidade, leitura de posição, STOP) precisa dividir a
        # mesma conexão. Esse lock evita que dois comandos se atropelem nela.
        self.dashboard_lock = threading.Lock()

        # O STOP de emergência fica num grupo de callback separado, processado
        # em outra thread — assim ele NUNCA fica esperando na fila atrás de um
        # movimento em andamento (que bloqueia sua própria thread com sleep).
        normal_group = MutuallyExclusiveCallbackGroup()
        emergency_group = MutuallyExclusiveCallbackGroup()
        # Movimento manual em tempo real: grupo próprio pra não esperar atrás de um
        # movimento longo (ação move_joint) que esteja rodando.
        jog_group = MutuallyExclusiveCallbackGroup()
        # Leitura da posição (mini robô 3D e cabeçalho): também em grupo próprio,
        # pra continuar atualizando durante Home/Rotação.
        read_group = MutuallyExclusiveCallbackGroup()

        self.connect_service = self.create_service(
            Trigger, 'connect', self.connect_callback, callback_group=normal_group)
        self.disconnect_service = self.create_service(
            Trigger, 'disconnect', self.disconnect_callback, callback_group=normal_group)
        self.clear_alarm_service = self.create_service(
            Trigger, 'clear_alarm', self.clear_alarm_callback, callback_group=normal_group)
        self.set_speed_service = self.create_service(
            SetSpeed, 'set_speed', self.set_speed_callback, callback_group=normal_group)
        self.get_joints_service = self.create_service(
            GetJoints, 'get_joints', self.get_joints_callback, callback_group=read_group)
        self._action_server = ActionServer(
            self,
            MoveJoint,
            'move_joint',
            execute_callback=self.execute_callback,
            callback_group=normal_group,
        )
        # Percurso contínuo (movimento em 8): mesmo grupo do move_joint, então
        # os dois nunca rodam ao mesmo tempo.
        self._trajetoria_server = ActionServer(
            self,
            SeguirTrajetoria,
            'seguir_trajetoria',
            execute_callback=self.seguir_trajetoria_callback,
            callback_group=normal_group,
        )
        self.jog_joint_service = self.create_service(
            JogJoint, 'jog_joint', self.jog_joint_callback, callback_group=jog_group)
        self.emergency_stop_service = self.create_service(
            Trigger, 'emergency_stop', self.emergency_stop_callback,
            callback_group=emergency_group)
        # Verificação de caminho sem mover (aba Análise de Juntas): grupo
        # próprio, para não esperar atrás de um movimento em andamento.
        self.verificar_caminho_service = self.create_service(
            VerificarCaminho, 'verificar_caminho', self.verificar_caminho_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        # Parada normal: grupo próprio, para nunca esperar atrás de um
        # movimento em andamento (nem atrás do STOP de emergência).
        self.stop_motion_service = self.create_service(
            Trigger, 'stop_motion', self.stop_motion_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        # Modo arrastar (mover o robô com a mão): grupo próprio, para o
        # desligar nunca esperar atrás de outra coisa.
        self.arrastar_service = self.create_service(
            SetBool, 'arrastar', self.arrastar_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        # Ponteira em linha reta, parando ao encostar (aba Alinhamento da Mesa):
        # mesmo grupo dos outros movimentos. A força chega num grupo próprio,
        # para continuar chegando durante o movimento.
        self._ponta_server = ActionServer(
            self, MoverPonta, 'mover_ponta', execute_callback=self.mover_ponta_callback,
            callback_group=normal_group)
        self._forca_lock = threading.Lock()
        self._forcas = collections.deque(maxlen=1000)     # (monotonic, [fx, fy, fz])
        self.create_subscription(
            WrenchStamped, '/celula/wrench_bruto', self._on_forca, 50,
            callback_group=MutuallyExclusiveCallbackGroup())

        # Coletas (aba Coletas): o robô real segue o explorador do cr10twin.
        self._iniciar_coleta(VELOCIDADE_MAX_JUNTA_GRAUS_S, VELOCIDADE_MAXIMA_PERMITIDA,
                             SERVOJ_AHEADTIME, SERVOJ_GAIN)

        self.get_logger().info('dobot_action_server pronto. Aguardando chamada em /connect.')

    def _carregar_seguranca(self):
        try:
            from .seguranca import Seguranca
            arquivo = self.get_parameter('arquivo_seguranca').value
            with open(arquivo, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            if self.ferramenta:
                perfis = config.get('ferramentas') or {}
                if self.ferramenta not in perfis:
                    raise ValueError(
                        f'ferramenta "{self.ferramenta}" não existe em seguranca.yaml '
                        f'(perfis: {", ".join(perfis) or "nenhum"}).')
                perfil = perfis[self.ferramenta]
                config['geometria']['ferramenta'] = {
                    'ativa': True, 'tamanho_mm': perfil['tamanho_mm'],
                    'centro_mm': perfil['centro_mm'],
                    'ponteira_mm': perfil.get('ponteira_mm')}
                config['controlador']['carga_kg'] = perfil['carga_kg']
                config['controlador']['centro_massa_mm'] = perfil['centro_massa_mm']
            nivel = int(config['controlador']['nivel_colisao'])
            if not 1 <= nivel <= 5:
                raise ValueError(
                    f'nivel_colisao = {nivel}: tem de ser de 1 a 5 (0 desliga a '
                    'detecção de colisão do robô, e isso não é aceito).')
            twin = get_package_share_directory('dobot_twin')
            self.seguranca = Seguranca(
                config,
                os.path.join(twin, 'urdf', 'cr10_robot.xacro'),
                os.path.join(twin, 'meshes', 'cr10'))
            self.config_seguranca = config
            from .cinematica import Cinematica
            self.cinematica = Cinematica(self.seguranca.modelo,
                                         self.seguranca.comprimento_ferramenta)
            self.profundidade_contato = float(
                config['geometria'].get('profundidade_contato_mm', 6.0)) / 1000.0
            ferr = (config['geometria'].get('ferramenta') or {}).get('ativa')
            self.get_logger().info(
                f'Segurança carregada de {arquivo}: '
                f'{len(self.seguranca.obstaculos)} obstáculos, '
                f'margem {config["geometria"]["margem_ambiente_mm"]} mm, '
                f'colisão nível {nivel}, carga {config["controlador"]["carga_kg"]} kg, '
                f'ferramenta {self.ferramenta or "sensor S02 (padrão)"} '
                f'{config["geometria"]["ferramenta"].get("tamanho_mm")} mm.')
            if not ferr:
                self.get_logger().warn(
                    'Segurança: a ferramenta/sensores na ponta NÃO está no modelo '
                    '(ferramenta.ativa = false em seguranca.yaml). Só o flange é '
                    'verificado.')
        except Exception as e:  # noqa: BLE001 — qualquer falha bloqueia movimento
            self.seguranca = None
            self.erro_seguranca = f'{type(e).__name__}: {e}'
            self.get_logger().error(
                f'SEGURANÇA NÃO CARREGOU — nenhum movimento será aceito. {self.erro_seguranca}')

    def _robo_parado(self):
        """True só se o robô confirma RobotMode 5 (habilitado, parado). Qualquer
        outra resposta ou falha conta como "pode estar se movendo"."""
        try:
            res = self._send('RobotMode()', lock_timeout=3.0)
            return res.startswith('0,') and res.split('{', 1)[1].split('}', 1)[0].strip() == '5'
        except (TimeoutError, socket.error, OSError, IndexError):
            return False

    def _bloqueio_arrastar(self):
        """Mensagem de recusa enquanto o modo arrastar está ligado ('' se não)."""
        if self._arrastando:
            return ('O robô está no modo arrastar (sendo movido com a mão). Saia dele '
                    '(Parar) antes de mandar um movimento.')
        return ''

    def _sair_do_arrastar(self):
        """StopDrag(). Devolve (ok, mensagem). Marca o modo como desligado mesmo
        se o comando falhar: na dúvida, nenhum movimento novo fica bloqueado à toa,
        e o robô fora do modo arrastar segura a posição."""
        self._arrastando = False
        self._ultimo_alvo = None
        try:
            res = self._send('StopDrag()', lock_timeout=3.0)
        except (TimeoutError, socket.error, OSError) as e:
            return False, f'StopDrag() falhou: {e}'
        self.get_logger().warn(f'[ARRASTAR] StopDrag() -> {res.strip()}')
        if res.startswith('0,'):
            return True, 'Modo arrastar desligado: o robô voltou a segurar a posição.'
        return False, f'StopDrag() recusado pelo robô: {res.strip()}'

    def arrastar_callback(self, request, response):
        """Liga (data=True) ou desliga o modo arrastar do CR10A (StartDrag /
        StopDrag, TCP/IP Remote Control Interface Guide): com ele ligado, o braço é
        guiado com a mão, como pelo botão do próprio robô. Só no robô real, com o
        robô habilitado e parado."""
        if self.dashboard is None:
            response.success, response.message = False, 'Robô não conectado.'
            return response
        if not request.data:
            response.success, response.message = self._sair_do_arrastar()
            return response
        if self.modo == 'sim':
            response.success = False
            response.message = ('O modo arrastar só existe no robô real: o gêmeo virtual não '
                                'pode ser guiado com a mão.')
            return response
        if self.emergency_stop_flag:
            response.success = False
            response.message = 'Robô em PARADA DE EMERGÊNCIA. Limpe o alarme e reconecte.'
            return response
        if self._bloqueio_coleta():
            response.success, response.message = False, self._bloqueio_coleta()
            return response
        if not self._robo_parado():
            response.success = False
            response.message = ('Para entrar no modo arrastar, o robô precisa estar habilitado '
                                'e parado, sem movimento nem alarme.')
            return response
        try:
            res = self._send('StartDrag()', lock_timeout=3.0)
        except (TimeoutError, socket.error, OSError) as e:
            response.success, response.message = False, f'Sem resposta do robô: {e}'
            return response
        self.get_logger().warn(f'[ARRASTAR] StartDrag() -> {res.strip()}')
        if not res.startswith('0,'):
            erro = self._send('GetErrorID()')
            response.success = False
            response.message = (f'O robô recusou o modo arrastar: {res.strip()} | '
                                f'GetErrorID: {erro.strip()}')
            return response
        self._arrastando = True
        self._ultimo_alvo = None
        response.success = True
        response.message = 'Modo arrastar ligado: segure o braço e mova com a mão.'
        return response

    def _verificar_movimento(self, alvo):
        """Antes de mandar um MovJ: confere o caminho da posição atual (e do
        último destino ainda na fila) até `alvo`. Devolve (ok, mensagem)."""
        if self._arrastando:
            return False, self._bloqueio_arrastar()
        if self._bloqueio_coleta():
            return False, self._bloqueio_coleta()
        if self.seguranca is None:
            return False, ('Segurança não carregou, movimento bloqueado: '
                           f'{self.erro_seguranca}')
        # Primeiro pergunta ao robô se ele está PARADO (RobotMode 5 = habilitado
        # e sem movimento): só então a fila está garantidamente vazia.
        parado = self._robo_parado()
        atual = self._read_angles()
        if atual is None:
            return False, ('Não consegui ler a posição atual do robô para verificar '
                           'a segurança do caminho. Movimento bloqueado.')
        trechos = [atual]
        # Um MovJ novo entra na FILA do Dobot e começa de onde o anterior
        # termina. Se o robô não confirmou que está parado (ou a consulta
        # falhou), o trecho "último destino -> alvo" também é verificado. Se
        # ele está parado, a fila está vazia e o novo movimento sai exatamente
        # da posição lida — o trecho já verificado; conferir de novo seria só
        # trabalho em dobro.
        if self._ultimo_alvo is not None and not parado:
            trechos.append(self._ultimo_alvo)
        for inicio in trechos:
            ok, motivo = self.seguranca.verificar_caminho(inicio, alvo)
            if not ok:
                self.get_logger().warn(f'[SEGURANÇA] movimento para {alvo} recusado: {motivo}')
                return False, 'BLOQUEADO PELA SEGURANÇA: ' + motivo
        return True, ''

    def _aplicar_protecoes_controlador(self):
        """Liga as proteções do próprio controlador (mesmos comandos do
        cr10twin). A detecção de colisão é obrigatória; o resto vira aviso.
        Devolve (ok, resumo)."""
        c = self.config_seguranca['controlador']
        cx, cy, cz = (float(v) for v in c['centro_massa_mm'])
        obrigatorio = f'SetCollisionLevel({int(c["nivel_colisao"])})'
        res = self._send(obrigatorio)
        if not res.startswith('0,'):
            return False, f'{obrigatorio} recusado pelo robô: {res.strip()}'
        resumo = [f'colisão nível {int(c["nivel_colisao"])}']
        for cmd, texto in (
                (f'SetPayload({float(c["carga_kg"]):.3f},{cx:.1f},{cy:.1f},{cz:.1f})',
                 f'carga {float(c["carga_kg"])} kg'),
                (f'SetBackDistance({float(c["recuo_colisao_mm"]):.1f})',
                 f'recuo {float(c["recuo_colisao_mm"])} mm'),
                (f'SetPostCollisionMode({int(c["modo_pos_colisao"])})',
                 'para após colisão' if int(c['modo_pos_colisao']) == 0
                 else 'pausa após colisão')):
            res = self._send(cmd)
            if res.startswith('0,'):
                resumo.append(texto)
            else:
                resumo.append(f'AVISO: {cmd} recusado ({res.strip()})')
                self.get_logger().warn(f'[SEGURANÇA] {cmd} recusado: {res.strip()}')
        return True, ', '.join(resumo)

    def _open_dashboard(self):
        """Abre a conexão de comandos: o socket do robô real, ou o dashboard
        simulado do gêmeo no modo sim."""
        if self._sim_dashboard is not None:
            return self._sim_dashboard.open()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((self.get_parameter('robot_ip').value,
                      self.get_parameter('dashboard_port').value))
        sock.settimeout(COMMAND_TIMEOUT)
        return sock

    def _send(self, cmd, lock_timeout=None, registrar=True, antes_de_enviar=None):
        """Manda um comando e lê a resposta, com a conexão travada.
        `antes_de_enviar`: função chamada já com a conexão travada; se devolver
        um texto, o comando NÃO é enviado e esse texto vira um RuntimeError
        (usado pelo percurso contínuo para nunca mandar um ponto depois de um
        pedido de parada)."""
        acquired = self.dashboard_lock.acquire(timeout=lock_timeout if lock_timeout else -1)
        if not acquired:
            raise TimeoutError(f'Não consegui acesso à conexão do robô a tempo pra enviar: {cmd}')
        try:
            if self.dashboard is None:
                raise OSError('sem conexão com o robô')
            if antes_de_enviar is not None:
                motivo = antes_de_enviar()
                if motivo:
                    raise RuntimeError(motivo)
            self.dashboard.sendall((cmd + '\n').encode('ascii'))
            res = self.dashboard.recv(1024).decode('ascii')
            if registrar:
                self.get_logger().info(f'CMD: {cmd} | RES: {res.strip()}')
            return res
        except (socket.timeout, socket.error, OSError):
            # Uma resposta atrasada bagunçaria os próximos comandos: melhor
            # derrubar a conexão e deixar o usuário reconectar.
            try:
                if self.dashboard is not None:
                    self.dashboard.close()
            except OSError:
                pass
            self.dashboard = None
            raise
        finally:
            self.dashboard_lock.release()

    def connect_callback(self, request, response):
        recovering_from_estop = self.emergency_stop_flag

        if self.dashboard is not None and not recovering_from_estop:
            response.success = True
            response.message = 'Já estava conectado.'
            return response

        robot_ip = self.get_parameter('robot_ip').value
        dashboard_port = self.get_parameter('dashboard_port').value
        if self.modo == 'sim':
            robot_ip, dashboard_port = 'gêmeo virtual', 'simulação'

        try:
            if self.dashboard is None:
                self.dashboard = self._open_dashboard()

            if self.seguranca is None:
                raise RuntimeError(
                    f'segurança não carregou, conexão recusada ({self.erro_seguranca})')
            self.emergency_stop_flag = False
            self._ultimo_alvo = None
            self._velocidade_pct = None       # a interface manda de novo ao conectar
            self._send('ClearError()')
            self._send('EnableRobot()')
            time.sleep(1)
            modo = self._send('RobotMode()')

            ok, protecoes = self._aplicar_protecoes_controlador()
            if not ok:
                # Sem detecção de colisão o robô não fica habilitado.
                self._disable_and_close()
                raise RuntimeError(
                    f'proteção de colisão não foi aceita pelo robô ({protecoes}); '
                    'robô desabilitado')

            self.get_logger().info(
                f'Conectado ao robô em {robot_ip}:{dashboard_port}. Proteções: {protecoes}')
            response.success = True
            response.message = f'Conectado com sucesso ({modo.strip()}). Proteções: {protecoes}'
        except (socket.timeout, socket.error, OSError, TimeoutError, RuntimeError) as e:
            if self.dashboard is not None:
                try:
                    self.dashboard.close()
                except OSError:
                    pass
            self.dashboard = None
            response.success = False
            response.message = f'Falha ao conectar: {e}'

        return response

    def disconnect_callback(self, request, response):
        if self.dashboard is None:
            response.success = True
            response.message = 'Já estava desconectado.'
            return response

        if self._arrastando:
            self._sair_do_arrastar()
        self._col_interromper('Coleta parada: robô desconectado.')
        disable_info = self._disable_and_close()

        self.get_logger().info('Desconectado do robô.')
        response.success = True
        response.message = f'Desconectado ({disable_info})'
        return response

    def clear_alarm_callback(self, request, response):
        """Limpa o alarme (ex.: depois do STOP de emergência). Abre a conexão se
        ainda não houver uma. Não reabilita o robô: depois disso, "Conectar"
        faz o ClearError + EnableRobot de novo."""
        try:
            if self.dashboard is None:
                self.dashboard = self._open_dashboard()
                # Mantém a flag de emergência até o próximo "Conectar" reabilitar o robô.
                self.emergency_stop_flag = True

            messages = []
            ok = False
            for cmd in ('EmergencyStop(0)', 'ClearError()'):
                res = self._send(cmd, lock_timeout=3.0)
                messages.append(f'{cmd} -> {res.strip()}')
                if cmd == 'ClearError()' and res.startswith('0,'):
                    ok = True
            response.success = ok
            response.message = ' | '.join(messages)
        except (TimeoutError, socket.error, OSError) as e:
            self.dashboard = None
            response.success = False
            response.message = f'Falha ao limpar alarme: {e}'

        return response

    def _disable_and_close(self):
        """Manda DisableRobot() (o inverso do EnableRobot() do conectar) e fecha a conexão."""
        info = ''
        try:
            res = self._send('DisableRobot()', lock_timeout=3.0)
            info = f'DisableRobot() -> {res.strip()}'
        except (TimeoutError, socket.error, OSError) as e:
            info = f'DisableRobot() falhou: {e}'
            self.get_logger().error(info)

        acquired = self.dashboard_lock.acquire(timeout=3.0)
        try:
            if self.dashboard is not None:
                self.dashboard.close()
                self.dashboard = None
        finally:
            if acquired:
                self.dashboard_lock.release()
        return info

    def set_speed_callback(self, request, response):
        if self.dashboard is None:
            response.success = False
            response.message = 'Robô não conectado. Clique em Conectar primeiro.'
            return response

        percent = request.percent
        if percent < 0 or percent > VELOCIDADE_MAXIMA_PERMITIDA:
            response.success = False
            response.message = (
                f'Velocidade {percent}% fora do limite permitido '
                f'(0 a {VELOCIDADE_MAXIMA_PERMITIDA}%).'
            )
            return response

        try:
            res = self._send(f'SpeedFactor({percent})')
        except (TimeoutError, socket.error, OSError) as e:
            response.success = False
            response.message = f'Sem resposta do robô: {e}'
            return response
        if not res.startswith('0,'):
            response.success = False
            response.message = f'Robô recusou o comando: {res.strip()}'
            return response

        self._velocidade_pct = int(percent)
        response.success = True
        response.message = f'Velocidade ajustada para {percent}%'
        return response

    def _movj(self, juntas):
        """Reafirma a velocidade (SpeedFactor) e devolve o comando MovJ, com a
        aceleração limitada pela mesma porcentagem (pelo manual, a aceleração do
        MovJ NÃO é reduzida pela velocidade global, só pelo parâmetro `a`).
        Devolve (comando, None) ou (None, motivo da recusa)."""
        p = self._velocidade_pct
        if p is None:
            return None, ('Velocidade ainda não confirmada pelo robô. Mexa no controle '
                          'de velocidade e tente de novo.')
        p = max(1, min(int(p), VELOCIDADE_MAXIMA_PERMITIDA))
        res = self._send(f'SpeedFactor({p})', lock_timeout=3.0)
        if not res.startswith('0,'):
            return None, f'Robô não aceitou a velocidade {p}%: {res.strip()}'
        return 'MovJ(joint={' + ', '.join(str(j) for j in juntas) + f'}},a={p})', None

    def get_joints_callback(self, request, response):
        if self.dashboard is None:
            response.success = False
            response.message = 'Robô não conectado. Clique em Conectar primeiro.'
            response.joints = [0.0] * 6
            return response

        try:
            res = self._send('GetAngle()')
        except (TimeoutError, socket.error, OSError) as e:
            response.success = False
            response.message = f'Sem resposta do robô: {e}'
            response.joints = [0.0] * 6
            return response
        try:
            if not res.startswith('0,'):
                raise ValueError(res.strip())
            inner = res.split('{', 1)[1].split('}', 1)[0]
            values = [float(v) for v in inner.split(',')]
            if len(values) != 6:
                raise ValueError(f'esperava 6 valores, recebeu {len(values)}')

            response.success = True
            response.message = 'ok'
            response.joints = values
        except (IndexError, ValueError) as e:
            response.success = False
            response.message = f'Erro ao ler GetAngle(): {e} | resposta: {res.strip()}'
            response.joints = [0.0] * 6

        return response

    def verificar_caminho_callback(self, request, response):
        """Roda a mesma verificação de antes de um MovJ, mas não manda nada."""
        if self.dashboard is None:
            response.seguro = False
            response.message = 'Robô não conectado. Clique em Conectar primeiro.'
            return response
        ok, motivo = self._verificar_movimento([float(j) for j in request.joints])
        response.seguro = ok
        response.message = 'Caminho seguro.' if ok else motivo
        return response

    def stop_motion_callback(self, request, response):
        """Para o movimento SEM emergência: Stop() + Continue(), como o
        stop_motion do cr10twin. Stop() freia e limpa a fila de movimentos;
        Continue() deixa a fila pronta para o próximo comando. O robô continua
        habilitado e conectado — não precisa limpar alarme nem reconectar."""
        self._parada_geracao += 1
        self._ultimo_alvo = None
        self._col_interromper('Coleta parada pelo botão Parar.')

        if self.dashboard is None:
            response.success = False
            response.message = 'Robô não conectado.'
            return response

        mensagens = []
        ok = False
        if self._arrastando:
            mensagens.append(self._sair_do_arrastar()[1])
        for cmd in ('Stop()', 'Continue()'):
            try:
                res = self._send(cmd, lock_timeout=3.0)
                mensagens.append(f'{cmd} -> {res.strip()}')
                if cmd == 'Stop()' and res.startswith('0,'):
                    ok = True
            except (TimeoutError, socket.error, OSError) as e:
                mensagens.append(f'{cmd} -> falhou: {e}')
        self.get_logger().warn('[PARAR MOVIMENTO] ' + ' | '.join(mensagens))
        response.success = ok
        response.message = ('Movimento parado.' if ok else 'Falha ao parar: ') + \
            ' | '.join(mensagens)
        return response

    def emergency_stop_callback(self, request, response):
        """Roda num grupo/thread dedicado, então esse callback em si nunca fica
        preso atrás de um movimento em andamento. O robô só aceita UMA conexão
        na porta do dashboard, então reaproveita a mesma (self.dashboard) —
        mas com um timeout curto pra não ficar refém se algo travar de vez."""
        self.emergency_stop_flag = True
        self._ultimo_alvo = None
        self._arrastando = False         # a emergência desabilita o robô
        # Congela o explorador (o robô já vai receber EmergencyStop logo abaixo).
        if self._col_armada or self._coleta_ativa():
            self._col_armada = False
            self._col_freeze_pub.publish(Empty())
            self._col_desarmar('Coleta parada pela EMERGÊNCIA.', erro=True)

        if self.dashboard is None:
            response.success = False
            response.message = 'Robô não está com conexão aberta — use o botão físico de emergência.'
            return response

        messages = []
        ok_any = False
        estop_ok = False
        # EmergencyStop(1) é a sintaxe com parâmetro (aciona); a sem parâmetro fica
        # de fallback caso o firmware use essa. Stop() sempre vai por último.
        for cmd in ('EmergencyStop(1)', 'EmergencyStop()', 'Stop()'):
            if cmd == 'EmergencyStop()' and estop_ok:
                continue
            try:
                res = self._send(cmd, lock_timeout=3.0)
                self.get_logger().warn(f'[PARADA DE EMERGÊNCIA] CMD: {cmd} | RES: {res.strip()}')
                messages.append(f'{cmd} -> {res.strip() or "(sem resposta, conexão fechada pelo robô)"}')
                if res.startswith('0,'):
                    ok_any = True
                    if cmd.startswith('EmergencyStop'):
                        estop_ok = True
            except (TimeoutError, socket.error, OSError) as e:
                messages.append(f'{cmd} -> falhou: {e}')
                self.get_logger().error(f'[PARADA DE EMERGÊNCIA] Falha ao enviar {cmd}: {e}')

        # O robô derruba a conexão quando entra em emergência; fecha a nossa também,
        # assim o próximo "Conectar" abre uma conexão nova e limpa.
        acquired = self.dashboard_lock.acquire(timeout=1.0)
        try:
            if self.dashboard is not None:
                self.dashboard.close()
                self.dashboard = None
        finally:
            if acquired:
                self.dashboard_lock.release()

        response.success = ok_any
        response.message = ' | '.join(messages)
        return response

    def _read_angles(self):
        """Posição real das 6 juntas, ou None se a leitura falhar."""
        try:
            res = self._send('GetAngle()', lock_timeout=3.0)
            if not res.startswith('0,'):
                return None
            inner = res.split('{', 1)[1].split('}', 1)[0]
            values = [float(v) for v in inner.split(',')]
            return values if len(values) == 6 else None
        except (TimeoutError, socket.error, OSError, IndexError, ValueError):
            return None

    @staticmethod
    def _angle_diff(a, b):
        return abs((a - b + 180.0) % 360.0 - 180.0)

    def _wait_until_reached(self, target, goal_handle, feedback_msg, geracao):
        """Espera o robô CHEGAR no destino (lendo a posição real), em vez de um
        tempo fixo. Devolve (resultado, quanto faltava em graus):
        'ok', 'estop', 'stuck' (parou longe), 'timeout'.
        Se não der pra ler a posição, cai no tempo fixo antigo."""
        start = time.time()
        time.sleep(0.5)  # dá tempo do robô começar a andar
        last = None
        still_since = None
        failed_reads = 0
        faltam = 0.0
        while time.time() - start < TEMPO_MAXIMO_MOVIMENTO:
            if self.emergency_stop_flag:
                return 'estop', faltam
            if self._parada_geracao != geracao:
                return 'parado', faltam
            current = self._read_angles()
            if current is None:
                failed_reads += 1
                if failed_reads >= 3:
                    return self._fixed_wait(start, geracao), 0.0
                time.sleep(SLEEP_STEP)
                continue

            faltam = max(self._angle_diff(c, t) for c, t in zip(current, target))
            feedback_msg.status = f'Movendo... faltam {faltam:.1f}°'
            goal_handle.publish_feedback(feedback_msg)
            if faltam <= TOLERANCIA_GRAUS:
                return 'ok', faltam

            moved = last is None or any(abs(c - l) > 0.2 for c, l in zip(current, last))
            if moved:
                still_since = None
            elif still_since is None:
                still_since = time.time()
            elif time.time() - still_since > TEMPO_PARADO_SEM_CHEGAR:
                return 'stuck', faltam
            last = current
            time.sleep(SLEEP_STEP)
        return 'timeout', faltam

    def _fixed_wait(self, start, geracao):
        while time.time() - start < TEMPO_MOVIMENTO:
            if self.emergency_stop_flag:
                return 'estop'
            if self._parada_geracao != geracao:
                return 'parado'
            time.sleep(SLEEP_STEP)
        return 'ok'

    def jog_joint_callback(self, request, response):
        if self.dashboard is None:
            response.success = False
            response.message = 'Robô não conectado. Clique em Conectar primeiro.'
            return response

        if self.emergency_stop_flag:
            response.success = False
            response.message = 'Robô em PARADA DE EMERGÊNCIA. Limpe o alarme e reconecte.'
            return response

        alvo = [float(j) for j in request.joints]
        ok, motivo = self._verificar_movimento(alvo)
        if not ok:
            response.success = False
            response.message = motivo
            return response

        t0 = time.time()
        try:
            cmd, motivo = self._movj(alvo)
            if cmd is None:
                response.success = False
                response.message = motivo
                return response
            res = self._send(cmd, lock_timeout=3.0)
        except (TimeoutError, socket.error, OSError) as e:
            response.success = False
            response.message = f'Falha ao enviar movimento: {e}'
            return response
        elapsed_ms = (time.time() - t0) * 1000

        response.success = res.startswith('0,')
        if response.success:
            self._ultimo_alvo = alvo
        response.message = (f'MovJ aceito pelo robô em {elapsed_ms:.0f} ms'
                            if response.success else f'Robô recusou: {res.strip()}')
        return response

    def execute_callback(self, goal_handle):
        result = MoveJoint.Result()

        if self.dashboard is None:
            result.success = False
            result.message = 'Robô não conectado. Clique em Conectar primeiro.'
            goal_handle.abort()
            return result

        if self.emergency_stop_flag:
            result.success = False
            result.message = 'Robô em PARADA DE EMERGÊNCIA. Reconecte para liberar o movimento.'
            goal_handle.abort()
            return result

        geracao = self._parada_geracao
        joints = [float(j) for j in goal_handle.request.joints]

        feedback_msg = MoveJoint.Feedback()
        feedback_msg.status = 'Verificando segurança do caminho...'
        goal_handle.publish_feedback(feedback_msg)
        ok, motivo = self._verificar_movimento(joints)
        if not ok:
            result.success = False
            result.message = motivo
            goal_handle.abort()
            return result
        if self.emergency_stop_flag:
            result.success = False
            result.message = 'Movimento abortado: PARADA DE EMERGÊNCIA acionada.'
            goal_handle.abort()
            return result
        if self._parada_geracao != geracao:
            result.success = False
            result.message = 'Movimento parado pelo usuário (antes de começar).'
            goal_handle.abort()
            return result

        feedback_msg.status = 'Enviando movimento...'
        goal_handle.publish_feedback(feedback_msg)

        try:
            cmd, motivo = self._movj(joints)
            if cmd is None:
                result.success = False
                result.message = motivo
                goal_handle.abort()
                return result
            res = self._send(cmd)
        except (TimeoutError, socket.error, OSError) as e:
            result.success = False
            result.message = f'Sem resposta do robô: {e}'
            goal_handle.abort()
            return result

        if res.startswith('0,'):
            self._ultimo_alvo = joints
        if not res.startswith('0,'):
            erro = self._send('GetErrorID()')
            result.success = False
            result.message = f'Comando falhou: {res.strip()} | GetErrorID: {erro.strip()}'
            goal_handle.abort()
            return result

        feedback_msg.status = 'Movendo...'
        goal_handle.publish_feedback(feedback_msg)

        outcome, faltam = self._wait_until_reached(joints, goal_handle, feedback_msg, geracao)

        if outcome == 'estop':
            result.success = False
            result.message = 'Movimento abortado: PARADA DE EMERGÊNCIA acionada.'
            goal_handle.abort()
            return result
        if outcome == 'parado':
            result.success = False
            result.message = 'Movimento parado pelo usuário.'
            goal_handle.abort()
            return result
        if outcome in ('stuck', 'timeout'):
            result.success = False
            motivo = 'parou antes do destino' if outcome == 'stuck' else 'demorou demais'
            result.message = f'O robô {motivo} (faltavam {faltam:.1f}°).'
            goal_handle.abort()
            return result

        result.success = True
        result.message = 'Movimento concluído'
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Percurso contínuo (ServoJ) — movimento em 8
    # ------------------------------------------------------------------

    @staticmethod
    def _validar_trajetoria(dados, periodo):
        """Confere o percurso ANTES de qualquer coisa: formato, limites das
        juntas, velocidade (o ServoJ ignora o SpeedFactor) e suavidade.
        Devolve (ok, motivo, pontos)."""
        if len(dados) % 6 or len(dados) < 12:
            return False, 'Percurso inválido: precisa de pelo menos 2 pontos de 6 juntas.', None
        if len(dados) // 6 > MAX_PONTOS_TRAJETORIA:
            return False, f'Percurso longo demais (máximo {MAX_PONTOS_TRAJETORIA} pontos).', None
        if not PERIODO_SERVOJ_S[0] <= periodo <= PERIODO_SERVOJ_S[1]:
            return False, (f'Intervalo entre pontos de {periodo} s fora da faixa '
                           f'{PERIODO_SERVOJ_S[0]}–{PERIODO_SERVOJ_S[1]} s.'), None
        if not all(v == v and abs(v) < 1e9 for v in dados):     # NaN / infinito
            return False, 'Percurso inválido: há valores que não são números.', None
        pontos = [list(dados[i:i + 6]) for i in range(0, len(dados), 6)]
        for k, p in enumerate(pontos):
            for j, (v, lim) in enumerate(zip(p, LIMITE_JUNTA_GRAUS)):
                if abs(v) > lim:
                    return False, f'Ponto {k}: J{j + 1} = {v:.1f}° passa do limite de ±{lim:.0f}°.', None
        fator = VELOCIDADE_MAXIMA_PERMITIDA / 100.0
        vel_ant = None
        for k in range(1, len(pontos)):
            vel = [(b - a) / periodo for a, b in zip(pontos[k - 1], pontos[k])]
            for j, (v, vmax) in enumerate(zip(vel, VELOCIDADE_MAX_JUNTA_GRAUS_S)):
                if abs(v) > vmax * fator:
                    return False, (f'Rápido demais: J{j + 1} chegaria a {abs(v):.0f}°/s no ponto {k} '
                                   f'(limite {vmax * fator:.0f}°/s, {VELOCIDADE_MAXIMA_PERMITIDA}% da '
                                   'velocidade máxima). Aumente o tempo de cada volta.'), None
            if vel_ant is not None:
                for j, (a, b) in enumerate(zip(vel_ant, vel)):
                    if abs(b - a) / periodo > ACELERACAO_MAX_GRAUS_S2:
                        return False, (f'Percurso com tranco: J{j + 1} muda de velocidade bruscamente '
                                       f'no ponto {k}.'), None
            vel_ant = vel
        return True, '', pontos

    def _motivo_para_nao_enviar(self, geracao):
        """Conferido com a conexão travada, logo antes de cada ponto."""
        if self.emergency_stop_flag:
            return 'Percurso abortado: PARADA DE EMERGÊNCIA acionada.'
        if self._parada_geracao != geracao:
            return 'Percurso parado pelo usuário.'
        return ''

    def _parar_servo(self):
        """Encerra um percurso contínuo interrompido: Stop() freia e Continue()
        deixa o robô pronto para o próximo comando (como no Parar movimento)."""
        for cmd in ('Stop()', 'Continue()'):
            try:
                self._send(cmd, lock_timeout=2.0)
            except (TimeoutError, socket.error, OSError):
                pass
        self._ultimo_alvo = None

    def seguir_trajetoria_callback(self, goal_handle):
        result = SeguirTrajetoria.Result()
        feedback = SeguirTrajetoria.Feedback()

        def falhar(msg):
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result

        def avisar(texto, indice):
            feedback.status = texto
            feedback.indice = indice
            goal_handle.publish_feedback(feedback)

        if self.dashboard is None:
            return falhar('Robô não conectado. Clique em Conectar primeiro.')
        if self.emergency_stop_flag:
            return falhar('Robô em PARADA DE EMERGÊNCIA. Reconecte para liberar o movimento.')
        if self._arrastando:
            return falhar(self._bloqueio_arrastar())
        if self._bloqueio_coleta():
            return falhar(self._bloqueio_coleta())
        if self.seguranca is None:
            return falhar(f'Segurança não carregou, movimento bloqueado: {self.erro_seguranca}')
        geracao = self._parada_geracao
        periodo = float(goal_handle.request.periodo_s)

        # 1) Formato, limites, velocidade e suavidade.
        ok, motivo, pontos = self._validar_trajetoria(list(goal_handle.request.pontos), periodo)
        if not ok:
            return falhar(motivo)
        n = len(pontos)

        # 2) O robô tem de estar parado, exatamente no 1º ponto.
        avisar('Conferindo a posição do robô...', 0)
        # Logo depois de um movimento o robô pode levar um instante para se
        # declarar parado: espera um pouco antes de recusar.
        limite = time.monotonic() + 3.0
        while not self._robo_parado():
            if time.monotonic() > limite:
                return falhar('O robô precisa estar parado para começar o percurso contínuo.')
            motivo = self._motivo_para_nao_enviar(geracao)
            if motivo:
                return falhar(motivo)
            time.sleep(0.2)
        atual = self._read_angles()
        if atual is None:
            return falhar('Não consegui ler a posição atual do robô. Movimento bloqueado.')
        dif = max(self._angle_diff(a, b) for a, b in zip(atual, pontos[0]))
        if dif > TOLERANCIA_INICIO_GRAUS:
            return falhar(f'O percurso tem de começar na posição atual do robô (diferença de {dif:.1f}°).')

        # 3) Segurança do percurso INTEIRO antes de mandar o primeiro ponto.
        avisar(f'Verificando segurança do percurso inteiro ({n} pontos)...', 0)
        # Percursos repetidos (várias voltas do 8) passam pelos mesmos trechos:
        # cada trecho diferente é verificado uma vez e o resultado reaproveitado.
        verificados = set()
        anterior = atual
        ultimo_aviso = time.monotonic()
        for k, p in enumerate(pontos):
            motivo = self._motivo_para_nao_enviar(geracao)
            if motivo:
                return falhar(motivo)
            chave = (tuple(round(v, 3) for v in anterior), tuple(round(v, 3) for v in p))
            if chave not in verificados:
                ok, motivo = self.seguranca.verificar_caminho(anterior, p)
                if not ok:
                    self.get_logger().warn(f'[SEGURANÇA] percurso contínuo recusado no ponto {k}: {motivo}')
                    return falhar(f'BLOQUEADO PELA SEGURANÇA (ponto {k} de {n}): {motivo}')
                verificados.add(chave)
            anterior = p
            if time.monotonic() - ultimo_aviso > 0.5:
                ultimo_aviso = time.monotonic()
                avisar(f'Verificando segurança do percurso inteiro: {100 * k // n}%', 0)
        self.get_logger().info(f'Percurso contínuo: {n} pontos verificados, a cada {periodo:.3f} s.')

        # 4) Envio, um ponto a cada `periodo` (o robô interpola entre eles).
        avisar('Percurso contínuo em andamento...', 0)
        t0 = time.monotonic()
        ultimo_aviso = 0.0
        for k in range(1, n):
            espera = t0 + (k - 1) * periodo - time.monotonic()
            if espera > 0:
                time.sleep(espera)
            cmd = 'ServoJ({},t={:.3f},aheadtime={},gain={})'.format(
                ','.join(f'{v:.4f}' for v in pontos[k]), periodo, SERVOJ_AHEADTIME, SERVOJ_GAIN)
            try:
                res = self._send(cmd, lock_timeout=4 * periodo, registrar=False,
                                 antes_de_enviar=lambda: self._motivo_para_nao_enviar(geracao))
            except RuntimeError as e:          # pedido de parada/emergência
                return falhar(str(e))
            except (TimeoutError, socket.error, OSError) as e:
                self._parar_servo()
                return falhar(f'Sem resposta do robô no ponto {k} de {n}: {e}. Parado por segurança.')
            if not res.startswith('0,'):
                self._parar_servo()
                return falhar(f'O robô recusou o ponto {k} de {n}: {res.strip()}. Parado por segurança.')
            atraso = time.monotonic() - (t0 + (k - 1) * periodo)
            if atraso > 4 * periodo + 0.1:
                self._parar_servo()
                return falhar(f'Comunicação lenta demais para o percurso contínuo (atraso de '
                              f'{atraso:.2f} s no ponto {k}). Parado por segurança.')
            agora = time.monotonic()
            if agora - ultimo_aviso > 0.25:
                ultimo_aviso = agora
                avisar(f'Percurso contínuo: ponto {k} de {n - 1}', k)
        self._ultimo_alvo = pontos[-1]

        # 5) Confere que terminou no último ponto.
        time.sleep(periodo + 0.3)
        fim = self._read_angles()
        if fim is not None:
            faltam = max(self._angle_diff(a, b) for a, b in zip(fim, pontos[-1]))
            if faltam > TOLERANCIA_FIM_GRAUS:
                return falhar(f'O robô terminou longe do fim do percurso (faltavam {faltam:.1f}°).')
        result.success = True
        result.message = f'Percurso contínuo concluído ({n} pontos).'
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Ponteira em linha reta, parando ao encostar (medição da mesa)
    # ------------------------------------------------------------------

    def _on_forca(self, msg):
        f = msg.wrench.force
        with self._forca_lock:
            self._forcas.append((time.monotonic(), np.array([f.x, f.y, f.z])))

    def _forca_base(self):
        """Média da força no último JANELA_FORCA_BASE_S, ou None se a célula
        não está mandando dados agora."""
        agora = time.monotonic()
        with self._forca_lock:
            recentes = [f for t, f in self._forcas if agora - t <= JANELA_FORCA_BASE_S]
            ultimo = self._forcas[-1][0] if self._forcas else 0.0
        if len(recentes) < 3 or agora - ultimo > FORCA_VELHA_S:
            return None
        return np.mean(recentes, axis=0)

    def _forca_atual(self):
        with self._forca_lock:
            if not self._forcas:
                return None, None
            t, f = self._forcas[-1]
        return time.monotonic() - t, f

    def mover_ponta_callback(self, goal_handle):
        req = goal_handle.request
        result = MoverPonta.Result()
        feedback = MoverPonta.Feedback()

        def falhar(msg):
            result.success = False
            result.message = msg
            goal_handle.abort()
            return result

        def avisar(texto, percorrido=0.0, forca=0.0):
            feedback.status = texto
            feedback.percorrido_mm = float(percorrido)
            feedback.forca_n = float(forca)
            goal_handle.publish_feedback(feedback)

        if self.dashboard is None:
            return falhar('Robô não conectado. Clique em Conectar primeiro.')
        if self.emergency_stop_flag:
            return falhar('Robô em PARADA DE EMERGÊNCIA. Reconecte para liberar o movimento.')
        if self._arrastando:
            return falhar(self._bloqueio_arrastar())
        if self._bloqueio_coleta():
            return falhar(self._bloqueio_coleta())
        if self.seguranca is None or self.cinematica is None:
            return falhar(f'Segurança não carregou, movimento bloqueado: {self.erro_seguranca}')
        geracao = self._parada_geracao
        v = float(req.velocidade_mms)
        vmax = VELOCIDADE_CONTATO_MAX_MMS if req.parar_no_contato else VELOCIDADE_PONTA_MAX_MMS
        if not 0.05 <= v <= vmax:
            return falhar(f'Velocidade da ponteira de {v:.2f} mm/s fora da faixa 0,05–{vmax:.0f} mm/s.')
        if req.tocar_mesa and self.seguranca.ferramenta_contato is None:
            return falhar('Encostar na mesa só com a célula de carga na ponta (abra com iris, sem sem-nada nem com-sensor).')
        limiar = float(req.limiar_n)
        if req.parar_no_contato and not LIMIAR_CONTATO_N[0] <= limiar <= LIMIAR_CONTATO_N[1]:
            return falhar(f'Limiar de contato de {limiar:.2f} N fora da faixa '
                          f'{LIMIAR_CONTATO_N[0]}–{LIMIAR_CONTATO_N[1]} N.')

        # 1) Robô parado; posição de partida.
        avisar('Conferindo a posição do robô...')
        limite = time.monotonic() + 3.0
        while not self._robo_parado():
            if time.monotonic() > limite:
                return falhar('O robô precisa estar parado para mover a ponteira.')
            motivo = self._motivo_para_nao_enviar(geracao)
            if motivo:
                return falhar(motivo)
            time.sleep(0.2)
        atual = self._read_angles()
        if atual is None:
            return falhar('Não consegui ler a posição atual do robô. Movimento bloqueado.')
        T0 = self.cinematica.direta(atual)
        if req.ate_altura:
            desloc = np.array([0.0, 0.0, float(req.altura_mm) - T0[2, 3] * 1000.0])
        else:
            desloc = np.array(req.deslocamento_mm, dtype=float)
        dist = float(np.linalg.norm(desloc))
        if not np.all(np.isfinite(desloc)):
            return falhar('Deslocamento da ponteira inválido.')
        if dist < 0.05:                         # já está lá
            result.success = True
            result.juntas = [float(x) for x in atual]
            result.ponta_mm = [float(x) * 1000.0 for x in T0[:3, 3]]
            result.message = 'A ponteira já está na posição pedida.'
            goal_handle.succeed()
            return result
        if req.tocar_mesa:
            # Para encostar, a ponteira aponta para baixo; sem parar ao encostar,
            # só é permitido SAIR da mesa (subir na vertical).
            desvio = math.degrees(math.acos(max(-1.0, min(1.0, -float(T0[2, 2])))))
            if desvio > INCLINACAO_PONTEIRA_MAX_GRAUS:
                return falhar(f'A ponteira precisa apontar para baixo para encostar na mesa '
                              f'(está a {desvio:.0f}° da vertical; limite '
                              f'{INCLINACAO_PONTEIRA_MAX_GRAUS:.0f}°).')
            subindo = desloc[2] > 0 and float(np.linalg.norm(desloc[:2])) < 0.01
            if not req.parar_no_contato and not subindo:
                return falhar('Perto da mesa, a ponteira só pode descer parando ao encostar '
                              '(ou subir na vertical).')

        # 2) Juntas ao longo da linha reta (orientação da ferramenta fixa).
        avisar('Calculando o caminho da ponteira...')
        qs, motivo = self.cinematica.linha_reta(atual, desloc / 1000.0)
        if qs is None:
            return falhar(f'Caminho da ponteira impossível: {motivo}.')
        if req.tocar_mesa:
            z_fim = float(self.cinematica.direta(qs[-1])[2, 3])
            if z_fim < -self.profundidade_contato - 1e-6:
                return falhar(f'A descida iria até {-z_fim * 1000:.1f} mm abaixo do tampo da mesa do '
                              f'modelo de segurança (limite {self.profundidade_contato * 1000:.0f} mm).')

        # 3) Segurança de todos os trechos.
        avisar('Verificando segurança do caminho da ponteira...')
        for a, b in zip(qs, qs[1:]):
            motivo = self._motivo_para_nao_enviar(geracao)
            if motivo:
                return falhar(motivo)
            ok, motivo = self.seguranca.verificar_caminho(a, b, contato_mesa=req.tocar_mesa)
            if not ok:
                self.get_logger().warn(f'[SEGURANÇA] ponteira em linha reta recusada: {motivo}')
                return falhar('BLOQUEADO PELA SEGURANÇA: ' + motivo)

        # 4) Pontos no tempo (um a cada PERIODO_PONTA_S, na velocidade pedida).
        n = max(1, int(math.ceil(dist / (v * PERIODO_PONTA_S))))
        pontos = []
        for k in range(n + 1):
            s = (len(qs) - 1) * k / n
            i = min(int(s), len(qs) - 2) if len(qs) > 1 else 0
            f = s - i
            pontos.append(list(qs[i] * (1 - f) + qs[min(i + 1, len(qs) - 1)] * f))
        ok, motivo, pontos = self._validar_trajetoria(
            [x for p in pontos for x in p], PERIODO_PONTA_S)
        if not ok:
            return falhar(motivo)

        # 5) Força de referência (o "zero" deste movimento).
        base = self._forca_base()
        if req.parar_no_contato and base is None:
            return falhar('A célula de carga não está mandando dados: sem ela não dá para '
                          'perceber o contato. Movimento bloqueado.')

        # 6) Envio, conferindo a força antes de cada ponto.
        avisar('Movendo a ponteira...')
        contato = False
        forca = 0.0
        t0 = time.monotonic()
        ultimo_aviso = 0.0
        for k in range(1, len(pontos)):
            if base is not None:
                idade, f = self._forca_atual()
                if req.parar_no_contato and (idade is None or idade > FORCA_VELHA_S):
                    self._parar_servo()
                    return falhar('A célula de carga parou de mandar dados durante o movimento. '
                                  'Parado por segurança.')
                forca = float(np.linalg.norm(f - base))
                if forca > FORCA_MAXIMA_N:
                    self._parar_servo()
                    return falhar(f'Força de {forca:.1f} N na ponteira (limite {FORCA_MAXIMA_N:.0f} N). '
                                  'Parado por segurança.')
                if req.parar_no_contato and forca >= limiar:
                    contato = True
                    break
            espera = t0 + (k - 1) * PERIODO_PONTA_S - time.monotonic()
            if espera > 0:
                time.sleep(espera)
            cmd = 'ServoJ({},t={:.3f},aheadtime={},gain={})'.format(
                ','.join(f'{x:.4f}' for x in pontos[k]), PERIODO_PONTA_S, SERVOJ_AHEADTIME, SERVOJ_GAIN)
            try:
                res = self._send(cmd, lock_timeout=4 * PERIODO_PONTA_S, registrar=False,
                                 antes_de_enviar=lambda: self._motivo_para_nao_enviar(geracao))
            except RuntimeError as e:          # pedido de parada/emergência
                return falhar(str(e))
            except (TimeoutError, socket.error, OSError) as e:
                self._parar_servo()
                return falhar(f'Sem resposta do robô: {e}. Parado por segurança.')
            if not res.startswith('0,'):
                self._parar_servo()
                return falhar(f'O robô recusou um ponto: {res.strip()}. Parado por segurança.')
            if time.monotonic() - (t0 + (k - 1) * PERIODO_PONTA_S) > 4 * PERIODO_PONTA_S + 0.1:
                self._parar_servo()
                return falhar('Comunicação lenta demais para mover a ponteira. Parado por segurança.')
            agora = time.monotonic()
            if agora - ultimo_aviso > 0.25:
                ultimo_aviso = agora
                avisar('Movendo a ponteira...', dist * k / n, forca)

        if contato:
            self._parar_servo()                # freia já, sem esperar o fim da linha
        else:
            self._ultimo_alvo = pontos[-1]
        time.sleep(PERIODO_PONTA_S + 0.3)
        fim = self._read_angles()
        if fim is None:
            return falhar('Não consegui ler a posição final do robô.')
        T = self.cinematica.direta(fim)
        idade, f = self._forca_atual()
        if base is not None and f is not None:
            forca = float(np.linalg.norm(f - base))
        result.success = True
        result.contato = contato
        result.percorrido_mm = float(np.linalg.norm(T[:3, 3] - T0[:3, 3]) * 1000.0)
        result.juntas = [float(x) for x in fim]
        result.ponta_mm = [float(x) * 1000.0 for x in T[:3, 3]]
        result.forca_n = forca
        if contato:
            result.message = (f'Encostou depois de {result.percorrido_mm:.2f} mm '
                              f'({forca:.2f} N).')
        elif req.parar_no_contato:
            result.message = f'Chegou ao fim ({result.percorrido_mm:.1f} mm) sem encostar.'
        else:
            result.message = f'Ponteira movida {result.percorrido_mm:.1f} mm.'
        goal_handle.succeed()
        return result

    def destroy(self):
        self.emergency_stop_flag = True
        if self.dashboard:
            self._disable_and_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DobotActionServer()

    # Dois backends ao mesmo tempo brigam pelo robô (um conecta, o outro responde
    # "não conectado"). Dá um tempinho pro ROS descobrir os outros nós e confere.
    end = time.time() + 1.5
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.get_node_names().count('dobot_action_server') > 1:
        node.get_logger().error(
            'Já existe outro dobot_action_server rodando! Feche o outro '
            '(pkill -f dobot_action_server) antes de abrir este.')
        node.destroy_node()
        rclpy.try_shutdown()
        return
    # 6 threads: movimento, jog, leitura, emergência e (no modo sim) a
    # assinatura de /joint_states do gêmeo, com folga.
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    def handle_sigint(signum, frame):
        # Aborta rápido qualquer movimento em andamento (mesmo mecanismo do
        # STOP de emergência), em vez de deixar o processo preso esperando o
        # sleep de até 8s do movimento atual terminar sozinho.
        # try_shutdown() é o que acorda o executor.spin() (só sinalizar não basta:
        # ele fica preso esperando evento e o Ctrl+C nunca terminava).
        node.emergency_stop_flag = True
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, handle_sigint)
    # Sem isso a thread principal fica presa numa espera em C e o Python nunca
    # chega a executar o handler acima; o timer acorda ela a cada 0,2s.
    node.create_timer(0.2, lambda: None)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
