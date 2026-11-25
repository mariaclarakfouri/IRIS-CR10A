"""Servidor de movimento do CR10A (IRIS).

Fala com o controlador do robô pela porta do dashboard (29999, TCP): conectar,
habilitar, velocidade, ler a posição, mover as juntas (MovJ), parar e
emergência. No modo simulação os mesmos comandos vão para o gêmeo virtual.

Segurança: todo movimento passa por `_verificar_movimento` antes de ir para o
robô. Ali o caminho inteiro do MovJ (da posição atual até o destino) é conferido
contra o campo de trabalho calculado (dobot_gui.seguranca): se o flange ou a
ponta da ferramenta sairiam do campo, ou qualquer peça do robô desceria abaixo
do tampo da mesa, em algum momento, o movimento é recusado e nada é enviado ao
robô. O drag mode (mover com a mão) não passa por aqui.
"""
import signal
import socket
import threading
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from dobot_gui.seguranca import COMPRIMENTO_CELULA_MM, VerificadorCampo
from dobot_interfaces.action import MoveJoint
from dobot_interfaces.srv import GetJoints, JogJoint, SetSpeed

TEMPO_MOVIMENTO = 8  # segundos de espera fixa após cada MovJ
VELOCIDADE_MAXIMA_PERMITIDA = 50  # limite de velocidade, em % da velocidade máxima do robô
SLEEP_STEP = 0.2  # granularidade da espera durante o movimento, pra poder abortar rápido
TOLERANCIA_GRAUS = 0.5  # o robô "chegou" quando todas as juntas estão a menos disso do destino
TEMPO_MAXIMO_MOVIMENTO = 180  # desiste de esperar o robô chegar depois disso (s)
COMMAND_TIMEOUT = 5.0  # segundos esperando a resposta do robô antes de considerar que ele parou de responder
MODO_DRAG = 6  # RobotMode do controlador Dobot com o drag mode ligado
TEMPO_PARADO_SEM_CHEGAR = 3.0  # se o robô fica parado esse tempo longe do destino, desiste (s)


class DobotActionServer(Node):

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

        # Último destino mandado ao robô (um MovJ novo entra na FILA do Dobot).
        self._ultimo_alvo = None
        # Velocidade pedida pela interface (%). É reenviada ao robô antes de cada
        # MovJ: o SpeedFactor é global e pode ser trocado por fora (tablet,
        # DobotStudio, religar o robô) sem o servidor ficar sabendo.
        self._velocidade_pct = None

        # Segurança: o campo de trabalho calculado (pontos da aba Campo de
        # Trabalho). Com a célula de carga na ponta, a ponta dela também é
        # conferida. Sem o verificador, nenhum movimento é aceito.
        self.declare_parameter('ponta', 'nada')
        ponta = str(self.get_parameter('ponta').value).strip().lower()
        ferramenta_mm = COMPRIMENTO_CELULA_MM if ponta == 'celula' else 0.0
        self._seguranca = None
        self._erro_seguranca = ''
        try:
            self._seguranca = VerificadorCampo(ferramenta_mm=ferramenta_mm)
            campo, erro = self._seguranca.campo()
            if campo is None:
                self.get_logger().warn(f'Segurança: {erro}. Movimentos serão recusados.')
            else:
                self.get_logger().info(
                    f'Segurança: campo de trabalho calculado carregado (chão '
                    f'{campo.chao_z:.0f} mm, topo {campo.teto_z:.0f} mm, mesa '
                    f'{self._seguranca.mesa_z:.1f} mm; ferramenta {ferramenta_mm:.1f} mm).')
        except Exception as e:     # sem o verificador, nada se move
            self._erro_seguranca = str(e) or e.__class__.__name__
            self.get_logger().error(f'Segurança: verificador não carregou ({self._erro_seguranca}).'
                                    ' Movimentos serão recusados.')

        self.dashboard = None
        self.emergency_stop_flag = False
        # Modo arrastar (StartDrag): o braço é guiado com a mão. Enquanto ligado,
        # nenhum movimento por comando é aceito.
        self._arrastando = False
        # "Parar movimento" (sem emergência): cada pedido incrementa este
        # contador. Um movimento guarda o valor de quando começou; se mudou,
        # ele foi parado.
        self._parada_geracao = 0
        # O robô só aceita UMA conexão na porta do dashboard, então todo mundo
        # (movimento, velocidade, leitura de posição, STOP) divide a mesma
        # conexão. Esse lock evita que dois comandos se atropelem nela.
        self.dashboard_lock = threading.Lock()

        # A emergência e o parar ficam em grupos próprios (outra thread), para
        # nunca esperarem atrás de um movimento em andamento.
        normal_group = MutuallyExclusiveCallbackGroup()
        emergency_group = MutuallyExclusiveCallbackGroup()
        jog_group = MutuallyExclusiveCallbackGroup()
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
            self, MoveJoint, 'move_joint', execute_callback=self.execute_callback,
            callback_group=normal_group)
        self.jog_joint_service = self.create_service(
            JogJoint, 'jog_joint', self.jog_joint_callback, callback_group=jog_group)
        self.emergency_stop_service = self.create_service(
            Trigger, 'emergency_stop', self.emergency_stop_callback,
            callback_group=emergency_group)
        self.stop_motion_service = self.create_service(
            Trigger, 'stop_motion', self.stop_motion_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        # Modo arrastar (mover o robô com a mão): grupo próprio, para o
        # desligar nunca esperar atrás de outra coisa.
        self.arrastar_service = self.create_service(
            SetBool, 'arrastar', self.arrastar_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.get_logger().info('dobot_action_server pronto. Aguardando chamada em /connect.')

    def _verificar_movimento(self, alvo):
        """Ponto único por onde todo movimento passa antes de ser enviado.

        Recusa: drag mode ligado, ou o caminho do MovJ (da partida até o
        `alvo`) saindo do campo de trabalho calculado em qualquer momento.
        Devolve (ok, motivo)."""
        bloqueio = self._bloqueio_arrastar()
        if bloqueio:
            return False, bloqueio
        if self._seguranca is None:
            return False, ('NÃO É SEGURO: o verificador do campo de trabalho não carregou '
                           f'({self._erro_seguranca}). Movimento recusado.')
        inicio = self._partida_do_movimento()
        if inicio is None:
            return False, ('Não deu para ler a posição atual do robô, e sem ela não dá para '
                           'conferir se o caminho é seguro. Movimento recusado.')
        try:
            ok, motivo = self._seguranca.verificar(inicio, alvo)
        except Exception as e:
            ok, motivo = False, f'NÃO É SEGURO: erro ao conferir o caminho ({e}). Recusado.'
        if not ok:
            self.get_logger().warn(f'Movimento recusado para {alvo}: {motivo}')
        return ok, motivo

    def _partida_do_movimento(self):
        """De onde o próximo MovJ começa: a posição atual ou, se o robô ainda
        está indo para o destino anterior (o MovJ novo entra na FILA do Dobot),
        esse destino. None se não der para ler a posição."""
        atual = self._read_angles()
        if atual is None:
            return None
        ultimo = self._ultimo_alvo
        if ultimo is not None and \
                max(self._angle_diff(a, u) for a, u in zip(atual, ultimo)) > TOLERANCIA_GRAUS \
                and not self._robo_parado():
            return list(ultimo)
        return atual

    # --- modo arrastar (mover o robô com a mão) ---

    def _bloqueio_arrastar(self):
        """Mensagem de recusa enquanto o drag mode está ligado ('' se não). No
        robô real, confere no próprio controlador (RobotMode 6 = drag mode): ele
        pode ter entrado ou saído pelo botão do braço, sem passar pela interface."""
        if self.modo != 'sim':
            modo = self._modo_robo()
            if modo is not None:
                self._arrastando = (modo == MODO_DRAG)
        if self._arrastando:
            return ('O robô está em drag mode (sendo movido com a mão). Saia dele (botão '
                    'do robô ou "Desabilitar drag") antes de mandar um movimento.')
        return ''

    def _modo_robo(self):
        """RobotMode do controlador (int), ou None se não der para ler."""
        try:
            res = self._send('RobotMode()', lock_timeout=3.0)
            if not res.startswith('0,'):
                return None
            return int(res.split('{', 1)[1].split('}', 1)[0].strip())
        except (TimeoutError, socket.error, OSError, IndexError, ValueError):
            return None

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
        self.get_logger().warn(f'[DRAG] StopDrag() -> {res.strip()}')
        if res.startswith('0,'):
            return True, 'Drag mode desligado: o robô voltou a segurar a posição.'
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
            response.message = ('O drag mode só existe no robô real: o gêmeo virtual não '
                                'pode ser guiado com a mão.')
            return response
        if self.emergency_stop_flag:
            response.success = False
            response.message = 'Robô em PARADA DE EMERGÊNCIA. Limpe o alarme e reconecte.'
            return response
        if not self._robo_parado():
            response.success = False
            response.message = ('Para entrar em drag mode, o robô precisa estar habilitado '
                                'e parado, sem movimento nem alarme.')
            return response
        try:
            res = self._send('StartDrag()', lock_timeout=3.0)
        except (TimeoutError, socket.error, OSError) as e:
            response.success, response.message = False, f'Sem resposta do robô: {e}'
            return response
        self.get_logger().warn(f'[DRAG] StartDrag() -> {res.strip()}')
        if not res.startswith('0,'):
            erro = self._send('GetErrorID()')
            response.success = False
            response.message = (f'O robô recusou o drag mode: {res.strip()} | '
                                f'GetErrorID: {erro.strip()}')
            return response
        self._arrastando = True
        self._ultimo_alvo = None
        response.success = True
        response.message = 'Drag mode ligado: segure o braço e mova com a mão.'
        return response

    def _robo_parado(self):
        """True só se o robô confirma RobotMode 5 (habilitado, parado). Qualquer
        outra resposta ou falha conta como "pode estar se movendo"."""
        try:
            res = self._send('RobotMode()', lock_timeout=3.0)
            return res.startswith('0,') and res.split('{', 1)[1].split('}', 1)[0].strip() == '5'
        except (TimeoutError, socket.error, OSError, IndexError):
            return False

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

            self.emergency_stop_flag = False
            self._arrastando = False
            self._ultimo_alvo = None
            self._velocidade_pct = None       # a interface manda de novo ao conectar
            self._send('ClearError()')
            self._send('EnableRobot()')
            time.sleep(1)
            modo = self._send('RobotMode()')

            self.get_logger().info(f'Conectado ao robô em {robot_ip}:{dashboard_port}.')
            response.success = True
            response.message = f'Conectado com sucesso ({modo.strip()}).'
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
            response.modo_robo = -1
            return response

        try:
            res = self._send('GetAngle()')
        except (TimeoutError, socket.error, OSError) as e:
            response.success = False
            response.message = f'Sem resposta do robô: {e}'
            response.joints = [0.0] * 6
            response.modo_robo = -1
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
        # O modo do robô vai junto: a interface percebe o drag mode ligado ou
        # desligado pelo botão do braço (RobotMode 6).
        modo = self._modo_robo() if (response.success and self.modo != 'sim') else None
        response.modo_robo = -1 if modo is None else int(modo)
        if modo is not None:
            self._arrastando = (modo == MODO_DRAG)
        return response

    def stop_motion_callback(self, request, response):
        """Para o movimento SEM emergência: Stop() + Continue(), como o
        stop_motion do cr10twin. Stop() freia e limpa a fila de movimentos;
        Continue() deixa a fila pronta para o próximo comando. O robô continua
        habilitado e conectado — não precisa limpar alarme nem reconectar."""
        self._parada_geracao += 1
        self._ultimo_alvo = None

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
        response.message = ('Movimento parado. ' if ok else 'Falha ao parar: ') + \
            ' | '.join(mensagens)
        return response

    def emergency_stop_callback(self, request, response):
        """Roda num grupo/thread dedicado, então esse callback em si nunca fica
        preso atrás de um movimento em andamento. O robô só aceita UMA conexão
        na porta do dashboard, então reaproveita a mesma (self.dashboard) —
        mas com um timeout curto pra não ficar refém se algo travar de vez."""
        self.emergency_stop_flag = True
        self._arrastando = False         # a emergência desabilita o robô
        self._ultimo_alvo = None

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

    def destroy(self):
        self.emergency_stop_flag = True
        if self.dashboard and self._arrastando:
            self._sair_do_arrastar()
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

