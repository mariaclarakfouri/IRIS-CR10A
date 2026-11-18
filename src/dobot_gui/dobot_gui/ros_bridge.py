"""Node ROS 2 que roda em segundo plano e fala com o dobot_action_server."""
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool, Trigger

from dobot_interfaces.action import MoveJoint, MoverPonta, SeguirTrajetoria
from dobot_interfaces.msg import EstadoJuntas
from dobot_interfaces.srv import GetJoints, JogJoint, SetSpeed, VerificarCaminho

from .constantes import ROBOT_IP


class RosBridge(Node):

    def __init__(self):
        super().__init__('dobot_gui_node')
        # Vêm do dobot_bringup.launch.py. 'sim' = o backend comanda só o
        # gêmeo virtual, e a janela avisa isso bem visível.
        self.modo = str(self.declare_parameter('modo', 'real').value).strip().lower()
        self.robot_ip = str(self.declare_parameter('robot_ip', ROBOT_IP).value).strip()
        self.connect_client = self.create_client(Trigger, 'connect')
        self.disconnect_client = self.create_client(Trigger, 'disconnect')
        self.clear_alarm_client = self.create_client(Trigger, 'clear_alarm')
        self.jog_client = self.create_client(JogJoint, 'jog_joint')
        self.speed_client = self.create_client(SetSpeed, 'set_speed')
        self.get_joints_client = self.create_client(GetJoints, 'get_joints')
        self.verificar_caminho_client = self.create_client(VerificarCaminho, 'verificar_caminho')
        self.emergency_stop_client = self.create_client(Trigger, 'emergency_stop')
        self.stop_motion_client = self.create_client(Trigger, 'stop_motion')
        self.arrastar_client = self.create_client(SetBool, 'arrastar')
        self.move_joint_client = ActionClient(self, MoveJoint, 'move_joint')
        self.seguir_trajetoria_client = ActionClient(self, SeguirTrajetoria, 'seguir_trajetoria')
        self.mover_ponta_client = ActionClient(self, MoverPonta, 'mover_ponta')
        self._assinantes_estado = []
        # Célula de carga (nó celula_carga, só com a ferramenta célula, o padrão do IRIS).
        self.ferramenta = str(self.declare_parameter('ferramenta', '').value).strip()
        # O que está no flange (celula | sensor | nada): define o desenho do Robô 3D.
        self.ponta = str(self.declare_parameter(
            'ponta', 'celula' if self.ferramenta == 'celula_fa7155' else 'nada').value).strip()
        self._assinantes_celula = []
        self._assinantes_celula_estado = []
        self.create_subscription(WrenchStamped, '/celula/wrench', self._on_celula, 50)
        self.create_subscription(String, '/celula/estado', self._on_celula_estado, 10)
        self._pub_tara = self.create_publisher(Empty, '/celula/tara', 10)
        self._pub_limpar_tara = self.create_publisher(Empty, '/celula/limpar_tara', 10)
        self.create_subscription(EstadoJuntas, '/dobot/estado_juntas', self._on_estado, 10)

    # --- estado das juntas (aba Análise de Juntas) ---

    def assinar_estado(self, callback):
        """`callback(msg)` a cada estado recebido (chamado na thread do ROS)."""
        self._assinantes_estado.append(callback)

    def _on_estado(self, msg):
        for cb in list(self._assinantes_estado):
            cb(msg)

    # --- célula de carga (aba Célula de Carga) ---

    def assinar_celula(self, ao_receber, ao_estado):
        """`ao_receber(msg WrenchStamped)` e `ao_estado(texto JSON)`, chamados na
        thread do ROS."""
        self._assinantes_celula.append(ao_receber)
        self._assinantes_celula_estado.append(ao_estado)

    def _on_celula(self, msg):
        for cb in list(self._assinantes_celula):
            cb(msg)

    def _on_celula_estado(self, msg):
        for cb in list(self._assinantes_celula_estado):
            cb(msg.data)

    def pedir_tara(self):
        self._pub_tara.publish(Empty())

    def pedir_limpar_tara(self):
        self._pub_limpar_tara.publish(Empty())

    # --- chamada genérica de serviço ---

    def _chamar(self, client, nome, request, callback, espera_s, extrair):
        """Chama o serviço e devolve o resultado em `callback` (nunca lança)."""
        if not client.wait_for_service(timeout_sec=espera_s):
            callback(*extrair(None, f'Serviço /{nome} indisponível — o '
                                    'dobot_action_server está rodando?'))
            return
        future = client.call_async(request)

        def _done(f):
            try:
                callback(*extrair(f.result(), None))
            except Exception as e:  # noqa: BLE001
                callback(*extrair(None, f'Erro ao chamar /{nome}: {e}'))

        future.add_done_callback(_done)

    @staticmethod
    def _ok_msg(res, erro):
        return (False, erro) if res is None else (res.success, res.message)

    def _trigger(self, client, nome, callback, espera_s=3.0):
        self._chamar(client, nome, Trigger.Request(), callback, espera_s, self._ok_msg)

    # --- serviços ---

    def call_emergency_stop(self, callback):
        self._trigger(self.emergency_stop_client, 'emergency_stop', callback, 2.0)

    def call_stop_motion(self, callback):
        self._trigger(self.stop_motion_client, 'stop_motion', callback, 2.0)

    def call_arrastar(self, ligar, callback):
        """Liga ou desliga o modo arrastar (mover o robô com a mão).
        `callback(ok, mensagem)`."""
        request = SetBool.Request()
        request.data = bool(ligar)
        self._chamar(self.arrastar_client, 'arrastar', request, callback, 2.0, self._ok_msg)

    def call_connect(self, callback):
        self._trigger(self.connect_client, 'connect', callback)

    def call_clear_alarm(self, callback):
        self._trigger(self.clear_alarm_client, 'clear_alarm', callback)

    def call_disconnect(self, callback):
        self._trigger(self.disconnect_client, 'disconnect', callback)

    def call_jog(self, joints, callback):
        request = JogJoint.Request()
        request.joints = [float(j) for j in joints]
        self._chamar(self.jog_client, 'jog_joint', request, callback, 2.0, self._ok_msg)

    def call_set_speed(self, percent, callback):
        request = SetSpeed.Request()
        request.percent = percent
        self._chamar(self.speed_client, 'set_speed', request, callback, 3.0, self._ok_msg)

    def call_get_joints(self, callback):
        def extrair(res, erro):
            if res is None:
                return False, erro, None
            return res.success, res.message, list(res.joints)
        self._chamar(self.get_joints_client, 'get_joints', GetJoints.Request(),
                     callback, 3.0, extrair)

    def call_verificar_caminho(self, joints, callback):
        """Pergunta ao servidor se o caminho até `joints` (graus) é seguro, sem
        mover nada. `callback(seguro, mensagem)`."""
        request = VerificarCaminho.Request()
        request.joints = [float(j) for j in joints]

        def extrair(res, erro):
            return (False, erro) if res is None else (res.seguro, res.message)
        self._chamar(self.verificar_caminho_client, 'verificar_caminho', request,
                     callback, 3.0, extrair)

    def call_seguir_trajetoria(self, pontos, periodo_s, feedback_callback, done_callback):
        """Percurso contínuo (movimento em 8). `pontos`: lista de 6 juntas
        (graus); `feedback_callback(status, indice)`; `done_callback(ok, msg)`."""
        if not self.seguir_trajetoria_client.wait_for_server(timeout_sec=3.0):
            done_callback(False, 'Servidor da action /seguir_trajetoria indisponível')
            return
        goal = SeguirTrajetoria.Goal()
        goal.pontos = [float(v) for p in pontos for v in p]
        goal.periodo_s = float(periodo_s)

        def _feedback(msg):
            feedback_callback(msg.feedback.status, msg.feedback.indice)

        def _goal_response(f):
            try:
                goal_handle = f.result()
            except Exception as e:  # noqa: BLE001
                done_callback(False, f'Erro ao enviar o percurso: {e}')
                return
            if not goal_handle.accepted:
                done_callback(False, 'O robô recusou o pedido de percurso.')
                return

            def _result(rf):
                try:
                    res = rf.result().result
                    done_callback(res.success, res.message)
                except Exception as e:  # noqa: BLE001
                    done_callback(False, f'Erro ao obter resultado do percurso: {e}')

            goal_handle.get_result_async().add_done_callback(_result)

        self.seguir_trajetoria_client.send_goal_async(
            goal, feedback_callback=_feedback).add_done_callback(_goal_response)

    def call_mover_ponta(self, deslocamento_mm, velocidade_mms, parar_no_contato, limiar_n,
                         tocar_mesa, feedback_callback, done_callback, altura_mm=None):
        """Ponteira em linha reta (medição da mesa). `feedback_callback(status,
        percorrido_mm, forca_n)`; `done_callback(ok, msg, resultado)`, com
        resultado = dict(contato, percorrido_mm, juntas, ponta_mm, forca_n).
        Com `altura_mm`, sobe/desce na vertical até essa altura da ponteira."""
        if not self.mover_ponta_client.wait_for_server(timeout_sec=3.0):
            done_callback(False, 'Servidor da action /mover_ponta indisponível', None)
            return
        goal = MoverPonta.Goal()
        goal.deslocamento_mm = [float(v) for v in deslocamento_mm]
        goal.velocidade_mms = float(velocidade_mms)
        goal.parar_no_contato = bool(parar_no_contato)
        goal.limiar_n = float(limiar_n)
        goal.tocar_mesa = bool(tocar_mesa)
        if altura_mm is not None:
            goal.ate_altura = True
            goal.altura_mm = float(altura_mm)

        def _feedback(msg):
            fb = msg.feedback
            feedback_callback(fb.status, fb.percorrido_mm, fb.forca_n)

        def _goal_response(f):
            try:
                goal_handle = f.result()
            except Exception as e:  # noqa: BLE001
                done_callback(False, f'Erro ao enviar o movimento da ponteira: {e}', None)
                return
            if not goal_handle.accepted:
                done_callback(False, 'O robô recusou o movimento da ponteira.', None)
                return

            def _result(rf):
                try:
                    res = rf.result().result
                    done_callback(res.success, res.message, {
                        'contato': res.contato, 'percorrido_mm': res.percorrido_mm,
                        'juntas': list(res.juntas), 'ponta_mm': list(res.ponta_mm),
                        'forca_n': res.forca_n})
                except Exception as e:  # noqa: BLE001
                    done_callback(False, f'Erro ao obter resultado da ponteira: {e}', None)

            goal_handle.get_result_async().add_done_callback(_result)

        self.mover_ponta_client.send_goal_async(
            goal, feedback_callback=_feedback).add_done_callback(_goal_response)

    def call_move_joint(self, joints, feedback_callback, done_callback):
        if not self.move_joint_client.wait_for_server(timeout_sec=3.0):
            done_callback(False, 'Servidor da action /move_joint indisponível')
            return

        goal = MoveJoint.Goal()
        goal.joints = [float(j) for j in joints]

        def _feedback(feedback_msg):
            feedback_callback(feedback_msg.feedback.status)

        send_goal_future = self.move_joint_client.send_goal_async(
            goal, feedback_callback=_feedback)

        def _goal_response(f):
            try:
                goal_handle = f.result()
            except Exception as e:  # noqa: BLE001
                done_callback(False, f'Erro ao enviar movimento: {e}')
                return
            if not goal_handle.accepted:
                done_callback(False, 'O robô recusou o pedido de movimento.')
                return

            def _result(rf):
                try:
                    res = rf.result().result
                    done_callback(res.success, res.message)
                except Exception as e:  # noqa: BLE001
                    done_callback(False, f'Erro ao obter resultado do movimento: {e}')

            goal_handle.get_result_async().add_done_callback(_result)

        send_goal_future.add_done_callback(_goal_response)
