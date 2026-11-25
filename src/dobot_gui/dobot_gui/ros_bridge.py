"""Nó ROS 2 que roda em segundo plano e fala com o dobot_action_server."""
from rclpy.action import ActionClient
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool, Trigger

from dobot_interfaces.action import MoveJoint
from dobot_interfaces.msg import EstadoJuntas
from dobot_interfaces.srv import GetJoints, JogJoint, SetSpeed

from .constantes import ROBOT_IP


class RosBridge(Node):
    """Nó ROS da interface: chama os serviços e a ação do dobot_action_server."""

    def __init__(self):
        super().__init__('dobot_gui_node')
        self.modo = str(self.declare_parameter('modo', 'real').value).strip().lower()
        self.robot_ip = str(self.declare_parameter('robot_ip', ROBOT_IP).value).strip()
        self.connect_client = self.create_client(Trigger, 'connect')
        self.disconnect_client = self.create_client(Trigger, 'disconnect')
        self.clear_alarm_client = self.create_client(Trigger, 'clear_alarm')
        self.jog_client = self.create_client(JogJoint, 'jog_joint')
        self.speed_client = self.create_client(SetSpeed, 'set_speed')
        self.get_joints_client = self.create_client(GetJoints, 'get_joints')
        self.emergency_stop_client = self.create_client(Trigger, 'emergency_stop')
        self.stop_motion_client = self.create_client(Trigger, 'stop_motion')
        self.arrastar_client = self.create_client(SetBool, 'arrastar')
        self.move_joint_client = ActionClient(self, MoveJoint, 'move_joint')
        # O que está no flange (celula | nada): define o desenho do Robô 3D.
        self.ponta = str(self.declare_parameter('ponta', 'nada').value).strip()
        # Estado das juntas (~25 por segundo, nó estado_juntas): o Robô 3D usa
        # para o movimento liso.
        self._assinantes_estado = []
        self.create_subscription(EstadoJuntas, '/dobot/estado_juntas', self._on_estado, 10)
        # Célula de carga (nó celula_carga).
        self._assinantes_celula = []
        self._assinantes_celula_estado = []
        self.create_subscription(WrenchStamped, '/celula/wrench', self._on_celula, 50)
        self.create_subscription(String, '/celula/estado', self._on_celula_estado, 10)
        self._pub_tara = self.create_publisher(Empty, '/celula/tara', 10)
        self._pub_limpar_tara = self.create_publisher(Empty, '/celula/limpar_tara', 10)

    # --- estado das juntas (Robô 3D) ---

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

    def call_emergency_stop(self, callback):
        self._trigger(self.emergency_stop_client, 'emergency_stop', callback, 2.0)

    def call_stop_motion(self, callback):
        self._trigger(self.stop_motion_client, 'stop_motion', callback, 2.0)

    def call_arrastar(self, ligar, callback):
        """Liga ou desliga o drag mode (mover o robô com a mão).
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

    def call_jog(self, joints, callback, experimento=''):
        request = JogJoint.Request()
        request.joints = [float(j) for j in joints]
        request.experimento = experimento
        self._chamar(self.jog_client, 'jog_joint', request, callback, 2.0, self._ok_msg)

    def call_set_speed(self, percent, callback):
        request = SetSpeed.Request()
        request.percent = percent
        self._chamar(self.speed_client, 'set_speed', request, callback, 3.0, self._ok_msg)

    def call_get_joints(self, callback):
        def extrair(res, erro):
            if res is None:
                return False, erro, None
            # O modo do robô (RobotMode) chega junto: 6 = drag mode.
            self.ultimo_modo_robo = int(getattr(res, 'modo_robo', -1))
            return res.success, res.message, list(res.joints)
        self._chamar(self.get_joints_client, 'get_joints', GetJoints.Request(),
                     callback, 3.0, extrair)

    def call_move_joint(self, joints, feedback_callback, done_callback, experimento=''):
        if not self.move_joint_client.wait_for_server(timeout_sec=3.0):
            done_callback(False, 'Servidor da action /move_joint indisponível')
            return

        goal = MoveJoint.Goal()
        goal.joints = [float(j) for j in joints]
        goal.experimento = experimento

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
