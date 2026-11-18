"""Dashboard SIMULADO do Dobot: responde como a porta 29999 do robô real, mas
move o gêmeo virtual no Gazebo.

O dobot_action_server conversa com o robô por um socket TCP: manda
"MovJ(joint={...})", "GetAngle()" etc. e lê respostas no formato
"ErrorID,{valor},Comando();". No modo simulação ele recebe um objeto desta
classe no lugar do socket (mesmos métodos sendall/recv/settimeout/close), então
TODA a lógica do servidor e da interface — espera de chegada, checagem do home,
STOP, giros — roda igual à do robô real. É isso que torna o teste útil.

O que é aproximado (não é o planejador do controlador Dobot):
  * a duração de um MovJ vem de velocidades máximas de junta típicas do CR10
    vezes o SpeedFactor; o caminho no espaço de juntas é o mesmo (todas as
    juntas saem e chegam juntas), o tempo é só parecido;
  * colisões não param o gêmeo — elas aparecem na tela do Gazebo.
"""
import re
import socket
import threading

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .conversao import JOINT_NAMES, dobot_deg_to_urdf_rad, urdf_rad_to_dobot_deg

TRAJECTORY_TOPIC = '/cr10_group_controller/joint_trajectory'

# Velocidade máxima de cada junta do CR10 (graus/s) a 100% de SpeedFactor.
# Valores típicos da ficha do CR10; servem para o gêmeo levar um tempo da
# ordem do real, não para cronometrar ensaio.
JOINT_MAX_SPEED_DEG_S = [120.0, 120.0, 150.0, 180.0, 180.0, 180.0]
# A trajetória do controlador sai e chega parada; o pico de velocidade dela
# fica ~1,5x acima da média. Este fator mantém o pico dentro do limite.
PEAK_FACTOR = 1.5
MIN_DURATION_S = 0.3

# Limites de junta do modelo (graus, convenção Dobot). Movimento pedido fora
# disso é RECUSADO, como o controlador real recusaria.
JOINT_LIMITS_DEG = [(-359.2, 359.2), (-359.2, 359.2), (-163.9, 163.9),
                    (-359.2, 359.2), (-359.2, 359.2), (-359.2, 359.2)]

# RobotMode() do Dobot: 4 = desabilitado, 5 = habilitado parado,
# 7 = executando, 9 = erro/alarme.
MODE_DISABLED, MODE_ENABLED, MODE_RUNNING, MODE_ERROR = 4, 5, 7, 9


class SimDashboard:
    """Imita o socket do dashboard. Uma instância por servidor; "conectar" de
    novo reaproveita a mesma (open())."""

    def __init__(self, node, callback_group=None):
        self._node = node
        self._pub = node.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        node.create_subscription(JointState, '/joint_states', self._on_joint_states,
                                 10, callback_group=callback_group)
        self._lock = threading.Lock()
        self._q_rad = None            # posição atual do gêmeo, ordem joint1..6
        self._responses = []
        self._open = False
        self.enabled = False
        self.estop = False
        self.speed_percent = 100      # o robô real guarda a última; aqui começa no máximo
        self._moving_until = 0.0

    # --- interface de socket usada pelo dobot_action_server ---

    def open(self):
        with self._lock:
            self._open = True
            self._responses.clear()
        return self

    def settimeout(self, _t):
        pass

    def close(self):
        with self._lock:
            self._open = False
            self._responses.clear()

    def sendall(self, data):
        cmd = data.decode('ascii').strip()
        if not self._open:
            raise OSError('gêmeo virtual: conexão fechada')
        resp = self._handle(cmd)
        with self._lock:
            self._responses.append(resp)

    def recv(self, _n):
        with self._lock:
            if not self._responses:
                raise socket.timeout('gêmeo virtual: sem resposta pendente')
            return self._responses.pop(0).encode('ascii')

    # --- estado do gêmeo ---

    def _on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in JOINT_NAMES):
            with self._lock:
                self._q_rad = [pos[n] for n in JOINT_NAMES]

    def current_deg(self):
        with self._lock:
            q = None if self._q_rad is None else list(self._q_rad)
        return None if q is None else urdf_rad_to_dobot_deg(q)

    def _now(self):
        return self._node.get_clock().now().nanoseconds / 1e9

    def _publish(self, target_deg, duration_s, parar_no_ponto=True):
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = dobot_deg_to_urdf_rad(target_deg)
        if parar_no_ponto:
            point.velocities = [0.0] * len(JOINT_NAMES)
        sec = int(duration_s)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration_s - sec) * 1e9))
        traj.points = [point]
        try:
            self._pub.publish(traj)
        except Exception:  # noqa: BLE001 — ROS já desligando (Ctrl+C)
            return False
        return True

    def _hold(self):
        """Para o gêmeo onde ele está (STOP / emergência / desabilitar)."""
        q = self.current_deg()
        if q is not None:
            self._publish(q, 0.05)
        self._moving_until = 0.0

    def _duration(self, start_deg, target_deg):
        factor = max(self.speed_percent, 1) / 100.0
        t = max(abs(b - a) / (vmax * factor)
                for a, b, vmax in zip(start_deg, target_deg, JOINT_MAX_SPEED_DEG_S))
        return max(MIN_DURATION_S, PEAK_FACTOR * t)

    # --- comandos ---

    @staticmethod
    def _reply(code, value, cmd):
        return f'{code},{{{value}}},{cmd};'

    def _handle(self, cmd):
        name = cmd.split('(', 1)[0]
        args = cmd[len(name) + 1:-1] if cmd.endswith(')') else ''

        if name == 'ClearError':
            return self._reply(0, '', cmd)

        if name == 'EnableRobot':
            if self.estop:
                return self._reply(-1, '', cmd)
            self.enabled = True
            return self._reply(0, '', cmd)

        if name == 'DisableRobot':
            self._hold()
            self.enabled = False
            return self._reply(0, '', cmd)

        if name == 'RobotMode':
            if self.estop:
                mode = MODE_ERROR
            elif not self.enabled:
                mode = MODE_DISABLED
            elif self._now() < self._moving_until:
                mode = MODE_RUNNING
            else:
                mode = MODE_ENABLED
            return self._reply(0, mode, cmd)

        if name == 'SpeedFactor':
            try:
                value = int(float(args))
            except ValueError:
                return self._reply(-1, '', cmd)
            if not 0 <= value <= 100:
                return self._reply(-1, '', cmd)
            self.speed_percent = value
            return self._reply(0, '', cmd)

        if name == 'GetAngle':
            q = self.current_deg()
            if q is None:
                # Gazebo ainda não publicou /joint_states.
                return self._reply(-1, '', cmd)
            return self._reply(0, ','.join(f'{v:.4f}' for v in q), cmd)

        if name == 'MovJ':
            return self._movj(cmd, args)

        if name == 'ServoJ':
            return self._servoj(cmd, args)

        if name == 'Stop':
            self._hold()
            return self._reply(0, '', cmd)

        if name == 'EmergencyStop':
            if args.strip() == '0':
                self.estop = False
            else:
                self.estop = True
                self.enabled = False
                self._hold()
            return self._reply(0, '', cmd)

        if name in ('SetCollisionLevel', 'SetPayload', 'SetBackDistance',
                    'SetPostCollisionMode'):
            # Proteções do controlador real: o gêmeo aceita (para o fluxo de
            # conexão ser o mesmo), mas não sente impacto nem peso.
            self._node.get_logger().info(
                f'[GÊMEO] {cmd} aceito (no gêmeo não tem efeito físico).')
            return self._reply(0, '', cmd)

        if name == 'GetErrorID':
            return self._reply(0, '[[],[],[],[],[],[],[]]', cmd)

        # Comando que o gêmeo não conhece: aceita sem efeito.
        return self._reply(0, '', cmd)

    def _servoj(self, cmd, args):
        """ServoJ(J1,...,J6,t=<s>,aheadtime=<n>,gain=<n>): segue um ponto de
        um percurso contínuo em `t` segundos, sem parar nele (o próximo ponto
        chega antes). Como no robô real, não usa o SpeedFactor."""
        if self.estop or not self.enabled:
            return self._reply(-1, '', cmd)
        partes = [p.strip() for p in args.split(',')]
        try:
            target = [float(v) for v in partes[:len(JOINT_NAMES)]]
            opcoes = dict(p.split('=', 1) for p in partes[len(JOINT_NAMES):])
            t = float(opcoes.get('t', 0.1))
        except ValueError:
            return self._reply(-1, '', cmd)
        if len(target) != len(JOINT_NAMES) or not 0.02 <= t <= 3600.0:
            return self._reply(-1, '', cmd)
        for v, (lo, hi) in zip(target, JOINT_LIMITS_DEG):
            if not lo <= v <= hi:
                self._node.get_logger().error(
                    f'[GÊMEO] ServoJ recusado: {target} fora dos limites das juntas.')
                return self._reply(-1, '', cmd)
        if not self._publish(target, t, parar_no_ponto=False):
            return self._reply(-1, '', cmd)
        self._moving_until = self._now() + t + 0.1
        return self._reply(0, '', cmd)

    def _movj(self, cmd, args):
        if self.estop or not self.enabled:
            return self._reply(-1, '', cmd)
        m = re.search(r'joint=\{([^}]*)\}', args)
        try:
            target = [float(v) for v in m.group(1).split(',')]
        except (AttributeError, ValueError):
            return self._reply(-1, '', cmd)
        if len(target) != len(JOINT_NAMES):
            return self._reply(-1, '', cmd)
        for v, (lo, hi) in zip(target, JOINT_LIMITS_DEG):
            if not lo <= v <= hi:
                self._node.get_logger().error(
                    f'[GÊMEO] MovJ recusado: {target} fora dos limites das juntas.')
                return self._reply(-1, '', cmd)
        start = self.current_deg()
        if start is None:
            return self._reply(-1, '', cmd)
        duration = self._duration(start, target)
        if not self._publish(target, duration):
            return self._reply(-1, '', cmd)
        self._moving_until = self._now() + duration
        self._node.get_logger().info(
            f'[GÊMEO] MovJ {target} em {duration:.1f} s ({self.speed_percent}%)')
        return self._reply(0, '', cmd)
