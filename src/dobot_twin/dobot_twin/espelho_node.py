"""Espelho robô real -> gêmeo virtual.

Lê a posição real das juntas na porta de FEEDBACK do CR10 (30004, um pacote de
1440 bytes a cada 8 ms) e faz o gêmeo no Gazebo seguir. A porta 30004 é só de
leitura e é separada da porta de comandos (29999), então este nó NÃO disputa a
conexão com o dobot_action_server e não comanda o robô em nada.

Formato do pacote (o mesmo que o real_driver.py do cr10twin usa): começa com
MessageSize = 1440 (uint16 little-endian) e traz q_actual — 6 float64 em graus —
no byte 432.

Durante uma coleta (aba Coletas) quem move o gêmeo é o explorador do cr10twin, e
o robô real é que segue o gêmeo. Então, com a coleta armada ou em andamento
(/dobot/coleta_estado, publicado pelo dobot_action_server), este nó fica quieto.
"""
import json
import math
import socket
import struct
import threading
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .conversao import JOINT_NAMES, dobot_deg_to_urdf_rad

FEEDBACK_PORT = 30004
PACKET_SIZE = 1440
Q_ACTUAL_OFFSET = 432
MARKER = struct.pack('<H', PACKET_SIZE)
RECONNECT_BACKOFF_S = (2.0, 5.0, 10.0)
STALE_AFTER_S = 0.5          # sem pacote novo há mais que isso = não publica
FIRST_MOVE_S = 3.0           # primeira sincronização: o gêmeo vai até a pose real devagar
TRAJECTORY_TOPIC = '/cr10_group_controller/joint_trajectory'
FASES_PARADAS = ('IDLE', 'DONE', 'ABORTED', 'FROZEN')
COLETA_ESTADO_VELHO_S = 2.0  # sem notícia do servidor há mais que isso: volta a espelhar


def parse_q_actual(packet):
    """Devolve as 6 juntas (graus) de um pacote de 1440 B, ou None se inválido."""
    if len(packet) != PACKET_SIZE or packet[:2] != MARKER:
        return None
    q = struct.unpack_from('<6d', packet, Q_ACTUAL_OFFSET)
    if not all(math.isfinite(v) and abs(v) <= 400.0 for v in q):
        return None
    return list(q)


class EspelhoNode(Node):

    def __init__(self):
        super().__init__('gemeo_espelho')
        self.declare_parameter('robot_ip', '192.168.5.2')
        self.declare_parameter('taxa_hz', 30.0)

        self._ip = self.get_parameter('robot_ip').value
        taxa = float(self.get_parameter('taxa_hz').value)
        self._periodo = 1.0 / taxa

        self._pub = self.create_publisher(JointTrajectory, TRAJECTORY_TOPIC, 10)
        self._lock = threading.Lock()
        self._q_deg = None
        self._q_time = 0.0
        self._synced = False
        self._stop = threading.Event()
        self._coleta = False
        self._coleta_t = 0.0
        self.create_subscription(String, '/dobot/coleta_estado', self._on_coleta, 10)

        threading.Thread(target=self._reader_loop, daemon=True).start()
        self.create_timer(self._periodo, self._publish)
        self.get_logger().info(
            f'Espelho do gêmeo: lendo {self._ip}:{FEEDBACK_PORT} (só leitura).')

    # --- leitura da porta 30004 (thread própria) ---

    def _reader_loop(self):
        tentativa = 0
        while not self._stop.is_set():
            try:
                with socket.create_connection((self._ip, FEEDBACK_PORT), timeout=3.0) as s:
                    s.settimeout(2.0)
                    self.get_logger().info('Espelho conectado ao feedback do robô real.')
                    tentativa = 0
                    self._read_packets(s)
            except OSError as e:
                espera = RECONNECT_BACKOFF_S[min(tentativa, len(RECONNECT_BACKOFF_S) - 1)]
                if tentativa == 0:
                    self.get_logger().warn(
                        f'Espelho sem feedback do robô ({e}). O gêmeo fica parado; '
                        f'tentando de novo a cada {espera:.0f}-'
                        f'{RECONNECT_BACKOFF_S[-1]:.0f} s.')
                tentativa += 1
                self._synced = False
                self._stop.wait(espera)

    def _read_packets(self, s):
        buf = b''
        while not self._stop.is_set():
            chunk = s.recv(65536)
            if not chunk:
                raise OSError('robô fechou a porta de feedback')
            buf += chunk
            latest = None
            while len(buf) >= PACKET_SIZE:
                if buf[:2] != MARKER:
                    # Desalinhado: pula até o próximo marcador.
                    idx = buf.find(MARKER, 1)
                    buf = buf[idx:] if idx >= 0 else buf[-1:]
                    continue
                q = parse_q_actual(buf[:PACKET_SIZE])
                buf = buf[PACKET_SIZE:]
                if q is not None:
                    latest = q
            if latest is not None:
                with self._lock:
                    self._q_deg = latest
                    self._q_time = time.monotonic()

    # --- coleta ---

    def _on_coleta(self, msg):
        try:
            estado = json.loads(msg.data)
        except ValueError:
            return
        coleta = bool(estado.get('armada')) or str(estado.get('fase', 'IDLE')) not in FASES_PARADAS
        if coleta and not self._coleta:
            self.get_logger().info('Coleta em andamento: o gêmeo segue o explorador.')
        elif not coleta and self._coleta:
            self.get_logger().info('Coleta encerrada: o gêmeo volta a espelhar o robô real.')
        self._coleta, self._coleta_t = coleta, time.monotonic()

    # --- publicação para o Gazebo ---

    def _publish(self):
        if self._coleta and time.monotonic() - self._coleta_t < COLETA_ESTADO_VELHO_S:
            return
        with self._lock:
            q = None if self._q_deg is None else list(self._q_deg)
            age = time.monotonic() - self._q_time
        if q is None or age > STALE_AFTER_S:
            return
        dur = FIRST_MOVE_S if not self._synced else 2.0 * self._periodo
        self._synced = True

        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = dobot_deg_to_urdf_rad(q)
        sec = int(dur)
        point.time_from_start = Duration(sec=sec, nanosec=int((dur - sec) * 1e9))
        traj.points = [point]
        self._pub.publish(traj)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EspelhoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
