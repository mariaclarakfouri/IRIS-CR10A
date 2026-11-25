"""Publica o estado das 6 juntas em /dobot/estado_juntas (para a aba Análise de Juntas).

Robô real: lê a porta de FEEDBACK do CR10 (30004, um pacote de 1440 bytes a
cada 8 ms). É só leitura e separada da porta de comandos (29999): não disputa a
conexão com o dobot_action_server nem comanda nada.

Posições no pacote (bytes), pelo mapa de registros Modbus do Dobot ("Appendix A
Modbus Register Definition", incluído no projeto cr10twin): o pacote da 30004
tem a mesma ordem dos registros de entrada, com byte = (registro - 1000) x 2.
Conferido com os dois campos que o cr10twin já usa (ângulos no byte 432 e
força no 720) e, a cada pacote, com o valor de teste fixo do byte 48.

Simulação: ângulo e velocidade vêm de /joint_states do gêmeo virtual; corrente,
temperatura e torque não existem no gêmeo e são enviados como NaN.
"""
import math
import socket
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from dobot_interfaces.msg import EstadoJuntas

FEEDBACK_PORT = 30004
PACKET_SIZE = 1440
MARCADOR = struct.pack('<H', PACKET_SIZE)          # byte 0: MessageSize
VALOR_TESTE = 0x0123456789ABCDEF                    # byte 48: "Memory structure test"
OFFSET_TESTE = 48
OFFSET_MODO_ROBO = 24     # RobotMode (uint64): 6 = drag mode
OFFSET_VELOCIDADE_GLOBAL = 64  # SpeedScaling: a velocidade global que o robô está usando
OFFSET_ANGULO = 432       # Actual joint position (registro 1216)
OFFSET_VELOCIDADE = 480   # Actual joint speed (1240)
OFFSET_CORRENTE = 528     # Actual joint current (1264)
OFFSET_TEMPERATURA = 864  # Joint temperature (1432)
OFFSET_TORQUE = 1120      # Actual torque (1560)
RECONEXAO_S = (2.0, 5.0, 10.0)

JUNTAS_URDF = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
# Mesma convenção do gêmeo: só a junta 1 tem o sinal invertido.
SINAL_URDF = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
NAN6 = [float('nan')] * 6


def ler_pacote(pacote):
    """Dicionário com os campos de um pacote de 1440 B, ou None se o pacote
    não passar nas conferências (tamanho, marcador, valor de teste, faixas)."""
    if len(pacote) != PACKET_SIZE or pacote[:2] != MARCADOR:
        return None
    if struct.unpack_from('<Q', pacote, OFFSET_TESTE)[0] != VALOR_TESTE:
        return None

    def seis(offset):
        return list(struct.unpack_from('<6d', pacote, offset))

    dados = {
        'angulo': seis(OFFSET_ANGULO),
        'velocidade': seis(OFFSET_VELOCIDADE),
        'corrente': seis(OFFSET_CORRENTE),
        'temperatura': seis(OFFSET_TEMPERATURA),
        'torque': seis(OFFSET_TORQUE),
        'velocidade_global': struct.unpack_from('<d', pacote, OFFSET_VELOCIDADE_GLOBAL)[0],
        'modo_robo': struct.unpack_from('<Q', pacote, OFFSET_MODO_ROBO)[0],
    }
    if not 0 <= dados['modo_robo'] <= 20:
        dados['modo_robo'] = -1
    if not all(math.isfinite(v) and abs(v) <= 400.0 for v in dados['angulo']):
        return None
    # Esforço fora de qualquer faixa plausível = campo que este firmware não
    # preenche: vira NaN (aparece como "sem dado"), em vez de um número errado.
    for chave, limite in (('corrente', 100.0), ('temperatura', 150.0), ('torque', 1000.0)):
        if not all(math.isfinite(v) and abs(v) <= limite for v in dados[chave]):
            dados[chave] = list(NAN6)
    return dados


class EstadoJuntasNode(Node):

    def __init__(self):
        super().__init__('estado_juntas')
        self.declare_parameter('modo', 'real')
        self.declare_parameter('robot_ip', '192.168.5.2')
        self.declare_parameter('taxa_hz', 25.0)
        self.modo = str(self.get_parameter('modo').value).strip().lower()
        self.ip = str(self.get_parameter('robot_ip').value)
        taxa = float(self.get_parameter('taxa_hz').value)

        self.pub = self.create_publisher(EstadoJuntas, '/dobot/estado_juntas', 10)
        self._lock = threading.Lock()
        self._ultimo = None
        self._vel_global = None        # última SpeedScaling lida (só para o registro)
        self._ultimo_t = 0.0
        self._parar = threading.Event()

        if self.modo == 'sim':
            self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
            self.get_logger().info('Estado das juntas: lendo o gêmeo virtual (/joint_states).')
        else:
            threading.Thread(target=self._ler_feedback, daemon=True).start()
            self.get_logger().info(
                f'Estado das juntas: lendo {self.ip}:{FEEDBACK_PORT} (só leitura).')
        self.create_timer(1.0 / taxa, self._publicar)

    # --- simulação ---

    def _on_joint_states(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if not all(n in pos for n in JUNTAS_URDF):
            return
        angulo = [s * math.degrees(pos[n]) for s, n in zip(SINAL_URDF, JUNTAS_URDF)]
        # O gêmeo move as juntas por posição e informa velocidade 0: a
        # velocidade é calculada pela variação do ângulo no tempo da simulação
        # (carimbo da mensagem), com uma suavização leve.
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        anterior = getattr(self, '_sim_anterior', None)
        velocidade = getattr(self, '_sim_vel', [0.0] * 6)
        if anterior is not None and t > anterior[0]:
            dt = t - anterior[0]
            bruta = [(a - b) / dt for a, b in zip(angulo, anterior[1])]
            velocidade = [0.6 * v + 0.4 * b for v, b in zip(velocidade, bruta)]
        self._sim_anterior = (t, angulo)
        self._sim_vel = velocidade
        dados = {
            'angulo': angulo,
            'velocidade': velocidade,
            'corrente': list(NAN6), 'temperatura': list(NAN6), 'torque': list(NAN6),
        }
        with self._lock:
            self._ultimo, self._ultimo_t = dados, time.monotonic()

    # --- robô real ---

    def _ler_feedback(self):
        tentativa = 0
        while not self._parar.is_set():
            try:
                with socket.create_connection((self.ip, FEEDBACK_PORT), timeout=3.0) as s:
                    s.settimeout(2.0)
                    self.get_logger().info('Estado das juntas: conectado ao feedback do robô.')
                    tentativa = 0
                    self._ler_pacotes(s)
            except OSError as e:
                espera = RECONEXAO_S[min(tentativa, len(RECONEXAO_S) - 1)]
                if tentativa == 0:
                    self.get_logger().warn(f'Estado das juntas: sem feedback do robô ({e}).')
                tentativa += 1
                self._parar.wait(espera)

    def _ler_pacotes(self, s):
        buf = b''
        rejeitados = 0
        while not self._parar.is_set():
            chunk = s.recv(65536)
            if not chunk:
                raise OSError('robô fechou a porta de feedback')
            buf += chunk
            ultimo = None
            while len(buf) >= PACKET_SIZE:
                if buf[:2] != MARCADOR:
                    i = buf.find(MARCADOR, 1)
                    buf = buf[i:] if i >= 0 else buf[-1:]
                    continue
                dados = ler_pacote(buf[:PACKET_SIZE])
                buf = buf[PACKET_SIZE:]
                if dados is None:
                    rejeitados += 1
                    if rejeitados in (1, 100, 10000):
                        self.get_logger().warn(
                            f'Estado das juntas: {rejeitados} pacote(s) recusado(s) na '
                            'conferência (valor de teste ou faixa).')
                else:
                    ultimo = dados
            if ultimo is not None:
                # Registra quando a velocidade global do robô muda: confere se o
                # SpeedFactor mandado pela interface é o que o robô está usando.
                vg = ultimo['velocidade_global']
                if math.isfinite(vg) and (self._vel_global is None
                                          or abs(vg - self._vel_global) > 1e-6):
                    self._vel_global = vg
                    self.get_logger().info(f'Velocidade global no robô (SpeedScaling): {vg:g}')
                with self._lock:
                    self._ultimo, self._ultimo_t = ultimo, time.monotonic()

    # --- publicação ---

    def _publicar(self):
        with self._lock:
            dados, t = self._ultimo, self._ultimo_t
        if dados is None or time.monotonic() - t > 1.0:
            return
        msg = EstadoJuntas()
        msg.stamp = self.get_clock().now().to_msg()
        msg.fonte = 'simulacao' if self.modo == 'sim' else 'real'
        msg.angulo_graus = [float(v) for v in dados['angulo']]
        msg.velocidade_graus_s = [float(v) for v in dados['velocidade']]
        msg.corrente_a = [float(v) for v in dados['corrente']]
        msg.temperatura_c = [float(v) for v in dados['temperatura']]
        msg.torque_nm = [float(v) for v in dados['torque']]
        msg.modo_robo = int(dados.get('modo_robo', -1))
        msg.esforco_disponivel = any(math.isfinite(v) for v in dados['corrente'] + dados['temperatura'])
        self.pub.publish(msg)

    def destroy_node(self):
        self._parar.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EstadoJuntasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
