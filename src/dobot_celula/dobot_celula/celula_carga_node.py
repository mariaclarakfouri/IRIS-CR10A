"""Leitura da célula de carga FA7155 (6 eixos) para a aba Célula de Carga.

Robô real: lê a célula pelo conversor USB-RS485, com o leitor de quadros do
projeto cr10twin (ft_serial.py: cabeçalho + CRC, ressincronização, reconexão
quando o cabo sai e volta). A célula fala sozinha: o nó só escuta.
Simulação: lê o sensor de força simulado do gêmeo (plugin do Gazebo na junta
da célula, /sim/celula/wrench).

Nos dois casos publica:
  /celula/wrench        geometry_msgs/WrenchStamped  filtrado e com a tara (N, N·m)
  /celula/wrench_bruto  geometry_msgs/WrenchStamped  como chegou, sem filtro nem tara
  /celula/estado        std_msgs/String (JSON)       link, taxa, erros, tara (1 Hz)
e escuta:
  /celula/tara          std_msgs/Empty  zera os 6 eixos com a média do último meio segundo
  /celula/limpar_tara   std_msgs/Empty  volta a mostrar o valor sem tara

No IRIS este nó é o dono da porta (`fonte:=serial`, o padrão). A opção
`fonte:=ft_receiver` (só ouvir um ft_receiver do cr10twin em
/ft_sensor/wrench_raw) continua no código, mas não é usada.

A tara é do PROGRAMA (uma subtração aqui) e some ao fechar. Zerar o sensor
por dentro (Set_Zero pelo Modbus) não está aqui de propósito: o mapa de
registradores ainda não foi confirmado (ver o cr10twin) e um endereço errado
pode deixar o sensor inacessível.
"""
import collections
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Empty, String

from .constantes import (
    FT_AXES,
    FT_MODE_CHOICES,
    FT_MODE_POLLED,
    FT_MIN_RATE_HZ,
    FT_NOMINAL_RATE_HZ,
    FT_SERIAL_BAUD,
    ft_max_rate_hz,
)
from .lc_filter import _LoadCellFilter

TOPICO_SIM = '/sim/celula/wrench'
TOPICO_FT_RECEIVER = '/ft_sensor/wrench_raw'
JANELA_TARA_S = 0.5          # média usada pela tara
DADO_VELHO_S = 1.0           # sem amostra há mais que isso = link caído


class CelulaCargaNode(Node):

    def __init__(self):
        super().__init__('celula_carga')
        self.modo = str(self.declare_parameter('modo', 'real').value).strip().lower()
        porta = str(self.declare_parameter('porta', '').value).strip()
        # 0 = automático: tenta 1 Mbaud e 115200 (a de fábrica) até achar a da célula.
        baud = int(self.declare_parameter('baud', 0).value)
        taxa = float(self.declare_parameter('taxa_hz', FT_NOMINAL_RATE_HZ).value)
        # Como a célula entrega os dados: 'polled' (o computador pergunta e ela
        # responde; é o modo em que o exemplar do laboratório funciona, o
        # padrão do cr10twin) ou 'stream' (ela manda sozinha e o computador só
        # escuta).
        modo_leitura = str(self.declare_parameter('modo_leitura', FT_MODE_POLLED).value)
        modo_leitura = modo_leitura.strip().lower()
        if modo_leitura not in FT_MODE_CHOICES:
            modo_leitura = FT_MODE_POLLED
        self._taxa_pub_hz = float(self.declare_parameter('taxa_publicacao_hz', 200.0).value)
        self._frame_id = str(self.declare_parameter('frame_id', 'load_cell_link').value)
        # 'serial' = este nó abre a porta; 'ft_receiver' = só ouve o ft_receiver.
        self._fonte_param = str(self.declare_parameter('fonte', 'serial').value).strip().lower()

        self._pub = self.create_publisher(WrenchStamped, '/celula/wrench', 50)
        self._pub_bruto = self.create_publisher(WrenchStamped, '/celula/wrench_bruto', 50)
        self._pub_estado = self.create_publisher(String, '/celula/estado', 10)
        self.create_subscription(Empty, '/celula/tara', self._on_tara, 10)
        self.create_subscription(Empty, '/celula/limpar_tara', self._on_limpar_tara, 10)

        self._lock = threading.Lock()
        self._filtros = []
        for _ in FT_AXES:
            f = _LoadCellFilter()
            f.set_sensitivity(1.0)       # o sinal já vem em N e N·m
            self._filtros.append(f)
        self._tara = [0.0] * len(FT_AXES)
        self._tarado = False
        self._janela = collections.deque()          # (t, valores filtrados)
        self._chegadas = collections.deque()        # carimbos, para a taxa medida
        self._ultimo_t = None
        self._ultimo_rx = 0.0
        self._ultima_pub = 0.0
        self._amostras = 0
        self._fonte = None
        self._polled = None
        self._modo_leitura = modo_leitura
        self._bytes_antes = 0

        if self.modo != 'sim' and self._fonte_param == 'ft_receiver':
            from rclpy.qos import qos_profile_sensor_data
            self.create_subscription(WrenchStamped, TOPICO_FT_RECEIVER, self._on_sim,
                                     qos_profile_sensor_data)
            self.get_logger().info(
                f'Célula de carga: ouvindo o ft_receiver ({TOPICO_FT_RECEIVER}).')
        elif self.modo == 'sim':
            self.create_subscription(WrenchStamped, TOPICO_SIM, self._on_sim, 50)
            self.get_logger().info(f'Célula de carga: lendo o sensor simulado do gêmeo ({TOPICO_SIM}).')
        else:
            from .ft_serial import FtSerialSource
            limite = ft_max_rate_hz(baud or FT_SERIAL_BAUD)
            if baud and taxa > limite:
                self.get_logger().warn(
                    f'taxa_hz={taxa:.0f} Hz não cabe em {baud} baud (teto {limite:.0f} Hz): os '
                    'quadros vão chegar picotados.')
            self._fonte = FtSerialSource(port=porta or None, baud=baud, rate_hz=taxa,
                                         on_sample=self._on_amostra_serial)
            if modo_leitura == FT_MODE_POLLED:
                # O driver não abre porta: faz as perguntas pela porta da fonte
                # acima, que continua dona da linha (como no cr10twin).
                from .ft_polled import FtPolledDriver
                self._polled = FtPolledDriver(self._fonte, on_sample=self._on_amostra_serial)
                self._fonte.ultimo_dado = lambda: self._polled.last_rx
            if not self._fonte.start():
                self.get_logger().error(f'Célula de carga: não deu para ler a serial ({self._fonte.error}).')
            else:
                if self._polled is not None:
                    self._polled.start()
                self.get_logger().info(
                    'Célula de carga: procurando o conversor USB-RS485 '
                    f'({"porta " + porta if porta else "detecção automática"}, '
                    f'{f"{baud} baud" if baud else "velocidade automática"}, '
                    f'modo {modo_leitura}).')
        self.create_timer(1.0, self._publicar_estado)

    # --- entrada ---

    def _on_amostra_serial(self, _seq, t_us, valores):
        self._processar(t_us * 1e-6, valores)

    def _on_sim(self, msg):
        f, m = msg.wrench.force, msg.wrench.torque
        self._processar(time.monotonic(), (f.x, f.y, f.z, m.x, m.y, m.z))

    def _processar(self, t, valores):
        if not all(math.isfinite(v) for v in valores):
            return
        agora = time.monotonic()
        with self._lock:
            dt = None
            if self._ultimo_t is not None:
                d = t - self._ultimo_t
                if 0.0 < d < 1.0:
                    dt = d
            self._ultimo_t = t
            filtrados = [f.update(v, dt) for f, v in zip(self._filtros, valores)]
            self._janela.append((agora, filtrados))
            while self._janela and agora - self._janela[0][0] > JANELA_TARA_S:
                self._janela.popleft()
            self._chegadas.append(agora)
            while self._chegadas and agora - self._chegadas[0] > 1.0:
                self._chegadas.popleft()
            self._ultimo_rx = agora
            self._amostras += 1
            tarados = [v - z for v, z in zip(filtrados, self._tara)]
        # Publica no máximo `taxa_publicacao_hz` (a célula manda até 1 kHz).
        if agora - self._ultima_pub >= 1.0 / self._taxa_pub_hz:
            self._ultima_pub = agora
            self._publicar(self._pub, tarados)
            self._publicar(self._pub_bruto, valores)

    def _publicar(self, pub, v):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = (float(x) for x in v[:3])
        msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = (float(x) for x in v[3:])
        pub.publish(msg)

    # --- tara ---

    def _on_tara(self, _msg):
        with self._lock:
            if not self._janela or time.monotonic() - self._ultimo_rx > DADO_VELHO_S:
                self.get_logger().warn('Tara recusada: sem leitura recente da célula.')
                return
            n = len(self._janela)
            self._tara = [sum(v[i] for _, v in self._janela) / n for i in range(len(FT_AXES))]
            self._tarado = True
        self.get_logger().info('Tara feita: ' + ', '.join(
            f'{a} {v:+.3f}' for a, v in zip(FT_AXES, self._tara)))

    def _on_limpar_tara(self, _msg):
        with self._lock:
            self._tara = [0.0] * len(FT_AXES)
            self._tarado = False
        self.get_logger().info('Tara limpa.')

    # --- estado ---

    def _publicar_estado(self):
        with self._lock:
            ligado = time.monotonic() - self._ultimo_rx < DADO_VELHO_S
            taxa = float(len(self._chegadas))
            estado = {
                'fonte': 'simulacao' if self.modo == 'sim' else 'real',
                'recebendo': ligado,
                'taxa_hz': taxa,
                'amostras': self._amostras,
                'tarado': self._tarado,
                'tara': list(self._tara),
            }
        if self._fonte is not None:
            p = self._fonte.parser
            bytes_rx = self._fonte.bytes_rx
            estado.update({
                'modo_leitura': self._modo_leitura,
                'baud': self._fonte.baud,
                'baud_auto': self._fonte.baud_auto,
                'bytes_por_s': bytes_rx - self._bytes_antes,     # o estado sai 1×/s
                'porta': self._fonte.port or '',
                'conectado': bool(getattr(self._fonte, 'connected', False)),
                'erro': self._fonte.error or '',
                'erros_crc': p.crc_errors,
                'ressincronizacoes': p.resyncs,
                'valores_invalidos': p.bad_values,
            })
            if self._polled is not None:
                # No polled o que dói é a pergunta sem resposta (timeout), não CRC.
                d = self._polled
                estado.update({'sem_resposta': d.timeouts, 'erros_resposta': d.errors,
                               'valores_invalidos': d.bad_values})
            self._bytes_antes = bytes_rx
            if ligado and taxa < FT_MIN_RATE_HZ:
                estado['aviso'] = (f'taxa baixa ({taxa:.0f} Hz < {FT_MIN_RATE_HZ:.0f} Hz): cabo, '
                                   'baud ou taxa do sensor')
        else:
            topico = TOPICO_SIM if self.modo == 'sim' else TOPICO_FT_RECEIVER
            estado.update({'porta': topico, 'conectado': ligado, 'erro': ''})
            if self.modo != 'sim':
                estado['leitor'] = 'ft_receiver'
        self._pub_estado.publish(String(data=json.dumps(estado)))

    def destroy_node(self):
        # O driver polled antes da porta: ele usa a porta da fonte.
        if self._polled is not None:
            self._polled.stop()
        if self._fonte is not None:
            self._fonte.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CelulaCargaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
