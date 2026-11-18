"""Coletas (aba Coletas): o robô real segue o explorador do cr10twin.

Como no cr10twin, quem planeja e controla a coleta é o `tactile_explorer`
(pacote dobot_coletas): ele move o GÊMEO, publicando a 33 Hz em
/cr10_group_controller/joint_trajectory, e lê a força da célula. No cr10twin o
robô real copia o gêmeo pelo `mirror_node` (ou pelo laço da palpation_gui):
nas fases ativas, um ServoJ a cada 30 ms com a posição de /joint_states.

Aqui essa cópia é feita pelo servidor, que é o único dono da conexão com o
robô, com o mesmo comportamento do mirror_node (fases paradas, banda morta,
t/aheadtime/gain do ServoJ) e, a mais, as proteções do IRIS:

  * a coleta só move o robô real ARMADA pela aba, depois do pré-home e com o
    robô real e o gêmeo na mesma posição (até 0,3°, a tolerância do pré-home
    da palpation_gui). Se o explorador começar a se mexer sem estar armado,
    ele é congelado (/palpation/freeze);
  * cada ponto passa pela verificação de segurança do IRIS (mesa,
    paredes, armários e o próprio braço; a ponteira pode encostar) e pelo
    limite de velocidade de cada junta (50% da máxima, como o percurso
    contínuo). Um ponto recusado congela o explorador e para o robô;
  * Parar, EMERGÊNCIA e Desconectar congelam o explorador e param o robô;
  * enquanto a coleta está armada ou em andamento, nenhum outro movimento é
    aceito.
"""
import json
import math
import socket
import threading
import time

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool

try:
    from touch_pack_msgs.msg import PalpationStatus
except ImportError:                      # pacote das coletas não compilado
    PalpationStatus = None

# Mesmas definições do mirror_node do cr10twin.
FASES_PARADAS = ('IDLE', 'DONE', 'ABORTED')
PERIODO_S = 0.030                        # 33 Hz (servoj_period_s padrão)
BANDA_MORTA_RAD = 1.0e-5                 # SERVOJ_DEADBAND_RAD
# _PREHOME_TOL_RAD da palpation_gui: o primeiro ServoJ descarrega de uma vez a
# diferença entre o gêmeo e o robô real, então ela tem de ser pequena.
TOLERANCIA_SINCRONIA_GRAUS = 0.3
JUNTAS_URDF = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')
SINAL_URDF = (-1.0, 1.0, 1.0, 1.0, 1.0, 1.0)   # só a J1 tem o sinal invertido
GEMEO_MUDO_S = 0.25                      # /joint_states parado há mais que isso: para
ESPERA_INICIO_S = 15.0                   # armada sem o explorador começar: desarma


class ColetaMixin:
    """Parte do DobotActionServer (ver dobot_action_server.py)."""

    def _iniciar_coleta(self, vel_max_junta, vel_max_pct, aheadtime, gain):
        self._col_vel_max = [v * vel_max_pct / 100.0 for v in vel_max_junta]
        self._col_aheadtime = aheadtime
        self._col_gain = gain
        self._col_lock = threading.Lock()
        self._col_armada = False
        self._col_armada_t = 0.0
        self._col_viu_ativa = False
        self._col_fase = 'IDLE'
        self._col_q = None                # posição do gêmeo (rad, URDF)
        self._col_q_t = 0.0
        self._col_ultimo = None           # último ponto mandado (graus, Dobot)
        self._col_mensagem = ''
        self._col_parar = threading.Event()

        grupo = MutuallyExclusiveCallbackGroup()
        self._col_freeze_pub = self.create_publisher(Empty, '/palpation/freeze', 10)
        self._col_estado_pub = self.create_publisher(String, '/dobot/coleta_estado', 10)
        self.create_service(SetBool, 'coleta', self._coleta_callback, callback_group=grupo)
        self.create_subscription(JointState, '/joint_states', self._col_on_juntas, 50,
                                 callback_group=MutuallyExclusiveCallbackGroup())
        if PalpationStatus is not None:
            self.create_subscription(PalpationStatus, '/palpation/status',
                                     self._col_on_status, 10,
                                     callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(0.5, self._col_publicar_estado, callback_group=grupo)
        threading.Thread(target=self._col_laco, daemon=True, name='coleta-servoj').start()

    # --- estado ---

    def _coleta_ativa(self):
        return self._col_fase not in FASES_PARADAS

    def _bloqueio_coleta(self):
        """Mensagem de recusa para outros movimentos ('' se não há coleta)."""
        if self._col_armada or (self._coleta_ativa() and self._col_fase != 'FROZEN'):
            return ('Há uma coleta em andamento (aba Coletas). Espere terminar ou use '
                    'Parar antes de mandar outro movimento.')
        return ''

    def _col_publicar_estado(self):
        estado = {'armada': self._col_armada, 'fase': self._col_fase,
                  'mensagem': self._col_mensagem}
        self._col_estado_pub.publish(String(data=json.dumps(estado, ensure_ascii=False)))

    def _col_avisar(self, texto, erro=False):
        self._col_mensagem = texto
        # Duas linhas separadas de propósito: o rclpy recusa (ValueError) uma
        # mesma linha que registra ora como info, ora como erro.
        if erro:
            self.get_logger().error(f'[COLETA] {texto}')
        else:
            self.get_logger().info(f'[COLETA] {texto}')
        self._col_publicar_estado()

    # --- entradas ---

    def _col_on_juntas(self, msg):
        pos = dict(zip(msg.name, msg.position))
        try:
            q = [float(pos[j]) for j in JUNTAS_URDF]
        except KeyError:
            return
        with self._col_lock:
            self._col_q, self._col_q_t = q, time.monotonic()

    def _col_on_status(self, msg):
        try:
            self._col_tratar_status(msg)
        except Exception as e:         # nunca derrubar o servidor por aqui
            self.get_logger().error(f'[COLETA] erro ao tratar o estado do explorador: {e}')

    def _col_tratar_status(self, msg):
        fase = str(msg.phase or 'IDLE')
        anterior, self._col_fase = self._col_fase, fase
        ativa = fase not in FASES_PARADAS
        if ativa and not self._col_armada and fase != 'FROZEN':
            if anterior in FASES_PARADAS or anterior == 'FROZEN':
                # Coleta começada por fora da aba (sem pré-home nem conferência):
                # o gêmeo se mexeria sem o robô real acompanhar. Congela.
                self._col_freeze_pub.publish(Empty())
                self._col_avisar('O explorador começou a se mexer sem a coleta estar armada '
                                 'pela aba Coletas: congelado.', erro=True)
            return
        if ativa:
            self._col_viu_ativa = True
        elif self._col_armada and self._col_viu_ativa:
            self._col_desarmar(f'Coleta terminada ({fase}).')

    # --- armar / desarmar ---

    def _coleta_callback(self, request, response):
        if request.data:
            ok, texto = self._col_armar()
        else:
            ok, texto = True, self._col_interromper('Coleta desarmada pela interface.')
        response.success, response.message = ok, texto
        return response

    def _col_armar(self):
        if self.dashboard is None:
            return False, 'Robô não conectado.'
        if self.emergency_stop_flag:
            return False, 'Robô em PARADA DE EMERGÊNCIA. Limpe o alarme e reconecte.'
        if self._arrastando:
            return False, self._bloqueio_arrastar()
        if self.seguranca is None:
            return False, f'Segurança não carregou: {self.erro_seguranca}'
        if self.seguranca.ferramenta_contato is None:
            return False, ('A coleta precisa da ferramenta com ponteira (a célula de carga): '
                           'abra com `iris` (célula é o padrão).')
        if PalpationStatus is None:
            return False, 'O pacote das coletas (touch_pack_msgs) não está compilado.'
        if self._coleta_ativa() and self._col_fase != 'FROZEN':
            return False, f'O explorador ainda está em {self._col_fase}.'
        with self._col_lock:
            q, idade = self._col_q, time.monotonic() - self._col_q_t
        if q is None or idade > 1.0:
            return False, 'O gêmeo virtual não está publicando a posição (/joint_states).'
        gemeo = [s * math.degrees(v) for s, v in zip(SINAL_URDF, q)]
        if self.modo == 'real':
            if not self._robo_parado():
                return False, 'O robô precisa estar habilitado e parado para começar a coleta.'
            real = self._read_angles()
            if real is None:
                return False, 'Não consegui ler a posição do robô.'
            dif = max(self._angle_diff(a, b) for a, b in zip(real, gemeo))
            if dif > TOLERANCIA_SINCRONIA_GRAUS:
                return False, (f'O robô real e o gêmeo estão {dif:.2f}° diferentes (máximo '
                               f'{TOLERANCIA_SINCRONIA_GRAUS}°). Leve o robô à home de novo.')
            ok, motivo = self.seguranca.verificar_pose(real, contato_mesa=True)
            if not ok:
                return False, 'BLOQUEADO PELA SEGURANÇA: ' + motivo
            self._col_ultimo = list(real)
        else:
            self._col_ultimo = gemeo
        self._ultimo_alvo = None
        self._col_viu_ativa = False
        self._col_armada_t = time.monotonic()
        self._col_armada = True
        self._col_avisar('Coleta armada: o robô segue o explorador.')
        return True, 'Coleta armada.'

    def _col_desarmar(self, texto, erro=False):
        self._col_armada = False
        self._col_ultimo = None
        self._col_avisar(texto, erro)

    def _col_interromper(self, texto, erro=False):
        """Congela o explorador e, se ele estava movendo o robô, para o robô."""
        estava = self._col_armada or (self._coleta_ativa() and self._col_fase != 'FROZEN')
        self._col_armada = False
        if not estava:
            return texto
        self._col_freeze_pub.publish(Empty())
        if self.modo == 'real' and self.dashboard is not None:
            self._parar_servo()
        self._col_desarmar(texto, erro)
        return texto

    # --- laço do ServoJ (robô real) ---

    def _col_laco(self):
        proximo = time.monotonic() + PERIODO_S
        while not self._col_parar.is_set():
            self._col_parar.wait(max(0.0, proximo - time.monotonic()))
            proximo += PERIODO_S
            if proximo < time.monotonic():
                proximo = time.monotonic() + PERIODO_S
            if not self._col_armada:
                continue
            if not self._col_viu_ativa and self._col_fase in FASES_PARADAS:
                if time.monotonic() - self._col_armada_t > ESPERA_INICIO_S:
                    self._col_desarmar('O explorador não começou a coleta: desarmada.', erro=True)
                continue
            if self._col_fase in FASES_PARADAS or self.modo != 'real':
                continue
            try:
                self._col_passo()
            except Exception as e:      # qualquer falha: para tudo
                self._col_interromper(f'Erro no envio da coleta ({e}): parado.', erro=True)

    def _col_passo(self):
        with self._col_lock:
            q, idade = self._col_q, time.monotonic() - self._col_q_t
        if q is None or idade > GEMEO_MUDO_S:
            self._col_interromper('O gêmeo parou de publicar a posição: coleta parada.', erro=True)
            return
        alvo = [s * math.degrees(v) for s, v in zip(SINAL_URDF, q)]
        ultimo = self._col_ultimo
        if ultimo is not None and max(
                abs(a - b) for a, b in zip(alvo, ultimo)) < math.degrees(BANDA_MORTA_RAD):
            return                                  # parado: sem ServoJ repetido
        if ultimo is not None:
            for j, (a, b, vmax) in enumerate(zip(alvo, ultimo, self._col_vel_max)):
                if abs(a - b) / PERIODO_S > vmax:
                    self._col_interromper(
                        f'J{j + 1} andaria {abs(a - b):.2f}° em {PERIODO_S * 1000:.0f} ms '
                        f'({abs(a - b) / PERIODO_S:.0f}°/s, máximo {vmax:.0f}°/s): '
                        'coleta parada.', erro=True)
                    return
        ok, motivo = self.seguranca.verificar_pose(alvo, contato_mesa=True)
        if not ok:
            self._col_interromper('BLOQUEADO PELA SEGURANÇA: ' + motivo, erro=True)
            return
        cmd = 'ServoJ({},t={:.3f},aheadtime={},gain={})'.format(
            ','.join(f'{v:.6f}' for v in alvo), PERIODO_S, self._col_aheadtime, self._col_gain)
        try:
            res = self._send(cmd, lock_timeout=4 * PERIODO_S, registrar=False,
                             antes_de_enviar=self._col_motivo_para_nao_enviar)
        except RuntimeError as e:
            self._col_interromper(str(e), erro=True)
            return
        except TimeoutError:
            return                       # conexão ocupada neste tique: tenta no próximo
        except (socket.error, OSError) as e:
            self._col_interromper(f'Sem resposta do robô: {e}. Coleta parada.', erro=True)
            return
        if not res.startswith('0,'):
            self._col_interromper(f'O robô recusou o ServoJ: {res.strip()}. Coleta parada.',
                                  erro=True)
            return
        self._col_ultimo = alvo

    def _col_motivo_para_nao_enviar(self):
        if self.emergency_stop_flag:
            return 'PARADA DE EMERGÊNCIA acionada: coleta parada.'
        if not self._col_armada:
            return 'Coleta desarmada.'
        return ''
