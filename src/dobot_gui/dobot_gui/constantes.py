"""Valores fixos da interface (compartilhados pela base comum e pelos módulos)."""
import os

APP_NOME = 'IRIS CR10A'
APP_SUBTITULO = 'Interface Robótica de Inspeção Sensorial'
APP_VERSION = '0.1.0'

VELOCIDADE_MAXIMA_PERMITIDA = 50  # espelha o limite de segurança do dobot_action_server
VELOCIDADE_INICIAL = 5
ROBOT_IP = '192.168.5.2'  # espelha o parâmetro robot_ip padrão do dobot_action_server

JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
JOINT_MIN = -180
JOINT_MAX = 180
# Posição home do projeto original (sensores_inerciais.py) — usada como valor
# inicial dos controles, assumindo que o robô está nessa posição ao conectar.
HOME_JOINTS = [-52, 7, -136, 40, -90, 180]

# Sequência de rotação inercial (sensores_inerciais.py original): índices 5, 4, 3
# correspondem a J6, J5, J4 (lista de juntas é 0-indexada).
ROTATION_SEQUENCE = [(5, 'J6'), (4, 'J5'), (3, 'J4')]

# Tempos de espera padrão da rotação inercial (segundos), ajustáveis na interface.
WAIT_SECONDS_MAX = 60
WAIT_BETWEEN_JOINTS_DEFAULT = 5
WAIT_AT_180_DEFAULT = 5
WAIT_AT_NEG180_DEFAULT = 5
ESPERA_ENTRE_JUNTAS = 'Entre juntas'
ESPERA_EM_180 = 'Ao chegar em 180°'
ESPERA_EM_NEG180 = 'Ao chegar em -180°'

# Giro repetido de UM eixo: quantas vezes cada eixo faz o vai e volta
# 180° -> -179° -> 180°.
REPEAT_MAX = 50
REPEAT_DEFAULT = 1
# Modo "por tempo": o eixo repete o vai e volta até completar este tempo
# (minutos), contado pelo relógio. Sempre começa um novo giro enquanto o tempo
# não acabou e nunca para no meio de um — então termina um pouco DEPOIS do
# tempo pedido (pelo menos esse tempo de coleta).
TEMPO_MIN_MAX = 240
TEMPO_MIN_DEFAULT = 30
# Pasta de tudo o que a plataforma grava (cada tipo de dado numa subpasta).
REGISTROS_DIR = os.path.expanduser('~/iris_registros')

# Ao conferir se o robô chegou na home (posição REAL lida do robô): nenhuma
# junta pode estar mais longe que isso.
HOME_TOLERANCE_DEG = 1.0
GET_JOINTS_TIMEOUT_SECONDS = 5.0

# Onde ficam salvos os últimos valores usados (juntas e tempos de espera),
# pra continuarem os mesmos da próxima vez que a interface for aberta.
SETTINGS_PATH = os.path.expanduser('~/.config/iris/settings.json')

LOG_PATH = os.path.expanduser('~/.config/iris/registro.log')
LOG_HISTORY_LINES = 200  # quantas linhas de sessões anteriores aparecem no Registro

# Detecção de conexão perdida (a leitura de posição funciona como "batimento").
POLL_TIMEOUT_SECONDS = 2.0
POLL_FAILS_BEFORE_LOST = 2

# Sensor inercial S02 preso no flange (mesmas medidas do gêmeo e da segurança):
# usado para desenhá-lo no modelo 3D da interface.
SENSOR_S02_MM = (65.36, 38.13, 23.10)

# Estados do robô: (texto, cor, aviso mostrado nos controles bloqueados).
ROBOT_STATES = {
    'disconnected': ('Desconectado', '#94a3b8', 'Conecte ao robô primeiro.'),
    'connected': ('Conectado', '#22c55e', ''),
    'moving': ('Em movimento', '#3b82f6', 'Aguarde o movimento terminar.'),
    'lost': ('Sem resposta do robô', '#f97316',
             'Sem resposta do robô: verifique o cabo e a rede.'),
    'alarm': ('Alarme limpo — clique em Conectar', '#f59e0b',
              'Clique em Conectar para reabilitar o robô.'),
    'emergency': ('Emergência', '#ef4444',
                  'Robô em emergência: limpe o alarme e reconecte.'),
}
