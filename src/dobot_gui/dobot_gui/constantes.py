"""Valores fixos da interface (compartilhados pela base comum e pelos módulos)."""
import os

APP_NOME = 'IRIS CR10A'
APP_SUBTITULO = 'Interface Robótica de Inspeção Sensorial'
APP_VERSION = '0.1.0'

VELOCIDADE_MAXIMA_PERMITIDA = 50  # espelha o limite de velocidade do dobot_action_server
VELOCIDADE_INICIAL = 5
ROBOT_IP = '192.168.5.2'  # espelha o parâmetro robot_ip padrão do dobot_action_server

JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
JOINT_MIN = -180
JOINT_MAX = 180
# Posição de repouso do robô: valor inicial da home do Controle Juntas.
HOME_JOINTS = [-52, 7, -136, 40, -90, 180]

# Ao conferir se o robô chegou na home (posição REAL lida do robô): nenhuma
# junta pode estar mais longe que isso.
HOME_TOLERANCE_DEG = 1.0
GET_JOINTS_TIMEOUT_SECONDS = 5.0

# Onde ficam salvos os últimos valores usados (juntas e homes), pra continuarem
# os mesmos da próxima vez que a interface for aberta.
SETTINGS_PATH = os.path.expanduser('~/.config/iris/settings.json')

LOG_PATH = os.path.expanduser('~/.config/iris/registro.log')
LOG_HISTORY_LINES = 200  # quantas linhas de sessões anteriores aparecem no Registro

# Detecção de conexão perdida (a leitura de posição funciona como "batimento").
POLL_TIMEOUT_SECONDS = 2.0
POLL_FAILS_BEFORE_LOST = 2

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
