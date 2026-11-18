"""Sobe o servidor do robô + a interface (+ o gêmeo virtual).

Argumentos:
    modo      real | sim  (default real)
                real -> comanda o robô de verdade
                sim  -> comanda SÓ o gêmeo virtual no Gazebo; o robô real não
                        é tocado. Use para testar sequências antes.
    gemeo     true | false  (default true)
                No modo real, abre o gêmeo seguindo o robô em tempo real.
                No modo sim o gêmeo sempre abre (é ele que se move).
    robot_ip  IP do CR10 (default 192.168.5.2)
    cenario   laboratorio | vazio  (default laboratorio) — cenário do gêmeo
    janela    true | false (default true) — false abre o gêmeo sem a janela 3D
    sensor    false | true (default false) — gêmeo com o sensor inercial S02 na ponta
    celula    false | true (default false) — célula de carga FA7155 com a ponteira na
              ponta: gêmeo com a célula, SEGURANÇA e peso do robô com a geometria
              dela (vale também no robô real) e o nó celula_carga lendo a célula
              (no gêmeo, a força simulada). Não dá para usar junto com sensor:=true.

Exemplos:
    ros2 launch dobot_control dobot_bringup.launch.py
    ros2 launch dobot_control dobot_bringup.launch.py modo:=sim
    ros2 launch dobot_control dobot_bringup.launch.py gemeo:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


# --- Coletas (aba Coletas): os nós do cr10twin, pacote dobot_coletas ---
# Os valores são os padrões do tactile_cell.launch.py do cr10twin.
PASTA_COLETAS = os.path.expanduser('~/iris_registros/coletas')
PASTA_CONFIG_COLETAS = os.path.expanduser('~/.config/iris_coletas')
SERVOJ_PERIODO_S = 0.030


def _nos_coletas(modo, com_gemeo):
    """Leitor da célula (contrato /load_cell/*), explorador e registrador."""
    try:
        get_package_share_directory('dobot_coletas')
    except Exception:
        return []                      # pacote das coletas não compilado
    env = {'TOUCH_PACK_DATA_DIR': PASTA_COLETAS,
           'TOUCH_PACK_CONFIG_DIR': PASTA_CONFIG_COLETAS}
    nos = []
    if modo == 'sim':
        # Força do sensor simulado do gêmeo (o IRIS publica em /sim/celula).
        nos.append(Node(package='dobot_coletas', executable='sim_force_bridge',
                        name='sim_force_bridge', output='screen', additional_env=env,
                        parameters=[{'use_sim_time': True}],
                        remappings=[('/sim/load_cell/wrench', '/sim/celula/wrench')]))
    else:
        # Dono da porta da célula, com os padrões do cr10twin.
        nos.append(Node(package='dobot_coletas', executable='ft_receiver',
                        name='ft_receiver', output='screen', additional_env=env,
                        parameters=[{'ft_serial_port': '', 'ft_mode': 'polled',
                                     'ft_baud': 1000000, 'ft_autozero': False,
                                     'ft_filter': True}]))
    if not com_gemeo:
        return nos                     # o explorador move o gêmeo: sem ele, sem coletas
    nos.append(Node(package='dobot_coletas', executable='tactile_explorer',
                    name='tactile_explorer', output='screen', additional_env=env,
                    parameters=[{'arm_base_z': 0.78, 'use_sim_time': True,
                                 'learned_contact_persist': False,
                                 'servoj_period_s': SERVOJ_PERIODO_S}]))
    if modo == 'sim':
        return nos                     # no gêmeo nada é gravado (a força é simulada)
    nos.append(Node(package='dobot_coletas', executable='palpation_logger',
                    name='palpation_logger', output='screen', additional_env=env,
                    parameters=[{'sensor': '5'}]))
    return nos


def _setup(context):
    modo = context.launch_configurations['modo'].strip().lower()
    gemeo = context.launch_configurations['gemeo'].strip().lower() in ('true', '1', 'sim')
    robot_ip = context.launch_configurations['robot_ip'].strip()
    cenario = context.launch_configurations['cenario'].strip()
    janela = context.launch_configurations['janela'].strip()
    sensor = context.launch_configurations['sensor'].strip()
    celula = context.launch_configurations['celula'].strip().lower() in ('true', '1', 'sim')
    if celula and sensor.lower() in ('true', '1', 'sim'):
        raise RuntimeError('escolha sensor:=true OU celula:=true: só cabe uma ferramenta na ponta')
    # Perfil de ferramenta do seguranca.yaml ('' = sensor S02, o padrão).
    ferramenta = 'celula_fa7155' if celula else ''

    if modo not in ('real', 'sim'):
        raise RuntimeError(f"modo deve ser 'real' ou 'sim', não {modo!r}")

    actions = [
        # Estado das juntas (ângulo, velocidade, corrente, temperatura, torque)
        # para a aba Análise de Juntas. Só leitura.
        Node(
            package='dobot_control',
            executable='estado_juntas',
            name='estado_juntas',
            output='screen',
            parameters=[{'robot_ip': robot_ip, 'modo': modo}],
        ),
        Node(
            package='dobot_control',
            executable='dobot_action_server',
            name='dobot_action_server',
            output='screen',
            parameters=[{'robot_ip': robot_ip, 'modo': modo, 'ferramenta': ferramenta}],
        ),
        Node(
            package='dobot_gui',
            executable='dobot_gui',
            name='dobot_gui_node',
            output='screen',
            # `ponta`: o que o Robô 3D desenha no flange (a ferramenta com que abriu).
            parameters=[{'modo': modo, 'robot_ip': robot_ip, 'ferramenta': ferramenta,
                         'ponta': ('celula' if celula else
                                   'sensor' if sensor.lower() in ('true', '1', 'sim') else 'nada')}],
        ),
    ]
    if celula:
        # Leitura da célula para a aba Célula de Carga. No robô real quem abre a
        # porta é o ft_receiver do cr10twin (abaixo), e este nó só o ouve.
        actions.append(Node(
            package='dobot_celula',
            executable='celula_carga',
            name='celula_carga',
            output='screen',
            parameters=[{'modo': modo, 'fonte': 'ft_receiver'}],
        ))
        actions += _nos_coletas(modo, gemeo or modo == 'sim')

    if gemeo or modo == 'sim':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('dobot_twin'), 'launch', 'gemeo.launch.py')),
            launch_arguments={'modo': modo, 'robot_ip': robot_ip,
                              'cenario': cenario, 'janela': janela,
                              'sensor': sensor,
                              'celula': 'true' if celula else 'false'}.items()))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='real'),
        DeclareLaunchArgument('gemeo', default_value='true'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.5.2'),
        DeclareLaunchArgument('cenario', default_value='laboratorio'),
        DeclareLaunchArgument('janela', default_value='true'),
        DeclareLaunchArgument('sensor', default_value='false'),
        DeclareLaunchArgument('celula', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
