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
              dela (vale também no robô real). Não dá para usar junto com sensor:=true.

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
