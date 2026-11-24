"""Sobe o servidor do robô + a interface (+ o gêmeo virtual).

Argumentos:
    modo      real | sim  (default real)
                real -> comanda o robô de verdade
                sim  -> comanda SÓ o gêmeo virtual no Gazebo; o robô real não
                        é tocado. Use para testar antes.
    gemeo     true | false  (default true)
                No modo real, abre o gêmeo seguindo o robô em tempo real.
                No modo sim o gêmeo sempre abre (é ele que se move).
    robot_ip  IP do CR10 (default 192.168.5.2)
    cenario   laboratorio | vazio  (default laboratorio) — cenário do gêmeo
    janela    true | false (default true) — false abre o gêmeo sem a janela 3D
    celula    true | false (default false) — célula de carga FA7155 com a ponteira na
              ponta: o nó celula_carga lê a célula (no gêmeo, a força simulada) e o
              gêmeo e o Robô 3D mostram a célula. false = nada na ponta.

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
    celula = context.launch_configurations['celula'].strip().lower() in ('true', '1', 'sim')

    if modo not in ('real', 'sim'):
        raise RuntimeError(f"modo deve ser 'real' ou 'sim', não {modo!r}")

    actions = [
        # Estado das juntas (ângulo, velocidade...) ~25 vezes por segundo, só
        # leitura: deixa o Robô 3D liso.
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
            # 'ponta': com a célula, a ponta dela também não pode sair do campo.
            parameters=[{'robot_ip': robot_ip, 'modo': modo,
                         'ponta': 'celula' if celula else 'nada'}],
        ),
        Node(
            package='dobot_gui',
            executable='dobot_gui',
            name='dobot_gui_node',
            output='screen',
            parameters=[{'modo': modo, 'robot_ip': robot_ip,
                         'ponta': 'celula' if celula else 'nada'}],
        ),
    ]
    if celula:
        # Leitura da célula (real: conversor USB-RS485; sim: sensor simulado).
        actions.append(Node(
            package='dobot_celula',
            executable='celula_carga',
            name='celula_carga',
            output='screen',
            parameters=[{'modo': modo}],
        ))
    if gemeo or modo == 'sim':
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('dobot_twin'), 'launch', 'gemeo.launch.py')),
            launch_arguments={'modo': modo, 'robot_ip': robot_ip,
                              'cenario': cenario, 'janela': janela,
                              'celula': 'true' if celula else 'false'}.items()))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='real'),
        DeclareLaunchArgument('gemeo', default_value='true'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.5.2'),
        DeclareLaunchArgument('cenario', default_value='laboratorio'),
        DeclareLaunchArgument('janela', default_value='true'),
        DeclareLaunchArgument('celula', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
