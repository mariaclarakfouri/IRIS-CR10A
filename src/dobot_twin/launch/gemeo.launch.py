"""Gêmeo virtual do CR10 no Gazebo.

Argumentos:
    modo      real | sim   (default real)
                real -> o gêmeo SEGUE o robô real (espelho pela porta 30004,
                        só leitura)
                sim  -> o gêmeo é comandado pelo dobot_action_server no lugar
                        do robô real (o espelho não sobe)
    robot_ip  IP do CR10 para o espelho (default 192.168.5.2)
    janela    true | false (default true) — false sobe o Gazebo sem a janela 3D
    celula    false | true (default false)
                true  -> robô com a célula de carga FA7155 e a ponteira no
                         flange (cr10_robot_com_celula.xacro), com o sensor de
                         força simulado publicando em /sim/celula/wrench.
    cenario   laboratorio | vazio  (default laboratorio)
                laboratorio -> o laboratório do cr10twin (research_lab.world):
                               piso, paredes, armários e a mesa onde o robô
                               fica, na mesma posição que lá
                vazio       -> só o chão, robô na origem

Normalmente sobe junto com a interface (dobot_bringup.launch.py). Sozinho:
    ros2 launch dobot_twin gemeo.launch.py modo:=sim
"""
import os
import re
import tempfile

import xacro

# Cenários: arquivo do mundo e onde o robô é colocado (x, y, z). No
# laboratório o robô fica sobre a lab_table, igual ao tactile_cell.launch.py
# do cr10twin (spawn em 0,30 / 0 / 0,75).
CENARIOS = {
    'laboratorio': ('laboratorio.world', ('0.30', '0', '0.75')),
    'vazio': (None, ('0', '0', '0')),
}
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _setup(context):
    modo = context.launch_configurations['modo'].strip().lower()
    robot_ip = context.launch_configurations['robot_ip'].strip()
    janela = context.launch_configurations['janela'].strip().lower()
    com_celula = context.launch_configurations['celula'].strip().lower() in ('true', '1', 'sim')
    cenario = context.launch_configurations['cenario'].strip().lower()
    if cenario not in CENARIOS:
        raise RuntimeError(
            f"cenario deve ser um de {sorted(CENARIOS)}, não {cenario!r}")
    world_name, (x, y, z) = CENARIOS[cenario]

    pkg = get_package_share_directory('dobot_twin')
    modelo = 'cr10_robot_com_celula.xacro' if com_celula else 'cr10_robot.xacro'
    full_urdf = xacro.process_file(os.path.join(pkg, 'urdf', modelo)).toxml()

    # O gazebo_ros2_control lê o robot_description do robot_state_publisher e
    # repassa como argumento de linha de comando; comentários e quebras de
    # linha do URDF quebram esse parser. Mesma solução do cr10twin: o
    # robot_state_publisher recebe um URDF enxuto, e o Gazebo recebe o
    # completo (malhas, colisão, inércia) por arquivo.
    minimal_urdf = full_urdf
    for tag in ('visual', 'collision', 'inertial'):
        minimal_urdf = re.sub(rf'<{tag}\b[^>]*>.*?</{tag}>', '', minimal_urdf, flags=re.DOTALL)
    minimal_urdf = re.sub(r'<gazebo\s+reference\s*=\s*"[^"]*"\s*>.*?</gazebo>', '',
                          minimal_urdf, flags=re.DOTALL)
    minimal_urdf = re.sub(r'<!--.*?-->', '', minimal_urdf, flags=re.DOTALL)
    minimal_urdf = re.sub(r'<\?xml[^?]*\?>', '', minimal_urdf)
    minimal_urdf = ' '.join(minimal_urdf.split())

    fd, spawn_path = tempfile.mkstemp(prefix='dobot_twin_cr10_', suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(full_urdf)

    gazebo_args = {'verbose': 'false', 'gui': janela}
    if world_name:
        gazebo_args['world'] = os.path.join(pkg, 'worlds', world_name)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments=gazebo_args.items())

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': minimal_urdf, 'use_sim_time': True}])

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', spawn_path, '-entity', 'cr10', '-x', x, '-y', y, '-z', z,
                   '-timeout', '120'],
        parameters=[{'use_sim_time': True}])

    spawners = [
        Node(package='controller_manager', executable='spawner',
             arguments=[name, '--controller-manager', '/controller_manager',
                        '--controller-manager-timeout', '120'])
        for name in ('joint_state_broadcaster', 'cr10_group_controller')
    ]

    actions = [gazebo, rsp, spawn, *spawners]

    if modo == 'real':
        actions.append(Node(
            package='dobot_twin',
            executable='gemeo_espelho',
            name='gemeo_espelho',
            output='screen',
            parameters=[{'robot_ip': robot_ip}]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='real'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.5.2'),
        DeclareLaunchArgument('janela', default_value='true'),
        DeclareLaunchArgument('cenario', default_value='laboratorio'),
        DeclareLaunchArgument('celula', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
