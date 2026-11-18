"""Conversão de ângulos entre o controlador do Dobot e o modelo (URDF) do CR10.

O Dobot fala em GRAUS; o URDF/Gazebo em RADIANOS. A junta 1 do URDF tem eixo
"0 0 -1", então o sinal dela é invertido; as juntas 2 a 6 são iguais nos dois
lados. Mesma regra do cr10twin (kinematics.py, verificada no robô real) e da
aba "Robô 3D" desta interface.
"""
import math

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
DOBOT_TO_URDF_SIGN = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


def dobot_deg_to_urdf_rad(q_deg):
    return [math.radians(s * float(v)) for s, v in zip(DOBOT_TO_URDF_SIGN, q_deg)]


def urdf_rad_to_dobot_deg(q_rad):
    return [s * math.degrees(float(v)) for s, v in zip(DOBOT_TO_URDF_SIGN, q_rad)]
