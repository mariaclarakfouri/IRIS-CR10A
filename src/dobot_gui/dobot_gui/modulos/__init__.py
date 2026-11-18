"""Módulos da plataforma, na ordem em que aparecem na barra lateral.

Para acrescentar um módulo, importe a classe e coloque-a nesta lista
(veja modulos/base.py).
"""
from .alinhamento_mesa import AlinhamentoMesa
from .celula_carga import CelulaCarga
from .coletas import Coletas
from .controle_juntas import ControleJuntas
from .registro import Registro
from .robo_3d import Robo3D
from .sensor_tatil import Tatil4x4, Tatil5x5

MODULOS = [ControleJuntas, CelulaCarga, AlinhamentoMesa, Coletas, Tatil5x5, Tatil4x4, Robo3D]

# O Registro fica separado, no fim da barra lateral (é da plataforma, não um ensaio).
MODULO_REGISTRO = Registro
