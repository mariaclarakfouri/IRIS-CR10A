"""Módulos da plataforma, na ordem em que aparecem na barra lateral.

Para acrescentar um módulo, importe a classe e coloque-a nesta lista
(veja modulos/base.py).
"""
from .celula_carga import CelulaCarga
from .controle_juntas import ControleJuntas
from .registro import Registro
from .robo_3d import Robo3D

MODULOS = [ControleJuntas, CelulaCarga, Robo3D]

# O Registro fica separado, no fim da barra lateral (é da plataforma, não um ensaio).
MODULO_REGISTRO = Registro
