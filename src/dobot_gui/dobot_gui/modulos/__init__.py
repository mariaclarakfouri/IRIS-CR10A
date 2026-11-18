"""Módulos da plataforma, na ordem em que aparecem na barra lateral.

Para acrescentar um módulo, importe a classe e coloque-a nesta lista
(veja modulos/base.py).
"""
from .controle_juntas import ControleJuntas
from .registro import Registro

MODULOS = [ControleJuntas]

# O Registro fica separado, no fim da barra lateral (é da plataforma, não um ensaio).
MODULO_REGISTRO = Registro
