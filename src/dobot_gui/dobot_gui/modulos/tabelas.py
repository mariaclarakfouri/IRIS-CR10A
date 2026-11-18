"""Tabelas no estilo da plataforma (usadas pela Célula de Carga e pelo
Alinhamento da Mesa)."""
import math

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QHeaderView, QTableWidget


def _fmt(v, casas=1):
    return f'{v:.{casas}f}' if isinstance(v, (int, float)) and math.isfinite(v) else '—'


def _tabela(colunas, linhas):
    t = QTableWidget(linhas, len(colunas))
    t.setHorizontalHeaderLabels(colunas)
    t.verticalHeader().setVisible(False)
    t.setEditTriggers(QTableWidget.NoEditTriggers)
    t.setSelectionMode(QTableWidget.NoSelection)
    t.setFocusPolicy(Qt.NoFocus)
    # O cabeçalho é desenhado em negrito (tema): medir com a mesma fonte.
    fonte = t.horizontalHeader().font()
    fonte.setBold(True)
    t.horizontalHeader().setFont(fonte)
    t.horizontalHeader().setDefaultAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    t.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
    t.horizontalHeader().setStretchLastSection(True)
    t.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    t.setShowGrid(False)
    return t


def _ajustar_altura(t):
    altura = t.horizontalHeader().height() + 2
    for i in range(t.rowCount()):
        altura += t.rowHeight(i)
    t.setFixedHeight(altura + 2)
