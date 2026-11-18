"""Gráfico do ROBÔ de uma coleta (aba Coletas), salvo ao lado dos dados.

Lê o samples.csv que o registrador do cr10twin grava em cada coleta no robô real
e desenha só o lado do robô: força na ponteira, trajetória da ponteira da
célula e as fases (descida, HOLD, deslizamento...). O sensor tátil tem o seu
próprio gráfico e gravação, na aba Tátil, e não entra aqui.

  grafico_robo.png
    (a) força × tempo, com a força alvo e as fases no fundo
    (b) altura da ponteira × tempo (descida, HOLD, subida)
    (c) trajetória da ponteira vista de cima (cinza no ar, vermelho em contato)
    (d) força × afundamento, um traço por toque
"""
import csv
import os

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from matplotlib.patches import Patch

from .graficos import CORES_SIMPLES, FUNDO, PRETO, _estilizar_simples, _legenda_simples

NOME_ARQUIVO = 'grafico_robo.png'
CONTATO_N = 0.12                      # CONTACT_ON_N do cr10twin
FASES_COM_CONTATO = (1, 2, 9)         # descida, HOLD, onda de força
ESCALA_MIN_XY_MM = 5.0

# Código numérico da coluna `phase` (PHASE_CODES do cr10twin) -> nome e cor de fundo.
FASES = {
    0: ('Home', '#e8e8e8'),
    1: ('Descida', '#fde2c8'),
    2: ('HOLD', '#cfe8cf'),
    3: ('Deslizamento', '#d6e4f5'),
    7: ('Entre pontos', '#ece0f3'),
    8: ('Calibração', '#f6f0c4'),
    9: ('Onda de força', '#cdeee8'),
}


def _ler(caminho):
    with open(caminho, newline='') as fh:
        linhas = list(csv.DictReader(fh))
    if not linhas:
        return None

    def col(nome, conv=float):
        out = []
        for ln in linhas:
            try:
                out.append(conv(ln.get(nome, '')))
            except (TypeError, ValueError):
                out.append(np.nan)
        return np.array(out, dtype=float)

    return {'t': col('t_rel_s'), 'fase': col('phase'), 'ciclo': col('cycle'),
            'forca': col('force_net_n'), 'alvo': col('setpoint_n'),
            'x': col('tcp_x') * 1000.0, 'y': col('tcp_y') * 1000.0,
            'z': col('tcp_z') * 1000.0}


def _trechos(t, fase):
    """[(inicio, fim, codigo)] dos trechos contínuos de mesma fase."""
    out = []
    if not len(t):
        return out
    ini = 0
    for i in range(1, len(t) + 1):
        if i == len(t) or fase[i] != fase[ini]:
            out.append((t[ini], t[i - 1], fase[ini]))
            ini = i
    return out


def _fundo_fases(ax, trechos):
    for a, b, f in trechos:
        if f in FASES and b > a:
            ax.axvspan(a, b, color=FASES[f][1], lw=0, zorder=0)


def gerar_grafico_robo(pasta, titulo=''):
    """Desenha grafico_robo.png em `pasta` (que tem o samples.csv). Devolve o
    caminho do PNG, ou None se não há dados (ex.: coleta de aprendizado)."""
    csv_path = os.path.join(pasta, 'samples.csv')
    if not os.path.isfile(csv_path):
        return None
    d = _ler(csv_path)
    if d is None or not np.isfinite(d['forca']).any():
        return None
    t, fase = d['t'], d['fase']
    trechos = _trechos(t, fase)
    tem_tcp = np.isfinite(d['z']).any()

    fig = Figure(figsize=(13, 9), facecolor=FUNDO)
    FigureCanvasAgg(fig)
    gs = fig.add_gridspec(2, 2, hspace=0.34, wspace=0.24)
    ax_f = fig.add_subplot(gs[0, 0])
    ax_z = fig.add_subplot(gs[0, 1], sharex=ax_f)
    ax_xy = fig.add_subplot(gs[1, 0])
    ax_fd = fig.add_subplot(gs[1, 1])

    # (a) força × tempo
    _fundo_fases(ax_f, trechos)
    ax_f.plot(t, d['forca'], color=CORES_SIMPLES[0], lw=1.2, label='Força medida')
    if np.isfinite(d['alvo']).any():
        ax_f.plot(t, d['alvo'], color=PRETO, lw=1.0, ls='--', label='Força alvo')
    _estilizar_simples(ax_f, 'Força na ponteira (N)', 'Tempo (s)')
    presentes = [f for f in FASES if np.any(fase == f)]
    marcas = [Patch(facecolor=FASES[f][1], edgecolor='none', label=FASES[f][0])
              for f in presentes]
    h, rotulos = ax_f.get_legend_handles_labels()
    _legenda_simples(ax_f, handles=h + marcas,
                     labels=rotulos + [m.get_label() for m in marcas], loc='center left')
    ax_f.set_title('(a) Força e fases', fontsize=10, color=PRETO)

    if tem_tcp:
        # (b) altura da ponteira × tempo
        _fundo_fases(ax_z, trechos)
        z0 = np.nanmax(d['z'])
        ax_z.plot(t, d['z'] - z0, color=CORES_SIMPLES[1], lw=1.2)
        _estilizar_simples(ax_z, 'Altura da ponteira (mm; 0 = ponto mais alto)', 'Tempo (s)')
        ax_z.set_title('(b) Trajetória vertical da ponteira', fontsize=10, color=PRETO)

        # (c) trajetória vista de cima
        x0 = d['x'][np.isfinite(d['x'])][0]
        y0 = d['y'][np.isfinite(d['y'])][0]
        contato = d['forca'] > CONTATO_N
        ax_xy.plot(d['x'] - x0, d['y'] - y0, color='#7f7f7f', lw=1.0, label='No ar')
        ax_xy.plot(np.where(contato, d['x'] - x0, np.nan), np.where(contato, d['y'] - y0, np.nan),
                   color=CORES_SIMPLES[3], lw=2.0, label='Em contato')
        ax_xy.plot([0], [0], 'o', color=PRETO, ms=4, label='Início')
        ax_xy.set_aspect('equal', adjustable='datalim')
        dx = np.nanmax(np.abs(d['x'] - x0))
        dy = np.nanmax(np.abs(d['y'] - y0))
        if max(dx, dy) < ESCALA_MIN_XY_MM:       # toque parado: não ampliar o ruído
            ax_xy.set_xlim(-ESCALA_MIN_XY_MM, ESCALA_MIN_XY_MM)
            ax_xy.set_ylim(-ESCALA_MIN_XY_MM, ESCALA_MIN_XY_MM)
        _estilizar_simples(ax_xy, 'y (mm)', 'x (mm)')
        _legenda_simples(ax_xy, loc='best')
        ax_xy.set_title('(c) Trajetória da ponteira vista de cima', fontsize=10, color=PRETO)

        # (d) força × afundamento, um traço por ciclo (toque)
        curvas = grafico_afundamento(d)
        for i, (c, prof, forca) in enumerate(curvas):
            ax_fd.plot(prof, forca, color=CORES_SIMPLES[i % len(CORES_SIMPLES)], lw=1.2,
                       label=f'Ciclo {c}')
        _estilizar_simples(ax_fd, 'Força na ponteira (N)',
                           'Afundamento depois do contato (mm)')
        if 1 < len(curvas) <= 12:
            _legenda_simples(ax_fd, loc='upper left')
        if not curvas:
            ax_fd.text(0.5, 0.5, 'sem contato nesta coleta', ha='center', va='center',
                       transform=ax_fd.transAxes, color=PRETO)
        ax_fd.set_title('(d) Força × afundamento', fontsize=10, color=PRETO)
    else:
        for ax in (ax_z, ax_xy, ax_fd):
            ax.text(0.5, 0.5, 'sem posição do robô nesta coleta', ha='center', va='center',
                    transform=ax.transAxes, color=PRETO)
            ax.set_axis_off()

    if titulo:
        fig.suptitle(titulo, fontsize=11, color=PRETO)
    destino = os.path.join(pasta, NOME_ARQUIVO)
    fig.savefig(destino, dpi=140, facecolor=FUNDO, bbox_inches='tight')
    return destino


def grafico_afundamento(d):
    """[(ciclo, afundamento_mm, força_N)]: o afundamento conta a partir da
    altura em que a força passou do limiar de contato naquele ciclo."""
    out = []
    for c in np.unique(d['ciclo'][np.isfinite(d['ciclo'])]):
        sel = (d['ciclo'] == c) & np.isfinite(d['z']) & np.isfinite(d['forca'])
        if not sel.any():
            continue
        contato = np.flatnonzero(sel & (d['forca'] > CONTATO_N))
        if not len(contato):
            continue
        z_contato = d['z'][contato[0]]
        # Do contato em diante, só nas fases com a ponteira na amostra
        # (descida, HOLD, onda de força); a subida de volta fica de fora.
        idx = np.arange(len(sel))
        no_contato = sel & (idx >= contato[0]) & np.isin(d['fase'], FASES_COM_CONTATO)
        if no_contato.any():
            out.append((int(c), z_contato - d['z'][no_contato], d['forca'][no_contato]))
    return out
