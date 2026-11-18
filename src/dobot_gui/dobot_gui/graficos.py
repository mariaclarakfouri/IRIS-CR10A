"""Gráficos (matplotlib embutido no Qt) no estilo da plataforma.

Regras seguidas (guia de visualização de dados do projeto):
  * cada junta tem SEMPRE a mesma cor, na ordem fixa J1..J6 (paleta categórica
    de referência, ordem validada para séries vizinhas);
  * linhas de 2 px, grade e eixos discretos, um eixo y por gráfico (nunca dois);
  * legenda sempre que há mais de uma série, e textos na cor de texto (a cor
    identifica só a marca);
  * ao passar o mouse: linha vertical e os valores de todas as séries naquele
    ponto. Os valores também aparecem em tabela ao lado (três cores da paleta
    têm pouco contraste com o branco, então a cor nunca é a única pista).
"""
import math

import numpy as np
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.ticker import MaxNLocator
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QIcon, QPainter, QPixmap
from PyQt5.QtWidgets import QSizePolicy

# Paleta categórica de referência, ordem fixa (slot 1 = J1 ... slot 6 = J6).
CORES_SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300']
TEXTO = '#0f172a'
TEXTO_FRACO = '#64748b'
GRADE = '#eef1f5'
EIXO = '#cbd5e1'
FUNDO = '#ffffff'


def icone_cor(cor, tamanho=10):
    """Bolinha colorida para usar em tabelas e legendas Qt."""
    pix = QPixmap(tamanho * 2, tamanho * 2)
    pix.setDevicePixelRatio(2)
    pix.fill(Qt.transparent)
    p = QPainter(pix)
    p.setRenderHint(QPainter.Antialiasing)
    p.setBrush(QColor(cor))
    p.setPen(Qt.NoPen)
    p.drawEllipse(1, 1, tamanho - 2, tamanho - 2)
    p.end()
    return QIcon(pix)


# --- Estilo SIMPLES (gráficos das juntas): o de gráfico de laboratório ---
# Caixa preta inteira, marcas para dentro nos quatro lados, grade pontilhada,
# cores padrão do matplotlib, títulos centralizados e legenda numa caixa.
CORES_SIMPLES = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']
PRETO = '#000000'
GRADE_SIMPLES = '#b0b0b0'


def _estilizar_simples(ax, rotulo_y, rotulo_x):
    ax.set_facecolor(FUNDO)
    for lado in ax.spines.values():
        lado.set_visible(True)
        lado.set_color(PRETO)
        lado.set_linewidth(0.8)
    ax.tick_params(direction='in', top=True, right=True, colors=PRETO, labelsize=9,
                   length=4, width=0.8)
    ax.grid(True, linestyle=':', color=GRADE_SIMPLES, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.set_ylabel(rotulo_y, color=PRETO, fontsize=10)
    if rotulo_x:
        ax.set_xlabel(rotulo_x, color=PRETO, fontsize=10)


def _legenda_simples(ax, **kw):
    """Legenda numa caixa com borda (como a padrão do matplotlib)."""
    leg = ax.legend(frameon=True, fancybox=False, edgecolor=PRETO, framealpha=1.0,
                    fontsize=8, labelcolor=PRETO, handlelength=1.8, borderpad=0.4, **kw)
    leg.get_frame().set_linewidth(0.8)
    return leg


def _estilizar(ax, rotulo_y, rotulo_x):
    ax.set_facecolor(FUNDO)
    for lado in ('top', 'right'):
        ax.spines[lado].set_visible(False)
    for lado in ('left', 'bottom'):
        ax.spines[lado].set_color(EIXO)
    ax.tick_params(colors=TEXTO_FRACO, labelsize=9, length=0, pad=6)
    ax.grid(True, axis='y', color=GRADE, linewidth=1)
    ax.set_axisbelow(True)
    ax.set_ylabel(rotulo_y, color=TEXTO_FRACO, fontsize=9)
    if rotulo_x:
        ax.set_xlabel(rotulo_x, color=TEXTO_FRACO, fontsize=9)


class _Base(FigureCanvasQTAgg):

    def __init__(self, altura_px):
        self.fig = Figure(figsize=(6, altura_px / 100), dpi=100, facecolor=FUNDO)
        super().__init__(self.fig)
        self.setMinimumHeight(altura_px)
        self.setMinimumWidth(200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._aviso = None

    def mostrar_aviso(self, texto):
        """Texto no meio do gráfico (ex.: 'aguardando dados')."""
        if self._aviso is None:
            self._aviso = self.ax.text(0.5, 0.5, '', transform=self.ax.transAxes,
                                       ha='center', va='center', fontsize=10,
                                       color=TEXTO_FRACO)
        self._aviso.set_text(texto or '')
        self.draw_idle()


class GraficoLinhas(_Base):
    """Várias séries (uma por junta) ao longo de x, com dica ao passar o mouse.

    rapido=True é para gráficos ao vivo: o fundo (eixos, números, legenda) fica
    guardado e cada quadro redesenha só as linhas, enquanto os limites dos
    eixos não mudam (um quadro inteiro custa ~20 ms; só as linhas, ~2 ms).
    """

    def __init__(self, rotulo_y, nomes, unidade='', rotulo_x='', fmt_x=None,
                 altura_px=240, marcadores=False, cores=None, rapido=False, estilo='padrao'):
        super().__init__(altura_px)
        self.nomes = list(nomes)
        self.unidade = unidade
        self.fmt_x = fmt_x or (lambda x: f'{x:.1f}')
        self.simples = estilo == 'simples'
        if self.simples:
            # Legenda ao lado, numa caixa: o gráfico fica livre.
            largura = 0.78 if len(self.nomes) > 1 else 0.86
            self.ax = self.fig.add_axes([0.1, 0.19 if rotulo_x else 0.1, largura,
                                         0.74 if rotulo_x else 0.83])
            _estilizar_simples(self.ax, rotulo_y, rotulo_x)
            cores = cores or CORES_SIMPLES
        else:
            self.ax = self.fig.add_axes([0.1, 0.17 if rotulo_x else 0.1, 0.88, 0.69])
            _estilizar(self.ax, rotulo_y, rotulo_x)
            cores = cores or CORES_SERIES
        self.linhas = [
            self.ax.plot([], [], color=cores[i % len(cores)],
                         linewidth=1.5 if self.simples else 2, label=n,
                         marker='o' if marcadores else None,
                         markersize=4 if self.simples else 4.5,
                         markeredgecolor=cores[i % len(cores)] if self.simples else FUNDO,
                         markeredgewidth=1)[0]
            for i, n in enumerate(self.nomes)]
        if len(self.nomes) > 1 and self.simples:
            _legenda_simples(self.ax, loc='upper left', bbox_to_anchor=(1.01, 1.0),
                             borderaxespad=0)
        elif len(self.nomes) > 1:
            self.ax.legend(loc='lower left', bbox_to_anchor=(0, 1.02), ncol=len(self.nomes),
                           frameon=False, fontsize=9, labelcolor=TEXTO,
                           handlelength=1.4, columnspacing=1.4, borderaxespad=0)
        # dica do mouse
        self._x = np.array([])
        self._ys = [np.array([]) for _ in self.nomes]
        self._guia = self.ax.axvline(0, color=PRETO if self.simples else TEXTO_FRACO,
                                     linewidth=0.8 if self.simples else 1,
                                     linestyle='--' if self.simples else '-',
                                     alpha=0.6, visible=False)
        self._pontos = self.ax.scatter([], [], s=36 if self.simples else 64, zorder=5,
                                       edgecolors=PRETO if self.simples else FUNDO,
                                       linewidths=0.8 if self.simples else 2)
        self._dica = self.ax.annotate(
            '', xy=(0, 0), xytext=(12, 0), textcoords='offset points', fontsize=9,
            color=PRETO if self.simples else TEXTO, va='center', family='monospace',
            bbox=(dict(boxstyle='square,pad=0.4', fc=FUNDO, ec=PRETO, lw=0.8) if self.simples
                  else dict(boxstyle='round,pad=0.5', fc=FUNDO, ec=EIXO, lw=1)), zorder=6,
            visible=False)
        self._cores = [ln.get_color() for ln in self.linhas]
        self.mpl_connect('motion_notify_event', self._ao_mover)
        self.mpl_connect('figure_leave_event', lambda _e: self._esconder_dica())
        self._rapido = rapido
        self._fundo = None
        if rapido:
            for a in self._animados():
                a.set_animated(True)
            self.mpl_connect('draw_event', self._ao_desenhar)

    def _animados(self):
        return sorted(self.linhas + [self._guia, self._pontos, self._dica],
                      key=lambda a: a.get_zorder())

    def _ao_desenhar(self, _evento):
        # Depois de cada desenho inteiro: guarda o fundo sem as linhas e as põe
        # por cima. Sem blit aqui: este evento pode vir de dentro da pintura do
        # Qt, que já vai mostrar o resultado (um repaint aqui seria recursivo).
        self._fundo = self.copy_from_bbox(self.fig.bbox)
        self._desenhar_animados()

    def _desenhar_animados(self):
        for a in self._animados():
            if a.get_visible():
                self.ax.draw_artist(a)

    def _pintar_animados(self):
        self._desenhar_animados()
        self.blit(self.fig.bbox)

    def _atualizar(self, rapido_ok=True):
        if self._rapido and rapido_ok and self._fundo is not None:
            self.restore_region(self._fundo)
            self._pintar_animados()
        else:
            self.draw_idle()

    def definir_dados(self, x, ys, xlim=None, ylim=None):
        self._x = np.asarray(x, dtype=float)
        self._ys = [np.asarray(y, dtype=float) for y in ys]
        for ln, y in zip(self.linhas, self._ys):
            ln.set_data(self._x, y)
        antes = (self.ax.get_xlim(), self.ax.get_ylim())
        if xlim is None or ylim is None:
            self.ax.relim()
            self.ax.autoscale_view(scalex=xlim is None, scaley=ylim is None)
        if xlim is not None:
            self.ax.set_xlim(*xlim)
        if ylim is not None:
            self.ax.set_ylim(*ylim)
        mudou = (self.ax.get_xlim(), self.ax.get_ylim()) != antes
        if len(self._x) and self._aviso is not None and self._aviso.get_text():
            self._aviso.set_text('')
            mudou = True
        self._atualizar(rapido_ok=not mudou)

    def definir_visiveis(self, visiveis):
        for ln, v in zip(self.linhas, visiveis):
            ln.set_visible(v)
        self.draw_idle()

    def _esconder_dica(self):
        if self._dica.get_visible():
            self._guia.set_visible(False)
            self._dica.set_visible(False)
            self._pontos.set_offsets(np.empty((0, 2)))
            self._atualizar()

    def _ao_mover(self, evento):
        if evento.inaxes is not self.ax or not len(self._x):
            self._esconder_dica()
            return
        i = int(np.argmin(np.abs(self._x - evento.xdata)))
        xv = self._x[i]
        pontos, cores, linhas_txt = [], [], [f'{self.fmt_x(xv)}']
        for nome, y, cor, ln in zip(self.nomes, self._ys, self._cores, self.linhas):
            if i >= len(y) or not ln.get_visible():
                continue
            v = y[i]
            if math.isfinite(v):
                pontos.append((xv, v))
                cores.append(cor)
                linhas_txt.append(f'{nome:<6}{v:9.1f} {self.unidade}')
            else:
                linhas_txt.append(f'{nome:<6}{"—":>9}')
        self._guia.set_xdata([xv, xv])
        self._guia.set_visible(True)
        self._pontos.set_offsets(np.array(pontos) if pontos else np.empty((0, 2)))
        self._pontos.set_facecolors(cores)
        self._dica.set_text('\n'.join(linhas_txt))
        y0, y1 = self.ax.get_ylim()
        self._dica.xy = (xv, (y0 + y1) / 2)
        # perto da borda direita, a caixa vai para a esquerda do cursor
        x0, x1 = self.ax.get_xlim()
        perto_direita = (xv - x0) / ((x1 - x0) or 1) > 0.6
        self._dica.set_position((-12, 0) if perto_direita else (12, 0))
        self._dica.set_ha('right' if perto_direita else 'left')
        self._dica.set_visible(True)
        self._atualizar()


class GraficoBarras(_Base):
    """Uma barra por categoria (uma série, uma cor), com o valor em cima."""

    def __init__(self, rotulo_y, categorias, unidade='', altura_px=240, cor=CORES_SIMPLES[0]):
        super().__init__(altura_px)
        self.categorias = list(categorias)
        self.unidade = unidade
        self.ax = self.fig.add_axes([0.1, 0.12, 0.86, 0.8])
        _estilizar_simples(self.ax, rotulo_y, '')
        self.ax.grid(False, axis='x')
        x = np.arange(len(self.categorias))
        self.barras = self.ax.bar(x, np.zeros(len(x)), width=0.6, color=cor,
                                  edgecolor=PRETO, linewidth=0.8)
        self.ax.set_xticks(x)
        self.ax.set_xticklabels(self.categorias)
        self.rotulos = [self.ax.text(xi, 0, '', ha='center', va='bottom', fontsize=9,
                                     color=PRETO) for xi in x]

    def definir_valores(self, valores):
        validos = [v for v in valores if math.isfinite(v)]
        topo = max(validos) if validos else 1.0
        for barra, rot, v in zip(self.barras, self.rotulos, valores):
            altura = v if math.isfinite(v) else 0.0
            barra.set_height(altura)
            rot.set_y(altura)
            rot.set_text(f'{v:.1f} {self.unidade}' if math.isfinite(v) else '—')
        self.ax.set_ylim(0, max(topo * 1.18, 1.0))
        self.draw_idle()


# Fases de uma sequência gravada: cor na faixa "O que o robô estava fazendo".
CORES_FASE = {'inicio': '#f2f2f2', 'conferindo': '#c9c9c9', 'movendo': '#cfe0f1',
              'parado': '#ffffff', 'interrompido': '#f4c7c3'}
FUNDO_CONFERINDO = '#ececec'      # robô parado conferindo o caminho, nos gráficos


def _rotulo_fase(tipo, k, dur):
    texto = {'inicio': 'início', 'conferindo': f'conferindo o\ncaminho até P{k}',
             'movendo': f'indo para P{k}', 'parado': f'parado em P{k}',
             'interrompido': f'interrompido\n(P{k})'}.get(tipo, tipo)
    return f'{texto}\n{dur:.1f} s'


def texto_fase(tipo, k):
    """Descrição de uma fase gravada (no CSV e na dica do mouse)."""
    return {'inicio': 'começo da gravação',
            'conferindo': f'conferindo o caminho até o ponto {k} (robô parado)',
            'movendo': f'indo para o ponto {k}', 'parado': f'parado no ponto {k}',
            'interrompido': f'interrompido no caminho para o ponto {k}'}.get(tipo, tipo)


class GraficoMovimento(FigureCanvasQTAgg):
    """Figura estática de um movimento gravado: em cima, o que o robô estava
    fazendo em cada trecho (se a gravação tiver as fases); depois, todas as
    juntas juntas, na largura toda; embaixo, um gráfico por junta (cada um na
    sua escala). Ao passar o mouse, linha vertical, valor(es) e a fase."""

    TAMANHO_PNG = (18, 12)          # polegadas da imagem salva (a 150 dpi)

    def __init__(self, nomes, altura_px=1100):
        self.fig = Figure(figsize=(16, altura_px / 100), dpi=100, facecolor=FUNDO)
        super().__init__(self.fig)
        self.setMinimumHeight(altura_px)
        self.setMinimumWidth(300)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.nomes = list(nomes)
        self._eixos = []          # (ax, [índices das séries], unidade)
        self._t = np.array([])
        self._ys = []
        self._unidade = ''
        self._fases = []
        self._guias = []
        self.mpl_connect('motion_notify_event', self._ao_mover)
        self.mpl_connect('figure_leave_event', lambda _e: self._esconder())
        self.vazio('Monte uma sequência de pontos e clique em "Realizar movimento e gravar": '
                   'o gráfico do movimento aparece aqui.')

    def vazio(self, texto):
        self.fig.clear()
        self._eixos = []
        self.fig.text(0.5, 0.5, texto, ha='center', va='center', fontsize=11,
                      color=TEXTO_FRACO)
        self.draw_idle()

    def desenhar(self, t, ys, grandeza, unidade, titulo='', fases=()):
        """t: segundos desde o início; ys: uma série por junta; fases: lista de
        (início, fim, tipo, ponto), em segundos."""
        self.fig.clear()
        self._t = np.asarray(t, dtype=float)
        self._ys = [np.asarray(y, dtype=float) for y in ys]
        self._unidade = unidade
        self._fases = list(fases)
        self._eixos = []
        t_fim = max(float(self._t[-1]) if len(self._t) else 1.0, 1e-3)
        if titulo:
            self.fig.suptitle(titulo, y=0.99, fontsize=13, color=PRETO)
        com_fases = bool(self._fases)
        externo = self.fig.add_gridspec(2 if com_fases else 1, 1, left=0.05, right=0.99,
                                        top=0.93, bottom=0.055, hspace=0.12,
                                        height_ratios=[0.3, 3.9] if com_fases else [1])
        gs = externo[-1].subgridspec(3, 3, wspace=0.2, hspace=0.5, height_ratios=[1.7, 1, 1])
        linha0 = 0
        rotulo_y = f'{grandeza.capitalize()} ({unidade})'

        # todas as juntas (largura toda)
        ax = self.fig.add_subplot(gs[linha0, :])
        _estilizar_simples(ax, rotulo_y, 'Tempo (s)')
        for i, nome in enumerate(self.nomes):
            ax.plot(self._t, self._ys[i], color=CORES_SIMPLES[i], linewidth=1.6, label=nome)
        ax.set_title('Todas as juntas', fontsize=11, color=PRETO, pad=6, loc='left')
        leg = _legenda_simples(ax, loc='lower right', bbox_to_anchor=(1.0, 1.01),
                               ncol=len(self.nomes), borderaxespad=0)
        for texto in leg.get_texts():
            texto.set_fontsize(9)
        self._eixos.append((ax, list(range(len(self.nomes)))))
        principal = ax

        # um por junta: 2 linhas x 3 colunas
        for i, nome in enumerate(self.nomes):
            ax = self.fig.add_subplot(gs[linha0 + 1 + i // 3, i % 3], sharex=principal)
            _estilizar_simples(ax, '', 'Tempo (s)' if i >= 3 else '')
            ax.plot(self._t, self._ys[i], color=CORES_SIMPLES[i], linewidth=1.5)
            ax.set_title(f'{nome} ({unidade})', fontsize=10, color=PRETO, pad=4)
            ax.yaxis.set_major_locator(MaxNLocator(5))
            y = self._ys[i][np.isfinite(self._ys[i])]
            # Junta que não se mexeu (no torque, quase constante não quer dizer parada).
            if grandeza != 'torque' and len(y) and float(np.ptp(y)) < 0.5:
                ax.text(0.98, 0.9, 'parada', transform=ax.transAxes, ha='right', va='top',
                        fontsize=8, color=PRETO, style='italic')
                centro = float(np.mean(y))
                ax.set_ylim(centro - 1, centro + 1)
            self._eixos.append((ax, [i]))

        for a, _ in self._eixos:
            a.set_xlim(0, t_fim)
            for ini, fim, tipo, _k in self._fases:
                if tipo == 'conferindo':
                    a.axvspan(ini, fim, color=FUNDO_CONFERINDO, lw=0, zorder=0)
                elif tipo == 'movendo':          # chegada ao ponto
                    a.axvline(fim, color=PRETO, lw=0.7, ls=':', zorder=1)

        if com_fases:
            self._faixa_fases(self.fig.add_subplot(externo[0], sharex=principal), t_fim)

        self._guias = []
        for a, _ in self._eixos:
            guia = a.axvline(0, color=PRETO, linewidth=0.8, linestyle='--', alpha=0.6,
                             visible=False)
            self._guias.append(guia)
        self._dica = None
        self.draw_idle()

    def _faixa_fases(self, ax, t_fim):
        """Faixa do tempo com o que o robô fazia: uma caixa por fase, com o nome
        e a duração (quando cabe; senão, só ao passar o mouse)."""
        ax.set_ylim(0, 1)
        ax.set_yticks([])
        ax.tick_params(axis='x', labelbottom=False, length=0)
        for lado in ax.spines.values():
            lado.set_color(PRETO)
            lado.set_linewidth(0.8)
        ax.set_title('O que o robô estava fazendo', fontsize=11, color=PRETO, pad=6,
                     loc='left')
        for ini, fim, tipo, k in self._fases:
            ax.axvspan(ini, fim, color=CORES_FASE.get(tipo, '#ffffff'), lw=0)
            ax.axvline(ini, color=PRETO, lw=0.8)
            if (fim - ini) / t_fim > 0.045:
                ax.text((ini + fim) / 2, 0.5, _rotulo_fase(tipo, k, fim - ini), ha='center',
                        va='center', fontsize=9.5, color=PRETO, linespacing=1.15)
        ax.set_xlim(0, t_fim)

    def salvar_png(self, caminho):
        # Sempre no mesmo tamanho grande, qualquer que seja o da janela.
        self._esconder(redesenhar=False)
        tamanho = self.fig.get_size_inches().copy()
        self.fig.set_size_inches(*self.TAMANHO_PNG)
        try:
            self.fig.savefig(caminho, dpi=150, facecolor=FUNDO)
        finally:
            self.fig.set_size_inches(*tamanho)
            self.draw_idle()

    def _esconder(self, redesenhar=True):
        mudou = False
        for g in self._guias:
            if g.get_visible():
                g.set_visible(False)
                mudou = True
        if getattr(self, '_dica', None) is not None:
            self._dica.remove()
            self._dica = None
            mudou = True
        if mudou and redesenhar:
            self.draw_idle()

    def _ao_mover(self, evento):
        alvo = next(((a, idx) for a, idx in self._eixos if a is evento.inaxes), None)
        if alvo is None or not len(self._t):
            self._esconder()
            return
        ax, indices = alvo
        k = int(np.argmin(np.abs(self._t - evento.xdata)))
        tk = self._t[k]
        for g in self._guias:
            g.set_xdata([tk, tk])
            g.set_visible(True)
        linhas = [f't = {tk:.2f} s']
        fase = next((f for f in self._fases if f[0] <= tk <= f[1]), None)
        if fase is not None:
            linhas.append(texto_fase(fase[2], fase[3]))
        for i in indices:
            v = self._ys[i][k]
            linhas.append(f'{self.nomes[i]:<4}{v:9.1f} {self._unidade}' if math.isfinite(v)
                          else f'{self.nomes[i]:<4}{"—":>9}')
        if getattr(self, '_dica', None) is not None:
            self._dica.remove()
        x0, x1 = ax.get_xlim()
        direita = (tk - x0) / ((x1 - x0) or 1) > 0.6
        y0, y1 = ax.get_ylim()
        self._dica = ax.annotate(
            '\n'.join(linhas), xy=(tk, (y0 + y1) / 2), xytext=(-10 if direita else 10, 0),
            textcoords='offset points', ha='right' if direita else 'left', va='center',
            fontsize=8.5, family='monospace', color=TEXTO, zorder=10,
            bbox=dict(boxstyle='square,pad=0.4', fc=FUNDO, ec=PRETO, lw=0.8))
        self.draw_idle()
