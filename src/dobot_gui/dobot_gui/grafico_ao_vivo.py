"""Gráfico de linhas ao vivo desenhado direto com o Qt (QPainter).

Visual de gráfico de laboratório, o mesmo dos gráficos das juntas: caixa preta,
marcas para dentro nos quatro lados, grade pontilhada, cores padrão do
matplotlib e a legenda numa caixa ao lado.

É o mesmo jeito de desenhar do painel tátil: cada quadro só pinta as linhas
(nada de matplotlib), então dá para redesenhar ~60 vezes por segundo sem
travar. O "agora" é o relógio na hora de pintar e as amostras são médias em
faixas de tempo presas ao relógio: de um quadro para o outro a linha só
desliza, sem mudar de forma.
"""

import math

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QFontMetrics, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QWidget

PRETO = QColor('#000000')
CINZA_GRADE = QColor('#c8c8c8')
CINZA_TEXTO = QColor('#666666')
CORES = tuple(QColor(c) for c in ('#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
                                   '#8c564b'))


def _marcas(lo, hi, n=5):
    """Marcas "redondas" (1, 2 ou 5 × 10^k) entre lo e hi."""
    bruto = (hi - lo) / max(n - 1, 1)
    if bruto <= 0:
        return [lo]
    base = 10 ** math.floor(math.log10(bruto))
    passo = next(m * base for m in (1, 2, 5, 10) if m * base >= bruto)
    k0 = math.ceil(lo / passo - 1e-9)
    return [k * passo for k in range(k0, int(math.floor(hi / passo + 1e-9)) + 1)]


def _fmt(v, passo):
    casas = max(0, -int(math.floor(math.log10(passo)))) if passo < 1 else 0
    return f'{v:.{casas}f}'.replace('-', '−')


class GraficoAoVivo(QWidget):
    """Séries ao longo dos últimos `janela_s` segundos, com o 0 na direita."""

    def __init__(self, rotulo_y, nomes, unidade='', altura_px=240, faixa_min=1.0):
        super().__init__()
        self.faixa_min = faixa_min      # faixa mínima do eixo y automático
        self.rotulo_y = rotulo_y
        self.nomes = list(nomes)
        self.unidade = unidade
        self.setMinimumHeight(altura_px)
        self.setMinimumWidth(200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.setMouseTracking(True)
        self._fonte = QFont(self.font())
        self._fonte.setPointSizeF(8.5)
        self._t = np.empty(0)
        self._ys = np.empty((len(self.nomes), 0))
        self._agora = 0.0
        self._janela_s = 10.0
        self._ylim = (-1.0, 1.0)
        self._auto_ok = False
        self._aviso = ''
        self._mouse_x = None
        self._ultimas = None        # (xs, [ys...]) em pixels, para a dica do mouse

    # --- dados ---

    def definir(self, t, ys, agora, janela_s, ylim=None):
        """t: horários (monotônicos) das amostras; ys: uma linha por série.
        Sem ylim, o eixo y se ajusta sozinho (ver _faixa_auto)."""
        self._t, self._ys = t, ys
        self._agora, self._janela_s = agora, float(janela_s)
        self._ylim = ylim if ylim is not None else self._faixa_auto(ys)
        self._aviso = ''
        self.update()

    def _faixa_auto(self, ys):
        """Eixo y estável: só muda quando o sinal sai da faixa ou passa a ocupar
        menos da metade dela; assim o eixo não fica pulando a cada quadro."""
        ys = ys[np.isfinite(ys)]
        if not ys.size:
            return self._ylim
        lo, hi = float(ys.min()), float(ys.max())
        a, b = self._ylim
        if (not self._auto_ok or lo < a or hi > b
                or (hi - lo) < 0.5 * (b - a) and (b - a) > self.faixa_min * 1.5):
            meio, meia = (lo + hi) / 2, max((hi - lo) / 2 * 1.3, self.faixa_min / 2)
            self._auto_ok = True
            return (meio - meia, meio + meia)
        return self._ylim

    def mostrar_aviso(self, texto):
        self._aviso = texto or ''
        self._auto_ok = False
        self._t = np.empty(0)
        self._ys = np.empty((len(self.nomes), 0))
        self.update()

    # --- desenho ---

    def paintEvent(self, _evento):
        p = QPainter(self)
        p.setFont(self._fonte)
        p.fillRect(self.rect(), QColor('#ffffff'))
        fm = QFontMetrics(self._fonte)
        lo, hi = self._ylim
        marcas_y = _marcas(lo, hi)
        passo_y = marcas_y[1] - marcas_y[0] if len(marcas_y) > 1 else 1.0
        textos_y = [_fmt(v, passo_y) for v in marcas_y]
        esquerda = 30 + max(fm.horizontalAdvance(s) for s in textos_y)
        legenda_w = (44 + max(fm.horizontalAdvance(n) for n in self.nomes)
                     if len(self.nomes) > 1 else 0)
        r = QRectF(esquerda, 10, self.width() - esquerda - 14 - legenda_w,
                   self.height() - 10 - 40)
        if r.width() < 20 or r.height() < 20:
            p.end()
            return

        # legenda numa caixa, ao lado do gráfico (como a das juntas)
        if legenda_w:
            caixa = QRectF(r.right() + 8, r.top(), legenda_w - 4, 16 * len(self.nomes) + 8)
            p.fillRect(caixa, QColor('#ffffff'))
            p.setPen(QPen(PRETO, 1))
            p.drawRect(caixa)
            for i, nome in enumerate(self.nomes):
                y = caixa.top() + 4 + 16 * i + 8
                p.setPen(QPen(CORES[i % len(CORES)], 1.5))
                p.drawLine(QPointF(caixa.left() + 6, y), QPointF(caixa.left() + 24, y))
                p.setPen(PRETO)
                p.drawText(QRectF(caixa.left() + 30, y - 8, caixa.width() - 30, 16),
                           Qt.AlignLeft | Qt.AlignVCenter, nome)

        # rótulo do eixo y (girado)
        p.save()
        p.translate(10, r.center().y())
        p.rotate(-90)
        p.setPen(PRETO)
        p.drawText(QRectF(-r.height() / 2, -8, r.height(), 16), Qt.AlignCenter, self.rotulo_y)
        p.restore()

        def y_px(v):
            return r.bottom() - r.height() * (v - lo) / (hi - lo)

        grade = QPen(CINZA_GRADE, 1, Qt.DotLine)
        for v, s in zip(marcas_y, textos_y):
            y = y_px(v)
            p.setPen(grade)
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y))
            p.setPen(PRETO)
            p.drawLine(QPointF(r.left(), y), QPointF(r.left() + 4, y))
            p.drawLine(QPointF(r.right(), y), QPointF(r.right() - 4, y))
            p.drawText(QRectF(r.left() - 60, y - 8, 56, 16), Qt.AlignRight | Qt.AlignVCenter, s)

        janela = self._janela_s
        passo_x = 1 if janela <= 10 else 5 if janela <= 30 else 10
        for k in range(0, int(janela) + 1, passo_x):
            x = r.right() - r.width() * k / janela
            p.setPen(grade)
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))
            p.setPen(PRETO)
            p.drawLine(QPointF(x, r.bottom()), QPointF(x, r.bottom() - 4))
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.top() + 4))
            p.drawText(QRectF(x - 20, r.bottom() + 3, 40, 16), Qt.AlignHCenter,
                       f'−{k}' if k else '0')
        p.drawText(QRectF(r.left(), r.bottom() + 20, r.width(), 18), Qt.AlignHCenter,
                   'tempo (s)')

        self._ultimas = None
        if len(self._t):
            self._linhas(p, r, y_px)
        if self._aviso:
            p.setPen(CINZA_TEXTO)
            p.drawText(r.adjusted(12, 0, -12, 0), Qt.AlignCenter | Qt.TextWordWrap, self._aviso)
        p.setPen(QPen(PRETO, 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(r)
        if self._ultimas is not None and self._mouse_x is not None:
            self._dica(p, r, fm)
        p.end()

    def _linhas(self, p, r, y_px):
        # Média em faixas de tempo fixas, uma por pixel, presas ao relógio.
        tt, agora, janela = self._t, self._agora, self._janela_s
        dt = janela / max(1, int(r.width()))
        faixa = np.floor(tt / dt).astype(np.int64)
        inicio = np.flatnonzero(np.r_[True, faixa[1:] != faixa[:-1]])
        conta = np.diff(np.r_[inicio, len(tt)])
        xs = r.right() + r.width() * ((faixa[inicio] + 0.5) * dt - agora) / janela
        dentro = xs >= r.left()
        xs = xs[dentro]
        lo, hi = self._ylim
        p.setClipRect(r)
        p.setRenderHint(QPainter.Antialiasing, True)
        todas = []
        for i, serie in enumerate(self._ys):
            media = (np.add.reduceat(serie, inicio) / conta)[dentro]
            todas.append(media)
            ys = r.bottom() - r.height() * (np.clip(media, lo - (hi - lo), hi + (hi - lo)) - lo) / (hi - lo)
            ok = np.isfinite(ys)             # amostra sem dado (NaN) fica de fora
            p.setPen(QPen(CORES[i % len(CORES)], 1.5))
            p.drawPolyline(QPolygonF([QPointF(x, y) for x, y in zip(xs[ok].tolist(),
                                                                    ys[ok].tolist())]))
        p.setRenderHint(QPainter.Antialiasing, False)
        p.setClipping(False)
        self._ultimas = (xs, todas, r.right(), r.width())

    def _dica(self, p, r, fm):
        xs, todas, direita, largura = self._ultimas
        mx = self._mouse_x
        if not len(xs) or mx < r.left() or mx > r.right():
            return
        j = int(np.argmin(np.abs(xs - mx)))
        x = xs[j]
        p.setPen(QPen(PRETO, 1, Qt.DashLine))
        p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))
        t = (x - direita) / largura * self._janela_s
        linhas = [f'{t:.1f} s'.replace('-', '−')] + [
            f'{n}: {s[j]:+.3f} {self.unidade}'.replace('-', '−') for n, s in zip(self.nomes, todas)]
        w = max(fm.horizontalAdvance(s) for s in linhas) + 12
        h = 16 * len(linhas) + 6
        bx = x + 10 if x + 10 + w < r.right() else x - 10 - w
        caixa = QRectF(bx, r.top() + 6, w, h)
        p.fillRect(caixa, QColor('#ffffff'))
        p.setPen(PRETO)
        p.drawRect(caixa)
        for k, s in enumerate(linhas):
            p.drawText(QRectF(bx + 6, caixa.top() + 3 + 16 * k, w - 8, 16),
                       Qt.AlignLeft | Qt.AlignVCenter, s)

    def mouseMoveEvent(self, e):
        self._mouse_x = e.x()
        self.update()               # parado (pausado), a dica também segue o mouse

    def leaveEvent(self, _e):
        self._mouse_x = None
        self.update()
