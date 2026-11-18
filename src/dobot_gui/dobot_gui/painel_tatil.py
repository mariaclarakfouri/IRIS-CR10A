"""Painel dos sensores táteis, desenhado direto com o Qt (QPainter).

Mostra o mesmo que a figura do cr10twin (mapa de calor, raster RA/SA, sinal
escalar, neurônio pós ou cuneiformes), mas bem mais leve. Os dados vêm já em
arrays da FonteTatil (leitor_tatil.py: a leitura da placa roda noutro processo),
então cada quadro só desenha. Os quatro gráficos são redesenhados ~60 vezes por
segundo; o "agora" é o relógio na hora de pintar, e por isso tudo rola contínuo;
o mapa mostra sempre o último frame (sem média: nada de atraso entre o dedo e a
cor).

Diferença de leitura em relação à figura do cr10twin:
  * o sinal escalar aparece ao longo dos últimos 5 s (lá eram as últimas 50
    amostras, ~50 ms a 1 kHz, o que deixava a linha tremendo).
A orientação do mapa e do raster é a mesma da figura do cr10twin.
"""

import time

import numpy as np
from matplotlib import cm
from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QImage, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QWidget

JANELA_S = 5.0                  # raster, sinal escalar e neurônios: últimos 5 s
# Tamanho dos pontos dos rasters na tela (altura, largura em px) e da linha escalar.
PONTO_RA_SA = (5.0, 4)
PONTO_CN = (12.0, 6)
LINHA_ESCALAR_PX = 2.2
VREF = 3.3
PRETO = QColor('#000000')
CINZA_GRADE = QColor('#c8c8c8')
COR_RA = QColor('#d62728')
COR_SA = QColor('#1f77b4')
COR_ESCALAR = QColor('#1f77b4')
COR_POST = QColor('#000000')
CORES_CN = (QColor('#006B6B'), QColor('#8C2D5D'), QColor('#3D4F8A'))   # CN_SA, CN_FA, CN_MM
_LUT = (cm.jet(np.linspace(0.0, 1.0, 256))[:, :3] * 255).astype(np.uint8)


def _ampliar(m, fator=24):
    """Interpolação bilinear de uma grade pequena (cada valor no centro da
    sua célula, como o imshow), para o mapa sair liso em vez de quadriculado."""
    h, w = m.shape

    def pos(n):
        x = (np.arange(n * fator) + 0.5) / fator - 0.5
        x = np.clip(x, 0.0, n - 1.0)
        i0 = np.minimum(np.floor(x).astype(int), max(n - 2, 0))
        return i0, x - i0

    y0, fy = pos(h)
    x0, fx = pos(w)
    if h == 1 or w == 1:
        return np.repeat(np.repeat(m, fator, 0), fator, 1)
    a = m[np.ix_(y0, x0)]
    b = m[np.ix_(y0, x0 + 1)]
    c = m[np.ix_(y0 + 1, x0)]
    d = m[np.ix_(y0 + 1, x0 + 1)]
    fy, fx = fy[:, None], fx[None, :]
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def _imagem(rgb):
    """QImage (com cópia própria dos bytes) a partir de um array h×w×3."""
    h, w, _ = rgb.shape
    bgra = np.empty((h, w, 4), dtype=np.uint8)
    bgra[..., 0] = rgb[..., 2]
    bgra[..., 1] = rgb[..., 1]
    bgra[..., 2] = rgb[..., 0]
    bgra[..., 3] = 255
    return QImage(bgra.tobytes(), w, h, 4 * w, QImage.Format_RGB32).copy()


class _Raster:
    """Pontos de raster numa imagem MINÚSCULA, que o Qt estica até o gráfico:
    uma faixa de `sub` pixels por linha do raster (o ponto ocupa as `grossura`
    do meio) e um pixel por coluna de tela. O custo é o número de disparos, não o
    tamanho da janela (a versão anterior carimbava cada ponto numa imagem do
    tamanho do gráfico: com ~900 disparos de cuneiforme por segundo e a janela
    maximizada, eram dezenas de ms por quadro, medido na placa real)."""

    SUB = 12                     # sub-linhas por linha do raster

    def __init__(self, n_linhas, largura_px, altura_px, cores, altura_ponto_px=5.0,
                 largura_ponto=4):
        self.n = max(1, int(n_linhas))
        self.w = max(1, int(largura_px))
        # O ponto tem `altura_ponto_px` de altura NA TELA, mas nunca mais que 85% da
        # linha (pontos de linhas vizinhas não se encostam).
        altura_linha = max(1.0, float(altura_px) / self.n)
        fracao = min(0.85, altura_ponto_px / altura_linha)
        self.sub = self.SUB
        self.grossura = max(1, int(round(self.SUB * fracao)))
        self.largura_ponto = max(1, int(largura_ponto))
        self.occ = np.zeros((self.n, self.w), dtype=np.uint8)
        # Imagem "pintada por números": o Qt aplica as cores (0 = transparente).
        self.cores = [0] + [c.rgba() for c in cores]

    def marcar(self, linha, x_px, cor_id):
        """linha: índice da linha do raster (0 = em cima); x_px: colunas."""
        x = np.asarray(x_px, dtype=np.int64)
        linha = np.broadcast_to(np.asarray(linha, dtype=np.int64), x.shape)
        ok = (x >= 0) & (x < self.w)
        self.occ[linha[ok], x[ok]] = cor_id

    def imagem(self):
        occ = self.occ
        for _ in range(self.largura_ponto - 1):        # engrossa o ponto para a direita
            occ[:, 1:] = np.where(occ[:, 1:] == 0, occ[:, :-1], occ[:, 1:])
        faixas = np.zeros((self.n, self.sub, self.w), dtype=np.uint8)
        a = (self.sub - self.grossura) // 2
        faixas[:, a:a + self.grossura, :] = occ[:, None, :]
        img = QImage(faixas.tobytes(), self.w, self.n * self.sub, self.w, QImage.Format_Indexed8)
        img.setColorTable(self.cores)
        return img.copy()


class PainelTatil(QWidget):

    def __init__(self, linhas, colunas):
        super().__init__()
        self.linhas, self.colunas = linhas, colunas
        self.n = linhas * colunas
        self.setMinimumSize(640, 520)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.fonte = None
        self.firmware_antigo = False
        self._mapa = None
        self._agora = 0.0
        self._barra = _imagem(_LUT[::-1].reshape(256, 1, 3))
        self._fonte_txt = QFont(self.font())
        self._fonte_txt.setPointSizeF(8.5)

    # --- dados ---

    def definir_fonte(self, fonte, firmware_antigo):
        self.fonte = fonte
        self.firmware_antigo = bool(firmware_antigo)
        self._mapa = None
        self.update()

    def limpar(self):
        self.fonte = None
        self._mapa = None
        self.update()

    def atualizar(self):
        """Redesenha com o que a FonteTatil já recebeu (ela é lida pela aba)."""
        if self.fonte is None:
            return
        self._mapa = self.fonte.volt
        self._agora = time.monotonic()
        self.update()

    # --- desenho ---

    def _quadros(self):
        w, h = self.width(), self.height()
        mx, my = w * 0.5, h * 0.5
        return (QRectF(0, 0, mx, my), QRectF(mx, 0, w - mx, my),
                QRectF(0, my, mx, h - my), QRectF(mx, my, w - mx, h - my))

    def paintEvent(self, evento):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, False)
        p.setFont(self._fonte_txt)
        quadros = self._quadros()
        pedido = QRectF(evento.rect())
        for quadro, desenhar in zip(quadros, (self._mapa_calor, self._raster,
                                              self._escalar, self._neuronios)):
            if quadro.intersects(pedido):
                p.fillRect(quadro, QColor('#ffffff'))
                desenhar(p, quadro)
        p.end()

    def _area(self, p, quadro, titulo, rotulo_x='', esquerda=46, direita=14):
        """Título em cima e caixa preta; devolve o retângulo de dentro."""
        r = QRectF(quadro.left() + esquerda, quadro.top() + 26,
                   quadro.width() - esquerda - direita,
                   quadro.height() - 26 - (40 if rotulo_x else 22))
        p.setPen(PRETO)
        p.drawText(QRectF(quadro.left(), quadro.top() + 4, quadro.width(), 20),
                   Qt.AlignHCenter | Qt.AlignVCenter, titulo)
        if rotulo_x:
            p.drawText(QRectF(r.left(), r.bottom() + 18, r.width(), 18), Qt.AlignHCenter, rotulo_x)
        return r

    def _caixa(self, p, r):
        p.setPen(QPen(PRETO, 1))
        p.setBrush(Qt.NoBrush)
        p.drawRect(r)

    def _eixo_tempo(self, p, r):
        """Marcas de -5 a 0 s embaixo, com grade pontilhada."""
        grade = QPen(CINZA_GRADE, 1, Qt.DotLine)
        for k in range(int(JANELA_S) + 1):
            x = r.left() + r.width() * k / JANELA_S
            p.setPen(grade)
            p.drawLine(QPointF(x, r.top()), QPointF(x, r.bottom()))
            p.setPen(PRETO)
            p.drawLine(QPointF(x, r.bottom()), QPointF(x, r.bottom() - 4))
            p.drawText(QRectF(x - 20, r.bottom() + 2, 40, 16), Qt.AlignHCenter,
                       f'{k - int(JANELA_S)}')

    def _mapa_calor(self, p, quadro):
        r = self._area(p, quadro, f'Tensão (0–3,3 V) — {self.linhas}×{self.colunas}',
                       esquerda=34, direita=70)
        lado = min(r.width(), r.height())
        r = QRectF(r.left() + (r.width() - lado) / 2, r.top() + (r.height() - lado) / 2, lado, lado)
        if self._mapa is not None:
            idx = np.clip(_ampliar(self._mapa) / VREF * 255.0, 0, 255).astype(np.uint8)
            img = _imagem(_LUT[idx])
            p.setRenderHint(QPainter.SmoothPixmapTransform, True)
            p.drawImage(r, img)
            p.setRenderHint(QPainter.SmoothPixmapTransform, False)
            celula_w, celula_h = r.width() / self.colunas, r.height() / self.linhas
            p.setPen(QColor('#ffffff'))
            for i in range(self.linhas):
                for j in range(self.colunas):
                    p.drawText(QRectF(r.left() + j * celula_w, r.top() + i * celula_h,
                                      celula_w, celula_h), Qt.AlignCenter, f'{self._mapa[i, j]:.2f}')
        else:
            p.fillRect(r, QColor('#f2f2f2'))
            p.setPen(QColor('#666666'))
            p.drawText(r, Qt.AlignCenter | Qt.TextWordWrap, 'Esperando a placa...')
        self._caixa(p, r)
        p.setPen(PRETO)
        for i in range(self.linhas):        # índices das linhas e colunas
            y = r.top() + (i + 0.5) * r.height() / self.linhas
            p.drawText(QRectF(r.left() - 20, y - 8, 16, 16), Qt.AlignRight | Qt.AlignVCenter, str(i))
        for j in range(self.colunas):
            x = r.left() + (j + 0.5) * r.width() / self.colunas
            p.drawText(QRectF(x - 8, r.bottom() + 2, 16, 16), Qt.AlignHCenter, str(j))
        # barra de cores
        b = QRectF(r.right() + 14, r.top(), 12, r.height())
        p.setRenderHint(QPainter.SmoothPixmapTransform, True)
        p.drawImage(b, self._barra)
        p.setRenderHint(QPainter.SmoothPixmapTransform, False)
        self._caixa(p, b)
        p.setPen(PRETO)
        for v in (0.0, 1.0, 2.0, 3.0):
            y = b.bottom() - b.height() * v / VREF
            p.drawLine(QPointF(b.right(), y), QPointF(b.right() + 3, y))
            p.drawText(QRectF(b.right() + 5, y - 8, 30, 16), Qt.AlignLeft | Qt.AlignVCenter, f'{v:.0f}')

    def _raster(self, p, quadro):
        r = self._area(p, quadro, 'Raster RA / SA', 't − agora (s)')
        self._eixo_tempo(p, r)
        n2 = 2 * self.n
        p.setPen(PRETO)
        for texto, y0 in (('RA', 0.25), ('SA', 0.75)):
            p.drawText(QRectF(r.left() - 40, r.bottom() - r.height() * y0 - 8, 34, 16),
                       Qt.AlignRight | Qt.AlignVCenter, texto)
        meio = r.bottom() - r.height() * 0.5
        p.setPen(QPen(CINZA_GRADE, 1, Qt.DotLine))
        p.drawLine(QPointF(r.left(), meio), QPointF(r.right(), meio))
        f = self.fonte
        if f is not None:
            agora = self._agora
            tela = _Raster(n2, r.width(), r.height(), (COR_RA, COR_SA),
                           altura_ponto_px=PONTO_RA_SA[0], largura_ponto=PONTO_RA_SA[1])
            for t, k, base, cor_id in ((f.ra_t, f.ra_k, 0, 1), (f.sa_t, f.sa_k, self.n, 2)):
                x = tela.w * (1.0 + (t - agora) / JANELA_S)
                dentro = x >= 0
                # linha 0 é a de cima: RA embaixo, SA em cima
                tela.marcar(n2 - 1 - (base + k[dentro].astype(np.int64)), x[dentro], cor_id)
            p.drawImage(r, tela.imagem())
        self._caixa(p, r)

    def _escalar(self, p, quadro):
        antigo = self.firmware_antigo
        titulo = 'I_final (linha TOTAL)' if antigo else 'Tensão média do sensor (V)'
        r = self._area(p, quadro, titulo, 't − agora (s)', esquerda=52)
        self._eixo_tempo(p, r)
        lo, hi = (-1000.0, 1000.0) if antigo else (0.0, VREF)
        marcas = (-1000, -500, 0, 500, 1000) if antigo else (0.0, 1.0, 2.0, 3.0)
        for v in marcas:
            y = r.bottom() - r.height() * (v - lo) / (hi - lo)
            p.setPen(QPen(CINZA_GRADE, 1, Qt.DotLine))
            p.drawLine(QPointF(r.left(), y), QPointF(r.right(), y))
            p.setPen(PRETO)
            p.drawText(QRectF(r.left() - 50, y - 8, 46, 16), Qt.AlignRight | Qt.AlignVCenter,
                       f'{v:.0f}' if antigo else f'{v:.1f}')
        f = self.fonte
        if f is not None and len(f.esc_t):
            agora = self._agora
            i0 = int(np.searchsorted(f.esc_t, agora - JANELA_S - 0.1))
            tt, vv = f.esc_t[i0:], f.esc_v[i0:]
        else:
            tt = np.empty(0)
        if len(tt):
            # Média em faixas de tempo FIXAS (uma por pixel, presas ao relógio):
            # de um quadro para o outro a linha só desliza, sem mudar de forma.
            largura = max(1, int(r.width()))
            dt = JANELA_S / largura
            faixa = np.floor(tt / dt).astype(np.int64)
            inicio = np.flatnonzero(np.r_[True, faixa[1:] != faixa[:-1]])
            soma = np.add.reduceat(vv, inicio)
            conta = np.diff(np.r_[inicio, len(tt)])
            t_faixa = (faixa[inicio] + 0.5) * dt
            xs = r.left() + r.width() * (1.0 + (t_faixa - agora) / JANELA_S)
            ys = r.bottom() - r.height() * (np.clip(soma / conta, lo, hi) - lo) / (hi - lo)
            dentro = xs >= r.left()
            xs, ys = xs[dentro], ys[dentro]
            linha = QPolygonF([QPointF(x, y) for x, y in zip(xs.tolist(), ys.tolist())])
            p.setRenderHint(QPainter.Antialiasing, True)
            p.setPen(QPen(COR_ESCALAR, LINHA_ESCALAR_PX))
            p.drawPolyline(linha)
            p.setRenderHint(QPainter.Antialiasing, False)
        self._caixa(p, r)

    def _neuronios(self, p, quadro):
        antigo = self.firmware_antigo
        r = self._area(p, quadro, 'Neurônio pós' if antigo else 'Raster cuneiforme',
                       't − agora (s)', esquerda=56)
        self._eixo_tempo(p, r)
        linhas = ((('POST', 'post', COR_POST),) if antigo else
                  (('CN_SA', 'cn_sa', CORES_CN[0]), ('CN_FA', 'cn_ra', CORES_CN[1]),
                   ('CN_MM', 'cn_mm', CORES_CN[2])))
        f = self.fonte
        n = len(linhas)
        tela = _Raster(n, r.width(), r.height(), [c for _, _, c in linhas],
                       altura_ponto_px=PONTO_CN[0], largura_ponto=PONTO_CN[1])
        for k, (nome, chave, cor) in enumerate(linhas):
            y = r.bottom() - r.height() * (k + 0.5) / n
            p.setPen(PRETO)
            p.drawText(QRectF(r.left() - 54, y - 8, 50, 16), Qt.AlignRight | Qt.AlignVCenter, nome)
            if f is None:
                continue
            x = tela.w * (1.0 + (f.eventos[chave] - self._agora) / JANELA_S)
            tela.marcar(n - 1 - k, x[x >= 0], k + 1)
        if f is not None:
            p.drawImage(r, tela.imagem())
        self._caixa(p, r)
