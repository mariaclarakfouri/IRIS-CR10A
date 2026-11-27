"""Módulo Campo do Experimento: cortar o campo de trabalho para um experimento.

O usuário escolhe o experimento e desenha, numa vista de cima da mesa, o polígono
onde quer trabalhar, mais uma faixa de alturas. O campo daquele experimento passa
a ser o campo de trabalho GERAL dentro desse corte: um corte só diminui o campo,
nunca aumenta, e o campo geral (o calculado, da aba Campo de Trabalho) não muda.

Os cortes ficam em seguranca.CORTES_PATH. O servidor de movimento confere o corte
nos movimentos que chegam com o nome do experimento (por enquanto, os da aba
Controle de Juntas).
"""
import datetime
import math
import os
import time

import numpy as np
from PyQt5.QtCore import QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF, QRegExpValidator
from PyQt5.QtCore import QRegExp
from PyQt5.QtWidgets import (QComboBox, QGridLayout, QHBoxLayout, QLineEdit, QPushButton,
                             QSizePolicy, QWidget)

from ..campo import eh_referencia
from ..robot_view import RobotView
from ..seguranca import (EXPERIMENTOS, NOME_EXPERIMENTO, PAREDES, PONTOS_PATH, altura_da_mesa,
                         area_poligono, ler_cortes, ler_pontos, modelo_robo, salvar_cortes,
                         validar_corte)
from ..tema import COR
from ..widgets import Cartao, com_icone, rotulo
from .base import Modulo

FLUXO_MUDO_S = 1.0          # sem estado em tempo real há isto: usa a leitura da janela
ALCANCE_VISTA_MM = 1450.0   # a vista de cima mostra até isto do eixo, para cada lado
PEGAR_CANTO_PX = 12         # distância (px) para pegar um canto com o mouse


class VistaDeCima(QWidget):
    """A mesa vista de cima (x para a direita, y para cima, mm no referencial do
    robô): o campo geral em azul, as paredes, a base do robô e o corte em laranja.

    Clicar num lugar vazio acrescenta um canto; arrastar um canto o move; botão
    direito num canto o apaga. A rodinha do mouse não faz nada aqui."""

    mudou = pyqtSignal(list)            # cantos do corte (mm)

    FUNDO = QColor('#0f172a')
    GRADE = QColor('#1e293b')
    CAMPO = QColor(56, 189, 248, 60)
    CAMPO_BORDA = QColor(125, 211, 252, 210)
    BASE_BORDA = QColor(250, 204, 21, 200)
    PAREDE = QColor(217, 164, 101, 230)
    CORTE = QColor(249, 115, 22, 70)
    CORTE_BORDA = QColor(251, 146, 60, 255)
    TEXTO = QColor('#cbd5e1')

    def __init__(self):
        super().__init__()
        self.setMinimumSize(420, 420)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.cantos = []                # [(x, y)] mm
        self.pegada = None              # (N, 2) mm: o maior contorno do campo geral
        self.base = None                # (N, 2) mm: contorno dos pontos da base
        self._arrastando = None
        self._sobre = None

    def heightForWidth(self, w):
        return w

    def hasHeightForWidth(self):
        return True

    # --- dados ---

    def definir_campo(self, pegada, base):
        self.pegada = None if pegada is None else np.asarray(pegada, float)
        self.base = None if base is None else np.asarray(base, float)
        self.update()

    def definir_cantos(self, cantos):
        self.cantos = [(float(x), float(y)) for x, y in (cantos or [])]
        self.update()

    # --- conversão mm <-> tela ---

    def _escala(self):
        lado = min(self.width(), self.height()) - 24
        return lado / (2 * ALCANCE_VISTA_MM)

    def _tela(self, x, y):
        e = self._escala()
        return QPointF(self.width() / 2 + x * e, self.height() / 2 - y * e)

    def _mm(self, px, py):
        e = self._escala()
        return ((px - self.width() / 2) / e, (self.height() / 2 - py) / e)

    def _canto_em(self, pos):
        for i, (x, y) in enumerate(self.cantos):
            p = self._tela(x, y)
            if math.hypot(p.x() - pos.x(), p.y() - pos.y()) <= PEGAR_CANTO_PX:
                return i
        return None

    # --- mouse ---

    def mousePressEvent(self, ev):
        i = self._canto_em(ev.pos())
        if ev.button() == Qt.RightButton:
            if i is not None:
                del self.cantos[i]
                self._avisar()
            return
        if ev.button() != Qt.LeftButton:
            return
        if i is not None:
            self._arrastando = i
            return
        x, y = self._mm(ev.x(), ev.y())
        if math.hypot(x, y) > ALCANCE_VISTA_MM:
            return
        self.cantos.append((round(x), round(y)))
        self._arrastando = len(self.cantos) - 1
        self._avisar()

    def mouseMoveEvent(self, ev):
        if self._arrastando is not None:
            x, y = self._mm(ev.x(), ev.y())
            x = max(-ALCANCE_VISTA_MM, min(ALCANCE_VISTA_MM, x))
            y = max(-ALCANCE_VISTA_MM, min(ALCANCE_VISTA_MM, y))
            self.cantos[self._arrastando] = (round(x), round(y))
            self._avisar()
            return
        sobre = self._canto_em(ev.pos())
        if sobre != self._sobre:
            self._sobre = sobre
            self.setCursor(Qt.OpenHandCursor if sobre is not None else Qt.CrossCursor)
            self.update()

    def mouseReleaseEvent(self, _ev):
        self._arrastando = None

    def wheelEvent(self, ev):
        ev.ignore()                     # a página rola; aqui nada muda

    def _avisar(self):
        self.update()
        self.mudou.emit(list(self.cantos))

    # --- desenho ---

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), self.FUNDO)
        # Grade de 200 em 200 mm e os eixos.
        p.setPen(QPen(self.GRADE, 1))
        for v in range(-1400, 1401, 200):
            p.drawLine(self._tela(v, -ALCANCE_VISTA_MM), self._tela(v, ALCANCE_VISTA_MM))
            p.drawLine(self._tela(-ALCANCE_VISTA_MM, v), self._tela(ALCANCE_VISTA_MM, v))
        # Campo geral (o maior contorno em qualquer altura) e o contorno da base.
        if self.pegada is not None and len(self.pegada) >= 3:
            p.setPen(QPen(self.CAMPO_BORDA, 1.6))
            p.setBrush(self.CAMPO)
            p.drawPolygon(QPolygonF([self._tela(x, y) for x, y in self.pegada]))
        if self.base is not None and len(self.base) >= 3:
            p.setPen(QPen(self.BASE_BORDA, 1.4, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawPolygon(QPolygonF([self._tela(x, y) for x, y in self.base]))
        # Paredes do laboratório (obstáculos do robô inteiro).
        f = QFont(self.font())
        f.setPointSize(9)
        p.setFont(f)
        for nome, (nx, ny), d in PAREDES:
            p.setPen(QPen(self.PAREDE, 2.2))
            if nx:
                x = nx * d
                p.drawLine(self._tela(x, -ALCANCE_VISTA_MM), self._tela(x, ALCANCE_VISTA_MM))
                t = self._tela(x, ALCANCE_VISTA_MM - 60)
                p.drawText(QPointF(t.x() + 6, t.y()), f'parede {nome}')
            else:
                y = ny * d
                p.drawLine(self._tela(-ALCANCE_VISTA_MM, y), self._tela(ALCANCE_VISTA_MM, y))
                t = self._tela(-ALCANCE_VISTA_MM + 40, y)
                p.drawText(QPointF(t.x(), t.y() - 6), f'parede {nome}')
        # A base do robô.
        p.setPen(Qt.NoPen)
        p.setBrush(QColor('#e2e8f0'))
        c = self._tela(0, 0)
        r = 90 * self._escala()
        p.drawEllipse(c, r, r)
        p.setPen(self.TEXTO)
        p.drawText(QPointF(c.x() + r + 4, c.y() + 4), 'robô')
        # O corte.
        if self.cantos:
            pts = [self._tela(x, y) for x, y in self.cantos]
            if len(pts) >= 3:
                p.setPen(QPen(self.CORTE_BORDA, 2.4))
                p.setBrush(self.CORTE)
                p.drawPolygon(QPolygonF(pts))
            elif len(pts) == 2:
                p.setPen(QPen(self.CORTE_BORDA, 2.4))
                p.drawLine(pts[0], pts[1])
            for i, q in enumerate(pts):
                p.setPen(QPen(QColor('#0f172a'), 1.5))
                p.setBrush(QColor('#ffffff') if i == self._sobre else self.CORTE_BORDA)
                p.drawEllipse(q, 6, 6)
        # Legenda.
        p.setPen(self.TEXTO)
        p.drawText(QRectF(10, self.height() - 24, self.width() - 20, 20), Qt.AlignLeft,
                   'Vista de cima · grade de 200 mm · azul: campo geral · tracejado: base')
        p.end()


class CampoExperimento(Modulo):
    NOME = 'Campo do Experimento'
    ICONE = 'corte'
    DESCRICAO = ('Escolha um experimento e desenhe a parte do campo de trabalho que ele pode '
                 'usar. O campo geral não muda.')

    estado_recebido = pyqtSignal(object)

    def construir(self):
        self._ultimo_fluxo = 0.0
        self._marca_pontos = None
        self._campo = None
        self._salvo = None                  # corte salvo do experimento escolhido
        self.estado_recebido.connect(self._ao_receber_estado)
        assinar = getattr(self.janela.ros_node, 'assinar_estado', None)
        if callable(assinar):
            assinar(lambda msg: self.estado_recebido.emit(list(msg.angulo_graus)))

        self.layout_modulo.addWidget(self._cartao_experimento())
        linha = QHBoxLayout()
        linha.setSpacing(16)
        linha.addWidget(self._cartao_desenho(), 1)
        linha.addWidget(self._cartao_3d(), 1)
        self.layout_modulo.addLayout(linha, 1)

        # O campo e o desenho só são montados na primeira vez que a aba aparece
        # (abrir a interface fica mais rápido).
        self._montada = False

    # ------------------------------------------------------------------
    # Tela
    # ------------------------------------------------------------------

    def _cartao_experimento(self):
        c = Cartao('Experimento',
                   'O corte vale para os movimentos feitos pela aba do experimento. O campo '
                   'do experimento é o campo de trabalho geral DENTRO do corte: o corte só '
                   'diminui o campo, nunca aumenta, e o campo geral continua o mesmo.')
        linha = QHBoxLayout()
        linha.setSpacing(12)
        linha.addWidget(rotulo('Experimento'))
        self.experimento = QComboBox()
        for chave, nome in EXPERIMENTOS:
            self.experimento.addItem(nome, chave)
        self.experimento.currentIndexChanged.connect(lambda _i: self._escolher_experimento())
        linha.addWidget(self.experimento)
        self.situacao = rotulo('', 'cartaoTexto')
        self.situacao.setWordWrap(True)
        linha.addWidget(self.situacao, 1)
        c.corpo.addLayout(linha)
        return c

    def _cartao_desenho(self):
        c = Cartao('Desenhe o corte',
                   'Clique na vista para marcar os cantos do corte, na ordem em volta da área. '
                   'Arraste um canto para mudar; botão direito num canto apaga.')
        self.vista = VistaDeCima()
        self.vista.mudou.connect(lambda _c: self._atualizar())
        c.corpo.addWidget(self.vista, 1)

        alturas = QGridLayout()
        alturas.setHorizontalSpacing(12)
        numero = QRegExpValidator(QRegExp(r'^-?\d{0,4}([.,]\d)?$'))
        self.z_min = QLineEdit()
        self.z_max = QLineEdit()
        for e in (self.z_min, self.z_max):
            e.setValidator(numero)
            e.setMaximumWidth(120)
            e.textChanged.connect(lambda _t: self._atualizar())
        alturas.addWidget(rotulo('Altura mínima (mm)'), 0, 0)
        alturas.addWidget(self.z_min, 0, 1)
        alturas.addWidget(rotulo('Altura máxima (mm)'), 0, 2)
        alturas.addWidget(self.z_max, 0, 3)
        alturas.setColumnStretch(4, 1)
        c.corpo.addLayout(alturas)
        self.alturas_txt = rotulo('', 'cartaoTexto')
        self.alturas_txt.setWordWrap(True)
        self.alturas_txt.setStyleSheet(f'color: {COR["texto_fraco"]};')
        c.corpo.addWidget(self.alturas_txt)

        botoes = QHBoxLayout()
        botoes.setSpacing(10)
        desfazer = QPushButton('Desfazer último canto')
        desfazer.setObjectName('discreto')
        desfazer.clicked.connect(self._desfazer)
        limpar = QPushButton('Limpar desenho')
        limpar.setObjectName('discreto')
        limpar.clicked.connect(self._limpar)
        self.remover_btn = QPushButton('Usar o campo geral')
        self.remover_btn.setObjectName('discreto')
        self.remover_btn.setToolTip('Apaga o corte deste experimento: ele volta a usar o campo '
                                    'de trabalho geral inteiro.')
        self.remover_btn.clicked.connect(self._remover)
        self.salvar_btn = com_icone(QPushButton('Salvar corte'), 'save')
        self.salvar_btn.clicked.connect(self._salvar)
        botoes.addWidget(desfazer)
        botoes.addWidget(limpar)
        botoes.addStretch(1)
        botoes.addWidget(self.remover_btn)
        botoes.addWidget(self.salvar_btn)
        c.corpo.addLayout(botoes)
        self.desenho_txt = rotulo('', 'cartaoTexto')
        self.desenho_txt.setWordWrap(True)
        c.corpo.addWidget(self.desenho_txt)
        return c

    def _cartao_3d(self):
        c = Cartao('Robô 3D', 'Em azul, o campo de trabalho geral; em laranja, o campo deste '
                              'experimento: a parte do campo geral que fica dentro do corte.')
        ponta = getattr(self.janela.ros_node, 'ponta', 'nada')
        self.robot_view = RobotView(ferramenta='celula' if ponta == 'celula' else None)
        self.robot_view.setMinimumHeight(460)
        self.robot_view.camera.dist = 3.9
        self.robot_view.camera.el = math.radians(28.0)
        self.robot_view.camera.target = np.array([0.0, 0.0, 0.55])
        c.corpo.addWidget(self.robot_view, 1)
        return c

    # ------------------------------------------------------------------
    # Campo geral
    # ------------------------------------------------------------------

    def _carregar_campo(self):
        """Lê os pontos e monta o campo geral calculado (o mesmo da segurança)."""
        try:
            st = os.stat(PONTOS_PATH)
            marca = (st.st_mtime_ns, st.st_size)
        except OSError:
            marca = None
        if marca == self._marca_pontos and self._campo is not None:
            return
        self._marca_pontos = marca
        try:
            pontos = ler_pontos()
        except OSError:
            pontos = []
        borda = [q for nome, q in pontos if not eh_referencia(nome)]
        refs = [q for nome, q in pontos if eh_referencia(nome)]
        self._campo = self.robot_view.definir_campo(borda, refs, so_base=True)
        try:
            self._mesa_z = altura_da_mesa(modelo_robo(), pontos)
        except Exception:
            self._mesa_z = None
        self.robot_view.definir_mesa(self._mesa_z)
        if self._campo is None:
            self.vista.definir_campo(None, None)
            return
        # Para a vista de cima: o maior alcance do campo em cada direção.
        angs = np.radians(np.arange(-180, 180, 2.0))
        zs = np.arange(self._campo.chao_z, self._campo.teto_z + 1e-6, 20.0)
        r = self._campo.raios_grade(angs, zs).max(axis=1)
        pegada = np.c_[r * np.cos(angs), r * np.sin(angs)]
        self.vista.definir_campo(pegada, self._campo.base.contorno)

    def showEvent(self, ev):
        super().showEvent(ev)
        self._carregar_campo()          # os pontos podem ter mudado (outra aba, sincronia)
        if not self._montada:
            self._montada = True
            self._escolher_experimento()
        else:
            self._atualizar()

    # ------------------------------------------------------------------
    # Experimento e corte
    # ------------------------------------------------------------------

    def _chave(self):
        return self.experimento.currentData()

    def _escolher_experimento(self):
        corte = ler_cortes().get(self._chave())
        self._salvo = corte
        if corte:
            self.vista.definir_cantos(corte['poligono_mm'])
            self.z_min.setText(f'{float(corte["z_min_mm"]):.0f}')
            self.z_max.setText(f'{float(corte["z_max_mm"]):.0f}')
        else:
            self.vista.definir_cantos([])
            if self._campo is not None:
                self.z_min.setText(f'{math.floor(self._campo.chao_z):.0f}')
                self.z_max.setText(f'{math.ceil(self._campo.teto_z):.0f}')
        self._atualizar()

    def _corte_da_tela(self):
        def num(e):
            try:
                return float(e.text().replace(',', '.'))
            except ValueError:
                return None
        return {'poligono_mm': [list(c) for c in self.vista.cantos],
                'z_min_mm': num(self.z_min), 'z_max_mm': num(self.z_max)}

    def _atualizar(self):
        """Situação do experimento, conferência do desenho e o corte no 3D."""
        nome = self.experimento.currentText()
        if self._salvo:
            quando = self._salvo.get('salvo_em', '')
            self.situacao.setText(f'“{nome}” usa um campo cortado'
                                  + (f' (salvo em {quando}).' if quando else '.'))
        else:
            self.situacao.setText(f'“{nome}” usa o campo de trabalho geral inteiro.')
        self.remover_btn.setEnabled(bool(self._salvo))

        corte = self._corte_da_tela()
        if self._campo is not None:
            mesa = f'a mesa fica a {self._mesa_z:.0f} mm' if self._mesa_z is not None else ''
            self.alturas_txt.setText(
                f'Alturas a partir da base do robô: {mesa}; o campo geral vai de '
                f'{self._campo.chao_z:.0f} a {self._campo.teto_z:.0f} mm.')
        z0, z1 = corte['z_min_mm'], corte['z_max_mm']
        if len(corte['poligono_mm']) >= 2 and z0 is not None and z1 is not None:
            self.robot_view.definir_corte(corte['poligono_mm'], z0, z1)
        else:
            self.robot_view.definir_corte(None)

        if not corte['poligono_mm']:
            msg, ok = 'Nenhum canto marcado.', False
            if self._salvo:
                msg = ('Desenho vazio, mas o corte salvo continua valendo. Para usar o campo '
                       'geral, clique em “Usar o campo geral”.')
        elif z0 is None or z1 is None:
            msg, ok = 'Escreva a altura mínima e a máxima.', False
        else:
            ok, motivo = validar_corte(corte)
            if ok:
                area = area_poligono(corte['poligono_mm']) / 1e6
                msg = (f'{len(corte["poligono_mm"])} cantos, {area:.2f} m² vistos de cima, de '
                       f'{z0:.0f} a {z1:.0f} mm de altura.')
            else:
                msg = motivo[0].upper() + motivo[1:] + '.'
        igual = self._salvo is not None and self._mesmo_corte(corte, self._salvo)
        if ok and not igual:
            msg += ' Ainda não salvo.'
        self.desenho_txt.setText(msg)
        self.desenho_txt.setStyleSheet(
            f'color: {COR["texto_fraco"] if ok else COR["aviso_texto"]};')
        self.salvar_btn.setEnabled(ok and not igual)

    @staticmethod
    def _mesmo_corte(a, b):
        try:
            return (np.allclose(np.asarray(a['poligono_mm'], float),
                                np.asarray(b['poligono_mm'], float))
                    and abs(float(a['z_min_mm']) - float(b['z_min_mm'])) < 0.05
                    and abs(float(a['z_max_mm']) - float(b['z_max_mm'])) < 0.05)
        except (ValueError, TypeError, KeyError):
            return False

    def _desfazer(self):
        if self.vista.cantos:
            self.vista.definir_cantos(self.vista.cantos[:-1])
            self._atualizar()

    def _limpar(self):
        """Limpa o desenho. Com um corte salvo, pergunta se é para apagá-lo também
        e voltar ao campo geral (só limpar a tela não muda o que está valendo)."""
        if self._salvo:
            nome = self.experimento.currentText()
            if self.janela.confirmar(
                    'Limpar desenho',
                    f'O experimento “{nome}” tem um corte salvo, que continua valendo até ser '
                    'apagado.\n\nApagar o corte salvo e usar o campo de trabalho geral?',
                    'Apagar e usar o campo geral'):
                self._remover(perguntar=False)
                return
        self.vista.definir_cantos([])
        self._atualizar()

    def _salvar(self):
        corte = self._corte_da_tela()
        ok, motivo = validar_corte(corte)
        if not ok:
            self.janela.alerta.mostrar(f'O corte não foi salvo: {motivo}.', 'aviso')
            return
        nome = self.experimento.currentText()
        if not self.janela.confirmar(
                'Salvar corte',
                f'A partir de agora, os movimentos do experimento “{nome}” só são aceitos se '
                'a ponta do robô ficar dentro deste corte (e, como sempre, dentro do campo '
                'de trabalho geral).\n\nO campo de trabalho geral não muda.', 'Salvar corte'):
            return
        corte['salvo_em'] = datetime.datetime.now().strftime('%d/%m/%Y %H:%M')
        cortes = ler_cortes()
        cortes[self._chave()] = corte
        try:
            salvar_cortes(cortes)
        except OSError as e:
            self.janela.alerta.mostrar(f'Não deu para salvar o corte: {e}', 'erro')
            return
        self._salvo = corte
        self.janela.log(f'Corte do campo salvo para o experimento “{nome}”: '
                        f'{len(corte["poligono_mm"])} cantos, de {corte["z_min_mm"]:.0f} a '
                        f'{corte["z_max_mm"]:.0f} mm.')
        self.janela.alerta.mostrar(f'Corte salvo: “{nome}” usa só esta parte do campo.', 'ok')
        self._avisar_abas()
        self._atualizar()

    def _remover(self, perguntar=True):
        nome = self.experimento.currentText()
        if perguntar and not self.janela.confirmar(
                'Usar o campo geral',
                f'Apagar o corte do experimento “{nome}”? Ele volta a usar o campo de trabalho '
                'geral inteiro.', 'Apagar o corte'):
            return
        cortes = ler_cortes()
        cortes.pop(self._chave(), None)
        try:
            salvar_cortes(cortes)
        except OSError as e:
            self.janela.alerta.mostrar(f'Não deu para apagar o corte: {e}', 'erro')
            return
        self._salvo = None
        self.janela.log(f'Corte do campo apagado: “{nome}” usa o campo geral inteiro.')
        self.janela.alerta.mostrar(f'“{nome}” voltou a usar o campo de trabalho geral.', 'ok')
        self._avisar_abas()
        self._escolher_experimento()

    def _avisar_abas(self):
        """As abas dos experimentos mostram se usam um corte."""
        for m in getattr(self.janela, '_paginas', []):
            f = getattr(m, 'atualizar_corte', None)
            if callable(f):
                f()

    # ------------------------------------------------------------------
    # Robô 3D seguindo o robô
    # ------------------------------------------------------------------

    def ao_ler_juntas(self, juntas, robo_parado):
        if time.monotonic() - self._ultimo_fluxo > FLUXO_MUDO_S:
            self.robot_view.set_joints_deg(juntas)

    def _ao_receber_estado(self, q):
        if len(q) == 6:
            self._ultimo_fluxo = time.monotonic()
            self.robot_view.seguir(q)
