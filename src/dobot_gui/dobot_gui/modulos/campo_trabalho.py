"""Módulo Campo de Trabalho: gravar os pontos medidos no laboratório.

Leve o robô até um ponto (pelo Controle Juntas ou à mão), dê um nome se quiser
e clique em "Gravar ponto base" ou "Gravar ponto teto". O IRIS guarda os 6
ângulos lidos do robô, a ferramenta que está na ponta e, se a célula estiver
ligada, a força na ponteira naquele instante; nada é digitado à mão.

  Base   ponta encostada no tampo (o círculo da base). Definem o chão
         imaginário, que continua além da borda da mesa. Mínimo 3.
  Teto   ponto lá no alto, na altura escolhida para o teto imaginário.

Os pontos medem o AMBIENTE (mesa e teto), não o robô: a ferramenta que for para
a ponta depois entra no cálculo pelas medidas dela, sem medir tudo de novo. Por
isso cada ponto guarda com que ferramenta foi medido (o que encostou na mesa).

Os pontos ficam em PONTOS_PATH (CSV, abre em planilha) e são regravados a cada
mudança. Só grava com o robô parado: um ponto lido no meio do movimento sai
errado.

Os pontos são os mesmos em todos os computadores: a cada mudança (e ao abrir a
interface) eles vão e vêm pelo GitHub, num ramo só deles (sincronia.py). Sem
internet, ficam salvos aqui e vão na próxima vez.

Robô 3D da aba: mostra o robô (ou o gêmeo) em tempo real e, ao clicar num ponto
da tabela, o ponto como um robô azul translúcido. Clicar no ponto NÃO move nada.
"Levar até o ponto" move o robô (no modo gêmeo, o gêmeo do Gazebo), com
confirmação, pelo mesmo caminho de todo movimento (verificação do servidor,
Parar e EMERGÊNCIA valendo). No robô real, só com velocidade até
VELOCIDADE_MAX_PONTO.
"""
import collections
import csv
import datetime
import math
import os
import shutil
import threading
import time
import uuid

import numpy as np
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QFileDialog,
    QHBoxLayout,
    QLineEdit,
    QPushButton,
    QTableWidgetItem,
)

from ..campo import eh_referencia
from ..seguranca import altura_da_mesa, modelo_robo
from ..constantes import JOINT_NAMES, SETTINGS_PATH
from ..robot_view import RobotView
from ..sincronia import Sincronia, id_do_ponto
from ..tema import COR
from ..widgets import Cartao, com_icone, rotulo
from .base import Modulo
from .tabelas import _fmt, _tabela

PONTOS_PATH = os.path.join(os.path.dirname(SETTINGS_PATH), 'pontos_campo.csv')

NOME_TIPO = {'base': 'Base', 'teto': 'Teto'}
# Arquivos da primeira versão desta aba: "mesa" virou "base".
TIPO_ANTIGO = {'mesa': 'base'}
BASE_MINIMO = 3              # pontos para definir um plano

# Força na ponteira: média das últimas amostras (tira o ruído do instante).
AMOSTRAS_MEDIA = 20
CELULA_ONLINE_S = 1.0
# Acima disso a ponteira está encostando (a resolução da célula é 0,12 N).
ENCOSTANDO_N = 0.5
SINAL_PONTEIRA_PADRAO = -1.0  # o mesmo da aba Célula de Carga

COLUNAS_CSV = (['numero', 'nome', 'tipo', 'ferramenta', 'ferramenta_comprimento_mm',
                'data_hora', 'fonte']
               + [f'{n}_graus' for n in JOINT_NAMES]
               + ['forca_ponteira_N', 'Fx_N', 'Fy_N', 'Fz_N', 'id', 'modificado_em'])
COLUNAS_TELA = ['Nº', 'Nome', 'Tipo'] + JOINT_NAMES + ['Ferramenta', 'Ponteira (N)', 'Hora']
COL_NOME = 1

# Velocidade máxima (%) para levar o robô REAL até um ponto: os pontos da base
# ficam no nível do tampo, então chega devagar.
VELOCIDADE_MAX_PONTO = 20
FLUXO_MUDO_S = 1.0           # sem o tópico de estado, o 3D segue a leitura da janela
# Depois de uma mudança, espera isto antes de enviar (várias mudanças seguidas vão juntas).
ESPERA_ENVIO_MS = 1500
# Sem mudança, busca os pontos do outro computador de tempos em tempos.
BUSCAR_A_CADA_MS = 120000


def _agora():
    return datetime.datetime.now().isoformat(timespec='seconds')


def _num(texto):
    try:
        return float(texto)
    except (TypeError, ValueError):
        return None


class CampoTrabalho(Modulo):
    NOME = 'Campo de Trabalho'
    ICONE = 'shield'
    DESCRICAO = ('Grave os pontos medidos no laboratório: os da base (mesa, o chão imaginário) '
                 'e os do teto. O IRIS guarda os 6 ângulos do robô e a ferramenta na ponta; '
                 'você não digita nada além do nome.')

    amostra_recebida = pyqtSignal(object)
    sincronizado = pyqtSignal(bool, str, bool)   # ok, texto, mudou aqui (vem de outra thread)
    estado_recebido = pyqtSignal(object)   # ângulos do tópico de estado (thread do ROS)
    movimento_terminou = pyqtSignal()

    def construir(self):
        self._mov_habilitado = False
        self._movendo = False
        self._ultimo_fluxo = 0.0
        self.estado_recebido.connect(self._ao_receber_estado)
        self.movimento_terminou.connect(self._ao_terminar_movimento)
        assinar_estado = getattr(self.janela.ros_node, 'assinar_estado', None)
        if callable(assinar_estado):
            assinar_estado(lambda msg: self.estado_recebido.emit(list(msg.angulo_graus)))
        self.pontos = []
        self._juntas = None
        self._parado = False
        self._forcas = collections.deque(maxlen=AMOSTRAS_MEDIA)
        self._ultima_amostra = 0.0
        self.amostra_recebida.connect(self._ao_receber)
        assinar = getattr(self.janela.ros_node, 'assinar_celula', None)
        if callable(assinar):
            assinar(self.amostra_recebida.emit, lambda _texto: None)

        self.layout_modulo.addWidget(self._cartao_gravar())
        linha = QHBoxLayout()
        linha.setSpacing(16)
        linha.addWidget(self._cartao_pontos(), 3)
        linha.addWidget(self._cartao_3d(), 2)
        self.layout_modulo.addLayout(linha, 1)
        self.layout_modulo.addWidget(self._cartao_3d_base())
        self.sinc = Sincronia(PONTOS_PATH)
        self._sinc_rodando = False
        self._sinc_de_novo = False
        self.sincronizado.connect(self._ao_sincronizar)
        self._carregar_pontos()
        self._timer_envio = QTimer(self)
        self._timer_envio.setSingleShot(True)
        self._timer_envio.setInterval(ESPERA_ENVIO_MS)
        self._timer_envio.timeout.connect(self._sincronizar)
        self._timer_buscar = QTimer(self)
        self._timer_buscar.setInterval(BUSCAR_A_CADA_MS)
        self._timer_buscar.timeout.connect(self._sincronizar)
        self._timer_buscar.start()
        QTimer.singleShot(500, self._sincronizar)       # ao abrir: busca os do outro computador

        self._timer = QTimer(self)
        self._timer.setInterval(200)
        self._timer.timeout.connect(self._atualizar_leitura)
        self._timer.start()
        self._atualizar_leitura()

    # ------------------------------------------------------------------
    # Tela
    # ------------------------------------------------------------------

    def _cartao_gravar(self):
        c = Cartao('Gravar ponto',
                   'Leve o robô até o ponto, espere ele parar e grave. Na base, encoste no '
                   'tampo bem devagar.')
        leitura = QHBoxLayout()
        leitura.setSpacing(24)
        self.juntas_txt = rotulo('—', 'valorMono')
        self.forca_txt = rotulo('', 'cartaoTexto')
        self.parado_txt = rotulo('', 'cartaoTexto')
        leitura.addWidget(self.juntas_txt)
        leitura.addWidget(self.forca_txt)
        leitura.addWidget(self.parado_txt)
        leitura.addStretch(1)
        c.corpo.addLayout(leitura)

        self.ferramenta_txt = rotulo('', 'cartaoTexto')
        self.ferramenta_txt.setStyleSheet(f'color: {COR["texto_fraco"]};')
        c.corpo.addWidget(self.ferramenta_txt)

        linha = QHBoxLayout()
        linha.setSpacing(10)
        linha.addWidget(rotulo('Nome do ponto'))
        self.nome_edit = QLineEdit()
        self.nome_edit.setMaxLength(60)
        linha.addWidget(self.nome_edit, 1)
        self.base_btn = com_icone(QPushButton('Gravar ponto base'), 'save')
        self.base_btn.clicked.connect(lambda: self._gravar('base'))
        self.teto_btn = com_icone(QPushButton('Gravar ponto teto'), 'save')
        self.teto_btn.clicked.connect(lambda: self._gravar('teto'))
        linha.addWidget(self.base_btn)
        linha.addWidget(self.teto_btn)
        c.corpo.addLayout(linha)

        self.contagem_txt = rotulo('', 'cartaoTexto')
        c.corpo.addWidget(self.contagem_txt)
        return c

    def _cartao_3d(self):
        c = Cartao('Robô 3D', 'Clique num ponto da tabela: ele aparece em azul. Só mostra; '
                              'o robô só se move em "Levar até o ponto".')
        ponta = getattr(self.janela.ros_node, 'ponta', 'celula')
        self.robot_view = RobotView(ferramenta='celula' if ponta == 'celula' else None)
        self.robot_view.setMinimumHeight(420)
        # Vista mais aberta que a do Robô 3D: o campo vai até ~1 m em volta e 1,4 m de altura.
        self.robot_view.camera.dist = 3.9
        self.robot_view.camera.el = math.radians(24.0)
        self.robot_view.camera.target = np.array([0.0, 0.0, 0.62])
        c.corpo.addWidget(self.robot_view, 1)
        self.mostrar_campo = QCheckBox('Mostrar o campo de trabalho calculado')
        self.mostrar_campo.setChecked(True)
        self.mostrar_campo.toggled.connect(lambda _v: self._atualizar_campo_3d())
        c.corpo.addWidget(self.mostrar_campo)
        self.campo_txt = rotulo('', 'cartaoTexto')
        self.campo_txt.setWordWrap(True)
        self.campo_txt.setStyleSheet(f'color: {COR["texto_fraco"]};')
        c.corpo.addWidget(self.campo_txt)
        self.ponto_txt = rotulo('Nenhum ponto selecionado.', 'cartaoTexto')
        self.ponto_txt.setWordWrap(True)
        c.corpo.addWidget(self.ponto_txt)
        linha = QHBoxLayout()
        linha.setSpacing(10)
        robo = 'o gêmeo' if self.janela.sim_mode else 'o robô'
        self.ir_btn = com_icone(QPushButton(f'Levar {robo} até o ponto'), 'play')
        self.ir_btn.clicked.connect(self._confirmar_e_ir)
        linha.addWidget(self.ir_btn)
        linha.addStretch(1)
        c.corpo.addLayout(linha)
        self.ir_dica = rotulo('', 'cartaoTexto')
        self.ir_dica.setWordWrap(True)
        self.ir_dica.setStyleSheet(f'color: {COR["texto_fraco"]};')
        c.corpo.addWidget(self.ir_dica)
        return c

    def _cartao_3d_base(self):
        c = Cartao('Campo calculado pela base · é o que vale na segurança',
                   'Com os pontos da base (o plano da mesa) e a cinemática do robô: paredes '
                   'imaginárias verticais subindo do contorno da base, cortadas por até onde o '
                   'braço chega com a ponta para baixo em cada altura. Nunca passa do campo '
                   'medido ponto a ponto. Todo movimento por comando é recusado se, no destino '
                   'ou no caminho, o flange (ou a ponta da célula) sair deste campo, ou se '
                   'qualquer peça do robô bater na mesa, nas paredes retas (computador e '
                   'esquerda) ou no teto.')
        ponta = getattr(self.janela.ros_node, 'ponta', 'celula')
        self.robot_view_base = RobotView(ferramenta='celula' if ponta == 'celula' else None)
        self.robot_view_base.setMinimumHeight(460)
        self.robot_view_base.camera.dist = 3.9
        self.robot_view_base.camera.el = math.radians(24.0)
        self.robot_view_base.camera.target = np.array([0.0, 0.0, 0.62])
        c.corpo.addWidget(self.robot_view_base, 1)
        self.campo_base_txt = rotulo('', 'cartaoTexto')
        self.campo_base_txt.setWordWrap(True)
        self.campo_base_txt.setStyleSheet(f'color: {COR["texto_fraco"]};')
        c.corpo.addWidget(self.campo_base_txt)
        return c

    def _vistas(self):
        """Os dois robôs 3D da aba (o do campo medido e o do calculado pela base)."""
        return [v for v in (getattr(self, 'robot_view', None),
                            getattr(self, 'robot_view_base', None)) if v is not None]

    def _cartao_pontos(self):
        c = Cartao('Pontos gravados', f'Salvos em {PONTOS_PATH} (abre em planilha) e iguais em '
                                      'todos os computadores, pelo GitHub. Para mudar o nome de '
                                      'um ponto, clique duas vezes no nome.')
        self.sinc_txt = rotulo('Sincronizando com o GitHub…', 'cartaoTexto')
        self.sinc_txt.setWordWrap(True)
        self.sinc_txt.setStyleSheet(f'color: {COR["texto_fraco"]};')
        c.corpo.addWidget(self.sinc_txt)
        self.tab = _tabela(COLUNAS_TELA, 0)
        self.tab.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tab.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tab.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        self.tab.setFocusPolicy(Qt.StrongFocus)
        self.tab.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.tab.setMinimumHeight(260)
        self.tab.itemSelectionChanged.connect(self._ponto_selecionado)
        self.tab.itemChanged.connect(self._nome_editado)
        c.corpo.addWidget(self.tab, 1)

        botoes = QHBoxLayout()
        botoes.setSpacing(10)
        self.apagar_btn = QPushButton('Apagar ponto selecionado')
        self.apagar_btn.setObjectName('discreto')
        self.apagar_btn.clicked.connect(self._apagar_selecionado)
        self.apagar_todos_btn = QPushButton('Começar de novo')
        self.apagar_todos_btn.setObjectName('discreto')
        self.apagar_todos_btn.setToolTip('Tira todos os pontos da lista. O arquivo antigo '
                                         'fica guardado ao lado, com a data no nome.')
        self.apagar_todos_btn.clicked.connect(self._apagar_todos)
        salvar = QPushButton('Salvar cópia como…')
        salvar.setObjectName('discreto')
        salvar.clicked.connect(self._salvar_como)
        botoes.addWidget(self.apagar_btn)
        botoes.addWidget(self.apagar_todos_btn)
        botoes.addStretch(1)
        botoes.addWidget(salvar)
        c.corpo.addLayout(botoes)
        return c

    # ------------------------------------------------------------------
    # Leituras
    # ------------------------------------------------------------------

    def ao_ler_juntas(self, juntas, robo_parado):
        self._juntas = list(juntas)
        self._parado = bool(robo_parado)
        if time.monotonic() - self._ultimo_fluxo > FLUXO_MUDO_S:
            for v in self._vistas():
                v.set_joints_deg(juntas)
        self._atualizar_botoes()

    def _ao_receber_estado(self, q):
        if len(q) == 6:
            self._ultimo_fluxo = time.monotonic()
            for v in self._vistas():
                v.seguir(q)

    # ------------------------------------------------------------------
    # Ponto no 3D e levar o robô até ele
    # ------------------------------------------------------------------

    def _selecionado(self):
        linhas = {i.row() for i in self.tab.selectedItems()}
        if not linhas or min(linhas) >= len(self.pontos):
            return None
        return self.pontos[min(linhas)]

    def _atualizar_campo_3d(self):
        if not hasattr(self, 'robot_view'):
            return
        self._atualizar_mesa()
        self._atualizar_campo_base()
        if not self.mostrar_campo.isChecked():
            self.robot_view.definir_campo([])
            self.campo_txt.setText('')
            return
        borda = [p['juntas'] for p in self.pontos if not eh_referencia(p['nome'])]
        refs = [p['juntas'] for p in self.pontos if eh_referencia(p['nome'])]
        campo = self.robot_view.definir_campo(borda, refs)
        if campo is None:
            self.campo_txt.setText(
                'Para calcular o campo: pelo menos 2 planos (alturas), cada um com 3 pontos '
                'em volta do robô.')
            return
        alturas = ', '.join(f'{p.z:.0f}' for p in campo.planos)
        aviso = (f' · {campo.ignorados} ponto(s) fora de qualquer plano (menos de 3 pontos '
                 'naquela altura) ficaram de fora.' if campo.ignorados else '')
        self.campo_txt.setText(
            f'{len(campo.planos)} planos medidos, alturas {alturas} mm. Chão no primeiro, '
            'topo no último. Em cada plano, a borda é o contorno dos pontos (amarelo); os '
            'pontos "mesa" e "começo …" (rosa) só dão a altura. Entre dois planos, a parede '
            'vai em linha reta de um contorno ao outro; o alcance do braço só corta onde for '
            'menor. Posição do flange (sem ferramenta), no referencial do robô.' + aviso)

    def _atualizar_mesa(self):
        """O tampo da mesa nos dois 3D, na altura que a segurança usa (pelo
        ponto "mesa": a parte mais baixa do robô naquela posição)."""
        try:
            self._mesa_z = altura_da_mesa(modelo_robo(),
                                          [(p['nome'], p['juntas']) for p in self.pontos])
        except Exception:
            self._mesa_z = None
        for v in self._vistas():
            v.definir_mesa(self._mesa_z)

    def _atualizar_campo_base(self):
        """O segundo 3D: campo só com os pontos da base + cinemática, e a
        comparação com os pontos medidos nos planos de cima."""
        vista = getattr(self, 'robot_view_base', None)
        if vista is None:
            return
        borda = [p for p in self.pontos if not eh_referencia(p['nome'])]
        refs = [p['juntas'] for p in self.pontos if eh_referencia(p['nome'])]
        campo = vista.definir_campo([p['juntas'] for p in borda], refs, so_base=True)
        if campo is None:
            self.campo_base_txt.setText('Para calcular: pelo menos 3 pontos da base (na mesma '
                                        'altura, em volta do robô).')
            return
        # Comparação: os pontos medidos acima da base ficam dentro do calculado?
        acima = [(p, xyz) for p, xyz in zip(borda, vista.flanges_mm([p['juntas'] for p in borda]))
                 if xyz[2] > campo.chao_z + 15.0]
        fora = []
        for p, (x, y, z) in acima:
            if not campo.dentro((x, y, z), -0.5):
                lim = campo.raio(math.atan2(y, x), z)
                fora.append((math.hypot(x, y) - lim if lim > 0 else float('inf'), p['nome'], z))
        comparacao = ''
        if acima:
            comparacao = (f' Dos {len(acima)} pontos medidos acima da base, '
                          f'{len(acima) - len(fora)} ficam dentro deste campo')
            if fora:
                fora.sort(reverse=True)
                d, nome, _z = fora[0]
                quanto = f'{d:.0f} mm' if d != float('inf') else 'acima do topo'
                comparacao += (f' e {len(fora)} ficam fora (o que mais passa: {nome}, '
                               f'{quanto}).')
            else:
                comparacao += '.'
        if campo.limite is not None:
            topo = (f'Topo a {campo.teto_z:.0f} mm (o plano medido mais alto; com a ponta para '
                    f'baixo o braço chegaria até {campo.teto_alcance_z:.0f} mm). Em cada direção e '
                    'altura, vale o menor entre o calculado e o medido: nunca passa do que você '
                    'mediu.')
        else:
            topo = (f'Topo a {campo.teto_z:.0f} mm: acima disso o braço não chega com a ponta '
                    'para baixo. Sem 2 planos medidos, não há campo medido para limitar.')
        mesa = getattr(self, '_mesa_z', None)
        mesa_txt = (f' Mesa (o tampo, em laranja) a {mesa:.1f} mm, pelo ponto "mesa": nenhuma '
                    'peça do robô (nem a ferramenta) pode descer abaixo dela.'
                    if mesa is not None else '')
        self.campo_base_txt.setText(
            f'Chão a {campo.chao_z:.1f} mm ({len(campo.pontos)} pontos da base). {topo}'
            + mesa_txt + ' '
            'Posição do flange (sem ferramenta), no referencial do robô.' + comparacao)

    def _ponto_selecionado(self):
        p = self._selecionado()
        if p is None:
            for v in self._vistas():
                v.definir_alvo(None)
            self.ponto_txt.setText('Nenhum ponto selecionado.')
        else:
            for v in self._vistas():
                v.definir_alvo(p['juntas'])
                v.definir_estado_alvo('ponto')
            self.ponto_txt.setText(f'{p["nome"]} ({NOME_TIPO[p["tipo"]]}): '
                                   + '  '.join(f'{n} {v:.1f}°'
                                               for n, v in zip(JOINT_NAMES, p['juntas'])))
        self._atualizar_botoes()

    def _motivo_bloqueio_ir(self):
        robo = 'gêmeo' if self.janela.sim_mode else 'robô'
        if self._selecionado() is None:
            return 'Clique num ponto da tabela.'
        if self._movendo:
            return f'O {robo} está indo até o ponto.'
        if not self._mov_habilitado:
            return f'Com o {robo} conectado e parado, dá para levá-lo até o ponto.'
        vel = self.janela.speed_slider.value()
        if not self.janela.sim_mode and vel > VELOCIDADE_MAX_PONTO:
            return (f'No robô real, só com velocidade até {VELOCIDADE_MAX_PONTO}% '
                    f'(agora: {vel}%).')
        return ''

    def definir_movimento_habilitado(self, habilitado):
        self._mov_habilitado = bool(habilitado)
        self._atualizar_botoes()

    def _confirmar_e_ir(self):
        j = self.janela
        p = self._selecionado()
        if p is None or self._motivo_bloqueio_ir() or j._action_running:
            self._atualizar_botoes()
            return
        robo = 'gêmeo' if j.sim_mode else 'robô'
        alvo = [float(v) for v in p['juntas']]
        de = j.ultimas_juntas() or alvo
        maior = max(abs(a - b) for a, b in zip(alvo, de))
        avisos = []
        if maior > 90:
            avisos.append(f'Movimento grande ({maior:.0f}° numa junta): confira se não há '
                          'ninguém nem nada perto do robô.')
        if not j.sim_mode and p['tipo'] == 'base':
            avisos.append('Ponto da BASE: o flange chega no nível do tampo. O caminho é '
                          'conferido pela segurança do campo de trabalho (só o flange e a '
                          'ponta da ferramenta, não o braço inteiro): acompanhe com a mão '
                          'perto da EMERGÊNCIA.')
        texto = (f'Levar o {robo} até "{p["nome"]}" ({NOME_TIPO[p["tipo"]]}).\n\n'
                 'De:  ' + ', '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, de))
                 + '\nPara: ' + ', '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, alvo))
                 + f'\n\nVelocidade: {j.speed_slider.value()}%'
                 + ''.join('\n\nATENÇÃO: ' + a for a in avisos)
                 + '\n\nParar e EMERGÊNCIA continuam valendo durante o movimento.')
        if not j.confirmar('Levar até o ponto', texto, f'Levar o {robo}'):
            return
        if j._action_running or self._motivo_bloqueio_ir():
            return
        self._movendo = True
        self._atualizar_botoes()
        j.iniciar_acao(f'Indo até o ponto "{p["nome"]}"', f'Levar até o ponto "{p["nome"]}": '
                       + ', '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, alvo)))
        threading.Thread(target=self._executar_ir, args=(alvo, p['nome']), daemon=True).start()

    def _executar_ir(self, alvo, nome):
        j = self.janela
        ok, msg = False, 'Movimento não concluído.'
        try:
            ok, msg = j.mover_e_esperar(alvo)
        finally:
            j.terminar_acao(ok, f'Chegou no ponto "{nome}".' if ok else msg)
            self.movimento_terminou.emit()

    def _ao_terminar_movimento(self):
        self._movendo = False
        self._atualizar_botoes()

    def _ao_receber(self, msg):
        f = msg.wrench.force
        self._forcas.append((f.x, f.y, f.z))
        self._ultima_amostra = time.monotonic()

    def _sinal_ponteira(self):
        celula = self.janela.modulo('Célula de Carga')
        sinal = getattr(celula, '_sinal_ponteira', None)
        return sinal() if callable(sinal) else SINAL_PONTEIRA_PADRAO

    def _forca_atual(self):
        """(ponteira, Fx, Fy, Fz) pela média das últimas amostras, ou None se a
        célula não está mandando dados (por exemplo, sem nada na ponta)."""
        if not self._forcas or time.monotonic() - self._ultima_amostra > CELULA_ONLINE_S:
            return None
        n = len(self._forcas)
        fx, fy, fz = (sum(a[i] for a in self._forcas) / n for i in range(3))
        # round(...) + 0.0 tira o "-0.00" da célula solta.
        return tuple(round(v, 3) + 0.0 for v in (fz * self._sinal_ponteira(), fx, fy, fz))

    def _juntas_atuais(self):
        """Ângulos atuais (só se a leitura do robô está em dia)."""
        atual = self.janela.ultimas_juntas()
        if atual is None or self.janela._robot_state not in ('connected', 'moving'):
            return None
        return atual

    def _ferramenta(self):
        """A ferramenta escolhida na aba Ferramentas: (nome, comprimento em mm)."""
        if not hasattr(self.janela, '_paginas'):       # a janela ainda está montando
            return '', None
        aba = self.janela.modulo('Ferramentas')
        f = aba.ativa() if aba is not None else None
        if f is None:
            return '', None
        return f['nome'], f['comprimento_mm']

    def _proximo_nome(self, tipo):
        usados = {p['nome'] for p in self.pontos}
        n = sum(1 for p in self.pontos if p['tipo'] == tipo) + 1
        while f'{NOME_TIPO[tipo]} {n}' in usados:
            n += 1
        return f'{NOME_TIPO[tipo]} {n}'

    def _atualizar_leitura(self):
        juntas = self._juntas_atuais()
        if juntas is None:
            self.juntas_txt.setText('Sem a posição do robô: conecte primeiro.')
            self.parado_txt.setText('')
        else:
            self.juntas_txt.setText('   '.join(f'{n} {v:.1f}°' for n, v in zip(JOINT_NAMES, juntas)))
            if self._parado:
                self.parado_txt.setText('robô parado')
                self.parado_txt.setStyleSheet(f'color: {COR["sucesso"]};')
            else:
                self.parado_txt.setText('esperando o robô parar…')
                self.parado_txt.setStyleSheet(f'color: {COR["aviso_texto"]};')
        forca = self._forca_atual()
        if forca is None:
            self.forca_txt.setText('')
        else:
            p = forca[0]
            encostando = p >= ENCOSTANDO_N
            self.forca_txt.setText(f'Ponteira: {p:.2f} N'
                                   + ('  (encostando)' if encostando else '  (solta)'))
            self.forca_txt.setStyleSheet(
                f'color: {COR["sucesso"] if encostando else COR["texto_fraco"]};')
        nome_f, comp = self._ferramenta()
        self.ferramenta_txt.setText(
            f'Ferramenta na ponta: {nome_f or "—"}'
            + (f' ({_fmt(comp)} mm)' if comp is not None else '')
            + '. Vai gravada com cada ponto; troque na aba Ferramentas.')
        self.nome_edit.setPlaceholderText(
            f'vazio = "{self._proximo_nome("base")}" ou "{self._proximo_nome("teto")}"')
        self._atualizar_botoes()

    def _atualizar_botoes(self):
        pode = self._juntas_atuais() is not None and self._parado
        self.base_btn.setEnabled(pode)
        self.teto_btn.setEnabled(pode)
        self.apagar_btn.setEnabled(bool(self.tab.selectedItems()))
        self.apagar_todos_btn.setEnabled(bool(self.pontos))
        if hasattr(self, 'ir_btn'):
            motivo = self._motivo_bloqueio_ir()
            self.ir_btn.setEnabled(motivo == '')
            self.ir_dica.setText(motivo)
            self.ir_dica.setVisible(bool(motivo))

    # ------------------------------------------------------------------
    # Gravar, renomear e apagar
    # ------------------------------------------------------------------

    def _gravar(self, tipo):
        juntas = self._juntas_atuais()
        if juntas is None or not self._parado:
            self.janela.alerta.mostrar('Espere o robô parar para gravar o ponto.', 'aviso')
            return
        forca = self._forca_atual()
        # Com a célula ligada dá para conferir se a ponteira encostou na mesa.
        if (tipo == 'base' and forca is not None and forca[0] < ENCOSTANDO_N
                and not self.janela.confirmar(
                    'Gravar ponto base',
                    f'A ponteira marca {forca[0]:.2f} N: parece que não está encostando na '
                    'mesa.\nGravar mesmo assim?', 'Gravar')):
            return
        nome = self.nome_edit.text().strip() or self._proximo_nome(tipo)
        if any(p['nome'] == nome for p in self.pontos) and not self.janela.confirmar(
                'Nome repetido', f'Já existe um ponto chamado "{nome}". Gravar com o mesmo '
                'nome?', 'Gravar'):
            return
        numero = max((p['numero'] for p in self.pontos), default=0) + 1
        nome_f, comp = self._ferramenta()
        ponto = {
            'numero': numero,
            'nome': nome,
            'tipo': tipo,
            'ferramenta': nome_f,
            'ferramenta_comprimento_mm': comp,
            'data_hora': datetime.datetime.now().isoformat(timespec='seconds'),
            'fonte': 'gemeo' if self.janela.sim_mode else 'robo',
            'juntas': [round(float(v), 3) for v in juntas],
            'forca': None if forca is None else list(forca),
            'id': uuid.uuid4().hex[:12],
            'modificado_em': _agora(),
        }
        self.pontos.append(ponto)
        if not self._salvar_pontos():
            self.pontos.pop()
            return
        self.nome_edit.clear()
        self._redesenhar_tabela()
        self.tab.scrollToBottom()
        texto = f'Ponto "{nome}" ({NOME_TIPO[tipo]}) gravado.'
        self.janela.log(texto + '  ' + '  '.join(
            f'{n} {v:.2f}°' for n, v in zip(JOINT_NAMES, ponto['juntas'])))
        self.janela.alerta.mostrar(texto, 'ok')

    def _nome_editado(self, item):
        if item.column() != COL_NOME or item.row() >= len(self.pontos):
            return
        ponto = self.pontos[item.row()]
        novo = item.text().strip()[:60]
        if not novo or novo == ponto['nome']:
            self._redesenhar_tabela()       # nome vazio: volta o anterior
            return
        antigo, antigo_mod = ponto['nome'], ponto.get('modificado_em', '')
        ponto['nome'] = novo
        ponto['modificado_em'] = _agora()
        if self._salvar_pontos():
            self.janela.log(f'Ponto "{antigo}" renomeado para "{novo}".')
        else:
            ponto['nome'], ponto['modificado_em'] = antigo, antigo_mod
        self._redesenhar_tabela()

    def _apagar_selecionado(self):
        linhas = {i.row() for i in self.tab.selectedItems()}
        if not linhas:
            return
        ponto = self.pontos[min(linhas)]
        if not self.janela.confirmar('Apagar ponto', f'Apagar o ponto "{ponto["nome"]}"?',
                                     'Apagar'):
            return
        self.pontos.remove(ponto)
        self._salvar_pontos()
        self._redesenhar_tabela()
        self.janela.log(f'Ponto "{ponto["nome"]}" apagado.')

    def _apagar_todos(self):
        if not self.pontos:
            return
        if not self.janela.confirmar('Começar de novo',
                                     f'Tirar os {len(self.pontos)} pontos da lista?\n\nEles '
                                     'saem também dos outros computadores (pelo GitHub). O '
                                     'arquivo atual fica guardado ao lado, com a data no nome.',
                                     'Começar de novo'):
            return
        carimbo = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        guardado = PONTOS_PATH.replace('.csv', f'-{carimbo}.csv')
        try:
            shutil.copyfile(PONTOS_PATH, guardado)
        except OSError as e:
            self.janela.log(f'Não foi possível guardar o arquivo antigo: {e}', 'ERRO')
            self.janela.alerta.mostrar('Não deu para guardar o arquivo antigo; nada foi apagado.',
                                       'aviso')
            return
        self.pontos = []
        self._salvar_pontos()
        self._redesenhar_tabela()
        self.janela.log(f'Pontos do campo zerados. Os antigos ficaram em {guardado}.')

    def _salvar_como(self):
        sugerido = os.path.expanduser('~/pontos_campo.csv')
        caminho, _ = QFileDialog.getSaveFileName(self, 'Salvar cópia dos pontos', sugerido,
                                                 'Planilha CSV (*.csv)')
        if caminho:
            self._escrever_csv(caminho)

    # ------------------------------------------------------------------
    # Arquivo
    # ------------------------------------------------------------------

    def _escrever_csv(self, caminho):
        try:
            os.makedirs(os.path.dirname(caminho) or '.', exist_ok=True)
            temporario = caminho + '.tmp'
            with open(temporario, 'w', newline='', encoding='utf-8') as f:
                w = csv.writer(f)
                w.writerow(COLUNAS_CSV)
                for p in self.pontos:
                    forca = p['forca'] or [None] * 4
                    comp = p.get('ferramenta_comprimento_mm')
                    w.writerow([p['numero'], p['nome'], p['tipo'], p['ferramenta'],
                                '' if comp is None else comp, p['data_hora'], p['fonte']]
                               + p['juntas']
                               + ['' if v is None else v for v in forca]
                               + [p.get('id', ''), p.get('modificado_em', '')])
            os.replace(temporario, caminho)
            return True
        except OSError as e:
            self.janela.log(f'Não foi possível salvar os pontos em {caminho}: {e}', 'ERRO')
            self.janela.alerta.mostrar('Não deu para salvar os pontos (veja o Registro).', 'aviso')
            return False

    def _salvar_pontos(self):
        with self.sinc.trava_arquivo:
            ok = self._escrever_csv(PONTOS_PATH)
        if ok:
            self._timer_envio.start()        # manda para o GitHub (os outros computadores)
        return ok

    # --- sincronia com os outros computadores ---

    def _sincronizar(self):
        if self._sinc_rodando:
            self._sinc_de_novo = True
            return
        self._sinc_rodando = True
        self.sinc_txt.setText('Sincronizando com o GitHub…')
        n = len(self.pontos)
        threading.Thread(target=lambda: self.sincronizado.emit(
            *self.sinc.sincronizar(f'pontos do campo de trabalho ({n} pontos)')),
            daemon=True).start()

    def _ao_sincronizar(self, ok, texto, mudou_aqui):
        self._sinc_rodando = False
        hora = datetime.datetime.now().strftime('%H:%M')
        if ok:
            self.sinc_txt.setText(f'✔ Sincronizado com o GitHub às {hora}: {texto}.')
            self.sinc_txt.setStyleSheet(f'color: {COR["sucesso"]};')
        else:
            self.sinc_txt.setText(f'Não sincronizou ({hora}): {texto}. Os pontos estão salvos '
                                  'neste computador e vão para o GitHub na próxima vez.')
            self.sinc_txt.setStyleSheet(f'color: {COR["aviso_texto"]};')
            self.janela.log(f'Sincronia dos pontos falhou: {texto}', 'AVISO')
        if mudou_aqui:
            selecionado = (self._selecionado() or {}).get('id')
            self._carregar_pontos()
            for i, p in enumerate(self.pontos):
                if p.get('id') == selecionado:
                    self.tab.selectRow(i)
            self.janela.log('Pontos do campo atualizados com os do outro computador.')
        if self._sinc_de_novo:
            self._sinc_de_novo = False
            self._sincronizar()

    def _carregar_pontos(self):
        self.pontos = []
        try:
            with open(PONTOS_PATH, newline='', encoding='utf-8') as f:
                for linha in csv.DictReader(f):
                    tipo = TIPO_ANTIGO.get(linha.get('tipo'), linha.get('tipo'))
                    juntas = [_num(linha.get(f'{n}_graus')) for n in JOINT_NAMES]
                    numero = _num(linha.get('numero'))
                    if None in juntas or numero is None or tipo not in NOME_TIPO:
                        continue
                    forca = [_num(linha.get(k)) for k in
                             ('forca_ponteira_N', 'Fx_N', 'Fy_N', 'Fz_N')]
                    self.pontos.append({
                        'numero': int(numero),
                        'nome': (linha.get('nome') or linha.get('nota') or '').strip()
                                or f'{NOME_TIPO[tipo]} {int(numero)}',
                        'tipo': tipo,
                        'ferramenta': linha.get('ferramenta') or '',
                        'ferramenta_comprimento_mm': _num(linha.get('ferramenta_comprimento_mm')),
                        'data_hora': linha.get('data_hora', ''),
                        'fonte': linha.get('fonte', ''),
                        'juntas': juntas,
                        'forca': None if None in forca else forca,
                        'id': id_do_ponto(linha),
                        'modificado_em': linha.get('modificado_em') or '',
                    })
        except FileNotFoundError:
            pass
        except (OSError, csv.Error) as e:
            self.janela.log(f'Não foi possível ler {PONTOS_PATH}: {e}', 'ERRO')
        self._redesenhar_tabela()

    def _redesenhar_tabela(self):
        self.tab.blockSignals(True)
        self.tab.setRowCount(len(self.pontos))
        for i, p in enumerate(self.pontos):
            hora = p['data_hora'].replace('T', ' ')[5:16] if p['data_hora'] else ''
            forca = _fmt(p['forca'][0], 2) if p['forca'] else '—'
            valores = ([str(p['numero']), p['nome'], NOME_TIPO[p['tipo']]]
                       + [_fmt(v, 1) for v in p['juntas']]
                       + [p['ferramenta'] or '—', forca, hora])
            for c, v in enumerate(valores):
                item = QTableWidgetItem(v)
                if c != COL_NOME:
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                self.tab.setItem(i, c, item)
        self.tab.blockSignals(False)
        conta = collections.Counter(p['tipo'] for p in self.pontos)
        base = conta['base']
        falta = (f' (faltam {BASE_MINIMO - base} para dar um plano)' if base < BASE_MINIMO else '')
        self.contagem_txt.setText(f'Gravados: Base {base}{falta}  ·  Teto {conta["teto"]}')
        ferramentas = {p['ferramenta'] for p in self.pontos if p['tipo'] == 'base'}
        if len(ferramentas) > 1:
            self.contagem_txt.setText(self.contagem_txt.text() + '  ·  atenção: pontos da base '
                                      'medidos com ferramentas diferentes')
        self._ponto_selecionado()
        self._atualizar_campo_3d()
