"""Módulo Célula de Carga: leitura da célula FA7155 de 6 eixos na ponta do robô.

Os dados vêm do nó `celula_carga` (pacote dobot_celula), que só sobe com a
ferramenta célula (o padrão do comando iris): no robô real ele lê a célula pelo conversor USB-RS485; no
gêmeo virtual, o sensor de força simulado do Gazebo.

  Ponteira    a força ao longo da ponteira: positiva apertando (empurrando
              contra algo), negativa puxando;
  Gráficos    força (Fx, Fy, Fz) e momento (Mx, My, Mz) ao longo do tempo;
  Valores     valor atual, % do fundo de escala e estatísticas da janela;
  Estado      porta, taxa medida, erros do link e tara;
  Tara        zera os 6 eixos pelo programa (some ao fechar).
"""
import json
import time

import numpy as np
from PyQt5.QtCore import QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QPushButton,
    QTableWidgetItem,
    QVBoxLayout,
)

from ..grafico_ao_vivo import GraficoAoVivo
from ..tema import COR
from ..widgets import Cartao, rotulo
from .tabelas import _ajustar_altura, _fmt, _tabela
from .base import Modulo

EIXOS = ('Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz')
# Na tela (gráfico Força e tabela) o Fz segue a convenção da ponteira: apertar
# = positivo. O tópico /celula/wrench e o CSV continuam com o Fz da célula.
EIXOS_TELA = ('Fx', 'Fy', 'Fz (apertar +)', 'Mx', 'My', 'Mz')
G_MS2 = 9.80665              # para mostrar a força como o peso equivalente, em gramas
UNIDADES = ('N', 'N', 'N', 'N·m', 'N·m', 'N·m')
# Fundo de escala da FA7155D (catálogo) e sobrecarga segura.
FUNDO_ESCALA = (400.0, 400.0, 400.0, 20.0, 20.0, 20.0)
SOBRECARGA_SEGURA_PCT = 300.0
JANELAS_S = (10, 30, 60)
JANELA_PADRAO_S = 10
TAXA_MAX_HZ = 200
QUADRO_MS = 16               # gráficos a ~60 quadros por segundo, como no tátil
QUADROS_POR_TABELA = 12      # tabela e estado a 5 por segundo (dá para ler)
# Faixa mínima do eixo y: sem ela, o ruído de 0,02 N vira um zigue-zague de
# tela cheia quando a célula está parada.
FAIXA_MIN_Y = (1.0, 0.05, 1.0)    # força (N), momento (N·m), ponteira (N)
# Força na ponteira = SINAL_PONTEIRA × Fz, com o z da célula apontando para
# fora do robô: Fz cresce quando a ponteira é PUXADA, então apertar dá Fz
# negativo. É a mesma convenção do cr10twin (ft_force_sign = -1, compressão
# positiva) e foi conferida no gêmeo: com a ponteira para baixo, o peso dela
# puxa e Fz = +3,25 N. Na célula real, conferir na bancada; se apertar der
# negativo, marque "Inverter sinal".
SINAL_PONTEIRA = -1.0
SOLTA_N = 0.2                # abaixo disso (em módulo) a ponteira está solta
# Resolução da FA7155: 0,03% do fundo de escala (manual, pág. 7) = 0,12 N. Abaixo
# disso o número é ruído, e o valor grande da ponteira mostra 0,00 N.
RESOLUCAO_N = 0.0003 * 400.0
SEM_DADOS = ('Sem dados da célula. Abra com a célula: iris (robô real) ou iris '
             'gemeo (gêmeo virtual); sem-nada abre sem ela.')


def _motivo_sem_dados(e):
    """O que está errado quando a porta abriu mas nenhuma medida chega, pelo
    número de bytes que a célula manda por segundo."""
    if not e.get('bytes_por_s'):
        if e.get('modo_leitura') == 'polled':
            return ('a célula não responde às perguntas do programa (nenhum byte volta). Confira '
                    'a fonte de 24 V (o USB não alimenta a célula), os fios A/B nos bornes e o '
                    'GND ligado também no conversor; o programa também tenta as velocidades '
                    '1000000 e 115200.')
        return ('nenhum byte chega do conversor: a célula não está falando. Confira a fonte de '
                '24 V (o USB não alimenta a célula), os fios A/B nos bornes e o GND ligado também '
                'no conversor.')
    return (f'chegam {e["bytes_por_s"]} bytes/s, mas nenhuma medida válida: velocidade diferente '
            + ('(o programa está tentando as duas, 1000000 e 115200)' if e.get('baud_auto')
               else f'de {e.get("baud")} baud')
            + ' ou fios A e B trocados.')


class CelulaCarga(Modulo):
    NOME = 'Célula de Carga'
    ICONE = 'sliders'
    DESCRICAO = ('Força e momento nos 6 eixos da célula de carga FA7155 na ponta do robô, '
                 'com tara e estatísticas.')

    amostra_recebida = pyqtSignal(object)
    estado_recebido = pyqtSignal(str)

    def construir(self):
        # Buffer circular: horário (monotônico) e os 6 eixos de cada amostra.
        n = max(JANELAS_S) * TAXA_MAX_HZ
        self._buf_t = np.zeros(n)
        self._buf_v = np.zeros((n, len(EIXOS)))
        self._n = 0                  # amostras recebidas desde que abriu
        self._estado = {}
        self._estado_t = 0.0
        self._ultima_amostra = 0.0
        self._status_atual = None
        self._mostrando_dados = False
        self._ylims = [None, None, None]
        self._quadro = 0
        self.amostra_recebida.connect(self._ao_receber)
        self.estado_recebido.connect(self._ao_estado)
        assinar = getattr(self.janela.ros_node, 'assinar_celula', None)
        if callable(assinar):
            assinar(self.amostra_recebida.emit, self.estado_recebido.emit)

        lay = self.layout_modulo
        barra = QHBoxLayout()
        barra.setSpacing(10)
        # Online / offline: o primeiro item da barra.
        self.status_txt = rotulo('', 'cartaoTexto')
        fonte = self.status_txt.font()
        fonte.setBold(True)
        self.status_txt.setFont(fonte)
        barra.addWidget(self.status_txt)
        barra.addSpacing(12)
        barra.addWidget(rotulo('Janela'))
        self.janela_combo = QComboBox()
        for s in JANELAS_S:
            self.janela_combo.addItem(f'últimos {s} s', s)
        self.janela_combo.setCurrentIndex(JANELAS_S.index(JANELA_PADRAO_S))
        barra.addWidget(self.janela_combo)
        self.tara_btn = QPushButton('Tarar (zerar os 6 eixos)')
        self.tara_btn.setObjectName('discreto')
        self.tara_btn.setToolTip('Zera os 6 eixos pela média do último meio segundo. É uma '
                                 'subtração no programa: some ao fechar. Faça com a ponteira '
                                 'livre, sem encostar em nada.')
        self.tara_btn.clicked.connect(self.janela.ros_node.pedir_tara)
        barra.addWidget(self.tara_btn)
        self.limpar_tara_btn = QPushButton('Limpar tara')
        self.limpar_tara_btn.setObjectName('discreto')
        self.limpar_tara_btn.clicked.connect(self.janela.ros_node.pedir_limpar_tara)
        barra.addWidget(self.limpar_tara_btn)
        barra.addStretch()
        lay.addLayout(barra)

        self.aviso = rotulo('', 'cartaoTexto')
        self.aviso.setWordWrap(True)
        self.aviso.hide()
        lay.addWidget(self.aviso)

        lay.addWidget(self._cartao_ponteira())

        grade = QGridLayout()
        grade.setHorizontalSpacing(16)
        grade.setVerticalSpacing(16)
        grade.setColumnStretch(0, 4)
        grade.setColumnStretch(1, 3)
        grade.setColumnMinimumWidth(1, 470)   # tabela de 7 colunas inteira
        c1 = Cartao('Força')
        self.graf_forca = GraficoAoVivo('força (N)', EIXOS_TELA[:3], 'N')
        c1.corpo.addWidget(self.graf_forca)
        c1.corpo.addStretch()     # o cartão estica com a tabela ao lado; o gráfico fica em cima
        grade.addWidget(c1, 0, 0)
        c2 = Cartao('Momento')
        self.graf_momento = GraficoAoVivo('momento (N·m)', EIXOS[3:], 'N·m')
        c2.corpo.addWidget(self.graf_momento)
        c2.corpo.addStretch()     # o cartão estica com a tabela ao lado; o gráfico fica em cima
        grade.addWidget(c2, 1, 0)

        c3 = Cartao('Valores', 'Valor atual, porcentagem do fundo de escala e estatísticas '
                               'da janela escolhida.')
        self.tab = _tabela(['Eixo', 'Atual', '% escala', 'Média', 'Desvio', 'Mín', 'Máx'],
                           len(EIXOS) + 2)
        for i, (e, u) in enumerate(list(zip(EIXOS_TELA, UNIDADES)) + [('|F|', 'N'), ('|M|', 'N·m')]):
            self.tab.setItem(i, 0, QTableWidgetItem(f'{e} ({u})'))
            for c in range(1, 7):
                self.tab.setItem(i, c, QTableWidgetItem('—'))
        _ajustar_altura(self.tab)
        c3.corpo.addWidget(self.tab)
        grade.addWidget(c3, 0, 1)

        c4 = Cartao('Estado da célula')
        self.estado_txt = rotulo('', 'cartaoTexto')
        self.estado_txt.setWordWrap(True)
        c4.corpo.addWidget(self.estado_txt)
        c4.corpo.addStretch()
        grade.addWidget(c4, 1, 1)
        lay.addLayout(grade)
        lay.addStretch()
        for g in (self.graf_forca, self.graf_momento, self.graf_ponteira):
            g.mostrar_aviso(SEM_DADOS)
        self.estado_txt.setText(SEM_DADOS)

        self._timer = QTimer(self)
        self._timer.setInterval(QUADRO_MS)
        self._timer.timeout.connect(self._redesenhar)
        self._timer.start()

    def _cartao_ponteira(self):
        c = Cartao('Força na ponteira', 'Ao longo do eixo da ponteira. Positiva = apertando '
                                        '(empurrando contra algo); negativa = puxando. Já com '
                                        'a tara.')
        linha = QHBoxLayout()
        linha.setSpacing(24)
        col = QVBoxLayout()
        col.setSpacing(4)
        self.ponteira_valor = rotulo('—')
        self._cor_ponteira = None
        self._pintar_ponteira(COR['texto'])
        self.ponteira_valor.setMinimumWidth(230)
        col.addWidget(self.ponteira_valor)
        self.ponteira_txt = rotulo('', 'cartaoTexto')
        col.addWidget(self.ponteira_txt)
        self.inverter_chk = QCheckBox('Inverter sinal')
        self.inverter_chk.setToolTip('Marque se, na bancada, apertar a ponteira com o dedo der '
                                     'número negativo (fiação ou montagem da célula ao contrário).')
        col.addWidget(self.inverter_chk)
        col.addStretch()
        linha.addLayout(col)
        self.graf_ponteira = GraficoAoVivo('força (N)', ['ponteira'], 'N')
        linha.addWidget(self.graf_ponteira, 1)
        c.corpo.addLayout(linha)
        return c

    def _sinal_ponteira(self):
        return -SINAL_PONTEIRA if self.inverter_chk.isChecked() else SINAL_PONTEIRA

    # --- dados ---

    def _ao_receber(self, msg):
        f, m = msg.wrench.force, msg.wrench.torque
        v = (f.x, f.y, f.z, m.x, m.y, m.z)
        agora = time.monotonic()
        self._ultima_amostra = agora
        i = self._n % len(self._buf_t)
        self._buf_t[i] = agora
        self._buf_v[i] = v
        self._n += 1

    def _limpar_graficos(self):
        """Célula offline: apaga as linhas, a tabela e o número da ponteira e
        mostra o aviso no lugar dos gráficos. Os dados antigos saem do buffer."""
        self._mostrando_dados = False
        self._n = 0
        self._ylims = [None, None, None]
        for g in (self.graf_forca, self.graf_momento, self.graf_ponteira):
            g.mostrar_aviso('Célula de carga offline: os gráficos voltam sozinhos quando ela '
                            'mandar dados.')
        for i in range(self.tab.rowCount()):
            for c in range(1, self.tab.columnCount()):
                self.tab.item(i, c).setText('—')
        self.ponteira_valor.setText('—')
        self._pintar_ponteira(COR['texto'])
        self.ponteira_txt.setText('')
        self.aviso.hide()

    def _ao_estado(self, texto):
        try:
            self._estado = json.loads(texto)
        except ValueError:
            return
        self._estado_t = time.monotonic()

    # --- desenho ---

    def _redesenhar(self):
        self._quadro += 1
        tabela = self._quadro % QUADROS_POR_TABELA == 0
        if tabela:
            self._atualizar_estado()
        # Sempre em tempo real enquanto a célula manda dados; offline, os gráficos
        # se limpam (em vez de ficarem parados no último valor).
        online = self._n and time.monotonic() - self._ultima_amostra < 1.0
        if not online:
            if self._mostrando_dados:
                self._limpar_graficos()
            return
        if not self.isVisible():
            return
        self._mostrando_dados = True
        janela_s = self.janela_combo.currentData()
        t_fim = time.monotonic()
        tam = len(self._buf_t)
        ordem = np.arange(self._n - min(self._n, tam), self._n) % tam   # mais antiga primeiro
        t = self._buf_t[ordem]
        i0 = min(int(np.searchsorted(t, t_fim - janela_s)), len(t) - 1)
        ordem = ordem[i0:]
        t, v = t[i0:], self._buf_v[ordem]
        # Fz na convenção da ponteira (apertar = positivo, e o "Inverter sinal").
        v = v.copy()
        v[:, 2] *= self._sinal_ponteira()
        for k, graf in enumerate((self.graf_forca, self.graf_momento)):
            ys = v[:, 3 * k:3 * k + 3]
            graf.definir(t, ys.T, t_fim, janela_s, self._faixa_y(k, ys))
        ponteira = v[:, 2]
        self.graf_ponteira.definir(t, ponteira[None, :], t_fim, janela_s,
                                   self._faixa_y(2, ponteira))
        if tabela:
            self._atualizar_tabela(v)
            self._atualizar_ponteira(float(ponteira[-1]))

    def _atualizar_ponteira(self, f):
        if abs(f) < RESOLUCAO_N:
            f = 0.0
        self.ponteira_valor.setText(f'{f:+.2f} N')
        if abs(f) < SOLTA_N:
            txt, cor = 'solta (sem força na ponteira)', COR['texto']
        elif f > 0:
            txt, cor = 'apertando', COR['sucesso']
        else:
            txt, cor = 'puxando', COR['perigo']
        self._pintar_ponteira(cor)
        pct = abs(f) / FUNDO_ESCALA[2] * 100
        gramas = abs(f) / G_MS2 * 1000
        peso = (f' · peso de {gramas:,.0f} g'.replace(',', '.') if abs(f) >= SOLTA_N else '')
        self.ponteira_txt.setText(f'{txt}{peso}\n{pct:.1f}% do fundo de escala '
                                  f'({FUNDO_ESCALA[2]:.0f} N)')

    def _pintar_ponteira(self, cor):
        """Verde apertando, vermelho puxando, cor do texto solta. Só troca a
        folha de estilo quando a cor muda (trocar a cada quadro custa caro)."""
        if cor == self._cor_ponteira:
            return
        self._cor_ponteira = cor
        # Pela folha de estilo: o QSS global ganharia de um setFont.
        self.ponteira_valor.setStyleSheet(f'font-size: 30pt; font-weight: 700; color: {cor};')

    def _faixa_y(self, k, ys):
        """Eixo y estável: só muda quando o sinal sai da faixa ou passa a
        ocupar menos da metade dela, com uma faixa mínima para o ruído não
        encher a tela, e o eixo não fica pulando a cada quadro."""
        lo, hi = float(ys.min()), float(ys.max())
        atual = self._ylims[k]
        if (atual is None or lo < atual[0] or hi > atual[1]
                or (hi - lo) < 0.5 * (atual[1] - atual[0]) - 1e-12
                and (atual[1] - atual[0]) > FAIXA_MIN_Y[k] * 1.5):
            meio, meia = (lo + hi) / 2, max((hi - lo) / 2 * 1.3, FAIXA_MIN_Y[k] / 2)
            self._ylims[k] = (meio - meia, meio + meia)
        return self._ylims[k]

    def _atualizar_tabela(self, v):
        series = list(v.T)
        series.append(np.linalg.norm(v[:, :3], axis=1))
        series.append(np.linalg.norm(v[:, 3:], axis=1))
        pior_pct = 0.0
        for i, s in enumerate(series):
            atual = float(s[-1])
            pct = abs(atual) / FUNDO_ESCALA[i] * 100 if i < 6 else None
            if pct is not None:
                pior_pct = max(pior_pct, pct)
            casas = 2 if i < 3 or i == 6 else 3
            valores = (atual, pct, float(s.mean()), float(s.std()), float(s.min()),
                       float(s.max()))
            for c, v in enumerate(valores, start=1):
                self.tab.item(i, c).setText('—' if v is None else _fmt(v, 1 if c == 2 else casas))
        if pior_pct >= SOBRECARGA_SEGURA_PCT:
            self._mostrar_aviso(f'ACIMA DA SOBRECARGA SEGURA: {pior_pct:.0f}% do fundo de escala '
                                f'(limite de catálogo {SOBRECARGA_SEGURA_PCT:.0f}%). Alivie a carga.',
                                COR['perigo'])
        elif pior_pct >= 100.0:
            self._mostrar_aviso(f'Acima do fundo de escala: {pior_pct:.0f}% (a medida perde '
                                'precisão; sobrecarga segura até '
                                f'{SOBRECARGA_SEGURA_PCT:.0f}%).', COR['aviso_texto'])
        else:
            self.aviso.hide()

    def _mostrar_aviso(self, texto, cor):
        self.aviso.setStyleSheet(f'color: {cor}; font-weight: 700;')
        self.aviso.setText(texto)
        self.aviso.show()

    def _mostrar_status(self, texto, cor, dica):
        if (texto, cor) != self._status_atual:          # setStyleSheet só quando muda
            self._status_atual = (texto, cor)
            self.status_txt.setText(texto)
            self.status_txt.setStyleSheet(f'color: {cor};')
        self.status_txt.setToolTip(dica)

    def _atualizar_status(self, e, agora):
        """● online / conectada sem dados / offline, a partir do estado que o nó
        da célula manda 1×/s e da chegada das amostras."""
        if not e or agora - self._estado_t > 3.0:
            self._mostrar_status('● offline', COR['perigo'],
                                 'O nó da célula não está rodando. Abra a plataforma com a '
                                 'célula: iris (robô real) ou iris gemeo (gêmeo '
                                 'virtual); sem-nada abre sem ela.')
            return
        gemeo = e.get('fonte') == 'simulacao'
        origem = 'gêmeo virtual' if gemeo else f'célula real em {e.get("porta") or "?"}'
        if e.get('recebendo') and agora - self._ultima_amostra < 1.0:
            self._mostrar_status(f'● online · {origem} · {e.get("taxa_hz", 0):.0f} amostras/s',
                                 COR['sucesso'], 'A célula está mandando dados.')
        elif e.get('leitor') == 'ft_receiver':
            self._mostrar_status('● offline', COR['perigo'],
                                 'O leitor da célula (ft_receiver, o mesmo do cr10twin) não está '
                                 'mandando dados. No terminal, as linhas do ft_receiver dizem a '
                                 'porta e o que falta (cabo, fonte de 24 V).')
        elif e.get('conectado') and not gemeo:
            self._mostrar_status('● conectada, sem dados', COR['aviso_texto'], _motivo_sem_dados(e))
        else:
            motivo = (f' ({e["erro"]})' if e.get('erro') else '')
            self._mostrar_status('● offline', COR['perigo'],
                                 ('O sensor simulado do gêmeo não está mandando dados.' if gemeo else
                                  f'Conversor USB-RS485 não encontrado{motivo}. Ligue a célula; '
                                  'o programa tenta de novo a cada 2 s.'))

    def _atualizar_estado(self):
        e = self._estado
        self._atualizar_status(e, time.monotonic())
        if not e or time.monotonic() - self._estado_t > 3.0:
            self.estado_txt.setText(SEM_DADOS)
            return
        if e.get('fonte') == 'simulacao':
            linhas = ['Fonte: gêmeo virtual (sensor de força simulado do Gazebo).']
        elif e.get('leitor') == 'ft_receiver':
            linhas = ['Fonte: célula real, lida pelo ft_receiver do cr10twin (o mesmo que '
                      'alimenta as Coletas).']
            linhas.append(f'Recebendo: {e.get("taxa_hz", 0):.0f} amostras/s.' if e.get('recebendo')
                          else 'Sem dados do ft_receiver: veja as linhas dele no terminal.')
            self.estado_txt.setText('\n'.join(linhas))
            return
        else:
            linhas = [f'Fonte: célula real, conversor USB-RS485 em {e.get("porta") or "(procurando)"}.']
        if e.get('recebendo'):
            linhas.append(f'Recebendo: {e.get("taxa_hz", 0):.0f} amostras/s.')
        elif e.get('conectado'):
            linhas.append('Porta aberta, mas sem medidas: ' + _motivo_sem_dados(e))
        if e.get('baud'):
            linhas.append(f'Velocidade: {e["baud"]} baud'
                          + (' (automática: tenta 1000000 e 115200)' if e.get('baud_auto') else '')
                          + '.')
        else:
            linhas.append('Conversor não encontrado' + (f' ({e["erro"]})' if e.get('erro') else '') +
                          '. Tentando de novo a cada 2 s.')
        if e.get('modo_leitura') == 'polled' and 'sem_resposta' in e:
            linhas.append('Modo: o programa pergunta e a célula responde (polled). Erros: '
                          f'{e["sem_resposta"]} perguntas sem resposta, {e["erros_resposta"]} '
                          f'respostas com erro, {e["valores_invalidos"]} valores inválidos.')
        elif 'erros_crc' in e:
            linhas.append(f'Erros do link: {e["erros_crc"]} CRC, {e["ressincronizacoes"]} '
                          f'ressincronizações, {e["valores_invalidos"]} valores inválidos.')
        if e.get('aviso'):
            linhas.append(f'Atenção: {e["aviso"]}.')
        if e.get('tarado'):
            linhas.append('Tara ligada: ' + ', '.join(
                f'{n} {v:+.2f}' for n, v in zip(EIXOS, e.get('tara', []))) + '.')
        else:
            linhas.append('Sem tara (valores como saem da célula).')
        self.estado_txt.setText('\n'.join(linhas))

    def carregar_config(self, dados):
        self.inverter_chk.setChecked(bool(dados.get('celula_inverter_sinal', False)))

    def salvar_config(self, dados):
        dados['celula_inverter_sinal'] = self.inverter_chk.isChecked()
