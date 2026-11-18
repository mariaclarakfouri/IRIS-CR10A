"""Módulos Tátil 5×5 e Tátil 4×4: os sensores táteis lidos pela placa STM32.

A leitura é a do projeto cr10twin, sem mudanças (dobot_tatil/touch_source.py:
TouchSensorSource), rodando num PROCESSO separado da janela (leitor_tatil.py:
assim ela não disputa o processador com o desenho). Esta página acrescenta:

  Conexão     automática: com a placa ligada no computador, a aba conecta sozinha.
              As duas placas são STM32 iguais na USB, então a aba abre cada
              porta livre, confere o tamanho do frame (16 ou 25 taxels) e fica
              com a deste sensor; o firmware (atual ou antigo, com TOTAL) também
              é reconhecido pelas linhas. Cabo solto: volta a procurar. A porta
              ainda pode ser escolhida à mão;
  Estado      "● conectado" no topo; passando o mouse, porta, firmware, frames
              bons e cortados e o que a placa está mandando;
  Gravar      dois CSVs: os frames (tensão de cada taxel, numeração física) e
              os eventos (spikes RA/SA, cuneiformes, neurônio pós, I_final).

O desenho (mapa de calor, raster RA/SA, sinal escalar e neurônio pós ou
cuneiformes) é o PainelTatil (painel_tatil.py), feito direto com o Qt: os
quatro gráficos ~60 vezes por segundo, o mapa sempre com o último frame.
"""
import datetime
import os
import threading
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QPushButton,
)

from ..constantes import REGISTROS_DIR
from ..leitor_tatil import FonteTatil
from ..painel_tatil import PainelTatil
from ..tema import COR
from ..widgets import com_icone, rotulo
from .base import Modulo

try:
    import importlib
    from dobot_tatil.constantes_tatil import FT_USB_VIDS
    importlib.import_module('dobot_tatil.touch_source')     # confere que a leitura existe
    _ERRO_IMPORT = None
except Exception as e:  # noqa: BLE001 — pacote ausente ou pyserial faltando
    _ERRO_IMPORT = str(e)

try:
    from serial.tools import list_ports
except Exception:  # noqa: BLE001
    list_ports = None

QUADRO_MS = 16                 # os quatro gráficos ~60×/s (medido com a placa real: sobra folga)
ESTADO_MS = 500
STM32_NATIVE_CDC_PID = 0x5740  # a USB nativa da placa (a do ST-Link é mais lenta)
STLINK_VCP_PID = 0x374b        # a ponte serial do ST-Link: os firmwares táteis não usam
BUSCA_MS = 1000                # procura/confere a placa a cada segundo
TESTE_S = 4.0                  # tempo para ver o tamanho do frame (inclui abrir o processo)
PULAR_OUTRA_PLACA_S = 15.0     # placa do outro sensor: não reabre por um tempo
PULAR_SEM_DADOS_S = 30.0       # porta que não manda dados do sensor
SEM_DADOS_S = 1.0
# Diagnóstico (aparece na tela e vai para o registro da interface):
JANELA_DIAG_S = 5.0            # frames cortados medidos nos últimos 5 s
CORTADOS_AVISO = 0.10          # acima de 10% de frames cortados: avisa
PARADA_TELA_S = 0.3            # a janela ficou sem atualizar mais que isso: anota
DIAG_LOG_S = 10.0              # resumo no registro no máximo a cada 10 s

# Portas abertas por qualquer aba tátil: a outra aba não pode pegar a mesma.
_PORTAS_EM_USO = set()

def _ler_portas():
    """Portas que podem ser a placa: /dev/ttyACM* e /dev/ttyUSB*, menos os
    conversores da célula de carga; a USB nativa da STM32 primeiro. LENTO às
    vezes (o Linux lê vários arquivos por porta): só a thread de busca chama."""
    if list_ports is None:
        return []
    portas = [p for p in list_ports.comports()
              if ('ACM' in p.device or 'USB' in p.device)
              and p.vid not in FT_USB_VIDS]
    portas.sort(key=lambda p: (p.pid != STM32_NATIVE_CDC_PID, p.device))
    return portas


class _BuscaPortas:
    """Lista de portas atualizada em SEGUNDO PLANO. Ler a lista do Linux leva de
    3 a ~50 ms; feito na thread da janela, isso dava uma travadinha na tela a
    cada busca (medido na interface completa, com a aba 4×4 procurando a placa
    dela). A janela só lê a última lista pronta, que é instantâneo."""

    def __init__(self):
        self._lock = threading.Lock()
        self._portas = []
        self._thread = None

    def lista(self):
        if self._thread is None:
            self._portas = _ler_portas()          # a primeira vez, já com a lista
            self._thread = threading.Thread(target=self._laco, daemon=True,
                                            name='busca-portas-tatil')
            self._thread.start()
        with self._lock:
            return list(self._portas)

    def _laco(self):
        while True:
            time.sleep(BUSCA_MS / 1000.0)
            try:
                portas = _ler_portas()
            except Exception:  # noqa: BLE001 — uma leitura ruim não derruba a busca
                continue
            with self._lock:
                self._portas = portas


_BUSCA = _BuscaPortas()


def _portas_candidatas():
    return _BUSCA.lista()


class _SensorTatil(Modulo):
    LINHAS = 5
    COLUNAS = 5
    ICONE = 'grid'

    def construir(self):
        self.n = self.LINHAS * self.COLUNAS
        self.fonte = None
        self.porta = None
        self._grav = None                      # {'base', 'inicio'} enquanto grava
        self._grav_parando = None              # base da gravação que está fechando
        self._por_segundo = {}
        self._ultimo_estado = 0.0
        self._estado_con = 'procurando'      # procurando | testando | conectado
        self._teste_ate = 0.0
        self._pular = {}                       # porta -> (até quando, motivo)
        self._firmware_antigo = False
        self._diag_frames = []                 # (t, frames_ok, frames_bad)
        self._diag_cortados = 0.0
        self._diag_ultimo_tick = None
        self._diag_paradas = []                # durações (s) das paradas da janela
        self._diag_ultimo_log = 0.0

        lay = self.layout_modulo
        if _ERRO_IMPORT:
            lay.addWidget(rotulo(f'A leitura do sensor tátil não carregou: {_ERRO_IMPORT}',
                                 'cartaoTexto'))
            lay.addStretch()
            return

        barra = QHBoxLayout()
        barra.setSpacing(10)
        barra.addWidget(rotulo('Porta'))
        self.porta_combo = QComboBox()
        self.porta_combo.setEditable(True)
        self.porta_combo.setMinimumContentsLength(34)
        self.porta_combo.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self.porta_combo.setToolTip(
            'Em "Automática", a aba acha sozinha a placa deste sensor: abre cada porta livre, '
            'confere o tamanho do frame (16 ou 25 taxels) e fica com a certa. A porta do '
            'ST-Link fica de fora (ela não manda os dados). Escolha uma porta só se precisar.')
        self.porta_combo.activated.connect(lambda _i: self._porta_mudou())
        self.porta_combo.lineEdit().editingFinished.connect(self._porta_mudou)
        barra.addWidget(self.porta_combo)
        self.conexao_txt = rotulo('', 'cartaoTexto')
        barra.addWidget(self.conexao_txt)
        barra.addStretch()
        self.gravacao_txt = rotulo('', 'cartaoTexto')
        barra.addWidget(self.gravacao_txt)
        self.gravar_btn = com_icone(QPushButton('Gravar em CSV'), 'record')
        self.gravar_btn.setObjectName('alternar')
        self.gravar_btn.setCheckable(True)
        self.gravar_btn.setEnabled(False)
        self.gravar_btn.setToolTip(f'Grava os frames e os eventos do sensor em {self._pasta()}.')
        self.gravar_btn.toggled.connect(self._alternar_gravacao)
        barra.addWidget(self.gravar_btn)
        lay.addLayout(barra)

        self.aviso = rotulo('', 'cartaoTexto')
        self.aviso.setWordWrap(True)
        self.aviso.hide()
        lay.addWidget(self.aviso)

        self.painel = PainelTatil(self.LINHAS, self.COLUNAS)
        lay.addWidget(self.painel, 1)
        self._listar_portas()
        self._atualizar_estado()

        self._timer = QTimer(self)
        self._timer.setInterval(QUADRO_MS)
        self._timer.timeout.connect(self._tick)
        self._timer.start()
        # Conexão automática: procura e confere a placa sem ninguém clicar.
        self._timer_busca = QTimer(self)
        self._timer_busca.setInterval(BUSCA_MS)
        self._timer_busca.timeout.connect(self._buscar)
        self._timer_busca.start()
        QTimer.singleShot(300, self._buscar)

    # --- figura ---

    def _tem_total(self):
        return self._firmware_antigo

    def _nova_fonte(self, porta):
        return FonteTatil(porta, self.LINHAS, self.COLUNAS, self._tem_total())

    # --- portas ---

    def _listar_portas(self):
        if self.porta_combo.view().isVisible() or self.porta_combo.lineEdit().hasFocus():
            return                           # não mexe na lista com ela aberta
        atual = self.porta_combo.currentData() if self.porta_combo.count() else None
        texto_atual = self.porta_combo.currentText()
        itens = [('Automática', None)]
        for p in _portas_candidatas():
            uso = ' (outra aba tátil)' if (p.device in _PORTAS_EM_USO
                                          and p.device != self.porta) else ''
            stlink = ' (ST-Link)' if p.pid == STLINK_VCP_PID else ''
            itens.append((f'{p.device} — {p.description}{stlink}{uso}', p.device))
        if itens == getattr(self, '_itens_porta', None):
            return                           # nada mudou: não mexe na caixa
        self._itens_porta = itens
        self.porta_combo.blockSignals(True)
        self.porta_combo.clear()
        for texto, dado in itens:
            self.porta_combo.addItem(texto, dado)
        i = self.porta_combo.findData(atual)
        if i >= 0:
            self.porta_combo.setCurrentIndex(i)
        elif texto_atual and not texto_atual.startswith('Automática') and atual is None \
                and self.porta_combo.findText(texto_atual) < 0:
            self.porta_combo.setEditText(texto_atual)
        self.porta_combo.blockSignals(False)

    def _porta_pedida(self):
        """A porta escolhida à mão, ou None em "Automática"."""
        texto = self.porta_combo.currentText().strip()
        i = self.porta_combo.currentIndex()
        if i >= 0 and self.porta_combo.itemText(i) == texto:
            return self.porta_combo.itemData(i)
        if not texto or texto.startswith('Automática'):
            return None
        return texto.split(' — ')[0].strip() or None       # caminho digitado

    def _porta_mudou(self):
        """Trocou a escolha de porta: larga a atual e procura de novo."""
        pedida = self._porta_pedida()
        if pedida == self.porta and self._estado_con != 'procurando':
            return
        self._pular.clear()
        if self.fonte is not None:
            self._liberar('troca de porta')
        QTimer.singleShot(0, self._buscar)

    def _proxima_porta(self):
        pedida = self._porta_pedida()
        if pedida:
            return pedida if pedida not in _PORTAS_EM_USO else None
        agora = time.monotonic()
        for p in _portas_candidatas():
            if p.pid == STLINK_VCP_PID:
                continue                       # a ponte do ST-Link não manda os dados
            if p.device in _PORTAS_EM_USO:
                continue
            ate, _motivo = self._pular.get(p.device, (0.0, ''))
            if agora < ate:
                continue
            return p.device
        return None

    # --- conexão automática ---

    def _buscar(self):
        """A cada BUSCA_MS: procura a placa, confere a que está em teste ou
        percebe que o cabo saiu."""
        if _ERRO_IMPORT:
            return
        self._listar_portas()
        if self._estado_con == 'conectado':
            if self.fonte is not None and not self.fonte.connected:
                self._liberar('cabo desconectado')
                self.janela.log(f'{self.NOME}: a placa saiu da USB; procurando de novo.')
            return
        if self._estado_con == 'testando':
            self._conferir_teste()
            return
        porta = self._proxima_porta()
        if porta is not None:
            self._abrir(porta)

    def _abrir(self, porta):
        fonte = self._nova_fonte(porta)
        if not fonte.start():
            self._pular[porta] = (time.monotonic() + PULAR_SEM_DADOS_S,
                                  f'não abriu ({fonte.error})')
            return
        self.fonte, self.porta = fonte, porta
        _PORTAS_EM_USO.add(porta)
        self._estado_con = 'testando'
        self._teste_ate = time.monotonic() + TESTE_S
        self._atualizar_estado()

    def _conferir_teste(self):
        """É a placa deste sensor? Decide pelo tamanho do frame (ADC) ou pelo
        maior índice de taxel (DATA, firmware antigo)."""
        f = self.fonte
        if f is None or not f.connected:
            motivo = f'não abriu ({f.error})' if f is not None and f.error else 'a porta fechou'
            self._liberar(motivo, pular=(PULAR_SEM_DADOS_S, motivo))
            return
        tam, max_idx, viu = f.tam_adc, f.max_idx, set(f.viu)
        n = self.n
        if tam == n or (tam is None and max_idx == n - 1):
            self._aceitar('TOTAL' in viu)
            return
        if (tam is not None and tam != n) or max_idx > n - 1:
            outro = tam if tam is not None else max_idx + 1
            self._liberar('placa do outro sensor',
                          pular=(PULAR_OUTRA_PLACA_S, f'placa de {outro} taxels (outro sensor)'))
            return
        if time.monotonic() > self._teste_ate:
            motivo = ('nada do sensor chegou' if not viu
                      else 'não deu para saber o tamanho do frame')
            self._liberar(motivo, pular=(PULAR_SEM_DADOS_S, motivo))

    def _aceitar(self, firmware_antigo):
        porta = self.porta
        if firmware_antigo != self._firmware_antigo:
            # O firmware antigo manda TOTAL: a leitura e a figura mudam de modo.
            self._firmware_antigo = firmware_antigo
            self.fonte.stop()
            fonte = self._nova_fonte(porta)
            if not fonte.start():
                self._liberar('não reabriu', pular=(PULAR_SEM_DADOS_S, 'não reabriu'))
                return
            self.fonte = fonte
        self.painel.definir_fonte(self.fonte, self._firmware_antigo)
        self._estado_con = 'conectado'
        self._pular.clear()
        self.gravar_btn.setEnabled(True)
        fw = 'antigo (DATA + TOTAL + POST)' if self._firmware_antigo else 'atual (ADC + cuneiformes)'
        self.janela.log(f'{self.NOME}: placa encontrada em {porta}, firmware {fw}.')
        self._atualizar_estado()

    def _liberar(self, motivo=None, pular=None):
        """Fecha a porta e volta a procurar."""
        if self._grav is not None:
            g, self._grav = self._grav, None
            self.gravar_btn.blockSignals(True)
            self.gravar_btn.setChecked(False)
            self.gravar_btn.blockSignals(False)
            self.gravar_btn.setText('Gravar em CSV')
            self.gravacao_txt.setText('')
            f = self.fonte
            n_f, n_e = (f.grav_frames, f.grav_eventos) if f is not None else (0, 0)
            msg = (f'{self.NOME}: gravação encerrada ({motivo}): {n_f} frames e {n_e} eventos '
                   f'em {g["base"]}_frames.csv e _eventos.csv.')
            self.janela.log(msg)
            self.janela.alerta.mostrar(msg, 'aviso')
        fonte, self.fonte = self.fonte, None
        if fonte is not None:
            fonte.stop()
        if self.porta is not None:
            _PORTAS_EM_USO.discard(self.porta)
            if pular is not None:
                self._pular[self.porta] = (time.monotonic() + pular[0], pular[1])
        self.porta = None
        self._estado_con = 'procurando'
        self.painel.limpar()
        self.gravar_btn.setEnabled(False)
        self._atualizar_estado()

    def _desconectar(self, motivo=None):
        self._liberar(motivo)

    # --- tick da tela ---

    def _tick(self):
        if _ERRO_IMPORT:
            return
        agora = time.monotonic()
        # A janela ficou parada? (o tick deveria vir a cada QUADRO_MS)
        if self._estado_con == 'conectado' and self.isVisible() \
                and self._diag_ultimo_tick is not None \
                and agora - self._diag_ultimo_tick > PARADA_TELA_S:
            self._diag_paradas.append(agora - self._diag_ultimo_tick)
        self._diag_ultimo_tick = agora
        if self.fonte is not None:
            self.fonte.receber()          # sempre (mesmo escondida): só junta os pacotes
            self._conferir_gravacao()
        if self._estado_con == 'conectado' and self.isVisible():
            self.painel.atualizar()
        agora = time.monotonic()
        if agora - self._ultimo_estado >= ESTADO_MS / 1000.0:
            self._atualizar_estado()

    def _atualizar_estado(self):
        agora = time.monotonic()
        dt = max(agora - self._ultimo_estado, 1e-3) if self._ultimo_estado else None
        self._ultimo_estado = agora
        f = self.fonte
        contagem = {}
        if f is not None:
            contagem = dict(f.contagem)
            f.contagem.clear()
        tam_adc = f.tam_adc if f is not None else None
        if dt:
            self._por_segundo = {k: v / dt for k, v in contagem.items()}
        self._diagnosticar(agora)
        self._mostrar_avisos(tam_adc)
        if self._grav is not None and f is not None:
            self.gravacao_txt.setText(f'gravando · {f.grav_frames} frames · {f.grav_eventos} '
                                      f'eventos · {agora - self._grav["inicio"]:.0f} s')
        if self._estado_con != 'conectado' or f is None:
            self.conexao_txt.setText('● procurando a placa' if self._estado_con == 'procurando'
                                     else f'● conferindo {self.porta}')
            self._cor_conexao(COR['aviso_texto'])
            linhas = ([f'Conferindo se a placa em {self.porta} é a deste sensor...']
                      if self._estado_con == 'testando' else
                      [f'Procurando a placa do sensor {self.LINHAS}×{self.COLUNAS} na USB. Ligue '
                       'a placa no computador: a aba conecta sozinha.'])
            vistos = []
            for p in _portas_candidatas():
                if p.pid == STLINK_VCP_PID:
                    vistos.append(f'{p.device}: ST-Link (não manda os dados)')
                elif p.device in _PORTAS_EM_USO and p.device != self.porta:
                    vistos.append(f'{p.device}: com a outra aba tátil')
                elif p.device in self._pular and agora < self._pular[p.device][0]:
                    vistos.append(f'{p.device}: {self._pular[p.device][1]}')
            if vistos:
                linhas.append('Portas vistas: ' + '; '.join(vistos) + '.')
            self.conexao_txt.setToolTip('\n'.join(linhas))
            return
        self.conexao_txt.setText(f'● conectado em {self.porta}')
        self._cor_conexao(COR['sucesso'])
        fw = 'antigo (DATA + TOTAL + POST)' if self._firmware_antigo else 'atual (ADC + cuneiformes)'
        linhas = [f'Porta: {self.porta}', f'Firmware: {fw}.']
        if f.is_fresh(SEM_DADOS_S):
            linhas.append('Recebendo dados.')
        elif f.last_rx > 0:
            linhas.append(f'Sem dados há {time.monotonic() - f.last_rx:.0f} s.')
        total = f.frames_ok + f.frames_bad
        if total:
            linhas.append(f'Frames: {f.frames_ok} bons, {f.frames_bad} cortados '
                          f'({100.0 * f.frames_bad / total:.1f}%).')
        if self._por_segundo:
            ordem = ('ADC', 'DATA', 'RA', 'SA', 'CN_SA', 'CN_RA', 'CN_MM', 'POST', 'TOTAL')
            partes = [f'{k} {self._por_segundo[k]:.0f}' for k in ordem if k in self._por_segundo]
            if partes:
                linhas.append('Linhas por segundo: ' + ', '.join(partes) + '.')
        self.conexao_txt.setToolTip('\n'.join(linhas))

    def _cor_conexao(self, cor):
        if cor != getattr(self, '_cor_atual', None):     # setStyleSheet só quando muda
            self._cor_atual = cor
            self.conexao_txt.setStyleSheet(f'color: {cor};')

    def _diagnosticar(self, agora):
        """Frames cortados nos últimos segundos, placa muda e paradas da janela:
        mostra na tela e anota no registro, para saber POR QUE algo travou."""
        f = self.fonte
        if self._estado_con != 'conectado' or f is None:
            self._diag_frames.clear()
            self._diag_cortados = 0.0
            return
        self._diag_frames.append((agora, f.frames_ok, f.frames_bad))
        while self._diag_frames and agora - self._diag_frames[0][0] > JANELA_DIAG_S:
            self._diag_frames.pop(0)
        t0, ok0, bad0 = self._diag_frames[0]
        ok, bad = f.frames_ok - ok0, f.frames_bad - bad0
        self._diag_cortados = bad / (ok + bad) if ok + bad >= 50 else 0.0
        if agora - self._diag_ultimo_log < DIAG_LOG_S:
            return
        partes = []
        if self._diag_cortados > CORTADOS_AVISO:
            partes.append(f'{100 * self._diag_cortados:.0f}% dos frames chegaram cortados nos '
                          f'últimos {JANELA_DIAG_S:.0f} s ({bad} de {ok + bad})')
        if f.last_rx and agora - f.last_rx > 2.0:
            partes.append(f'a placa não manda dados há {agora - f.last_rx:.0f} s')
        if self._diag_paradas:
            partes.append(f'a janela ficou parada {len(self._diag_paradas)} vez(es), a pior por '
                          f'{max(self._diag_paradas):.1f} s')
            self._diag_paradas.clear()
        if partes:
            self._diag_ultimo_log = agora
            por_s = ', '.join(f'{k} {v:.0f}' for k, v in sorted(self._por_segundo.items()))
            self.janela.log(f'{self.NOME} (diagnóstico): ' + '; '.join(partes)
                            + (f'. Linhas por segundo: {por_s}.' if por_s else '.'))

    def _mostrar_avisos(self, tam_adc):
        aviso = None
        pedida = self._porta_pedida()
        f = self.fonte
        if self._estado_con == 'conectado' and f is not None and f.last_rx \
                and time.monotonic() - f.last_rx > 2.0:
            aviso = (f'A placa parou de mandar dados há {time.monotonic() - f.last_rx:.0f} s (o '
                     'gráfico fica parado no último dado). Se não voltar, desligue e ligue a '
                     'USB da placa.')
        elif self._estado_con == 'conectado' and self._diag_cortados > CORTADOS_AVISO:
            aviso = (f'A placa está perdendo dados na USB: {100 * self._diag_cortados:.0f}% dos '
                     f'frames chegaram cortados nos últimos {JANELA_DIAG_S:.0f} s e foram '
                     'descartados, por isso o mapa parece travar. O firmware antigo do 5×5 '
                     'tinha esse defeito (corrigido no TouchFirmware do cr10twin).')
        elif pedida and self._estado_con == 'procurando' and pedida in _PORTAS_EM_USO:
            aviso = f'A porta {pedida} está com a outra aba tátil. Escolha outra ou "Automática".'
        elif pedida and self.porta is None and pedida in self._pular:
            aviso = f'A porta {pedida}: {self._pular[pedida][1]}. Escolha outra ou "Automática".'
        if aviso:
            self.aviso.setStyleSheet(f'color: {COR["aviso_texto"]}; font-weight: 700;')
            self.aviso.setText(aviso)
            self.aviso.show()
        else:
            self.aviso.hide()

    # --- gravação ---

    def _alternar_gravacao(self, ligado):
        if ligado:
            self._iniciar_gravacao()
        else:
            self._parar_gravacao()

    def _pasta(self):
        """Uma pasta por sensor: ~/iris_registros/sensores táteis 5x5 (ou 4x4)."""
        return os.path.join(REGISTROS_DIR, f'sensores táteis {self.LINHAS}x{self.COLUNAS}')

    def _iniciar_gravacao(self):
        f = self.fonte
        if f is None:
            self.gravar_btn.setChecked(False)
            return
        pasta = self._pasta()
        os.makedirs(pasta, exist_ok=True)
        agora = datetime.datetime.now()
        base = os.path.join(pasta, f'tatil{self.LINHAS}x{self.COLUNAS}_'
                                       f'{agora.strftime("%Y%m%d_%H%M%S")}')
        cab = (f'# sensor tátil {self.LINHAS}x{self.COLUNAS}; porta {self.porta}; '
               f'início {agora.isoformat(timespec="seconds")}; t_stm_s = relógio da placa '
               '(micros, volta a zero a cada ~71 min)\n')
        colunas = [f'v{r}{c}' for r in range(self.LINHAS) for c in range(self.COLUNAS)]
        cab_f = (cab + '# tensões em volts (adc x 3,3 / 4095), numeração física dos taxels '
                 '(v00 = taxel físico 00), como os CSVs do cr10twin\n'
                 + 't_stm_s,' + ','.join(colunas) + '\n')
        cab_e = (cab + '# tipo: RA, SA (taxel físico e adc), CN_SA, CN_RA, CN_MM, POST, '
                 'TOTAL (valor = I_final)\n' + 't_stm_s,tipo,taxel,valor\n')
        f.gravar(base, cab_f, cab_e)
        self._grav = {'base': base, 'inicio': time.monotonic()}
        self.gravar_btn.setText('Parar gravação')
        self.janela.log(f'{self.NOME}: gravando em {base}_frames.csv e _eventos.csv.')

    def _parar_gravacao(self):
        g, self._grav = self._grav, None
        self.gravar_btn.setText('Gravar em CSV')
        self.gravacao_txt.setText('')
        if g is None or self.fonte is None:
            return
        self._grav_parando = g['base']
        self.fonte.parar_gravacao()        # a mensagem sai quando os arquivos fecharem

    def _conferir_gravacao(self):
        """Respostas do processo de leitura sobre a gravação."""
        f = self.fonte
        if self._grav is not None and f.grav_erro:
            erro, f.grav_erro = f.grav_erro, None
            self._grav = None
            self.gravar_btn.blockSignals(True)
            self.gravar_btn.setChecked(False)
            self.gravar_btn.blockSignals(False)
            self.gravar_btn.setText('Gravar em CSV')
            self.gravacao_txt.setText('')
            self.janela.alerta.mostrar(f'Não foi possível gravar: {erro}', 'erro')
        if self._grav_parando is not None and f.grav_fim is not None:
            n_f, n_e = f.grav_fim
            f.grav_fim = None
            base, self._grav_parando = self._grav_parando, None
            msg = (f'{self.NOME} gravado: {n_f} frames e {n_e} eventos em '
                   f'{base}_frames.csv e _eventos.csv.')
            self.janela.log(msg)
            self.janela.alerta.mostrar(msg, 'ok')

    # --- configuração ---

    def carregar_config(self, dados):
        if _ERRO_IMPORT:
            return
        porta = dados.get(f'tatil{self.LINHAS}x{self.COLUNAS}_porta')
        if porta:
            i = self.porta_combo.findData(porta)
            if i >= 0:
                self.porta_combo.setCurrentIndex(i)
            else:
                self.porta_combo.setEditText(porta)
            self._porta_mudou()

    def salvar_config(self, dados):
        if _ERRO_IMPORT:
            return
        porta = self.porta_combo.currentData() if self.porta_combo.currentIndex() > 0 else None
        if porta is None and self.porta_combo.currentText() != 'Automática':
            porta = self.porta_combo.currentText().split(' — ')[0].strip() or None
        dados[f'tatil{self.LINHAS}x{self.COLUNAS}_porta'] = porta
        # Fechando a interface: fecha a gravação e a porta.
        self._timer_busca.stop()
        if self.fonte is not None:
            self._liberar('interface fechada')


class Tatil5x5(_SensorTatil):
    NOME = 'Tátil 5×5'
    CHAVE_HOME = 'tatil_5x5'
    DESCRICAO = ('Sensor tátil 5×5 (placa STM32): mapa de tensões, neurônios RA/SA de cada '
                 'taxel e cuneiformes, com gravação em CSV.')
    LINHAS = COLUNAS = 5


class Tatil4x4(_SensorTatil):
    NOME = 'Tátil 4×4'
    CHAVE_HOME = 'tatil_4x4'
    DESCRICAO = ('Sensor tátil 4×4 (placa STM32): mapa de tensões, neurônios RA/SA de cada '
                 'taxel e cuneiformes ou neurônio pós, com gravação em CSV.')
    LINHAS = COLUNAS = 4
    # O firmware é reconhecido sozinho: desde 24/09/2026 o 4×4 roda o mesmo do
    # 5×5 (frames ADC + cuneiformes); o antigo manda DATA por taxel, TOTAL e POST.
