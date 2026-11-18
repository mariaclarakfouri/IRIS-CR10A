"""Leitura do sensor tátil num PROCESSO separado da janela.

Por quê: o Python roda uma linha de trabalho por vez dentro de um processo. Com
a leitura da placa (o TouchSensorSource do cr10twin, sem mudanças, interpretando
~3000 linhas por segundo quando o sensor é apertado) no mesmo processo da
janela, os dois revezavam o processador e o desenho dava travadinhas. Aqui a
leitura roda noutro processo (noutro núcleo), e a janela só recebe ~60 pacotes
por segundo com o que é NOVO: o último frame, os disparos novos, as amostras
novas do sinal escalar e o estado. A gravação dos CSVs também acontece lá.

Este arquivo não importa Qt nem ROS: o processo filho é aberto do zero
("spawn"), sem herdar as threads da janela.

  FonteTatil      o lado da janela: abre o processo, recebe os pacotes e guarda
                  os últimos segundos em arrays (para o PainelTatil desenhar);
  _processo       o lado da placa: roda o TouchSensorSource e manda os pacotes.
"""
import bisect
import collections
import math
import multiprocessing as mp
import re
import sys
import time

import numpy as np

JANELA_S = 5.0                 # o que a janela guarda (o que os gráficos mostram)
PERIODO_S = 1.0 / 60.0         # pacotes por segundo do processo de leitura
VREF = 3.3
CHAVES_EVENTOS = ('cn_sa', 'cn_ra', 'cn_mm', 'post')

RE_IDX_ADC_T = re.compile(r'idx=(\d+),adc=(\d+),t=(\d+)')
RE_T = re.compile(r't=(\d+)')
RE_IFINAL = re.compile(r'Ifinal=([-+0-9.eE]+)')


# ======================================================================
# Lado da janela
# ======================================================================

class FonteTatil:
    """O que a aba usa: mesmos nomes do TouchSensorSource (connected, error,
    frames_ok, frames_bad, last_rx, is_fresh, start, stop) + os dados já em
    arrays e a gravação."""

    def __init__(self, porta, linhas, colunas, tem_total):
        self.port = porta
        self.linhas, self.colunas = linhas, colunas
        self.n = linhas * colunas
        self.tem_total = bool(tem_total)
        self.connected = False
        self.error = None
        self.frames_ok = self.frames_bad = 0
        self.last_rx = 0.0
        # detecção da placa (tamanho do frame, firmware) e linhas por tipo
        self.tam_adc = None
        self.max_idx = -1
        self.viu = set()
        self.contagem = collections.Counter()
        # gravação (acontece no processo de leitura)
        self.grav_frames = self.grav_eventos = 0
        self.grav_erro = None
        self.grav_fim = None           # (frames, eventos) quando a gravação fecha
        # dados para o desenho
        self.volt = None               # último frame, já na orientação da tela
        self.ra_t = np.empty(0)
        self.ra_k = np.empty(0, dtype=np.int16)
        self.sa_t = np.empty(0)
        self.sa_k = np.empty(0, dtype=np.int16)
        self.eventos = {c: np.empty(0) for c in CHAVES_EVENTOS}
        self.esc_t = np.empty(0)
        self.esc_v = np.empty(0)
        self._proc = None
        self._conn = None
        self._aberto = None            # None = esperando o processo responder

    # --- ciclo de vida ---

    def start(self):
        """Abre o processo de leitura e volta na hora (não espera a porta abrir:
        a resposta chega pelo receber()). Devolve False só se nem o processo abriu."""
        try:
            ctx = mp.get_context('spawn')
            self._conn, filho = ctx.Pipe()
            self._proc = ctx.Process(
                target=_processo, args=(self.port, self.linhas, self.colunas, self.tem_total,
                                        filho),
                daemon=True, name=f'leitor-tatil-{self.port}')
            with _sem_reimportar_principal():
                self._proc.start()
            filho.close()
        except Exception as e:  # noqa: BLE001
            self.error = f'não abriu o processo de leitura: {e}'
            return False
        self.connected = True        # "tentando": vira False se a porta não abrir
        return True

    def stop(self):
        conn, proc, self._conn, self._proc = self._conn, self._proc, None, None
        if conn is not None:
            try:
                conn.send(('sair',))
            except (OSError, EOFError, BrokenPipeError):
                pass
        if proc is not None:
            proc.join(timeout=1.5)
            if proc.is_alive():
                proc.terminate()
                proc.join(timeout=0.5)
        if conn is not None:
            conn.close()
        self.connected = False

    def is_fresh(self, max_age=1.0):
        return self.last_rx > 0.0 and (time.monotonic() - self.last_rx) < max_age

    # --- gravação ---

    def gravar(self, base, cab_frames, cab_eventos):
        self.grav_frames = self.grav_eventos = 0
        self.grav_erro = None
        self.grav_fim = None
        self._mandar(('gravar', base, cab_frames, cab_eventos))

    def parar_gravacao(self):
        self._mandar(('parar_gravacao',))

    def _mandar(self, msg):
        if self._conn is not None:
            try:
                self._conn.send(msg)
            except (OSError, EOFError, BrokenPipeError):
                self.connected = False

    # --- pacotes ---

    def receber(self):
        """Lê todos os pacotes que chegaram (chamado a cada quadro, rápido)."""
        conn = self._conn
        if conn is None:
            return
        try:
            while conn.poll():
                msg = conn.recv()
                tipo = msg[0]
                if tipo == 'dados':
                    self._juntar(msg[1])
                elif tipo == 'aberto':
                    _, ok, erro = msg
                    self._aberto = ok
                    if not ok:
                        self.connected = False
                        self.error = erro
                elif tipo == 'gravando':
                    if not msg[1]:
                        self.grav_erro = msg[2]
                elif tipo == 'gravacao_fim':
                    self.grav_fim = (msg[1], msg[2])
        except (EOFError, OSError, BrokenPipeError):
            self.connected = False
            self.error = self.error or 'o processo de leitura parou'
        if self._proc is not None and not self._proc.is_alive() and self.connected:
            self.connected = False
            self.error = self.error or 'o processo de leitura parou'

    def _juntar(self, p):
        st = p['estado']
        self.connected = st['connected'] and self._aberto is not False
        self.error = st['error']
        self.frames_ok, self.frames_bad = st['frames_ok'], st['frames_bad']
        self.last_rx = st['last_rx']
        self.tam_adc, self.max_idx = st['tam_adc'], st['max_idx']
        self.viu.update(st['viu'])
        self.contagem.update(st['contagem'])
        self.grav_frames, self.grav_eventos = st['grav_frames'], st['grav_eventos']
        if p['volt'] is not None:
            self.volt = p['volt']
        corte = time.monotonic() - JANELA_S - 0.5
        self.ra_t, self.ra_k = _anexar2(self.ra_t, self.ra_k, *p['ra'], corte)
        self.sa_t, self.sa_k = _anexar2(self.sa_t, self.sa_k, *p['sa'], corte)
        for c in CHAVES_EVENTOS:
            self.eventos[c] = _anexar1(self.eventos[c], p[c], corte)
        self.esc_t, self.esc_v = _anexar2(self.esc_t, self.esc_v, *p['escalar'], corte)


class _sem_reimportar_principal:
    """O "spawn" do multiprocessing roda de novo, no processo novo, o arquivo
    principal do programa (a janela, um script de teste...). O leitor não precisa
    dele: só deste módulo. Enquanto o processo sobe, o principal fica "sem
    arquivo", e o filho não o reimporta."""

    def __enter__(self):
        self.main = sys.modules.get('__main__')
        self.guardado = {}
        for nome in ('__file__', '__spec__'):
            if self.main is not None and hasattr(self.main, nome):
                self.guardado[nome] = getattr(self.main, nome)
        if self.main is not None:
            if '__file__' in self.guardado:
                del self.main.__file__
            self.main.__spec__ = None

    def __exit__(self, *_):
        if self.main is not None:
            for nome, valor in self.guardado.items():
                setattr(self.main, nome, valor)


def _anexar1(a, novos, corte):
    if len(novos):
        a = np.concatenate([a, novos])
    i = int(np.searchsorted(a, corte))
    return a[i:] if i else a


def _anexar2(t, v, t_novos, v_novos, corte):
    if len(t_novos):
        t = np.concatenate([t, t_novos])
        v = np.concatenate([v, v_novos])
    i = int(np.searchsorted(t, corte))
    return (t[i:], v[i:]) if i else (t, v)


# ======================================================================
# Lado da placa (processo filho)
# ======================================================================

def _novos(lista, ultimo):
    """Itens de uma lista ordenada de tempos depois de `ultimo`."""
    return lista[bisect.bisect_right(lista, ultimo):]


def _processo(porta, linhas, colunas, tem_total, conn):
    from dobot_tatil.constantes_tatil import (
        TOUCH_ROT180_GRIDS,
        taxel_frame_to_physical,
        taxel_index_to_physical,
    )
    from dobot_tatil.touch_source import TouchSensorSource

    n = linhas * colunas
    est = {'contagem': collections.Counter(), 'tam_adc': None, 'max_idx': -1, 'viu': set()}
    grav = {'f': None, 'e': None, 'frames': 0, 'eventos': 0, 'parcial': [0] * n}
    import threading
    trava = threading.Lock()

    # --- gravação (mesma leitura de linha do touch_source) ---
    def escrever_frame(t_us, adcs):
        volts = ','.join(f'{a * VREF / 4095.0:.4f}' for a in adcs)
        grav['f'].write(f'{t_us / 1e6:.6f},{volts}\n')
        grav['frames'] += 1

    def escrever_evento(t_us, tipo, taxel='', valor=''):
        t = f'{t_us / 1e6:.6f}' if t_us is not None else ''
        grav['e'].write(f'{t},{tipo},{taxel},{valor}\n')
        grav['eventos'] += 1

    def gravar_linha(ln):
        if ln.startswith('ADC'):
            partes = ln.split(',')
            vals = [int(v) for v in partes[1:-1]]
            if len(vals) != n:
                return                       # frame cortado: o touch_source também descarta
            t_us = int(partes[-1].replace('t=', '').strip())
            escrever_frame(t_us, taxel_frame_to_physical(vals, linhas, colunas))
        elif ln.startswith('DATA'):
            m = RE_IDX_ADC_T.search(ln)
            if m:
                idx = int(m.group(1))
                if 0 <= idx < n:
                    grav['parcial'][taxel_index_to_physical(idx, linhas, colunas)] = \
                        int(m.group(2))
                    if idx == n - 1:         # fim da varredura: frame completo
                        escrever_frame(int(m.group(3)), grav['parcial'])
        elif ln.startswith('CN_') or ln.startswith('POST'):
            tipo = ln.split(',', 1)[0]
            m, mi = RE_T.search(ln), RE_IFINAL.search(ln)
            escrever_evento(int(m.group(1)) if m else None, tipo, '', mi.group(1) if mi else '')
        elif ln.startswith('RA') or ln.startswith('SA'):
            m = RE_IDX_ADC_T.search(ln)
            if m and 0 <= int(m.group(1)) < n:
                escrever_evento(int(m.group(3)), ln[:2],
                                taxel_index_to_physical(int(m.group(1)), linhas, colunas),
                                m.group(2))
        elif ln.startswith('TOTAL'):
            m, mi = RE_T.search(ln), RE_IFINAL.search(ln)
            if mi:
                escrever_evento(int(m.group(1)) if m else None, 'TOTAL', '', mi.group(1))

    def tap(linhas_brutas):
        with trava:
            for ln in linhas_brutas:
                ln = ln.strip()
                if not ln:
                    continue
                tipo = ln.split(',', 1)[0]
                est['contagem'][tipo] += 1
                est['viu'].add(tipo)
                if tipo == 'ADC':
                    est['tam_adc'] = ln.count(',') - 1
                elif tipo == 'DATA':
                    m = RE_IDX_ADC_T.search(ln)
                    if m:
                        est['max_idx'] = max(est['max_idx'], int(m.group(1)))
                if grav['f'] is not None:
                    try:
                        gravar_linha(ln)
                    except (ValueError, OSError):
                        pass

    def fechar_gravacao():
        with trava:
            for k in ('f', 'e'):
                if grav[k] is not None:
                    grav[k].close()
                    grav[k] = None

    fonte = TouchSensorSource(port=porta, rows=linhas, cols=colunas, has_total=tem_total,
                              udp_broadcast=False, on_raw_lines=tap)
    ok = fonte.start()
    try:
        conn.send(('aberto', ok, fonte.error))
    except (OSError, BrokenPipeError):
        return
    if not ok:
        return

    girar = (linhas, colunas) not in TOUCH_ROT180_GRIDS
    ult = {'ra': [-math.inf] * n, 'sa': [-math.inf] * n, 'escalar': -math.inf}
    ult.update({c: -math.inf for c in CHAVES_EVENTOS})
    try:
        while True:
            t0 = time.monotonic()
            # comandos da janela
            while conn.poll():
                cmd = conn.recv()
                if cmd[0] == 'sair':
                    return
                if cmd[0] == 'gravar':
                    _, base, cab_f, cab_e = cmd
                    try:
                        ff = open(base + '_frames.csv', 'w', encoding='utf-8', newline='')
                        fe = open(base + '_eventos.csv', 'w', encoding='utf-8', newline='')
                        ff.write(cab_f)
                        fe.write(cab_e)
                    except OSError as e:
                        conn.send(('gravando', False, str(e)))
                        continue
                    with trava:
                        grav.update(f=ff, e=fe, frames=0, eventos=0, parcial=[0] * n)
                    conn.send(('gravando', True, ''))
                elif cmd[0] == 'parar_gravacao':
                    fechar_gravacao()
                    conn.send(('gravacao_fim', grav['frames'], grav['eventos']))

            snap = fonte.snapshot()
            pacote = {'volt': None}
            # último frame completo, na orientação da figura do cr10twin
            v = fonte.latest_voltages()
            pacote['volt'] = v if not girar else np.rot90(v, 2).copy()
            for chave in ('ra', 'sa'):
                ts, ks = [], []
                for k, lst in enumerate(snap[chave]):
                    novos = _novos(lst, ult[chave][k])
                    if novos:
                        ult[chave][k] = novos[-1]
                        ts.extend(novos)
                        ks.extend([k] * len(novos))
                ordem = np.argsort(ts, kind='stable') if ts else []
                pacote[chave] = (np.asarray(ts, dtype=float)[ordem] if ts else np.empty(0),
                                 np.asarray(ks, dtype=np.int16)[ordem] if ks else
                                 np.empty(0, dtype=np.int16))
            for c in CHAVES_EVENTOS:
                novos = _novos(snap[c], ult[c])
                if novos:
                    ult[c] = novos[-1]
                pacote[c] = np.asarray(novos, dtype=float)
            serie = snap['i_final']
            novos = serie[bisect.bisect_right(serie, (ult['escalar'], math.inf)):]
            if novos:
                ult['escalar'] = novos[-1][0]
                arr = np.asarray(novos, dtype=float)
                pacote['escalar'] = (arr[:, 0].copy(), arr[:, 1].copy())
            else:
                pacote['escalar'] = (np.empty(0), np.empty(0))
            with trava:
                contagem = dict(est['contagem'])
                est['contagem'].clear()
                estado = {'tam_adc': est['tam_adc'], 'max_idx': est['max_idx'],
                          'viu': list(est['viu'])}
            estado.update(connected=bool(fonte.connected), error=fonte.error,
                          frames_ok=fonte.frames_ok, frames_bad=fonte.frames_bad,
                          last_rx=fonte.last_rx, contagem=contagem,
                          grav_frames=grav['frames'], grav_eventos=grav['eventos'])
            pacote['estado'] = estado
            conn.send(('dados', pacote))
            espera = PERIODO_S - (time.monotonic() - t0)
            if espera > 0:
                time.sleep(espera)
    except (EOFError, OSError, BrokenPipeError):
        return                                  # a janela fechou
    finally:
        fechar_gravacao()
        fonte.stop()
