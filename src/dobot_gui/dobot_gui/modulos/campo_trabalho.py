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
errado. Este módulo não move o robô.
"""
import collections
import csv
import datetime
import os
import shutil
import time

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QFileDialog,
    QHBoxLayout,
    QLineEdit,
    QPushButton,
    QTableWidgetItem,
)

from ..constantes import JOINT_NAMES, SETTINGS_PATH
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
               + ['forca_ponteira_N', 'Fx_N', 'Fy_N', 'Fz_N'])
COLUNAS_TELA = ['Nº', 'Nome', 'Tipo'] + JOINT_NAMES + ['Ferramenta', 'Ponteira (N)', 'Hora']
COL_NOME = 1


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

    def construir(self):
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
        self.layout_modulo.addWidget(self._cartao_pontos(), 1)
        self._carregar_pontos()

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

    def _cartao_pontos(self):
        c = Cartao('Pontos gravados', f'Salvos em {PONTOS_PATH} (abre em planilha). Para mudar '
                                      'o nome de um ponto, clique duas vezes no nome.')
        self.tab = _tabela(COLUNAS_TELA, 0)
        self.tab.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tab.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tab.setEditTriggers(QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        self.tab.setFocusPolicy(Qt.StrongFocus)
        self.tab.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.tab.setMinimumHeight(260)
        self.tab.itemSelectionChanged.connect(self._atualizar_botoes)
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
        antigo = ponto['nome']
        ponto['nome'] = novo
        if self._salvar_pontos():
            self.janela.log(f'Ponto "{antigo}" renomeado para "{novo}".')
        else:
            ponto['nome'] = antigo
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
                                     f'Tirar os {len(self.pontos)} pontos da lista?\nO arquivo '
                                     'atual fica guardado ao lado, com a data no nome.',
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
                               + ['' if v is None else v for v in forca])
            os.replace(temporario, caminho)
            return True
        except OSError as e:
            self.janela.log(f'Não foi possível salvar os pontos em {caminho}: {e}', 'ERRO')
            self.janela.alerta.mostrar('Não deu para salvar os pontos (veja o Registro).', 'aviso')
            return False

    def _salvar_pontos(self):
        return self._escrever_csv(PONTOS_PATH)

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
        self._atualizar_botoes()
