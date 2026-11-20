"""Módulo Ferramentas: o que está na ponta do robô e as medidas de cada ferramenta.

A ferramenta escolhida aqui ("na ponta agora") é a que o IRIS usa nas contas:
o Campo de Trabalho grava com que ferramenta cada ponto foi medido, e o cálculo
do campo vai usar as medidas dela para o robô parar antes (uma ferramenta de
15 cm faz o flange parar 15 cm mais longe da mesa e das paredes), sem medir os
pontos de novo.

Medidas (no eixo do flange: z para fora do robô, x e y na face do flange):
  comprimento   do flange até a ponta da ferramenta (o TCP), em mm;
  desvio x, y   onde fica a ponta, se não estiver no centro do flange;
  raio          o maior raio da ferramenta em volta do eixo (para não bater de lado);
  massa         em kg, e o centro de massa (z): para torque e para a detecção de
                colisão da Dobot, que precisa saber o peso na ponta.

"Nada (flange)" e a célula de carga já vêm cadastradas e não mudam: as medidas
da célula são as do modelo do gêmeo (cr10_robot_com_celula.xacro). Para uma
variação delas, use "Duplicar".
"""
import copy

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLineEdit,
    QPushButton,
    QTableWidgetItem,
)

from ..tema import COR
from ..widgets import Cartao, com_icone, rotulo
from .base import Modulo
from .tabelas import _fmt, _tabela

NADA = 'Nada (flange)'
CELULA = 'Célula de carga FA7155'

# Do cr10_robot_com_celula.xacro: flange fixo 6 mm + célula 36,8 mm + acoplador
# 12,5 mm + ponteira 18 mm = face de contato a 67,7 mm do flange; massas 51,8 +
# 300 + 27,5 + 4,3 g; centro de massa do conjunto a 19,5 mm; corpo ⌀75 mm e o
# conector do cabo saindo até 44 mm do eixo. A ponteira não é usada, mas as
# medidas com ela ficam: dão folga de segurança.
FIXAS = [
    {'nome': NADA, 'comprimento_mm': 0.0, 'desvio_x_mm': 0.0, 'desvio_y_mm': 0.0,
     'raio_mm': 0.0, 'massa_kg': 0.0, 'cm_z_mm': 0.0,
     'nota': 'Sem ferramenta: o flange do robô é a ponta.'},
    {'nome': CELULA, 'comprimento_mm': 67.7, 'desvio_x_mm': 0.0, 'desvio_y_mm': 0.0,
     'raio_mm': 44.0, 'massa_kg': 0.384, 'cm_z_mm': 19.5,
     'nota': 'Medidas do modelo do gêmeo com a ponteira, mantidas de propósito para dar '
             'folga de segurança. Raio de 44 mm com o conector do cabo (corpo ⌀75).'},
]
NOMES_FIXOS = {f['nome'] for f in FIXAS}
# O que o comando iris abriu (parâmetro "ponta" da interface) → ferramenta.
DA_PONTA = {'celula': CELULA, 'nada': NADA}

# (chave, rótulo, mínimo, máximo, casas, sufixo)
CAMPOS = (
    ('comprimento_mm', 'Comprimento', 0, 1000, 1, ' mm'),
    ('desvio_x_mm', 'Desvio x', -500, 500, 1, ' mm'),
    ('desvio_y_mm', 'Desvio y', -500, 500, 1, ' mm'),
    ('raio_mm', 'Raio', 0, 500, 1, ' mm'),
    ('massa_kg', 'Massa', 0, 10, 3, ' kg'),       # carga máxima do CR10A: 10 kg
    ('cm_z_mm', 'Centro de massa (z)', 0, 1000, 1, ' mm'),
)
COLUNAS_TELA = ['Nome', 'Comprimento', 'Desvio x', 'Desvio y', 'Raio', 'Massa', 'CM (z)']


def _resumo(f):
    partes = [f'comprimento {_fmt(f["comprimento_mm"])} mm']
    if f['desvio_x_mm'] or f['desvio_y_mm']:
        partes.append(f'desvio ({_fmt(f["desvio_x_mm"])}, {_fmt(f["desvio_y_mm"])}) mm')
    partes += [f'raio {_fmt(f["raio_mm"])} mm', f'{_fmt(f["massa_kg"], 3)} kg']
    return ' · '.join(partes)


class Ferramentas(Modulo):
    NOME = 'Ferramentas'
    ICONE = 'tool'
    DESCRICAO = ('Qual ferramenta está na ponta do robô e as medidas de cada uma. O IRIS usa a '
                 'ferramenta escolhida para calcular até onde o robô pode ir, sem medir os '
                 'pontos de novo.')

    def construir(self):
        self.ferramentas = copy.deepcopy(FIXAS)
        ponta = str(getattr(self.janela.ros_node, 'ponta', 'nada') or 'nada')
        self.ponta_aberta = DA_PONTA.get(ponta, NADA)
        self.ativa_nome = self.ponta_aberta
        self._editando = None          # nome da ferramenta no formulário (None = nova)

        self.layout_modulo.addWidget(self._cartao_ativa())
        self.layout_modulo.addWidget(self._cartao_lista(), 1)
        self.layout_modulo.addWidget(self._cartao_formulario())
        self._redesenhar()
        self._fechar_formulario()

    # ------------------------------------------------------------------
    # API para a janela e os outros módulos
    # ------------------------------------------------------------------

    def ativa(self):
        """A ferramenta na ponta agora (cópia do dicionário)."""
        for f in self.ferramentas:
            if f['nome'] == self.ativa_nome:
                return dict(f)
        return dict(FIXAS[0])

    # ------------------------------------------------------------------
    # Tela
    # ------------------------------------------------------------------

    def _cartao_ativa(self):
        c = Cartao('Na ponta agora', 'Escolha a ferramenta que está montada no robô. Ela vale '
                                     'para o IRIS inteiro e vai gravada em cada ponto do Campo '
                                     'de Trabalho.')
        linha = QHBoxLayout()
        linha.setSpacing(12)
        self.ativa_combo = QComboBox()
        self.ativa_combo.setMinimumWidth(280)
        self.ativa_combo.activated.connect(self._escolher_ativa)
        linha.addWidget(self.ativa_combo)
        self.ativa_resumo = rotulo('', 'valorMono')
        linha.addWidget(self.ativa_resumo, 1)
        c.corpo.addLayout(linha)
        self.ativa_aviso = rotulo('', 'cartaoTexto')
        self.ativa_aviso.setWordWrap(True)
        self.ativa_aviso.setStyleSheet(f'color: {COR["aviso_texto"]};')
        c.corpo.addWidget(self.ativa_aviso)
        return c

    def _cartao_lista(self):
        c = Cartao('Ferramentas cadastradas',
                   'Medidas no eixo do flange: comprimento do flange até a ponta, desvio da '
                   'ponta para o lado, maior raio em volta do eixo, massa e centro de massa.')
        self.tab = _tabela(COLUNAS_TELA, 0)
        self.tab.setSelectionMode(QAbstractItemView.SingleSelection)
        self.tab.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.tab.setFocusPolicy(Qt.StrongFocus)
        self.tab.setMinimumHeight(170)
        self.tab.itemSelectionChanged.connect(self._ao_selecionar)
        self.tab.itemDoubleClicked.connect(lambda _i: self._editar())
        c.corpo.addWidget(self.tab, 1)
        self.nota_txt = rotulo('', 'cartaoTexto')
        self.nota_txt.setWordWrap(True)
        self.nota_txt.setStyleSheet(f'color: {COR["texto_fraco"]};')
        c.corpo.addWidget(self.nota_txt)

        botoes = QHBoxLayout()
        botoes.setSpacing(10)
        self.nova_btn = com_icone(QPushButton('Nova ferramenta'), 'tool', COR['texto'])
        self.nova_btn.setObjectName('secundario')
        self.nova_btn.clicked.connect(self._nova)
        self.duplicar_btn = QPushButton('Duplicar')
        self.duplicar_btn.setObjectName('discreto')
        self.duplicar_btn.clicked.connect(self._duplicar)
        self.editar_btn = QPushButton('Editar')
        self.editar_btn.setObjectName('discreto')
        self.editar_btn.clicked.connect(self._editar)
        self.apagar_btn = QPushButton('Apagar')
        self.apagar_btn.setObjectName('discreto')
        self.apagar_btn.clicked.connect(self._apagar)
        for b in (self.nova_btn, self.duplicar_btn, self.editar_btn, self.apagar_btn):
            botoes.addWidget(b)
        botoes.addStretch(1)
        c.corpo.addLayout(botoes)
        return c

    def _cartao_formulario(self):
        self.form = Cartao()
        self.form_titulo = rotulo('Ferramenta', 'cartaoTitulo')
        self.form.corpo.addWidget(self.form_titulo)
        grade = QGridLayout()
        grade.setHorizontalSpacing(12)
        grade.setVerticalSpacing(8)
        grade.addWidget(rotulo('Nome'), 0, 0)
        self.nome_edit = QLineEdit()
        self.nome_edit.setMaxLength(60)
        self.nome_edit.setPlaceholderText('ex.: Garra pneumática')
        grade.addWidget(self.nome_edit, 0, 1, 1, 6)
        self.spins = {}
        for i, (chave, texto, mini, maxi, casas, sufixo) in enumerate(CAMPOS):
            s = QDoubleSpinBox()
            s.setRange(mini, maxi)
            s.setDecimals(casas)
            s.setSingleStep(1.0 if casas == 1 else 0.01)
            s.setSuffix(sufixo)
            s.setKeyboardTracking(False)
            s.setMinimumWidth(120)
            linha, col = 1 + i // 3, (i % 3) * 2
            grade.addWidget(rotulo(texto), linha, col)
            grade.addWidget(s, linha, col + 1)
            self.spins[chave] = s
        grade.addWidget(rotulo('Nota'), 3, 0)
        self.nota_edit = QLineEdit()
        self.nota_edit.setMaxLength(160)
        self.nota_edit.setPlaceholderText('opcional, ex.: medido com paquímetro')
        grade.addWidget(self.nota_edit, 3, 1, 1, 6)
        grade.setColumnStretch(6, 1)
        self.form.corpo.addLayout(grade)
        botoes = QHBoxLayout()
        botoes.setSpacing(10)
        self.salvar_btn = com_icone(QPushButton('Salvar ferramenta'), 'save')
        self.salvar_btn.clicked.connect(self._salvar_formulario)
        cancelar = QPushButton('Cancelar')
        cancelar.setObjectName('discreto')
        cancelar.clicked.connect(self._fechar_formulario)
        botoes.addWidget(self.salvar_btn)
        botoes.addWidget(cancelar)
        botoes.addStretch(1)
        self.form.corpo.addLayout(botoes)
        return self.form

    # ------------------------------------------------------------------
    # Estado da tela
    # ------------------------------------------------------------------

    def _selecionada(self):
        linhas = {i.row() for i in self.tab.selectedItems()}
        if not linhas:
            return None
        return self.ferramentas[min(linhas)]

    def _redesenhar(self, selecionar=None):
        anterior = selecionar or (self._selecionada() or {}).get('nome')
        self.tab.blockSignals(True)
        self.tab.setRowCount(len(self.ferramentas))
        for i, f in enumerate(self.ferramentas):
            nome = f['nome'] + ('  (fixa)' if f['nome'] in NOMES_FIXOS else '')
            valores = [nome, f'{_fmt(f["comprimento_mm"])} mm', f'{_fmt(f["desvio_x_mm"])} mm',
                       f'{_fmt(f["desvio_y_mm"])} mm', f'{_fmt(f["raio_mm"])} mm',
                       f'{_fmt(f["massa_kg"], 3)} kg', f'{_fmt(f["cm_z_mm"])} mm']
            for c, v in enumerate(valores):
                self.tab.setItem(i, c, QTableWidgetItem(v))
            if f['nome'] == anterior:
                self.tab.selectRow(i)
        self.tab.blockSignals(False)

        self.ativa_combo.blockSignals(True)
        self.ativa_combo.clear()
        for f in self.ferramentas:
            self.ativa_combo.addItem(f['nome'])
        self.ativa_combo.setCurrentText(self.ativa_nome)
        self.ativa_combo.blockSignals(False)
        ativa = self.ativa()
        self.ativa_resumo.setText(_resumo(ativa))
        self._atualizar_aviso()
        self._ao_selecionar()

    def _atualizar_aviso(self):
        """Avisa quando a escolha não bate com o jeito que o iris foi aberto
        (a célula é a única ferramenta que liga coisas: o nó dela e o desenho)."""
        if self.ponta_aberta == CELULA and self.ativa_nome != CELULA:
            txt = ('O IRIS foi aberto com a célula na ponta (com-celula). Se a célula saiu '
                   'do robô, abra sem o "com-celula".')
        elif self.ponta_aberta != CELULA and self.ativa_nome == CELULA:
            txt = ('O IRIS foi aberto sem a célula: a aba Célula de Carga e o desenho da '
                   'célula no Robô 3D estão desligados. Para usar a célula, abra com '
                   '"com-celula" (iris com-celula ou iris gemeo com-celula).')
        else:
            txt = ''
        self.ativa_aviso.setText(txt)
        self.ativa_aviso.setVisible(bool(txt))

    def _ao_selecionar(self):
        f = self._selecionada()
        fixa = f is not None and f['nome'] in NOMES_FIXOS
        self.duplicar_btn.setEnabled(f is not None)
        self.editar_btn.setEnabled(f is not None and not fixa)
        self.apagar_btn.setEnabled(f is not None and not fixa)
        if f is None:
            self.nota_txt.setText('')
        else:
            extra = ('  Fixa: não dá para editar nem apagar; use "Duplicar" para uma variação.'
                     if fixa else '')
            self.nota_txt.setText((f.get('nota') or '') + extra)

    # ------------------------------------------------------------------
    # Ações
    # ------------------------------------------------------------------

    def _escolher_ativa(self, indice):
        nome = self.ativa_combo.itemText(indice)
        if nome == self.ativa_nome:
            return
        self.ativa_nome = nome
        self.janela.salvar_config()
        self.janela.log(f'Ferramenta na ponta: {nome} ({_resumo(self.ativa())}).')
        self.janela.alerta.mostrar(f'Ferramenta na ponta: {nome}.', 'ok')
        self._redesenhar()

    def _abrir_formulario(self, dados, editando, titulo):
        self._editando = editando
        self.form_titulo.setText(titulo)
        self.nome_edit.setText(dados.get('nome', ''))
        for chave, s in self.spins.items():
            s.setValue(float(dados.get(chave, 0.0)))
        self.nota_edit.setText(dados.get('nota', ''))
        self.salvar_btn.setText('Salvar alterações' if editando else 'Salvar ferramenta')
        self.form.show()
        self.nome_edit.setFocus()

    def _fechar_formulario(self):
        self._editando = None
        self.form.hide()

    def _nova(self):
        self._abrir_formulario({}, None, 'Nova ferramenta')

    def _duplicar(self):
        f = self._selecionada()
        if f is None:
            return
        dados = dict(f)
        dados['nome'] = self._nome_livre(f['nome'] + ' (cópia)')
        dados['nota'] = ''
        self._abrir_formulario(dados, None, 'Nova ferramenta')

    def _editar(self):
        f = self._selecionada()
        if f is None or f['nome'] in NOMES_FIXOS:
            return
        self._abrir_formulario(f, f['nome'], 'Editar ferramenta')

    def _nome_livre(self, base):
        nomes = {f['nome'] for f in self.ferramentas}
        nome, n = base, 2
        while nome in nomes:
            nome = f'{base} {n}'
            n += 1
        return nome

    def _salvar_formulario(self):
        nome = self.nome_edit.text().strip()
        if not nome:
            self.janela.alerta.mostrar('Dê um nome para a ferramenta.', 'aviso')
            return
        if nome in NOMES_FIXOS or (nome != self._editando
                                   and any(f['nome'] == nome for f in self.ferramentas)):
            self.janela.alerta.mostrar(f'Já existe uma ferramenta chamada "{nome}".', 'aviso')
            return
        nova = {'nome': nome, 'nota': self.nota_edit.text().strip()}
        for chave, s in self.spins.items():
            nova[chave] = round(s.value(), 3)
        if self._editando is None:
            self.ferramentas.append(nova)
            texto = f'Ferramenta "{nome}" cadastrada'
        else:
            i = next(i for i, f in enumerate(self.ferramentas) if f['nome'] == self._editando)
            self.ferramentas[i] = nova
            if self.ativa_nome == self._editando:
                self.ativa_nome = nome
            texto = f'Ferramenta "{nome}" alterada'
        self.janela.salvar_config()
        self.janela.log(f'{texto}: {_resumo(nova)}.')
        self.janela.alerta.mostrar(texto + '.', 'ok')
        self._fechar_formulario()
        self._redesenhar(selecionar=nome)

    def _apagar(self):
        f = self._selecionada()
        if f is None or f['nome'] in NOMES_FIXOS:
            return
        if not self.janela.confirmar('Apagar ferramenta', f'Apagar a ferramenta "{f["nome"]}"?',
                                     'Apagar'):
            return
        self.ferramentas.remove(f)
        if self.ativa_nome == f['nome']:
            self.ativa_nome = self.ponta_aberta
        self.janela.salvar_config()
        self.janela.log(f'Ferramenta "{f["nome"]}" apagada.')
        self._fechar_formulario()
        self._redesenhar()

    # ------------------------------------------------------------------
    # Configurações
    # ------------------------------------------------------------------

    def carregar_config(self, dados):
        self.ferramentas = copy.deepcopy(FIXAS)
        for f in dados.get('ferramentas') or []:
            try:
                nome = str(f['nome']).strip()
                if not nome or nome in NOMES_FIXOS or any(
                        x['nome'] == nome for x in self.ferramentas):
                    continue
                nova = {'nome': nome, 'nota': str(f.get('nota', ''))}
                for chave, _t, mini, maxi, _c, _s in CAMPOS:
                    nova[chave] = min(max(float(f.get(chave, 0.0)), mini), maxi)
            except (KeyError, TypeError, ValueError):
                continue
            self.ferramentas.append(nova)
        # A ferramenta salva só vale se combina com o jeito que o iris foi aberto:
        # aberto com a célula (com-celula), é a célula; sem ela, qualquer outra.
        salva = dados.get('ferramenta_ativa')
        nomes = {f['nome'] for f in self.ferramentas}
        if self.ponta_aberta == CELULA:
            self.ativa_nome = CELULA
        elif salva in nomes and salva != CELULA:
            self.ativa_nome = salva
        else:
            self.ativa_nome = NADA
        self._redesenhar()

    def salvar_config(self, dados):
        dados['ferramentas'] = [f for f in self.ferramentas if f['nome'] not in NOMES_FIXOS]
        dados['ferramenta_ativa'] = self.ativa_nome
