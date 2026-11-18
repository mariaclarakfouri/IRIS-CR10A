"""Módulo Registro: histórico do que foi feito e dos erros."""
import datetime
import html

from PyQt5.QtWidgets import QComboBox, QFileDialog, QHBoxLayout, QPlainTextEdit, QPushButton

from ..constantes import LOG_PATH
from ..widgets import Cartao, rotulo
from .base import Modulo

CORES_NIVEL = {'ERRO': '#b91c1c', 'AVISO': '#b45309'}
FILTROS = (
    ('Tudo', ('INFO', 'AVISO', 'ERRO')),
    ('Avisos e erros', ('AVISO', 'ERRO')),
    ('Só erros', ('ERRO',)),
)
MAX_LINHAS = 2000


class Registro(Modulo):
    NOME = 'Registro'
    ICONE = 'list'
    DESCRICAO = ('Histórico do que foi feito e dos erros, com detalhes técnicos. '
                 f'Também fica gravado em {LOG_PATH}.')

    def construir(self):
        self._linhas = []   # (texto, nível, apagada)
        cartao = Cartao()
        barra = QHBoxLayout()
        barra.setSpacing(10)
        barra.addWidget(rotulo('Mostrar'))
        self.filtro = QComboBox()
        for nome, _ in FILTROS:
            self.filtro.addItem(nome)
        self.filtro.currentIndexChanged.connect(self._redesenhar)
        barra.addWidget(self.filtro)
        barra.addStretch()
        salvar = QPushButton('Salvar como…')
        salvar.setObjectName('discreto')
        salvar.clicked.connect(self._salvar_como)
        limpar = QPushButton('Limpar tela')
        limpar.setObjectName('discreto')
        limpar.setToolTip('Só limpa a tela; o arquivo do registro continua igual.')
        limpar.clicked.connect(self._limpar)
        barra.addWidget(salvar)
        barra.addWidget(limpar)
        cartao.corpo.addLayout(barra)

        self.log_view = QPlainTextEdit()
        self.log_view.setObjectName('logView')
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(MAX_LINHAS)
        self.log_view.setMinimumHeight(420)
        cartao.corpo.addWidget(self.log_view, 1)
        self.layout_modulo.addWidget(cartao, 1)

    # --- usado pela janela ---

    def adicionar(self, texto, nivel='INFO', apagada=False):
        self._linhas.append((texto, nivel, apagada))
        del self._linhas[:-MAX_LINHAS]
        if nivel in FILTROS[self.filtro.currentIndex()][1]:
            self._escrever(texto, nivel, apagada)

    # --- interno ---

    def _escrever(self, texto, nivel, apagada):
        cor = '#94a3b8' if apagada else CORES_NIVEL.get(nivel, '#0f172a')
        self.log_view.appendHtml(f'<span style="color:{cor}">{html.escape(texto)}</span>')

    def _redesenhar(self):
        self.log_view.clear()
        niveis = FILTROS[self.filtro.currentIndex()][1]
        for texto, nivel, apagada in self._linhas:
            if nivel in niveis:
                self._escrever(texto, nivel, apagada)

    def _limpar(self):
        self._linhas.clear()
        self.log_view.clear()

    def _salvar_como(self):
        sugerido = f'registro-{datetime.date.today().isoformat()}.txt'
        caminho, _ = QFileDialog.getSaveFileName(self, 'Salvar registro', sugerido,
                                                 'Texto (*.txt)')
        if not caminho:
            return
        try:
            with open(caminho, 'w', encoding='utf-8') as f:
                f.write(self.log_view.toPlainText() + '\n')
        except OSError as e:
            self.janela.log(f'Não foi possível salvar o registro: {e}', 'ERRO')
            return
        self.janela.log(f'Registro salvo em {caminho}.')
