"""Base dos módulos da plataforma.

Cada módulo é uma página da interface (um item da barra lateral). Para criar um
módulo novo:

  1. crie um arquivo em `modulos/`, com uma classe que herda de `Modulo`;
  2. defina NOME, ICONE e DESCRICAO e monte a página em `construir()`;
  3. coloque a classe na lista MODULOS de `modulos/__init__.py`.

O módulo recebe a janela principal (`self.janela`), que oferece a base comum:
mover o robô (`mover_e_esperar`), ir para o home conferindo a
posição (`ir_para_home_e_conferir`), esperas que respeitam o botão Parar
(`esperar`, `parado`), registro (`log`), mensagens (`alerta`) e o painel de
progresso (`progresso`).
"""
from PyQt5.QtWidgets import QVBoxLayout, QWidget


class Modulo(QWidget):
    NOME = 'Módulo'
    ICONE = 'list'          # tipo de ícone de tema.make_icon
    DESCRICAO = ''
    CHAVE_HOME = None       # chave em homes.HOMES: a aba ganha o cartão da sua home
    TEXTO_HOME = None       # explicação opcional no cartão da home
    cartao_home = None

    def __init__(self, janela):
        super().__init__()
        self.janela = janela
        self.layout_modulo = QVBoxLayout(self)
        self.layout_modulo.setContentsMargins(0, 0, 0, 0)
        self.layout_modulo.setSpacing(16)
        self.construir()

    # --- a implementar pelos módulos ---

    def construir(self):
        """Monta os elementos da página em self.layout_modulo."""

    def widgets_de_movimento(self):
        """Controles que movem o robô: ficam travados enquanto o robô não
        está pronto (desconectado, em movimento, em emergência...)."""
        return []

    def carregar_config(self, dados):
        """Recebe o dicionário de configurações salvas."""

    def salvar_config(self, dados):
        """Acrescenta as configurações deste módulo ao dicionário."""

    def posicionar_cartao_home(self, cartao):
        """Onde o cartão da home entra na página (padrão: no alto)."""
        self.layout_modulo.insertWidget(0, cartao)

    def cartoes_extras_home(self):
        """Outros cartões de home que a aba montou (atualizados com os da base)."""
        return []

    def home_mudou(self, chave):
        """Chamado quando alguma home é gravada."""

    def ao_ler_juntas(self, juntas, robo_parado):
        """Chamado a cada leitura da posição do robô (graus)."""

    # --- comum ---

    def definir_movimento_habilitado(self, habilitado):
        for w in self.widgets_de_movimento():
            w.setEnabled(habilitado)
