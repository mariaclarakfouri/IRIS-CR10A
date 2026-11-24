"""Segurança dos movimentos: nada sai do campo de trabalho CALCULADO.

O campo é o da aba Campo de Trabalho, "Campo calculado pela base"
(campo.CampoDaBase): paredes imaginárias verticais subindo do contorno dos pontos
da base, cortadas por até onde o braço chega com a ponta para baixo, sem nunca
passar do campo medido ponto a ponto. Ele é montado com os pontos gravados em
PONTOS_PATH (os mesmos da aba, que vêm de todos os computadores) e é montado de
novo sozinho sempre que esse arquivo muda.

Como cada movimento é conferido (VerificadorCampo.verificar):
  * o robô move as juntas com MovJ: todas as juntas andam juntas, em linha reta
    nos ângulos, da posição de partida até o destino. O caminho é dividido em
    passos de no máximo PASSO_GRAUS em cada junta;
  * em cada passo, a cinemática direta (o modelo do CR10A, o mesmo do Robô 3D)
    dá a posição do flange e, com uma ferramenta, a da ponta dela (no eixo do
    flange, `ferramenta_mm` à frente);
  * se o flange ou a ponta saem do campo em QUALQUER passo, o movimento é
    recusado inteiro, antes de ir para o robô;
  * com o robô já fora do campo (por exemplo, depois de mover com a mão), só
    vale um movimento que volte para dentro e não saia mais.

Os OBSTÁCULOS são conferidos com o robô INTEIRO: em cada passo, nenhuma peça
(braço, cotovelo, antebraço, punho, flange e a ferramenta) pode descer abaixo do
tampo da mesa, passar das paredes retas criadas no campo (PAREDES: computador e
lado esquerdo) nem subir acima do teto (TETO_MM). Paredes e teto ficam onde o
corpo do robô chegou na medição sem bater. A
altura do tampo sai do ponto "mesa": é a parte mais baixa do robô naquela posição
(o flange estava encostado no tampo). As peças são as malhas 3D do modelo; de cada
uma só o casco de fora (a envoltória convexa), que dá o mesmo ponto mais baixo.
A base e a junta 1 só giram em volta do eixo vertical e não descem: ficam de fora.
O tampo vale como um plano sem fim (o "chão imaginário" continua além da mesa).

O campo de trabalho (a borda medida) é conferido com o flange e a ponta da
ferramenta (ele foi medido com o flange); os obstáculos, com o robô inteiro.

Sem Qt: usado pelo servidor de movimento (dobot_control).
"""
import csv
import math
import os
import threading
import xml.etree.ElementTree as ET

import numpy as np

from .campo import Campo, CampoDaBase, eh_referencia
from .constantes import SETTINGS_PATH
from .robot3d_scene import parse_urdf

PONTOS_PATH = os.path.join(os.path.dirname(SETTINGS_PATH), 'pontos_campo.csv')
PASSO_GRAUS = 0.5          # o caminho é conferido a cada passo deste tamanho (por junta)
TOLERANCIA_MM = 1.0        # arredondamento: até isto além da borda ainda conta como dentro
COMPRIMENTO_CELULA_MM = 67.7   # célula de carga FA7155 (valor com a ponteira, por folga)
RAIO_CELULA_MM = 44.0          # raio da célula (para a mesa: a ferramenta é um cilindro)
# Obstáculos do laboratório para o robô INTEIRO: as duas paredes retas criadas
# no campo de trabalho e o teto. Ficam onde o CORPO do robô chegou na medição sem
# bater em nada (o ponto mais longe de qualquer peça, nos pontos medidos que
# ficam dentro do campo), não onde o centro do flange encostou: assim o robô
# inteiro nunca vai além de onde já esteve. Calculados em 23/11/2026.
# Parede: (nome, direção para fora (x, y) no referencial do robô, distância do
# eixo da junta 1 em mm).
PAREDES = (
    ('do computador', (-1.0, 0.0), 591.8),
    ('da esquerda', (0.0, -1.0), 724.6),
)
TETO_MM = 1370.7
# Peças que só giram em volta do eixo vertical (base, junta 1): não descem.
PECAS_FIXAS_NA_ALTURA = ('base_link', 'Link1', 'ring_j1', 'cap_j1')
NOME_DA_PECA = {
    'Link2': 'o braço', 'ring_j2': 'o braço', 'cap_j2': 'o braço',
    'ring_j3': 'o cotovelo', 'cap_j3': 'o cotovelo', 'Link3': 'o antebraço',
    'Link4': 'o punho', 'ring_j4': 'o punho', 'cap_j4': 'o punho',
    'Link5': 'o punho', 'ring_j5': 'o punho', 'cap_j5': 'o punho',
    'Link6': 'o flange', 'ring_j6': 'o flange', 'cap_j6': 'o flange',
    'ferramenta': 'a ferramenta',
}
# Mesma convenção do Robô 3D (robot_view.DOBOT_TO_URDF_SIGN): só a junta 1 tem
# o sinal invertido entre o modelo e os ângulos que o Dobot reporta.
DOBOT_TO_URDF_SIGN = np.array([-1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
JUNTAS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')


def caminho_urdf():
    """cr10.urdf do pacote dobot_gui (instalado ou na pasta do código)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        p = os.path.join(get_package_share_directory('dobot_gui'), 'robot3d', 'cr10.urdf')
        if os.path.isfile(p):
            return p
    except Exception:
        pass
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'robot3d', 'cr10.urdf')


class ModeloRobo:
    """O CR10A do URDF (o mesmo do Robô 3D): cinemática direta de TODAS as peças
    para muitas posições de uma vez, e o casco de cada peça (mm, no frame dela)."""

    def __init__(self, urdf_caminho):
        with open(urdf_caminho, encoding='utf-8') as f:
            texto = f.read()
        pasta = os.path.dirname(urdf_caminho)
        texto = texto.replace('filename="meshes/', f'filename="{pasta}/meshes/')
        cena = parse_urdf(texto, triangle_budget=None)
        # Juntas da base_link para baixo, na ordem (pai antes do filho).
        por_pai = {}
        for j in cena.joints:
            por_pai.setdefault(j.parent, []).append(j)
        self.ordem, fila = [], ['base_link']
        while fila:
            pai = fila.pop(0)
            for j in por_pai.get(pai, ()):
                eixo = np.asarray(j.axis, float)
                self.ordem.append((j.name, j.jtype, pai, j.child, j.origin,
                                   eixo / (np.linalg.norm(eixo) or 1.0)))
                fila.append(j.child)
        # Cascos (envoltória convexa) de cada peça que pode descer, em mm.
        from scipy.spatial import ConvexHull
        cascos = {}
        for parte in cena.parts:
            if parte.link in PECAS_FIXAS_NA_ALTURA:
                continue
            v = np.unique(np.round(parte.tris.reshape(-1, 3) * 1000.0, 2), axis=0)
            try:
                v = v[ConvexHull(v).vertices]
            except Exception:
                pass
            cascos.setdefault(parte.link, []).append(v)
        self.cascos = {k: np.vstack(v) for k, v in cascos.items()}

    def transformacoes(self, q_dobot_graus):
        """{link: (N,4,4)} (metros) para N posições (graus, convenção Dobot)."""
        q = np.atleast_2d(np.asarray(q_dobot_graus, float))
        rad = np.radians(q * DOBOT_TO_URDF_SIGN)
        n = q.shape[0]
        T = {'base_link': np.broadcast_to(np.eye(4), (n, 4, 4))}
        for nome, tipo, pai, filho, origem, eixo in self.ordem:
            M = T[pai] @ origem
            if tipo in ('revolute', 'continuous') and nome in JUNTAS:
                R = np.broadcast_to(np.eye(4), (n, 4, 4)).copy()
                R[:, :3, :3] = _rotacoes(eixo, rad[:, JUNTAS.index(nome)])
                M = M @ R
            T[filho] = M
        return T

    def flange(self, q_dobot_graus):
        """(N,6) graus → (posições do flange (N,3) mm, eixo z do flange (N,3))."""
        T = self.transformacoes(q_dobot_graus)['Link6']
        return T[:, :3, 3] * 1000.0, T[:, :3, 2]

    def cascos_com_ferramenta(self, ferramenta_mm=0.0, raio_ferramenta_mm=0.0):
        """Cascos das peças (mm, no frame de cada uma) e, com ferramenta, um
        cilindro no eixo do flange (peça 'ferramenta', presa ao Link6)."""
        cascos = dict(self.cascos)
        if ferramenta_mm > 0:
            ang = np.linspace(0, 2 * math.pi, 24, endpoint=False)
            aro = np.c_[raio_ferramenta_mm * np.cos(ang), raio_ferramenta_mm * np.sin(ang)]
            cascos['ferramenta'] = np.vstack([np.c_[aro, np.zeros(24)],
                                              np.c_[aro, np.full(24, ferramenta_mm)]])
        return cascos

    def pontos_no_mundo(self, q_dobot_graus, ferramenta_mm=0.0, raio_ferramenta_mm=0.0):
        """{peça: (N, M, 3) mm}: os pontos do casco de cada peça em cada posição."""
        T = self.transformacoes(q_dobot_graus)
        saida = {}
        for link, pts in self.cascos_com_ferramenta(ferramenta_mm, raio_ferramenta_mm).items():
            M = T['Link6' if link == 'ferramenta' else link]
            saida[link] = (np.einsum('nij,mj->nmi', M[:, :3, :3], pts)
                           + M[:, None, :3, 3] * 1000.0)
        return saida

    def mais_baixo(self, q_dobot_graus, ferramenta_mm=0.0, raio_ferramenta_mm=0.0):
        """Para cada posição: (altura do ponto mais baixo do robô (N,) mm, nome da
        peça (N,)). Com ferramenta: um cilindro no eixo do flange."""
        T = self.transformacoes(q_dobot_graus)
        n = next(iter(T.values())).shape[0]
        menor = np.full(n, np.inf)
        quem = np.full(n, '', dtype=object)
        cascos = self.cascos_com_ferramenta(ferramenta_mm, raio_ferramenta_mm)
        for link, pts in cascos.items():
            M = T['Link6' if link == 'ferramenta' else link]
            # Só a altura: linha z da rotação · ponto + z da posição.
            z = np.einsum('nj,mj->nm', M[:, 2, :3], pts) + M[:, 2, 3][:, None] * 1000.0
            zmin = z.min(axis=1)
            melhor = zmin < menor
            menor[melhor] = zmin[melhor]
            quem[melhor] = link
        return menor, quem


def _rotacoes(eixo, angulos):
    """Rodrigues para muitos ângulos em volta do mesmo eixo → (N,3,3)."""
    x, y, z = eixo
    K = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    s, c = np.sin(angulos)[:, None, None], np.cos(angulos)[:, None, None]
    return np.eye(3) + s * K + (1 - c) * (K @ K)


def ler_pontos(caminho=PONTOS_PATH):
    """[(nome, [6 graus])] do arquivo de pontos do Campo de Trabalho."""
    pontos = []
    with open(caminho, encoding='utf-8', newline='') as f:
        for linha in csv.DictReader(f):
            try:
                q = [float(linha[f'J{k}_graus']) for k in range(1, 7)]
            except (KeyError, TypeError, ValueError):
                continue
            pontos.append((linha.get('nome') or '', q))
    return pontos


class VerificadorCampo:
    """Confere movimentos contra o campo calculado. Seguro para várias threads."""

    def __init__(self, ferramenta_mm=0.0, arquivo=PONTOS_PATH, urdf=None):
        self.ferramenta_mm = max(0.0, float(ferramenta_mm))
        self.arquivo = arquivo
        self.fk = ModeloRobo(urdf) if urdf else modelo_robo()
        self._trava = threading.Lock()
        self._marca = None          # (mtime, tamanho) do arquivo do campo em uso
        self._campo = None
        self.mesa_z = None          # altura do tampo (mm), pelo ponto "mesa"
        self._erro = 'campo de trabalho ainda não montado'

    def campo(self):
        """O campo calculado em uso (montado de novo se o arquivo mudou), ou None.
        Devolve (campo, motivo se None)."""
        with self._trava:
            try:
                st = os.stat(self.arquivo)
                marca = (st.st_mtime_ns, st.st_size)
            except OSError:
                self._campo, self._marca = None, None
                self._erro = (f'não há pontos do campo de trabalho ({self.arquivo} não '
                              'existe)')
                return None, self._erro
            if marca != self._marca:
                self._marca = marca
                try:
                    (self._campo, self.mesa_z), self._erro = self._montar(), ''
                except (OSError, ValueError) as e:
                    self._campo, self.mesa_z = None, None
                    self._erro = f'não deu para montar o campo de trabalho ({e})'
            return self._campo, self._erro

    def _montar(self):
        pontos = ler_pontos(self.arquivo)
        borda = [q for nome, q in pontos if not eh_referencia(nome)]
        refs = [q for nome, q in pontos if eh_referencia(nome)]
        if len(borda) < 3:
            raise ValueError('menos de 3 pontos da base')
        b_mm = self.fk.flange(borda)[0]
        r_mm = self.fk.flange(refs)[0] if refs else np.zeros((0, 3))
        try:
            medido = Campo(b_mm, r_mm)
        except ValueError:
            medido = None
        campo = CampoDaBase(b_mm, r_mm, limite=medido)
        return campo, altura_da_mesa(self.fk, pontos)

    def caminho(self, inicio, alvo):
        """Posições (N,6) do MovJ de `inicio` até `alvo`, a cada PASSO_GRAUS."""
        a, b = np.asarray(inicio, float), np.asarray(alvo, float)
        n = max(2, int(math.ceil(np.abs(b - a).max() / PASSO_GRAUS)) + 1)
        t = np.linspace(0.0, 1.0, n)[:, None]
        return a + (b - a) * t

    def verificar(self, inicio, alvo):
        """(ok, motivo). `inicio` e `alvo`: 6 ângulos (graus, convenção Dobot)."""
        campo, erro = self.campo()
        if campo is None:
            return False, (f'NÃO É SEGURO: {erro}. Sem o campo de trabalho não dá para '
                           'conferir o movimento, então ele foi recusado.')
        qs = self.caminho(inicio, alvo)
        flange, eixo = self.fk.flange(qs)
        pecas = [('o flange', flange)]
        if self.ferramenta_mm > 0:
            pecas.append(('a ponta da ferramenta', flange + eixo * self.ferramenta_mm))
        # Quanto cada passo está fora (o pior entre flange e ponta) e de quem.
        fora = np.full(len(qs), -np.inf)
        quem = [''] * len(qs)
        for nome, pts in pecas:
            for i, p in enumerate(pts):
                d = _fora_mm(campo, p)
                if d > fora[i]:
                    fora[i], quem[i] = d, nome
        # O robô INTEIRO contra os obstáculos: a mesa, as paredes retas criadas
        # no campo e o teto. Cada ponto do casco de cada peça (e da ferramenta).
        raio_f = RAIO_CELULA_MM if self.ferramenta_mm > 0 else 0.0
        for link, W in self.fk.pontos_no_mundo(qs, self.ferramenta_mm, raio_f).items():
            peca = NOME_DA_PECA.get(link, 'o robô')
            limites = [(f'bateria na parede {nome}|além dela', W[..., :2] @ np.asarray(n) - d)
                       for nome, n, d in PAREDES]
            limites.append(('bateria no teto|acima dele', W[..., 2] - TETO_MM))
            if self.mesa_z is not None:
                limites.append(('bateria na mesa|abaixo do tampo', self.mesa_z - W[..., 2]))
            for oque, excesso in limites:                    # (N, M)
                dmax = excesso.max(axis=1)
                for i in np.flatnonzero(dmax > fora):
                    fora[i] = dmax[i]
                    quem[i] = f'obst:{peca}|{oque}'
        dentro = fora <= TOLERANCIA_MM
        if dentro.all():
            return True, ''
        fim = '. Movimento recusado; o robô não se mexeu.'
        if not dentro[-1]:
            return False, ('NÃO É SEGURO ir para esse ponto: no destino, '
                           + self._onde(campo, quem[-1], pecas, len(qs) - 1, fora) + fim)
        primeiro = 0
        if not dentro[0]:
            # Já está fora: vale só voltar para dentro e não sair mais.
            primeiro = int(np.argmax(dentro))
            if dentro[primeiro:].all():
                return True, ''
        sai = primeiro + int(np.argmin(dentro[primeiro:]))       # onde sai do campo
        pior = primeiro + int(np.argmax(fora[primeiro:]))        # onde fica mais longe
        pct = 100.0 * sai / (len(qs) - 1)
        return False, ('NÃO É SEGURO ir para esse ponto: no caminho até ele, a partir de '
                       f'{pct:.0f}% do movimento, '
                       + self._onde(campo, quem[pior], pecas, pior, fora)
                       + ' (o destino é seguro, mas o caminho do robô até ele não)' + fim)

    @staticmethod
    def _onde(campo, quem, pecas, i, fora):
        """Texto de onde a peça sai do campo (ou bate num obstáculo)."""
        if quem.startswith('obst:'):
            peca, oque, onde = quem[5:].split('|', 2)
            return f'{peca} {oque} ({fora[i]:.0f} mm {onde})'
        x, y, z = dict(pecas)[quem][i]
        if z < campo.chao_z - TOLERANCIA_MM:
            return f'{quem} ficaria {campo.chao_z - z:.0f} mm abaixo do chão (a mesa)'
        if z > campo.teto_z + TOLERANCIA_MM:
            return f'{quem} ficaria {z - campo.teto_z:.0f} mm acima do topo do campo'
        return (f'{quem} sairia do campo de trabalho ({_fora_mm(campo, (x, y, z)):.0f} mm '
                f'além da parede, a {z:.0f} mm de altura)')


def altura_da_mesa(modelo, pontos):
    """Altura do tampo (mm): a parte mais baixa do robô (sem ferramenta) no ponto
    "mesa". Sem esse ponto, nos pontos mais baixos gravados (a mediana dos que
    estão a até 15 mm do mais baixo). None sem pontos. `pontos`: [(nome, q)]."""
    mesa = [q for nome, q in pontos if _normalizar(nome) == 'mesa']
    if not mesa:
        if not pontos:
            return None
        qs = [q for _n, q in pontos]
        z = modelo.flange(qs)[0][:, 2]
        mesa = [q for q, zz in zip(qs, z) if zz <= z.min() + 15.0]
    return float(np.median(modelo.mais_baixo(mesa)[0]))


_MODELO = None


def modelo_robo():
    """Um ModeloRobo só por processo (carregar leva ~0,1 s)."""
    global _MODELO
    if _MODELO is None:
        _MODELO = ModeloRobo(caminho_urdf())
    return _MODELO


def _normalizar(nome):
    import unicodedata
    n = unicodedata.normalize('NFKD', (nome or '').strip().lower())
    return ''.join(c for c in n if not unicodedata.combining(c))


def _fora_mm(campo, p):
    """Quanto (mm) o ponto está fora do campo; 0 ou menos = dentro. Abaixo do
    chão ou acima do topo conta a distância na altura; entre os dois, a
    distância além da parede (na altura do ponto, levada para dentro do campo
    se estiver só um pouco abaixo/acima)."""
    x, y, z = p
    if z < campo.chao_z:
        abaixo = campo.chao_z - z
        if abaixo > TOLERANCIA_MM:
            return abaixo
    if z > campo.teto_z:
        acima = z - campo.teto_z
        if acima > TOLERANCIA_MM:
            return acima
    z = min(max(z, campo.chao_z), campo.teto_z)
    return math.hypot(x, y) - campo.raio(math.atan2(y, x), z)
