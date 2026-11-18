"""Verificação geométrica de segurança ANTES de cada movimento.

Antes de mandar um MovJ, o servidor chama `Seguranca.verificar_caminho()` com a
posição atual e o destino. O caminho é percorrido como o MovJ do Dobot o faz —
interpolando as juntas juntas, do começo ao fim — em passos pequenos o bastante
para que nenhum ponto do braço ande mais que `passo_maximo_mm` entre duas
verificações. Em cada passo, cada parte do braço é testada contra:

  * os obstáculos do ambiente (mesa, chão, paredes, armários), e
  * as outras partes do próprio braço (autocolisão).

Se qualquer passo violar a margem, o movimento é RECUSADO, e a mensagem diz qual
parte chegaria perto demais de quê, e em que ponto do caminho.

Geometria: dos MESMOS arquivos do gêmeo virtual (pacote dobot_twin) — origens e
eixos das juntas do cr10_robot.xacro, forma de cada peça das malhas STL. Assim a
verificação e o Gazebo enxergam o mesmo robô.

Como a distância é medida (duas etapas):
  1. triagem: caixa orientada justa em volta de cada peça. Se as caixas (com a
     margem) não se tocam, as peças estão seguramente longe;
  2. confirmação: só quando as caixas se tocam, a distância é medida com pontos
     espalhados na SUPERFÍCIE real das malhas, a cada `espacamento_pontos_mm`.
     O erro dessa medida (no máximo o espaçamento) é somado à margem, então o
     resultado erra para o lado seguro.
Só caixas recusavam, por exemplo, o giro completo do J5, que no robô real passa
com ~31 mm de folga entre o flange e o antebraço.

Mesma ideia do collision.py do cr10twin (geometria dos links vs. obstáculos do
mundo), estendida para autocolisão e para o caminho inteiro.

Sem dependência de ROS: dá para testar e usar isoladamente.
"""
import math
import os
import struct
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.spatial import cKDTree

# Partes do braço, na ordem da cadeia. base_link fica parafusada na mesa.
LINKS = ['base_link', 'Link1', 'Link2', 'Link3', 'Link4', 'Link5', 'Link6']
FERRAMENTA = 'ferramenta'
MESA = 'mesa'                    # o obstáculo em que a ponteira pode encostar

# Pares que podem se encostar de verdade (não vizinhos na cadeia — vizinhos se
# tocam na própria junta). Punho/flange/ferramenta contra base, ombro, braço e
# antebraço (o caso de dobrar o punho para trás), e o antebraço contra a base.
PARES_AUTOCOLISAO = [
    ('Link4', 'base_link'), ('Link4', 'Link1'),
    ('Link5', 'base_link'), ('Link5', 'Link1'), ('Link5', 'Link2'),
    ('Link6', 'base_link'), ('Link6', 'Link1'), ('Link6', 'Link2'), ('Link6', 'Link3'),
    (FERRAMENTA, 'base_link'), (FERRAMENTA, 'Link1'), (FERRAMENTA, 'Link2'),
    (FERRAMENTA, 'Link3'), (FERRAMENTA, 'Link4'),
    ('Link3', 'base_link'),
]

DOBOT_TO_URDF_SIGN = [-1.0, 1.0, 1.0, 1.0, 1.0, 1.0]


# --------------------------------------------------------------------------
# Geometria básica
# --------------------------------------------------------------------------

def _rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _transform(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_angle(axis, angle):
    x, y, z = axis / np.linalg.norm(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    T = np.eye(4)
    T[:3, :3] = [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]
    return T


class Caixa:
    """Caixa orientada: centro (3,), eixos (3x3, colunas) e meias-medidas (3,)."""

    __slots__ = ('centro', 'eixos', 'meio')

    def __init__(self, centro, eixos, meio):
        self.centro = np.asarray(centro, dtype=float)
        self.eixos = np.asarray(eixos, dtype=float)
        self.meio = np.asarray(meio, dtype=float)


def caixas_se_tocam(a, b, margem):
    """Teste de eixo separador (15 eixos) entre duas caixas orientadas. True se
    a distância entre elas pode ser menor que `margem`.

    Duas etapas, ambas exatas (só aceleram, não mudam a resposta):
      1. esferas que envolvem as caixas: se elas estão a mais de `margem`, as
         caixas também estão — descarta a maioria dos pares de cara;
      2. os 15 eixos de uma vez, com numpy, em vez de um por um."""
    d = b.centro - a.centro
    alcance = math.sqrt(float(a.meio @ a.meio)) + math.sqrt(float(b.meio @ b.meio)) + margem
    if float(d @ d) > alcance * alcance:
        return False
    A, B = a.eixos, b.eixos
    # Eixos candidatos: 3 de A, 3 de B e os 9 produtos vetoriais (sem normalizar:
    # a margem é multiplicada pelo comprimento de cada eixo).
    cruz = np.cross(A.T[:, None, :], B.T[None, :, :]).reshape(9, 3)
    L = np.vstack((A.T, B.T, cruz))
    n = np.sqrt(np.einsum('ij,ij->i', L, L))
    validos = n > 1e-9
    L, n = L[validos], n[validos]
    separacao = np.abs(L @ d) - np.abs(L @ A) @ a.meio - np.abs(L @ B) @ b.meio
    return not bool(np.any(separacao > margem * n))


# Folga numérica do teste em lote: só pode marcar como "perto" um par a mais
# (que então passa pela conta fina), nunca deixar de marcar um que estava perto.
_EPS_LOTE = 1e-9


def caixas_se_tocam_lote(pares):
    """O mesmo teste de `caixas_se_tocam` para vários pares de uma vez.
    `pares`: lista de (caixa_a, caixa_b, margem). Devolve uma lista de bools.

    É a mesma conta (esferas e depois os 15 eixos separadores), só que em
    arrays com todos os pares juntos: o custo deixa de ser a sobrecarga do
    Python a cada par. Na dúvida numérica responde "toca" (lado seguro: o par
    vai para a medida fina, que é quem decide)."""
    if not pares:
        return []
    ca = np.array([a.centro for a, _, _ in pares])
    cb = np.array([b.centro for _, b, _ in pares])
    A = np.array([a.eixos for a, _, _ in pares])          # (P, 3, 3), eixos nas colunas
    B = np.array([b.eixos for _, b, _ in pares])
    ma = np.array([a.meio for a, _, _ in pares])
    mb = np.array([b.meio for _, b, _ in pares])
    margem = np.array([m for _, _, m in pares], dtype=float)
    d = cb - ca
    alcance = np.linalg.norm(ma, axis=1) + np.linalg.norm(mb, axis=1) + margem
    toca = np.einsum('pi,pi->p', d, d) <= alcance * alcance + _EPS_LOTE
    idx = np.flatnonzero(toca)
    if len(idx):
        At = np.transpose(A[idx], (0, 2, 1))              # linhas = eixos
        Bt = np.transpose(B[idx], (0, 2, 1))
        cruz = np.cross(At[:, :, None, :], Bt[:, None, :, :]).reshape(len(idx), 9, 3)
        L = np.concatenate((At, Bt, cruz), axis=1)        # (p, 15, 3)
        n = np.sqrt(np.einsum('pkj,pkj->pk', L, L))
        proj_d = np.abs(np.einsum('pkj,pj->pk', L, d[idx]))
        proj_a = np.einsum('pki,pi->pk', np.abs(np.einsum('pkj,pji->pki', L, A[idx])), ma[idx])
        proj_b = np.einsum('pki,pi->pk', np.abs(np.einsum('pkj,pji->pki', L, B[idx])), mb[idx])
        separacao = proj_d - proj_a - proj_b
        separa = (separacao > margem[idx, None] * n + _EPS_LOTE) & (n > 1e-9)
        toca[idx] = ~np.any(separa, axis=1)
    return toca.tolist()


def _ler_stl(path):
    """Triângulos (n, 3, 3) de um STL binário, no frame da peça."""
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack_from('<I', data, 80)[0]
    rec = np.frombuffer(data, dtype=np.dtype([
        ('normal', '<f4', 3), ('v', '<f4', (3, 3)), ('attr', '<u2')]),
        count=n, offset=84)
    return rec['v'].astype(float)


def _amostrar_superficie(tri, espacamento):
    """Pontos na superfície dos triângulos, ~1 a cada `espacamento` m (mais os
    vértices). Sorteio com semente fixa: o resultado é sempre o mesmo."""
    rng = np.random.default_rng(0)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    n_por_tri = np.ceil(area / (espacamento ** 2) * 2.0).astype(int)
    idx = np.repeat(np.arange(len(tri)), n_por_tri)
    r1 = np.sqrt(rng.random(len(idx)))
    r2 = rng.random(len(idx))
    p = ((1 - r1)[:, None] * a[idx] + (r1 * (1 - r2))[:, None] * b[idx]
         + (r1 * r2)[:, None] * c[idx])
    pts = np.vstack([tri.reshape(-1, 3), p])
    # Junta pontos muito próximos (grade de meio espaçamento).
    chave = np.round(pts / (espacamento / 2.0)).astype(np.int64)
    _, unicos = np.unique(chave, axis=0, return_index=True)
    return pts[np.sort(unicos)]


def _amostrar_caixa(meio, espacamento):
    """Pontos na superfície de uma caixa centrada na origem."""
    faces = []
    eixos = [np.linspace(-m, m, max(2, int(math.ceil(2 * m / espacamento)) + 1)) for m in meio]
    for k in range(3):
        i, j = [x for x in range(3) if x != k]
        U, V = np.meshgrid(eixos[i], eixos[j], indexing='ij')
        for sinal in (-1.0, 1.0):
            f = np.zeros((U.size, 3))
            f[:, i] = U.ravel()
            f[:, j] = V.ravel()
            f[:, k] = sinal * meio[k]
            faces.append(f)
    return np.vstack(faces)


# --------------------------------------------------------------------------
# Modelo do braço
# --------------------------------------------------------------------------

ESPACAMENTO_GROSSO = 0.020   # m — triagem intermediária antes dos pontos finos


def _reduzir(pontos, espacamento):
    """Um ponto por célula de uma grade de `espacamento`."""
    chave = np.floor(pontos / espacamento).astype(np.int64)
    _, idx = np.unique(chave, axis=0, return_index=True)
    return pontos[np.sort(idx)]


class Peca:
    """Uma parte do braço: caixa local, pontos de superfície (finos e grossos)
    e árvores de busca."""

    def __init__(self, nome, pontos):
        self.nome = nome
        lo, hi = pontos.min(axis=0), pontos.max(axis=0)
        self.centro_local = (lo + hi) / 2.0
        self.meio = (hi - lo) / 2.0
        self.pontos = pontos
        self.arvore = cKDTree(pontos)
        # Versão grossa: todo ponto fino está a menos de ESPACAMENTO_GROSSO*√3
        # de algum ponto grosso; esse é o erro dela.
        self.pontos_grossos = _reduzir(pontos, ESPACAMENTO_GROSSO)
        self.arvore_grossa = cKDTree(self.pontos_grossos)
        self.erro_grosso = ESPACAMENTO_GROSSO * math.sqrt(3.0)


class ModeloBraco:
    """Cinemática + forma das peças, lidas do xacro e das malhas do gêmeo."""

    def __init__(self, urdf_path, malhas_dir, espacamento):
        root = ET.parse(urdf_path).getroot()
        self._juntas = {}   # filho -> (pai, T_origem, eixo ou None, índice ou None)
        for j in root.findall('joint'):
            pai = j.find('parent').get('link')
            filho = j.find('child').get('link')
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            eixo = idx = None
            if j.get('type') in ('revolute', 'continuous'):
                eixo = np.array([float(v) for v in j.find('axis').get('xyz').split()])
                nome = j.get('name')
                if nome.startswith('joint') and nome[5:].isdigit():
                    idx = int(nome[5:]) - 1
            self._juntas[filho] = (pai, _transform(xyz, rpy), eixo, idx)

        self.pecas = {}
        for link in root.findall('link'):
            nome = link.get('name')
            if nome not in LINKS:
                continue
            mesh = link.find('collision/geometry/mesh')
            tri = _ler_stl(os.path.join(malhas_dir, os.path.basename(mesh.get('filename'))))
            self.pecas[nome] = Peca(nome, _amostrar_superficie(tri, espacamento))
        for nome in LINKS:
            if nome not in self.pecas:
                raise ValueError(f'modelo sem a peça {nome}')

    def transformadas(self, q_dobot_deg):
        """Pose (4x4) de cada peça no frame 'world' do URDF — a superfície onde o
        robô está parafusado, com origem no centro da base — e a posição do eixo
        de cada junta J1..J6 (para o cálculo do passo)."""
        q = [math.radians(s * float(v)) for s, v in zip(DOBOT_TO_URDF_SIGN, q_dobot_deg)]
        cache = {'world': np.eye(4)}
        origens = [None] * 6

        def pose(link):
            if link in cache:
                return cache[link]
            pai, T0, eixo, idx = self._juntas[link]
            T = pose(pai) @ T0
            if eixo is not None and idx is not None:
                origens[idx] = T[:3, 3].copy()
                T = T @ _axis_angle(eixo, q[idx])
            cache[link] = T
            return T

        Ts = {nome: pose(nome) for nome in LINKS}
        return Ts, origens


# --------------------------------------------------------------------------
# Verificação
# --------------------------------------------------------------------------

class Seguranca:

    def __init__(self, config, urdf_path, malhas_dir):
        geo = config['geometria']
        self.espacamento = float(geo.get('espacamento_pontos_mm', 5.0)) / 1000.0
        self.modelo = ModeloBraco(urdf_path, malhas_dir, self.espacamento)
        self.margem_ambiente = float(geo['margem_ambiente_mm']) / 1000.0
        self.margem_auto = float(geo['margem_autocolisao_mm']) / 1000.0
        self.passo_maximo = float(geo['passo_maximo_mm']) / 1000.0

        self.ferramenta = None
        # Encostar a ponteira na mesa (medição do plano da mesa): a ferramenta
        # SEM a ponteira é que é verificada contra a mesa, com esta margem.
        # Todo o resto (outras peças, outros obstáculos) continua igual.
        self.ferramenta_contato = None
        self.margem_contato = float(geo.get('margem_contato_mm', 5.0)) / 1000.0
        self.comprimento_ferramenta = 0.0     # flange -> ponta da ferramenta (m)
        ferr = geo.get('ferramenta') or {}
        if ferr.get('ativa'):
            meio = np.array(ferr['tamanho_mm'], dtype=float) / 2000.0
            centro = np.array(ferr['centro_mm'], dtype=float) / 1000.0
            pts = _amostrar_caixa(meio, self.espacamento) + centro
            self.ferramenta = Peca(FERRAMENTA, pts)
            self.comprimento_ferramenta = float(centro[2] + meio[2])
            ponteira = float(ferr.get('ponteira_mm') or 0.0) / 1000.0
            if 0.0 < ponteira < 2.0 * meio[2]:
                base, corte = centro[2] - meio[2], centro[2] + meio[2] - ponteira
                meio_c = np.array([meio[0], meio[1], (corte - base) / 2.0])
                centro_c = np.array([centro[0], centro[1], (corte + base) / 2.0])
                self.ferramenta_contato = Peca(
                    FERRAMENTA, _amostrar_caixa(meio_c, self.espacamento) + centro_c)

        self.obstaculos = []
        for nome, ob in (geo.get('obstaculos') or {}).items():
            c = np.array(ob['centro_m'], dtype=float)
            t = np.array(ob['tamanho_m'], dtype=float)
            R = _rpy_matrix(0.0, 0.0, math.radians(float(ob.get('giro_graus', 0.0))))
            # Peças que estão sempre encostadas neste obstáculo (ex.: a base
            # parafusada na mesa) não são testadas contra ele.
            ignora = set(ob.get('ignorar_pecas', []))
            self.obstaculos.append((nome, Caixa(c, R, t / 2.0), ignora))

    @classmethod
    def do_arquivo(cls, config_path, urdf_path, malhas_dir):
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        return cls(config, urdf_path, malhas_dir)

    def _pecas_na_pose(self, q_dobot_deg):
        Ts, origens = self.modelo.transformadas(q_dobot_deg)
        pecas = {nome: (self.modelo.pecas[nome], Ts[nome]) for nome in LINKS}
        if self.ferramenta is not None:
            pecas[FERRAMENTA] = (self.ferramenta, Ts['Link6'])
        return pecas, origens

    @staticmethod
    def _caixa(peca, T):
        return Caixa(T[:3, :3] @ peca.centro_local + T[:3, 3], T[:3, :3], peca.meio)

    @staticmethod
    def _dist_pontos_caixa(pontos, T, obst):
        pts = pontos @ T[:3, :3].T + T[:3, 3]
        local = (pts - obst.centro) @ obst.eixos
        d = np.abs(local) - obst.meio
        fora = np.linalg.norm(np.maximum(d, 0.0), axis=1)
        dentro = np.minimum(np.max(d, axis=1), 0.0)
        return float(np.min(fora + dentro))

    def _distancia_a_obstaculo(self, peca, T, obst, limite):
        """Menor distância (m) entre a superfície da peça e a caixa obstáculo
        (negativa se a peça entra nela). Se a triagem grossa já mostra que está
        longe (> limite), devolve esse valor sem olhar os pontos finos."""
        d = self._dist_pontos_caixa(peca.pontos_grossos, T, obst)
        if d - peca.erro_grosso >= limite:
            return d
        return self._dist_pontos_caixa(peca.pontos, T, obst)

    def _distancia_entre_pecas(self, pa, Ta, pb, Tb, limite):
        """Menor distância (m) entre as superfícies de duas peças (até `limite`)."""
        # Pontos de A no frame de B.
        R = Tb[:3, :3].T @ Ta[:3, :3]
        t = Tb[:3, :3].T @ (Ta[:3, 3] - Tb[:3, 3])
        erro = pa.erro_grosso + pb.erro_grosso
        d, _ = pb.arvore_grossa.query(pa.pontos_grossos @ R.T + t,
                                      distance_upper_bound=limite + erro)
        if float(np.min(d)) - erro >= limite:
            return float(np.min(d))
        d, _ = pb.arvore.query(pa.pontos @ R.T + t, distance_upper_bound=limite)
        return float(np.min(d))

    def verificar_pose(self, q_dobot_deg, contato_mesa=False):
        """(True, '') se a pose respeita as margens; senão (False, motivo).
        `contato_mesa`: a ponteira pode encostar na mesa (ver margem_contato)."""
        pecas, _ = self._pecas_na_pose(q_dobot_deg)
        # O erro da medida por pontos vai para o lado seguro.
        folga_extra = self.espacamento
        if contato_mesa and self.ferramenta_contato is None:
            return False, ('só a ferramenta com ponteira (a célula, padrão do IRIS) pode '
                           'encostar na mesa')
        # Todos os pares (peça × obstáculo e peça × peça) passam primeiro pelo
        # teste rápido de caixas, de uma vez; depois, NA MESMA ORDEM de sempre,
        # só os que podem estar perto vão para a medida fina, que decide.
        caixas = {nome: self._caixa(peca, T) for nome, (peca, T) in pecas.items()}
        ambiente = []
        for nome_p, (peca_p, T) in pecas.items():
            for nome_o, obst, ignora in self.obstaculos:
                if nome_p in ignora:
                    continue
                peca, limite = peca_p, self.margem_ambiente + folga_extra
                if contato_mesa and nome_p == FERRAMENTA and nome_o == MESA:
                    peca, limite = self.ferramenta_contato, self.margem_contato + folga_extra
                caixa = caixas[nome_p] if peca is peca_p else self._caixa(peca, T)
                ambiente.append((nome_p, nome_o, peca, T, obst, limite, caixa))
        auto = []
        for a, b in PARES_AUTOCOLISAO:
            if a in pecas and b in pecas:
                auto.append((a, b, self.margem_auto + folga_extra))
        perto = caixas_se_tocam_lote(
            [(caixa, obst, limite) for _, _, _, _, obst, limite, caixa in ambiente]
            + [(caixas[a], caixas[b], limite) for a, b, limite in auto])
        for (nome_p, nome_o, peca, T, obst, limite, _), p in zip(ambiente, perto):
            if not p:
                continue
            d = self._distancia_a_obstaculo(peca, T, obst, limite)
            if d < limite:
                return False, f'{_texto_peca(nome_p)} ficaria {_mm(d)} {_de(nome_o)}'
        for (a, b, limite), p in zip(auto, perto[len(ambiente):]):
            if not p:
                continue
            (pa, Ta), (pb, Tb) = pecas[a], pecas[b]
            d = self._distancia_entre_pecas(pa, Ta, pb, Tb, limite)
            if d < limite:
                return False, (f'{_texto_peca(a)} ficaria {_mm(d)} {_de(b)} '
                               '(o braço bateria nele mesmo)')
        return True, ''

    def _alcance_memo(self, q_dobot_deg):
        """_alcance_por_junta com as últimas poses guardadas: num percurso, o fim
        de um trecho é o começo do seguinte e a conta seria refeita."""
        chave = tuple(float(v) for v in q_dobot_deg)
        memo = self.__dict__.setdefault('_memo_alcance', {})
        r = memo.get(chave)
        if r is None:
            if len(memo) > 64:
                memo.clear()
            r = memo[chave] = self._alcance_por_junta(q_dobot_deg)
        return r

    def _alcance_por_junta(self, q_dobot_deg):
        """Para cada junta, a maior distância entre o eixo dela e qualquer ponto
        do braço que ela move (na pose dada) — quanto a ponta anda por grau."""
        pecas, origens = self._pecas_na_pose(q_dobot_deg)
        ordem = LINKS[1:] + ([FERRAMENTA] if FERRAMENTA in pecas else [])
        alcance = []
        for i in range(6):
            movidas = [p for p in ordem if p == FERRAMENTA or LINKS.index(p) >= i + 1]
            r = 0.0
            for nome in movidas:
                peca, T = pecas[nome]
                cantos = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                                   for sz in (-1, 1)]) * peca.meio + peca.centro_local
                cantos = cantos @ T[:3, :3].T + T[:3, 3]
                r = max(r, float(np.max(np.linalg.norm(cantos - origens[i], axis=1))))
            alcance.append(r)
        return np.array(alcance)

    def verificar_caminho(self, q_de_deg, q_para_deg, contato_mesa=False):
        """Verifica o caminho do MovJ (interpolação de juntas) do começo ao fim.
        Devolve (True, '') ou (False, mensagem explicando onde e por quê)."""
        q0 = np.asarray(q_de_deg, dtype=float)
        q1 = np.asarray(q_para_deg, dtype=float)
        # O destino primeiro: se ele já é perigoso, a recusa sai na hora, sem
        # percorrer o caminho todo.
        ok, motivo = self.verificar_pose(q1, contato_mesa)
        if not ok:
            pose = ', '.join(f'J{k + 1} {v:.0f}°' for k, v in enumerate(q1))
            return False, f'{motivo} no destino ({pose}).'
        dq = np.radians(np.abs(q1 - q0))
        # Limite de quanto um ponto anda num passo: soma, junta a junta, de
        # ângulo x maior raio (com 1,5x de folga para a mudança de pose no
        # meio do caminho).
        r = np.maximum(self._alcance_memo(q0), self._alcance_memo(q1)) * 1.5
        deslocamento = float(np.sum(dq * r))
        n = max(1, int(math.ceil(deslocamento / self.passo_maximo)))
        # O destino (i == n) já foi conferido acima, com o mesmo `contato_mesa`.
        for i in range(1, n):
            q = q0 + (q1 - q0) * (i / n)
            ok, motivo = self.verificar_pose(q, contato_mesa)
            if not ok:
                onde = ('no destino' if i == n else
                        f'no caminho, a {100.0 * i / n:.0f}% do movimento')
                pose = ', '.join(f'J{k + 1} {v:.0f}°' for k, v in enumerate(q))
                return False, f'{motivo} {onde} ({pose}).'
        return True, ''


def _mm(d):
    return 'a 0 mm (encostando ou atravessando)' if d <= 0 else f'a {d * 1000.0:.0f} mm'


def _de(nome):
    texto = {
        'mesa': 'da mesa', 'chao': 'do chão', 'parede_norte': 'da parede norte',
        'parede_oeste': 'da parede oeste', 'armario_norte': 'do armário norte',
        'armario_oeste': 'do armário oeste',
        'base_link': 'da base', 'Link1': 'do ombro (Link1)', 'Link2': 'do braço (Link2)',
        'Link3': 'do antebraço (Link3)', 'Link4': 'do punho (Link4)',
    }
    return texto.get(nome, f'de {nome}')


def _texto_peca(nome):
    return {
        'base_link': 'a base', 'Link1': 'o ombro (Link1)', 'Link2': 'o braço (Link2)',
        'Link3': 'o antebraço (Link3)', 'Link4': 'o punho (Link4)',
        'Link5': 'o punho (Link5)', 'Link6': 'o flange (Link6)',
        FERRAMENTA: 'a ferramenta/sensores na ponta',
    }.get(nome, nome)
