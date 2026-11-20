"""Sincronia dos pontos do Campo de Trabalho entre os computadores, pelo GitHub.

Os pontos ficam num ramo SÓ DELES no repositório do projeto (RAMO), com um único
arquivo (ARQUIVO_NO_RAMO). O código nunca entra nesse ramo e esse ramo nunca
entra no código: o envio automático não leva nada além dos pontos, não mexe na
pasta do projeto nem atrapalha o git pull / iris salvar.

Cada sincronização:
  1. busca o ramo no GitHub (git fetch);
  2. junta três versões dos pontos: a deste computador, a do GitHub e a da última
     sincronização (a "base"). Pela base dá para saber o que cada lado mudou:
       * ponto novo em qualquer lado entra;
       * ponto apagado num lado (e não mudado no outro) sai dos dois;
       * o mesmo ponto mudado nos dois lados: vale a mudança mais recente;
  3. grava o resultado aqui e, se o GitHub estiver diferente, envia (um commit em
     cima do ramo do GitHub, feito com o nome do git deste computador).
Sem internet ou sem login, os pontos continuam salvos aqui e vão na próxima vez.

Cada ponto tem um `id` que não muda e a hora da última mudança (`modificado_em`).
Pontos antigos, de antes do id, ganham um id pela data e hora em que foram
gravados: o mesmo ponto nos dois computadores ganha o mesmo id.
"""
import csv
import io
import os
import subprocess
import threading

RAMO = 'pontos-campo'
ARQUIVO_NO_RAMO = 'pontos_campo.csv'
TEMPO_MAXIMO_S = 30


def projeto_git():
    """Pasta do projeto (a que tem o .git), a partir de onde este arquivo está
    instalado (install/ fica dentro do projeto). None se não achar."""
    p = os.path.dirname(os.path.abspath(__file__))
    while p != os.path.dirname(p):
        if os.path.isdir(os.path.join(p, '.git')):
            return p
        p = os.path.dirname(p)
    alternativa = os.environ.get('IRIS_DIR') or os.path.expanduser('~/iris')
    return alternativa if os.path.isdir(os.path.join(alternativa, '.git')) else None


def id_do_ponto(linha):
    """id estável: o que já está na linha, ou um pela data/hora em que foi gravado."""
    if linha.get('id'):
        return linha['id']
    return 'p' + ''.join(c for c in (linha.get('data_hora') or '') if c.isdigit()) + \
        '-' + (linha.get('fonte') or '') + '-' + (linha.get('numero') or '')


def ler_csv(texto):
    """Texto do CSV → (colunas, {id: linha})."""
    if not texto:
        return [], {}
    leitor = csv.DictReader(io.StringIO(texto))
    linhas = {}
    for linha in leitor:
        linha = {k: (v or '') for k, v in linha.items() if k is not None}
        linha['id'] = id_do_ponto(linha)
        linhas[linha['id']] = linha
    return list(leitor.fieldnames or []), linhas


def escrever_csv(colunas, linhas):
    """{id: linha} → texto do CSV, em ordem de gravação e numerado de novo."""
    saida = io.StringIO()
    w = csv.DictWriter(saida, fieldnames=colunas, extrasaction='ignore', lineterminator='\n')
    w.writeheader()
    ordem = sorted(linhas.values(), key=lambda l: (l.get('data_hora', ''), l['id']))
    for i, linha in enumerate(ordem, 1):
        linha = dict(linha)
        linha['numero'] = str(i)
        w.writerow(linha)
    return saida.getvalue()


def juntar(base, local, remoto):
    """Junta três versões ({id: linha}); base None = nunca sincronizou (só soma)."""
    resultado = {}
    for i in set(local) | set(remoto):
        a, b = local.get(i), remoto.get(i)
        antes = base.get(i) if base is not None else None
        if a is not None and b is not None:
            if a == b:
                resultado[i] = a
            elif antes is not None and a == antes:
                resultado[i] = b                      # só o GitHub mudou
            elif antes is not None and b == antes:
                resultado[i] = a                      # só aqui mudou
            else:                                     # os dois mudaram: o mais recente
                resultado[i] = a if a.get('modificado_em', '') >= b.get('modificado_em', '') else b
        else:
            existe = a if a is not None else b
            if antes is None:
                resultado[i] = existe                 # ponto novo
            elif existe != antes:
                resultado[i] = existe                 # apagado de um lado, mudado no outro
            # senão: apagado de um lado e igual no outro → sai
    return resultado


def _explicar(err, acao):
    """Erro do git em português (o que dá para fazer a respeito)."""
    e = err.lower()
    if 'rejected' in e or 'fetch first' in e or 'non-fast-forward' in e:
        return 'rejected: o GitHub mudou'
    if 'could not resolve host' in e or 'unable to access' in e or 'network' in e:
        return 'sem internet (ou o GitHub fora do ar)'
    if ('authentication' in e or 'username' in e or 'password' in e or 'permission denied' in e
            or 'terminal prompts disabled' in e or '403' in e):
        return ('o GitHub pediu login e este computador não tem o login salvo (faça um git push '
                'uma vez no terminal, na pasta do projeto, para salvar)')
    linhas = [l for l in err.splitlines() if l.strip()]
    principal = next((l for l in linhas if l.lower().startswith(('fatal', 'error'))),
                     linhas[0] if linhas else '')
    return f'não deu para {acao} no GitHub ({principal or "erro do git"})'


class Sincronia:
    """Uma por interface. `sincronizar()` roda numa thread (git demora)."""

    def __init__(self, arquivo_local, arquivo_base=None, projeto=None):
        self.arquivo_local = arquivo_local
        self.arquivo_base = arquivo_base or arquivo_local.replace('.csv', '-sincronizado.csv')
        self.projeto = projeto if projeto is not None else projeto_git()
        self._trava = threading.Lock()
        # Gravar o arquivo local: a interface e a sincronia nunca ao mesmo tempo
        # (a interface também usa esta trava; ela é segurada só durante a gravação).
        self.trava_arquivo = threading.Lock()

    # --- git ---

    def _git(self, *args, entrada=None):
        env = dict(os.environ, GIT_TERMINAL_PROMPT='0', GIT_ASKPASS='true',
                   SSH_ASKPASS='true', LC_ALL='C')
        r = subprocess.run(['git', '-C', self.projeto, *args], input=entrada,
                           capture_output=True, text=True, timeout=TEMPO_MAXIMO_S, env=env)
        return r.returncode, r.stdout, r.stderr.strip()

    def _remoto(self):
        """(commit, texto) do ramo no GitHub; (None, '') se o ramo ainda não existe."""
        cod, _o, err = self._git('fetch', '--quiet', 'origin', f'+refs/heads/{RAMO}:refs/remotes/origin/{RAMO}')
        if cod != 0:
            if "couldn't find remote ref" in err or 'not found' in err.lower():
                return None, ''
            raise RuntimeError(_explicar(err, 'buscar'))
        cod, commit, _ = self._git('rev-parse', f'refs/remotes/origin/{RAMO}')
        if cod != 0:
            return None, ''
        cod, texto, _ = self._git('show', f'refs/remotes/origin/{RAMO}:{ARQUIVO_NO_RAMO}')
        return commit.strip(), (texto if cod == 0 else '')

    def _enviar(self, texto, pai, mensagem):
        cod, blob, err = self._git('hash-object', '-w', '--stdin', entrada=texto)
        if cod != 0:
            raise RuntimeError(err)
        cod, arvore, err = self._git('mktree', entrada=f'100644 blob {blob.strip()}\t{ARQUIVO_NO_RAMO}\n')
        if cod != 0:
            raise RuntimeError(err)
        args = ['commit-tree', arvore.strip(), '-m', mensagem]
        if pai:
            args += ['-p', pai]
        cod, commit, err = self._git(*args)
        if cod != 0:
            raise RuntimeError(err)
        cod, _o, err = self._git('push', '--quiet', 'origin', f'{commit.strip()}:refs/heads/{RAMO}')
        if cod != 0:
            raise RuntimeError(_explicar(err, 'enviar'))

    # --- arquivos ---

    @staticmethod
    def _ler(caminho):
        try:
            with open(caminho, encoding='utf-8') as f:
                return f.read()
        except FileNotFoundError:
            return None

    @staticmethod
    def _gravar(caminho, texto):
        os.makedirs(os.path.dirname(caminho), exist_ok=True)
        tmp = caminho + '.tmp'
        with open(tmp, 'w', encoding='utf-8', newline='') as f:
            f.write(texto)
        os.replace(tmp, caminho)

    # --- a sincronização ---

    def sincronizar(self, mensagem='pontos do campo de trabalho'):
        """Devolve (ok, texto, mudou_aqui). Nunca levanta exceção."""
        if self.projeto is None:
            return False, 'pasta do projeto (git) não encontrada', False
        with self._trava:
            try:
                for _tentativa in range(3):
                    pai, texto_remoto = self._remoto()
                    texto_local = self._ler(self.arquivo_local) or ''
                    texto_base = self._ler(self.arquivo_base)
                    col_l, local = ler_csv(texto_local)
                    col_r, remoto = ler_csv(texto_remoto)
                    _c, base = ler_csv(texto_base) if texto_base is not None else ([], None)
                    if not texto_local.strip() and base:
                        # Arquivo daqui sumiu ou está vazio (nem o cabeçalho): NÃO é "apagaram
                        # tudo" (o Começar de novo grava o cabeçalho). Vale a última sincronizada.
                        col_l, local = _c, dict(base)
                    colunas = list(col_l) + [c for c in col_r if c not in col_l]
                    for c in ('id', 'modificado_em'):
                        if c not in colunas:
                            colunas.append(c)
                    texto_novo = escrever_csv(colunas, juntar(base, local, remoto))
                    mudou_aqui = texto_novo != texto_local
                    if mudou_aqui:
                        with self.trava_arquivo:
                            if (self._ler(self.arquivo_local) or '') != texto_local:
                                continue      # gravaram um ponto enquanto buscava: junta de novo
                            self._gravar(self.arquivo_local, texto_novo)
                    if texto_novo != texto_remoto:
                        try:
                            self._enviar(texto_novo, pai, mensagem)
                        except RuntimeError as e:
                            if 'rejected' in str(e) or 'fetch first' in str(e):
                                continue          # outro computador enviou antes: junta de novo
                            raise
                    self._gravar(self.arquivo_base, texto_novo)
                    n = len(ler_csv(texto_novo)[1])
                    return True, f'{n} pontos iguais aqui e no GitHub', mudou_aqui
                return False, 'o GitHub mudou durante o envio; tenta de novo na próxima', False
            except (RuntimeError, OSError, subprocess.SubprocessError) as e:
                return False, str(e) or e.__class__.__name__, False
