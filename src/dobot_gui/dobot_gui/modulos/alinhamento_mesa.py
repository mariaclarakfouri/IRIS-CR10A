"""Módulo Alinhamento da Mesa: mede o plano da mesa com a ponteira da célula.

Como na calibração do ângulo de ataque do cr10twin (etapa de MEDIÇÃO; a
correção do punho não está aqui): a ponteira encosta na mesa num ponto central
e em N pontos num círculo em volta, a célula de carga percebe cada toque, e o
plano ajustado aos toques diz o quanto a mesa está inclinada em relação ao
robô. A conta do plano é a do cr10twin, sem mudanças (plane_probe.py).

Cada toque:
  1. desce rápido (parando se encostar) até perto da mesa: 8 mm acima do tampo
     do modelo de segurança no 1º toque; depois, uma margem acima do toque mais
     alto já medido;
  2. desce devagar até a força mudar `limiar` N (o servidor para na hora);
  3. sobe de volta à altura de partida e vai, na mesma altura, ao próximo ponto.

Os movimentos são do servidor (action mover_ponta): ponteira em linha reta,
orientação fixa, segurança verificada em todos os trechos (a ponteira, e só
ela, pode chegar à mesa) e parada ao sentir o contato.
"""
import datetime
import math
import os
import threading
import time

import numpy as np
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import (
    QDoubleSpinBox,
    QGridLayout,
    QPushButton,
    QSpinBox,
    QTableWidgetItem,
)

from ..constantes import REGISTROS_DIR
from ..plane_probe import (
    fit_plane,
    probe_pattern,
    sagitta_m,
    slope_along_deg,
    validate_fit,
)
from ..tema import COR
from ..widgets import Cartao, com_icone, rotulo
from .tabelas import _ajustar_altura, _fmt, _tabela
from .base import Modulo

FERRAMENTA_CELULA = 'celula_fa7155'
PASTA_MESA = os.path.join(REGISTROS_DIR, 'alinhamento', 'medicao_da_mesa')   # gêmeo e real juntos
RAIO_MM = (10, 80, 30)
PONTOS = (3, 8, 4)
LIMIAR_N = (0.3, 5.0, 1.0)
VEL_LENTA_MMS = (0.2, 2.0, 0.5)
VEL_RAPIDA_MMS = 4.0            # descendo até perto da mesa (parando se encostar)
VEL_LIVRE_MMS = 10.0            # subindo e indo de um ponto ao outro
Z_RAPIDO_1_MM = 8.0             # 1º toque: rápido até 8 mm acima do tampo do modelo
MARGEM_RAPIDO_MM = 3.0          # depois: margem acima do toque mais alto...
INCLINACAO_PREVISTA_GRAUS = 10.0    # ...mais o que essa inclinação daria no raio
# O servidor não deixa a ponteira passar de profundidade_contato_mm (6 mm) abaixo
# do tampo da mesa do modelo (seguranca.yaml).
PROFUNDIDADE_MM = 5.9
RECUO_MM = 3.0                  # encostou na descida rápida: recua e desce devagar
ALTURA_INICIAL_MM = (15.0, 150.0)


class AlinhamentoMesa(Modulo):
    NOME = 'Alinhamento da Mesa'
    CHAVE_HOME = 'alinhamento_mesa'
    ICONE = 'nivel'
    DESCRICAO = ('Mede a inclinação da mesa em relação ao robô: a ponteira da célula de '
                 'carga encosta em vários pontos e o plano dos toques mostra o desvio.')

    resultado_pronto = pyqtSignal(object)

    def construir(self):
        self.resultado_pronto.connect(self._mostrar_resultado)
        lay = self.layout_modulo
        self.aviso = rotulo('', 'cartaoTexto')
        self.aviso.setWordWrap(True)
        lay.addWidget(self.aviso)

        grade = QGridLayout()
        grade.setHorizontalSpacing(16)
        grade.setVerticalSpacing(16)
        grade.setColumnStretch(0, 1)
        grade.setColumnStretch(1, 1)

        c_med = Cartao('Medição',
                       'Deixe a ponteira apontando para baixo, de 20 a 80 mm acima do ponto da '
                       'mesa que quer medir (pelo Controle de Juntas), e clique em Medir. O robô '
                       'encosta no centro e nos pontos do círculo, um de cada vez, e volta.')
        g = QGridLayout()
        g.setHorizontalSpacing(12)
        g.setVerticalSpacing(10)
        g.setColumnStretch(1, 1)
        self.raio = QSpinBox()
        self.raio.setRange(RAIO_MM[0], RAIO_MM[1])
        self.raio.setSuffix(' mm')
        self.raio.setValue(RAIO_MM[2])
        self.raio.setToolTip('Quanto maior, mais precisa a inclinação (a conta tem mais braço '
                             'de alavanca); o círculo inteiro tem de caber na mesa.')
        self.pontos = QSpinBox()
        self.pontos.setRange(PONTOS[0], PONTOS[1])
        self.pontos.setValue(PONTOS[2])
        self.limiar = QDoubleSpinBox()
        self.limiar.setRange(LIMIAR_N[0], LIMIAR_N[1])
        self.limiar.setSingleStep(0.1)
        self.limiar.setDecimals(1)
        self.limiar.setSuffix(' N')
        self.limiar.setValue(LIMIAR_N[2])
        self.limiar.setToolTip('O toque é reconhecido quando a força muda isto em relação ao '
                               'começo da descida.')
        self.vel_lenta = QDoubleSpinBox()
        self.vel_lenta.setRange(VEL_LENTA_MMS[0], VEL_LENTA_MMS[1])
        self.vel_lenta.setSingleStep(0.1)
        self.vel_lenta.setDecimals(1)
        self.vel_lenta.setSuffix(' mm/s')
        self.vel_lenta.setValue(VEL_LENTA_MMS[2])
        self.vel_lenta.setToolTip('Velocidade dos últimos milímetros até encostar. Mais devagar '
                                  '= toque mais leve e mais preciso.')
        for linha, (texto, campo) in enumerate((('Raio do círculo', self.raio),
                                                ('Pontos no círculo', self.pontos),
                                                ('Força que conta como toque', self.limiar),
                                                ('Velocidade ao encostar', self.vel_lenta))):
            g.addWidget(rotulo(texto), linha, 0)
            g.addWidget(campo, linha, 1)
        c_med.corpo.addLayout(g)
        self.resumo = rotulo('', 'cartaoTexto')
        self.resumo.setWordWrap(True)
        c_med.corpo.addWidget(self.resumo)
        self.medir_btn = com_icone(QPushButton('Medir a mesa'), 'play')
        self.medir_btn.setEnabled(False)
        self.medir_btn.clicked.connect(self.on_medir_clicked)
        c_med.corpo.addWidget(self.medir_btn)
        c_med.corpo.addStretch()     # o cartão estica com o resultado ao lado; o conteúdo fica em cima
        grade.addWidget(c_med, 0, 0)

        c_res = Cartao('Resultado', 'A inclinação é a do plano dos toques do círculo; o '
                                    'toque do centro diz se a mesa é plana ali.')
        self.res_txt = rotulo('Nenhuma medição ainda.', 'cartaoTexto')
        self.res_txt.setWordWrap(True)
        c_res.corpo.addWidget(self.res_txt)
        self.tab = _tabela(['Ponto', 'x (mm)', 'y (mm)', 'z (mm)', 'Força (N)'], 0)
        c_res.corpo.addWidget(self.tab)
        c_res.corpo.addStretch()
        grade.addWidget(c_res, 0, 1)
        lay.addLayout(grade)

        pasta = PASTA_MESA.replace(os.path.expanduser('~'), '~', 1)
        nota = rotulo('Cada medição fica em '
                      f'{pasta}/mesa_*.csv (os toques e o resultado). As posições são do frame '
                      'do robô: origem no centro da base, na superfície em que ele está '
                      'parafusado; x para a frente da base, z para cima.', 'cartaoTexto')
        nota.setWordWrap(True)
        lay.addWidget(nota)
        lay.addStretch()

        for w in (self.raio, self.pontos, self.limiar, self.vel_lenta):
            w.valueChanged.connect(self._atualizar_resumo)
        self._atualizar_resumo()

    # --- base ---

    def _com_celula(self):
        return getattr(self.janela.ros_node, 'ferramenta', '') == FERRAMENTA_CELULA

    def widgets_de_movimento(self):
        return [self.medir_btn, self.raio, self.pontos, self.limiar, self.vel_lenta]

    def definir_movimento_habilitado(self, habilitado):
        super().definir_movimento_habilitado(habilitado)
        if not self._com_celula():
            self.medir_btn.setEnabled(False)

    def carregar_config(self, dados):
        salvos = dados.get('mesa', {})
        for chave, campo in (('raio', self.raio), ('pontos', self.pontos),
                             ('limiar', self.limiar), ('vel_lenta', self.vel_lenta)):
            v = salvos.get(chave)
            if isinstance(v, (int, float)) and not isinstance(v, bool):
                campo.setValue(v)

    def salvar_config(self, dados):
        dados['mesa'] = {'raio': self.raio.value(), 'pontos': self.pontos.value(),
                         'limiar': self.limiar.value(), 'vel_lenta': self.vel_lenta.value()}

    def _atualizar_resumo(self, *_):
        if not self._com_celula():
            self.aviso.setStyleSheet(f'color: {COR["aviso_texto"]}; font-weight: 700;')
            self.aviso.setText('A medição usa a célula de carga para sentir o toque: abra a '
                               'plataforma com ela (iris, ou iris gemeo para testar '
                               'no gêmeo; sem-nada e com-sensor abrem sem ela).')
            self.aviso.show()
        else:
            self.aviso.hide()
        n = self.pontos.value()
        self.resumo.setText(
            f'{n + 1} toques: 1 no centro e {n} num círculo de {self.raio.value()} mm de raio. '
            f'Cada toque leva perto de meio minuto. A ponteira não desce mais que '
            f'{PROFUNDIDADE_MM:.0f} mm abaixo do tampo da mesa do modelo de segurança.')

    # --- medição ---

    def on_medir_clicked(self):
        cfg = {'raio': float(self.raio.value()), 'n': self.pontos.value(),
               'limiar': float(self.limiar.value()), 'v_lenta': float(self.vel_lenta.value()),
               'origem': ('gêmeo virtual' if getattr(self.janela, 'sim_mode', False) is True
                          else 'robô real')}
        texto = (f'A ponteira vai encostar na mesa {cfg["n"] + 1} vezes: no ponto logo abaixo dela '
                 f'e em {cfg["n"]} pontos num círculo de {cfg["raio"]:.0f} mm em volta, '
                 f'descendo devagar ({cfg["v_lenta"]:.1f} mm/s) e parando ao sentir '
                 f'{cfg["limiar"]:.1f} N. Entre os toques ela sobe de volta à altura em que está '
                 'agora. Confira que a ponteira aponta para baixo, que está de 20 a 80 mm acima '
                 'da mesa e que o círculo inteiro cabe na mesa, sem nada no caminho.')
        if not self.janela.confirmar('Medir a mesa', texto, 'Medir'):
            return
        self.janela.iniciar_acao('Medindo a mesa',
                                 f'Medição da mesa iniciada: centro + {cfg["n"]} pontos, raio '
                                 f'{cfg["raio"]:.0f} mm, toque a {cfg["limiar"]:.1f} N.')
        threading.Thread(target=self._executar, args=(cfg,), daemon=True).start()

    def _executar(self, cfg):
        try:
            self._executar_corpo(cfg)
        finally:
            self.janela.definir_prefixo(None)

    def _mover(self, **kw):
        ok, msg, res = self.janela.mover_ponta_e_esperar(**kw)
        if ok and res is None:
            return False, 'sem resultado do servidor', None
        return ok, msg, res

    def _executar_corpo(self, cfg):
        j = self.janela
        offsets = probe_pattern(cfg['n'], cfg['raio'] / 1000.0, with_center=True) * 1000.0
        total = len(offsets)
        margem = MARGEM_RAPIDO_MM + cfg['raio'] * math.tan(math.radians(INCLINACAO_PREVISTA_GRAUS))
        toques = []
        z_ref = None
        anterior = np.zeros(2)
        inicio = datetime.datetime.now()
        j.progresso.emit({'cronometro': (time.monotonic(), None), 'fracao': 0.0})

        def falhar(texto):
            j.definir_prefixo(None)
            j.terminar_acao(False, texto)

        for k, off in enumerate(offsets):
            nome = 'centro' if k == 0 else f'ponto {k} do círculo'
            j.definir_prefixo(f'toque {k + 1} de {total} ({nome})')
            j.progresso.emit({'fracao': k / total})
            if j.parado():
                return falhar('Movimento parado pelo usuário.')
            if k > 0:
                d = off - anterior
                ok, msg, _ = self._mover(deslocamento_mm=[d[0], d[1], 0.0],
                                         velocidade_mms=VEL_LIVRE_MMS)
                if not ok:
                    return falhar(f'indo para o {nome}: {msg}')
            # 1) rápido até perto da mesa (parando se encostar)
            z_rapido = Z_RAPIDO_1_MM if not toques else max(t['z'] for t in toques) + margem
            if z_ref is None or z_rapido < z_ref:
                ok, msg, res = self._mover(deslocamento_mm=[0, 0, 0], velocidade_mms=VEL_RAPIDA_MMS,
                                           parar_no_contato=True, limiar_n=cfg['limiar'],
                                           tocar_mesa=True, altura_mm=z_rapido)
                if not ok:
                    return falhar(f'{nome}, descendo: {msg}')
                if z_ref is None:
                    z_ref = res['ponta_mm'][2] + res['percorrido_mm']
                    if not ALTURA_INICIAL_MM[0] <= z_ref <= ALTURA_INICIAL_MM[1]:
                        j.log_de_fundo(f'Atenção: a ponteira começou a {z_ref:.0f} mm do tampo do '
                                       'modelo (o recomendado é de 20 a 80 mm).')
                if res['contato']:
                    ok, msg, res = self._mover(deslocamento_mm=[0, 0, 0], velocidade_mms=2.0,
                                               tocar_mesa=True,
                                               altura_mm=res['ponta_mm'][2] + RECUO_MM)
                    if not ok:
                        return falhar(f'{nome}, recuando: {msg}')
            # 2) devagar até encostar
            ok, msg, res = self._mover(deslocamento_mm=[0, 0, 0], velocidade_mms=cfg['v_lenta'],
                                       parar_no_contato=True, limiar_n=cfg['limiar'],
                                       tocar_mesa=True, altura_mm=-PROFUNDIDADE_MM)
            if not ok:
                return falhar(f'{nome}, encostando: {msg}')
            if not res['contato']:
                self._subir(z_ref)
                return falhar(f'{nome}: a ponteira desceu até {PROFUNDIDADE_MM:.0f} mm abaixo do '
                              'tampo da mesa do modelo sem encostar. Ou a mesa real está mais baixa '
                              'que o modelo de segurança, ou a célula não está percebendo o toque.')
            p = res['ponta_mm']
            toques.append({'nome': nome, 'x': p[0], 'y': p[1], 'z': p[2],
                           'forca': res['forca_n'], 'juntas': res['juntas']})
            j.log_de_fundo(f'Mesa: {nome} em x={p[0]:.2f} y={p[1]:.2f} z={p[2]:.3f} mm '
                           f'({res["forca_n"]:.2f} N).')
            # 3) sobe de volta à altura de partida
            ok, msg = self._subir(z_ref)
            if not ok:
                return falhar(f'{nome}, subindo: {msg}')
            anterior = off

        # De volta ao ponto de partida (acima do centro).
        if np.linalg.norm(anterior) > 0.01:
            ok, msg, _ = self._mover(deslocamento_mm=[-anterior[0], -anterior[1], 0.0],
                                     velocidade_mms=VEL_LIVRE_MMS)
            if not ok:
                return falhar(f'voltando ao centro: {msg}')
        j.definir_prefixo(None)
        resultado = self._calcular(toques, cfg)
        resultado['caminho'] = self._gravar(toques, cfg, resultado, inicio)
        self.resultado_pronto.emit(resultado)
        j.progresso.emit({'fracao': 1.0})
        j.terminar_acao(True, f'Mesa medida: inclinação de {resultado["inclinacao"]:.2f}° '
                              f'(X {resultado["incl_x"]:+.2f}°, Y {resultado["incl_y"]:+.2f}°).'
                        + (f' Atenção: {resultado["aviso"]}' if resultado['aviso'] else ''))

    def _subir(self, z_ref):
        ok, msg, _ = self._mover(deslocamento_mm=[0, 0, 0], velocidade_mms=VEL_LIVRE_MMS,
                                 tocar_mesa=True, altura_mm=z_ref)
        return ok, msg

    @staticmethod
    def _calcular(toques, cfg):
        """Plano dos toques do círculo (como no cr10twin, o centro fica fora do
        ajuste e mede a planicidade)."""
        P = np.array([[t['x'], t['y'], t['z']] for t in toques]) / 1000.0
        fit = fit_plane(P[1:])
        ok, motivo = validate_fit(fit)
        sag = sagitta_m(fit, P[0])
        incl_x = slope_along_deg(fit.normal, (1.0, 0.0))
        incl_y = slope_along_deg(fit.normal, (0.0, 1.0))
        aviso = '' if ok else motivo
        if abs(sag) > 5e-4 and not aviso:
            aviso = (f'o centro está {sag * 1000:+.2f} mm fora do plano do círculo: a mesa não é '
                     'plana ali (curvatura, sujeira ou algo embaixo)')
        return {'inclinacao': fit.tilt_deg, 'incerteza': fit.tilt_unc_deg, 'incl_x': incl_x,
                'incl_y': incl_y, 'rms_um': fit.rms_m * 1e6, 'sagitta_um': sag * 1e6,
                'z_centro': toques[0]['z'], 'aviso': aviso, 'toques': toques,
                'normal': list(fit.normal)}

    @staticmethod
    def _gravar(toques, cfg, r, inicio):
        try:
            os.makedirs(PASTA_MESA, exist_ok=True)
            caminho = os.path.join(PASTA_MESA, f'mesa_{inicio.strftime("%Y%m%d_%H%M%S")}.csv')
            with open(caminho, 'w', encoding='utf-8', newline='') as f:
                f.write(f'# medição da mesa; feita no {cfg["origem"]}; '
                        f'início {inicio.isoformat(timespec="seconds")}; centro + '
                        f'{cfg["n"]} pontos, raio {cfg["raio"]:.0f} mm; toque a {cfg["limiar"]:.1f} N; '
                        f'{cfg["v_lenta"]:.1f} mm/s ao encostar\n')
                f.write(f'# inclinação {r["inclinacao"]:.4f} graus (incerteza {r["incerteza"]:.4f}); '
                        f'ao longo de x {r["incl_x"]:+.4f} graus; ao longo de y {r["incl_y"]:+.4f} '
                        f'graus; resíduo {r["rms_um"]:.1f} um; centro fora do plano '
                        f'{r["sagitta_um"]:+.1f} um; normal {", ".join(f"{v:.6f}" for v in r["normal"])}'
                        + (f'; aviso: {r["aviso"]}' if r['aviso'] else '') + '\n')
                f.write('# x, y, z: face da ponteira no frame do robô (origem no centro da base, na '
                        'superfície em que ele está parafusado; x para a frente; z para cima), mm\n')
                f.write('ponto,x_mm,y_mm,z_mm,forca_n,j1,j2,j3,j4,j5,j6\n')
                for t in toques:
                    f.write(f'{t["nome"]},{t["x"]:.3f},{t["y"]:.3f},{t["z"]:.4f},{t["forca"]:.3f},'
                            + ','.join(f'{v:.4f}' for v in t['juntas']) + '\n')
            return caminho
        except OSError:
            return None

    def _mostrar_resultado(self, r):
        def sentido(graus, eixo, frente, tras):
            if abs(graus) < 0.005:
                return f'{eixo}: nivelada'
            sobe = frente if graus > 0 else tras
            return (f'{eixo}: {graus:+.2f}°, sobe para {sobe} '
                    f'({abs(math.tan(math.radians(graus))) * 100:.2f} mm a cada 100 mm)')
        inc = r['incerteza']
        inc_txt = f' ± {inc:.2f}°' if math.isfinite(inc) else ''
        linhas = [f'Inclinação da mesa: {r["inclinacao"]:.2f}°{inc_txt}',
                  sentido(r['incl_x'], 'Ao longo de x (frente/trás do robô)', 'a frente', 'trás'),
                  sentido(r['incl_y'], 'Ao longo de y (lados)', '+y', '−y'),
                  f'Planicidade: resíduo de {r["rms_um"]:.0f} µm; o centro está '
                  f'{r["sagitta_um"]:+.0f} µm do plano do círculo.',
                  f'Altura no centro: {r["z_centro"]:+.2f} mm em relação ao tampo do modelo de '
                  'segurança (valor aproximado: depende do comprimento real da ferramenta).']
        if r['aviso']:
            linhas.append(f'Atenção: {r["aviso"]}.')
        if r.get('caminho'):
            linhas.append(f'Gravado em {r["caminho"]}.')
        self.res_txt.setText('\n'.join(linhas))
        self.tab.setRowCount(len(r['toques']))
        for i, t in enumerate(r['toques']):
            for c, v in enumerate((t['nome'], _fmt(t['x'], 2), _fmt(t['y'], 2),
                                   _fmt(t['z'], 3), _fmt(t['forca'], 2))):
                self.tab.setItem(i, c, QTableWidgetItem(v))
        _ajustar_altura(self.tab)
        self.janela.log(f'Mesa: inclinação {r["inclinacao"]:.3f}° (x {r["incl_x"]:+.3f}°, '
                        f'y {r["incl_y"]:+.3f}°), resíduo {r["rms_um"]:.0f} µm.')
