"""Vigia de travadinhas da janela (só para diagnóstico).

Ligado pela variável de ambiente DOBOT_VIGIA=<arquivo>: uma batida a cada 10 ms
na thread da janela e uma thread que, quando a batida atrasa mais que o limiar,
anota no arquivo quanto a janela ficou parada e o que ela estava fazendo (a pilha
de chamadas naquele instante). Desligado, não faz nada.
"""
import os
import sys
import threading
import time
import traceback

from PyQt5.QtCore import QTimer

LIMIAR_S = 0.040


def iniciar(app):
    caminho = os.environ.get('DOBOT_VIGIA')
    if not caminho:
        return None
    principal = threading.main_thread().ident
    batida = [time.monotonic()]
    timer = QTimer(app)
    timer.timeout.connect(lambda: batida.__setitem__(0, time.monotonic()))
    timer.start(10)
    arq = open(caminho, 'a', encoding='utf-8')
    arq.write(f'# vigia ligado {time.strftime("%H:%M:%S")} (limiar {LIMIAR_S * 1000:.0f} ms)\n')
    arq.flush()

    def laco():
        pilha, inicio = None, None
        while True:
            time.sleep(0.003)
            idade = time.monotonic() - batida[0]
            if idade > LIMIAR_S and pilha is None:
                inicio = batida[0]
                f = sys._current_frames().get(principal)
                pilha = ([f'{os.path.basename(q.filename)}:{q.lineno} {q.name}'
                          for q in traceback.extract_stack(f)[-12:]] if f else ['?'])
            elif pilha is not None and batida[0] != inicio:
                dur = (batida[0] - inicio) * 1000
                arq.write(f'{time.strftime("%H:%M:%S")} parada de {dur:.0f} ms: '
                          + ' <- '.join(reversed(pilha)) + '\n')
                arq.flush()
                pilha = None

    threading.Thread(target=laco, daemon=True, name='vigia').start()
    return timer
