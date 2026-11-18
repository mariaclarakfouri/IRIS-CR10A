"""Ponto de entrada da interface (executável `dobot_gui`).

A interface está dividida em:
  janela.py         base comum (conexão, velocidade, home, parar, emergência...)
  modulos/          um arquivo por módulo (página da barra lateral)
  tema.py           cores, estilo e ícones
  widgets.py        peças visuais reutilizáveis
  ros_bridge.py     comunicação com o dobot_action_server
  constantes.py     valores fixos
"""
import gc
import signal
import sys
import threading
import time

import rclpy
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication

from .constantes import APP_NOME
from .janela import JanelaPrincipal
from .ros_bridge import RosBridge
from .tema import STYLE_SHEET, app_icon

# Nomes antigos, mantidos para quem ainda os importa.
MainWindow = JanelaPrincipal


def main(args=None):
    inicio = time.monotonic()
    rclpy.init(args=args)
    ros_node = RosBridge()

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    app.setApplicationName(APP_NOME)
    app.setWindowIcon(app_icon())
    app.setStyleSheet(STYLE_SHEET)

    window = JanelaPrincipal(ros_node)
    window.show()
    # Quanto a janela levou para abrir, contado desde que este programa começou.
    QTimer.singleShot(0, lambda: print(
        f'Interface aberta em {time.monotonic() - inicio:.1f} s.', flush=True))

    # Sem isso, Ctrl+C no terminal fica "preso" até algum evento acontecer na
    # janela — o loop de eventos do Qt não entrega o sinal pro Python sozinho.
    signal.signal(signal.SIGINT, lambda *args: window.close())
    # Fechar a janela do terminal (SIGHUP) ou o launch encerrar (SIGTERM) também
    # fecha a interface — senão ela ficava aberta sem o servidor por trás.
    signal.signal(signal.SIGTERM, lambda *args: window.close())
    signal.signal(signal.SIGHUP, lambda *args: window.close())
    keepalive_timer = QTimer()
    keepalive_timer.timeout.connect(lambda: None)
    keepalive_timer.start(200)

    # Tudo o que foi criado até aqui (janela, gráficos, modelo 3D, ROS) vive o
    # programa inteiro: tirá-lo da faxina de memória do Python deixa cada
    # faxina curta (medido: ~28 ms -> <1 ms), sem travadinhas na tela quando os
    # sensores táteis criam e descartam muitos objetos por segundo.
    gc.collect()
    gc.freeze()
    # Diagnóstico opcional (DOBOT_VIGIA=<arquivo>): anota as travadinhas da janela.
    from . import vigia
    _vigia = vigia.iniciar(app)  # noqa: F841 — guardado para o timer não ser coletado

    exit_code = app.exec_()

    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
