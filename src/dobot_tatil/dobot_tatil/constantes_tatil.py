"""Constantes do sensor tátil usadas por touch_source.py.

Copiadas de src/touch_pack/touch_pack/constants.py do projeto cr10twin (ver
NOTICE.txt), com os mesmos nomes e valores, para o touch_source.py rodar sem
mudanças. Só o que ele importa veio para cá.
"""

# Touch sensor (STM32 → PC plotter → UDP). Porta DIFERENTE da célula, senão
# os fluxos se misturam no mesmo receptor.
TOUCH_SENSOR_UDP_PORT = 8081
# Payload (little-endian, 8 bytes): uint32 seq + float valor. Espelhado no
# plotter standalone (sensors/Touch_sensor).
TOUCH_PAYLOAD_FMT = '<If'
# Broadcast do I_final reemitido pelo TouchSensorSource a cada TOTAL.
# (No IRIS o reenvio fica DESLIGADO: a aba cria a fonte com
# udp_broadcast=False.)
TOUCH_UDP_BROADCAST_IP = '192.168.5.255'

# VIDs dos conversores USB-RS485 da célula de carga: a detecção automática do
# sensor tátil pula essas portas, que também aparecem como /dev/ttyUSB*.
FT_USB_VIDS = (
    0x1A86,   # WCH — CH340/CH341/CH343 (o ZK-U485 azul da bancada)
    0x10C4,   # Silicon Labs — CP2102/CP2104
    0x0403,   # FTDI — FT232
    0x067B,   # Prolific — PL2303
)

TOUCH_ROT180_GRIDS = frozenset({(5, 5)})


def taxel_frame_to_physical(vals: list, rows: int, cols: int) -> list:
    """Reordena um frame inteiro do firmware para a numeração FÍSICA.

    Depois disto o índice 0 é o taxel físico 00 e o índice rows*cols-1 é o
    último da última linha. Inverter a lista achatada É a rotação de 180° da
    grade, então a mesma linha serve para qualquer R×C. Grade não
    caracterizada (ou frame de tamanho inesperado) volta sem tocar."""
    if (rows, cols) in TOUCH_ROT180_GRIDS and len(vals) == rows * cols:
        return vals[::-1]
    return vals


def taxel_index_to_physical(idx: int, rows: int, cols: int) -> int:
    """O mesmo para um índice solto — as linhas RA/SA do firmware trazem
    `idx=` na convenção do frame, e o raster de spikes tem de casar com o
    heatmap. Grade não caracterizada volta sem tocar."""
    if (rows, cols) in TOUCH_ROT180_GRIDS:
        return rows * cols - 1 - idx
    return idx
