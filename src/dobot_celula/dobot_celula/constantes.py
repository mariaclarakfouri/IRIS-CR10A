"""Constantes da célula de carga FA7155 (6 eixos, RS485).

Valores copiados de src/touch_pack/touch_pack/constants.py do projeto cr10twin
(seções do FA7155 e a taxa nominal usada pelo filtro), com os comentários
resumidos. Ver NOTICE.txt.
"""

# --- Link serial (conversor USB-RS485) ---
FT_SERIAL_BAUD = 1_000_000      # exemplar da bancada (o manual dá 115200 como padrão geral)
# Velocidade automática (baud 0, o padrão do nó): tenta estas, em ordem, até
# chegar um quadro válido. 115200 é a de fábrica (manual §4.3: "the default baud
# rate ... upon power-on is 115200"); 1 Mbaud, a do exemplar da bancada.
FT_BAUDS_AUTO = (FT_SERIAL_BAUD, 115200)
FT_BAUD_TROCA_S = 2.5           # sem quadro válido por este tempo: tenta a próxima
FT_FRAME_HEADER = b'\x53\x54'   # "ST"
FT_FRAME_LEN = 28               # 2 (cabeçalho) + 6 x float32 + 2 (CRC-16/MODBUS)
FT_AXES = ('fx', 'fy', 'fz', 'mx', 'my', 'mz')
FT_NOMINAL_RATE_HZ = 400.0
FT_MIN_RATE_HZ = 100.0          # abaixo disto: cabo ruim, baud errado ou taxa diferente

# VIDs dos chips de ponte USB-serial dos conversores RS485 (o FA7155 não
# aparece na USB, quem aparece é o conversor).
FT_USB_VIDS = (
    0x1A86,   # WCH: CH340/CH341/CH343 (o ZK-U485 azul da bancada)
    0x10C4,   # Silicon Labs: CP2102/CP2104
    0x0403,   # FTDI: FT232
    0x067B,   # Prolific: PL2303
)

# --- Fundo de escala (FA7155D, catálogo) ---
FT_RATED_FORCE_N = 400.0
FT_RATED_TORQUE_NM = 20.0
FT_SAFE_OVERLOAD_PCT = 300.0    # sobrecarga segura do catálogo, em % do fundo de escala


def ft_max_rate_hz(baud: float = FT_SERIAL_BAUD) -> float:
    """Teto de quadros/s que cabe num baud (28 B x 10 bits por quadro)."""
    return float(baud) / (FT_FRAME_LEN * 10)


def ft_axis_rated(axis: str) -> float:
    """Fundo de escala do eixo: força para fx/fy/fz, torque para mx/my/mz."""
    return FT_RATED_FORCE_N if axis in ('fx', 'fy', 'fz') else FT_RATED_TORQUE_NM


# Taxa nominal usada pelo filtro (mediana + One-Euro) como chute até o 1º dt
# medido; o dt real vem do carimbo de cada amostra.
LC_NOMINAL_RATE_HZ = 24.0

# --- Modo POLLED (do cr10twin, constants.py): o computador pergunta e a célula
# responde. É o modo em que o exemplar do laboratório responde: ele NÃO manda
# os quadros sozinho. Só leitura (função 0x03); as escritas de ft_modbus ficam
# travadas enquanto FT_MODBUS_MAP_CONFIRMED for False.
FT_MODE_STREAM = 'stream'
FT_MODE_POLLED = 'polled'
FT_MODE_CHOICES = (FT_MODE_STREAM, FT_MODE_POLLED)

FT_MODBUS_MAP_CONFIRMED = False
# None = endereço desconhecido (escritas recusadas).
FT_MODBUS_MAP: dict = {
    'zero':       None,
    'zero_value': 1,
    'rate':       None,
    'node_id':    None,
    'baud':       None,
    'stream':     None,
    'stream_on':  1,
    'stream_off': 0,
    'device_id':  None,
}
FT_MODBUS_DATA_ADDR = 0x0003   # holding register inicial dos seis eixos
FT_MODBUS_DATA_REGS = 12       # 12 regs = 24 bytes = 6 x float32 LE
FT_MODBUS_SLAVE_ID = 1
FT_MODBUS_TIMEOUT_S = 0.5


def ft_polled_max_rate_hz(baud: float = 115200) -> float:
    """Teto de amostras/s no modo polled: requisição (8 B) + resposta (29 B) +
    dois silêncios de 3,5 caracteres do Modbus RTU = 440 bits por amostra."""
    return float(baud) / ((8 + 29) * 10 + 2 * 35)
