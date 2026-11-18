# Estado das 6 juntas do CR10A, publicado em /dobot/estado_juntas.
#
# Robô real: lido da porta de feedback 30004 (somente leitura).
# Simulação: ângulo e velocidade vêm do gêmeo virtual; o esforço (corrente,
# temperatura e torque) não existe no gêmeo -> esforco_disponivel = false e
# os campos ficam em NaN.

builtin_interfaces/Time stamp
string fonte                      # "real" ou "simulacao"
float64[6] angulo_graus
float64[6] velocidade_graus_s
bool esforco_disponivel
float64[6] corrente_a
float64[6] temperatura_c
float64[6] torque_nm
