#!/bin/bash
# Compila o workspace e abre a interface do Dobot (robô + GUI + gêmeo virtual).
#
#   iris                    robô REAL, com o gêmeo seguindo em tempo real
#   iris gemeo              SIMULAÇÃO: só o gêmeo virtual se move, o robô real não
#   iris sem-gemeo          robô REAL, sem abrir o gêmeo
#   iris sem-nada           sem a célula de carga na ponta (padrão: com a célula)
#
# As opções se combinam: iris gemeo sem-nada
set -e

# A pasta do projeto é a deste arquivo: funciona em qualquer computador e pasta.
WORKSPACE_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

FERRAMENTA=celula
ARGS=()
for opcao in "$@"; do
    case "$opcao" in
        gemeo|sim)   ARGS+=(modo:=sim) ;;
        sem-gemeo)   ARGS+=(gemeo:=false) ;;
        sem-nada)    FERRAMENTA=nada ;;
        *)
            echo "Opção desconhecida: $opcao"
            echo "Use: iris [gemeo | sem-gemeo] [sem-nada]   (iris ajuda para detalhes)"
            exit 1
            ;;
    esac
done

if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "O ROS 2 Humble não está instalado neste computador (falta /opt/ros/humble)."
    echo "Veja a seção Instalação do README.md."
    exit 1
fi
source /opt/ros/humble/setup.bash
cd "$WORKSPACE_DIR"
# Compila só se algum arquivo do projeto mudou desde a última compilação (ou
# depois de um git pull). Sem mudança, pula os ~4 s do colcon. Para forçar:
# apague install/.compilado (ou rode colcon build).
MARCA=install/.compilado
if [ ! -f "$MARCA" ] || [ -n "$(find src -newer "$MARCA" -not -path '*/__pycache__/*' -print -quit)" ]; then
    colcon build
    touch "$MARCA"
else
    echo "Nada mudou desde a última compilação: abrindo direto."
fi
source install/setup.bash

# Segurança de rede: o ROS 2 (DDS) só conversa dentro deste computador.
# Sem isso, outro computador na mesma rede rodando ROS 2 no mesmo
# ROS_DOMAIN_ID enxergaria estes nós e poderia chamar /connect, /move_joint...
# Não afeta o robô: a conexão com o CR10A é TCP (portas 29999 e 30004), não DDS.
# Fica depois dos "source" para nenhum deles mudar o valor.
export ROS_LOCALHOST_ONLY=1

if [ "$FERRAMENTA" = celula ]; then ARGS+=(celula:=true); else ARGS+=(celula:=false); fi

ros2 launch dobot_control dobot_bringup.launch.py "${ARGS[@]}"
