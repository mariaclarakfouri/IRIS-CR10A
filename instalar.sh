#!/bin/bash
# Deixa o comando `iris` pronto neste computador. Rode UMA vez, depois do
# git clone (e de novo se mudar o projeto de pasta):
#
#   cd <pasta do projeto>
#   ./instalar.sh
#
# O que ele faz:
#   * cria ~/.local/bin/iris apontando para o arquivo `iris` deste
#     repositório (atualizações chegam pelo git pull, sem instalar de novo);
#   * garante ~/.local/bin no PATH e ROS_LOCALHOST_ONLY=1 nos terminais;
#   * confere o que o projeto precisa (ROS 2 Humble, pacotes Python, grupo dialout,
#     nome no git) e diz o comando para o que faltar. Não usa sudo nem instala nada.

PROJETO="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
BIN="$HOME/.local/bin"
FALTA=0

ok()    { echo "  ✔ $*"; }
falta() { echo "  ✖ $*"; FALTA=1; }

echo "Projeto: $PROJETO"
echo
echo "Comando iris"
mkdir -p "$BIN"
if [ -e "$BIN/iris" ] && [ ! -L "$BIN/iris" ]; then
    mv "$BIN/iris" "$BIN/iris.antigo"
    ok "o comando antigo foi guardado em $BIN/iris.antigo"
fi
ln -sfn "$PROJETO/iris" "$BIN/iris"
chmod +x "$PROJETO/iris" "$PROJETO/abrir_interface.sh"
ok "$BIN/iris -> $PROJETO/iris"

case ":$PATH:" in
    *":$BIN:"*) ok "$BIN já está no PATH" ;;
    *)
        if ! grep -qs 'HOME/.local/bin' "$HOME/.bashrc"; then
            printf '\n# iris: comandos instalados em ~/.local/bin\nexport PATH="$HOME/.local/bin:$PATH"\n' \
                >> "$HOME/.bashrc"
        fi
        ok "$BIN acrescentado ao PATH (vale nos terminais abertos daqui para a frente)"
        ;;
esac

if grep -qs '^export ROS_LOCALHOST_ONLY=1' "$HOME/.bash_aliases"; then
    ok "ROS_LOCALHOST_ONLY=1 já está em ~/.bash_aliases"
else
    printf '\n# iris: o ROS 2 só conversa dentro deste computador (como o comando\n# iris faz), para os comandos "ros2 ..." enxergarem a interface aberta\n# e outros computadores da rede não. Para desfazer, apague estas linhas.\nexport ROS_LOCALHOST_ONLY=1\n' \
        >> "$HOME/.bash_aliases"
    ok "ROS_LOCALHOST_ONLY=1 acrescentado a ~/.bash_aliases"
fi

echo
echo "O que o projeto precisa"
if [ -f /opt/ros/humble/setup.bash ]; then
    ok "ROS 2 Humble"
else
    falta "ROS 2 Humble não encontrado: instale pelo guia oficial (docs.ros.org > Humble > Installation > Ubuntu)"
fi
if command -v colcon >/dev/null; then
    ok "colcon"
else
    falta "colcon: sudo apt install python3-colcon-common-extensions"
fi
if [ -f /opt/ros/humble/share/gazebo_ros/package.xml ]; then
    ok "Gazebo (gazebo_ros)"
else
    falta "Gazebo: sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-controllers ros-humble-xacro"
fi
for par in PyQt5:python3-pyqt5 numpy:python3-numpy scipy:python3-scipy yaml:python3-yaml \
           matplotlib:python3-matplotlib serial:python3-serial; do
    modulo=${par%%:*}; pacote=${par##*:}
    if python3 -c "import $modulo" 2>/dev/null; then
        ok "Python: $modulo"
    else
        falta "Python: $modulo (sudo apt install $pacote)"
    fi
done
if id -nG "$USER" | grep -qw dialout; then
    ok "grupo dialout (célula de carga e placas táteis pela USB)"
else
    falta "grupo dialout: sudo usermod -aG dialout $USER  (depois saia da sessão e entre de novo)"
fi
if [ -n "$(git -C "$PROJETO" config user.name)" ] && [ -n "$(git -C "$PROJETO" config user.email)" ]; then
    ok "git: $(git -C "$PROJETO" config user.name) <$(git -C "$PROJETO" config user.email)>"
else
    falta "nome no git: git config --global user.name \"Seu Nome\" && git config --global user.email \"seu@email\""
fi

echo
if [ "$FALTA" -eq 0 ]; then
    echo "Tudo pronto. Abra um terminal novo e use, por exemplo:  iris gemeo"
else
    echo "Falta o que está marcado com ✖. Depois de resolver, rode ./instalar.sh de novo."
fi
echo "(iris ajuda mostra todas as opções)"
