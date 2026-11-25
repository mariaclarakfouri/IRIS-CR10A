<div align="center">

# IRIS CR10A

**Interface Robótica de Inspeção Sensorial** para o robô colaborativo Dobot CR10A.

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros&logoColor=white)
![Gazebo Classic 11](https://img.shields.io/badge/Gazebo-Classic%2011-FCBA28)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu&logoColor=white)
![Python 3.10](https://img.shields.io/badge/Python-3.10-3776AB?logo=python&logoColor=white)
![Robô Dobot CR10A](https://img.shields.io/badge/Rob%C3%B4-Dobot%20CR10A-2563EB)

</div>

---

## O que tem hoje

A base da interface:

- **Conexão** com o controlador do CR10A pela rede (TCP/IP): conectar, desconectar, limpar
  alarme.
- **Controle Juntas**: movimento de cada junta em tempo real e a **home** (com **Ir para a
  home** e **Alterar home**).
- **Velocidade** ajustável na barra do robô (até 50%).
- **Drag mode**: o braço fica solto para ser guiado com a mão. O botão funciona como o
  Conectar: liga e fica ligado (o estado mostra "Drag mode"); para sair, aperte o botão no
  robô ou clique em **Desabilitar drag**. A interface percebe quando o robô entra ou sai do
  drag mode pelo botão do braço. Fica na barra de cima e no cartão da home. Só no robô real.
- **Parar movimento** e **EMERGÊNCIA**.
- **Célula de Carga** (FA7155): força e momento ao vivo em gráfico e tara.
- **Robô 3D**: o CR10A em 3D seguindo o robô em tempo real, com o que estiver na ponta (nada ou, com `com-celula`, a célula). O comando
  pelo 3D fica desligado até existir a verificação de segurança.
- **Ferramentas**: qual ferramenta está na ponta e as medidas de cada uma (comprimento,
  desvio, raio, massa). A célula de carga FA7155 já vem cadastrada.
- **Campo de Trabalho**: grava os pontos medidos no laboratório (base e teto), cada um com
  nome, os 6 ângulos do robô e a ferramenta na ponta, em `~/.config/iris/pontos_campo.csv`.
  Os pontos são os mesmos em todos os computadores: vão e vêm sozinhos pelo GitHub, num ramo
  só deles (`pontos-campo`), sem levar código. Sem internet, ficam salvos e vão depois.
  Com o robô 3D da aba: o campo de trabalho calculado em volta do robô (os pontos são
  juntados em planos pela altura; cada plano tem como borda o contorno dos pontos em volta,
  os pontos "mesa" e "começo …" só dão a altura do plano; entre dois planos a parede vai
  em linha reta de um contorno ao outro, sem passar do alcance do braço), clicar num ponto
  mostra ele em azul, e "Levar até o ponto" move o robô (ou o gêmeo) até lá.
  Um segundo robô 3D mostra o **campo calculado pela base**: paredes verticais subindo do
  contorno dos pontos da base, cortadas pelo alcance do braço com a ponta para baixo
  (cinemática inversa), sem passar do campo medido.
- **Campo do Experimento**: escolha um experimento (por enquanto, Controle das Juntas) e
  desenhe numa vista de cima da mesa o polígono onde ele pode trabalhar, com uma faixa de
  alturas. O campo do experimento é o campo de trabalho geral dentro desse corte (o corte só
  diminui, nunca aumenta), e o campo geral não muda. Os movimentos da aba do experimento são
  conferidos também contra o corte. Os cortes ficam em `~/.config/iris/campos_experimentos.json`.
- **Segurança dos movimentos**: todo movimento por comando (juntas, Home, Robô 3D, levar até
  um ponto) é conferido pelo servidor antes de ir para o robô. O caminho inteiro do MovJ é
  calculado (cinemática direta, a cada 0,5° nas juntas); se o flange, ou a ponta da célula de
  carga com `com-celula`, sairia do campo calculado pela base em qualquer momento, o
  movimento é recusado e a interface avisa que o ponto não é seguro. Os **obstáculos** são
  conferidos com o robô inteiro (braço, cotovelo, antebraço, punho, flange e ferramenta, pelas
  malhas 3D): nenhuma peça pode descer abaixo do tampo da mesa (altura pelo ponto "mesa"),
  passar das paredes retas do computador e da esquerda, nem subir acima do teto. Paredes e
  teto ficam onde o corpo do robô chegou na medição (`PAREDES` e `TETO_MM` em
  `dobot_gui/seguranca.py`). O drag mode
  (guiar com a mão) não passa por essa conferência.
- **Registro** de tudo o que foi feito.
- **Gêmeo virtual** do CR10A no Gazebo: no modo simulação, os comandos movem só o gêmeo, e
  o robô real não é tocado; no modo real, o gêmeo segue o robô.

> [!WARNING]
> **Ainda não há verificação de segurança dos movimentos** (colisão com a bancada, limites
> das juntas). Ela é a próxima etapa do projeto. Até lá, teste no gêmeo virtual
> (`iris gemeo`) e use o robô real com velocidade baixa.

---

## Como usar

| Comando | O que faz |
|---|---|
| `iris` | Abre a interface com o **robô real**, **sem nada na ponta**, com o gêmeo seguindo em tempo real |
| `iris gemeo` | Abre em **simulação**, **sem nada na ponta**: só o gêmeo virtual se move |
| `iris sem-gemeo` | Robô real sem nada na ponta, sem abrir o gêmeo |
| `iris com-celula` | Robô real com a **célula de carga** na ponta. Combina com os outros: `iris gemeo com-celula` abre a simulação com a célula |
| `iris code` | Abre o código no VS Code |
| `iris mudancas` | Mostra o que mudou desde a última versão salva |
| `iris salvar "descrição"` | Salva as mudanças e envia para este repositório |
| `iris ajuda` | Lista os comandos |

Para fechar a interface: **Ctrl+C** no terminal.

O robô deve estar na mesma rede do computador, no IP `192.168.5.2` (padrão), com o
controlador em modo **remoto (TCP/IP)**.

---

## Instalação

**Requisitos:** Ubuntu 22.04, ROS 2 Humble e Gazebo Classic 11.

```bash
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
                 ros-humble-ros2-controllers ros-humble-xacro python3-colcon-common-extensions \
                 python3-pyqt5 python3-numpy python3-serial
sudo usermod -aG dialout $USER     # célula de carga pela USB (saia da sessão e entre de novo)

git clone https://github.com/mariaclarakfouri/IRIS-CR10A.git ~/iris
cd ~/iris
./instalar.sh
```

O `instalar.sh` cria o comando **`iris`** (em `~/.local/bin`) e confere o que o computador
precisa. Depois, num terminal novo:

```bash
iris gemeo     # abre no gêmeo virtual
```

Sem o comando, `./abrir_interface.sh gemeo` (na pasta do projeto) faz o mesmo.

---

## Estrutura

```
src/
├── dobot_celula/       leitura da célula de carga FA7155 (RS-485) e da força simulada no gêmeo
├── dobot_control/      servidor que fala com o robô (conexão, movimento, parada, emergência)
├── dobot_gui/          interface gráfica (PyQt5)
│   └── dobot_gui/
│       ├── janela.py       base comum (conexão, velocidade, home, parar, emergência)
│       ├── modulos/        um arquivo por aba
│       ├── tema.py         cores, estilo e ícones
│       └── widgets.py      peças visuais reutilizáveis
├── dobot_interfaces/   mensagens ROS 2 (ação de movimento e serviços)
└── dobot_twin/         gêmeo virtual: modelo do CR10A, laboratório e espelho
iris                    o comando iris (instalado pelo instalar.sh)
instalar.sh             cria o comando iris e confere o computador
abrir_interface.sh      compila (só se algo mudou) e abre tudo
```

---

## Manuais oficiais

Os manuais da Dobot **não estão neste repositório** (a Dobot proíbe a reprodução sem
autorização por escrito). Baixe-os na
[Central de Downloads da Dobot](https://www.dobot-robots.com/service/download-center):

| Manual | Para que serve aqui |
|---|---|
| *Dobot CR A Series User Guide* | O robô: especificações, instalação, segurança e manutenção |
| *Dobot TCP/IP Remote Control Interface Guide* | Os comandos pela rede (porta 29999) e o *feedback* em tempo real (porta 30004) |

O manual da célula de carga FA7155 (protocolo Modbus RTU) vem com o sensor, do fabricante.

---

## Créditos

- **Gêmeo virtual** — modelo do CR10A (URDF e malhas), laboratório do Gazebo, conversão de
  ângulos e leitura do *feedback* do robô: baseados no projeto
  [cr10twin](https://github.com/Martins-Lucaas/cr10twin), de **Lucas Martins**
  ([@Martins-Lucaas](https://github.com/Martins-Lucaas)), licença Apache-2.0. O modelo do
  robô vem dos arquivos oficiais da Dobot
  ([DOBOT_6Axis_ROS2_V4](https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4)).
  Detalhes em [`src/dobot_twin/NOTICE.txt`](src/dobot_twin/NOTICE.txt).
- **Célula de carga** — a ferramenta (célula, acoplador e ponteira) no gêmeo e no Robô 3D, a
  força simulada e a leitura do protocolo da FA7155: baseadas no mesmo
  [cr10twin](https://github.com/Martins-Lucaas/cr10twin). Detalhes em
  [`src/dobot_celula/NOTICE.txt`](src/dobot_celula/NOTICE.txt).

---

**Autora:** Maria Clara Almeida Kfouri Trazzi
