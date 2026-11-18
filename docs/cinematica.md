# Cinemática do CR10A

[← Voltar para a plataforma CR10A](../README.md)

Como a plataforma calcula onde o braço está (**cinemática direta**), que juntas levam a
ponteira a um lugar (**cinemática inversa**) e o **Jacobiano** usado nessa conta. Tudo fica
no pacote `dobot_control` e roda no servidor de movimento (`dobot_action_server`).

| Peça | Arquivo | Usada em |
|---|---|---|
| Cinemática direta (cada peça do braço) | [`seguranca.py`](../src/dobot_control/dobot_control/seguranca.py), classe `ModeloBraco` | Verificação de segurança antes de **todo** movimento |
| Cinemática direta da ponteira, inversa e Jacobiano | [`cinematica.py`](../src/dobot_control/dobot_control/cinematica.py), classe `Cinematica` | Ponteira em linha reta (ação `MoverPonta`, aba [Alinhamento da Mesa](modulos/alinhamento_mesa.md)) |

---

## Um só modelo do braço

As medidas do braço vêm do **mesmo modelo do gêmeo virtual**: o `cr10_robot.xacro` do pacote
`dobot_twin`, com a posição e o eixo de cada junta, e as malhas STL de cada peça. A
verificação de segurança, a cinemática da ponteira e o Gazebo usam esse mesmo modelo, então os
três concordam sobre onde o braço está.

**Frame:** o do robô, com origem no centro da base, na superfície em que ele está
parafusado; **x** para a frente da base, **z** para cima. Distâncias em metros, ângulos em
graus (como o Dobot).

**Sinal das juntas:** o Dobot e o modelo do gêmeo usam o mesmo sentido em todas as juntas,
menos a **J1**, que tem o sinal invertido (`DOBOT_TO_URDF_SIGN = [-1, 1, 1, 1, 1, 1]`, em
`seguranca.py`). A conversão é feita ali, antes de qualquer conta.

---

## Cinemática direta (juntas → posição)

Dados os 6 ângulos, `ModeloBraco.transformadas(q)` devolve a **pose 4×4** (posição e
orientação) de cada peça: base, Link1 a Link6. A conta segue a cadeia do modelo, de junta em
junta:

```
T_peça = T_peça_anterior · T_origem_da_junta · Rot(eixo_da_junta, ângulo)
```

`T_origem_da_junta` e `eixo_da_junta` são lidos do xacro (`<origin xyz rpy>` e `<axis>`). Não
é usada uma tabela de Denavit–Hartenberg: as matrizes vêm direto do modelo URDF, o mesmo que
o Gazebo usa.

**A ponteira:** `Cinematica.direta(q)` pega a pose do flange (Link6) e anda, ao longo do eixo
**z do flange**, o comprimento da ferramenta: do flange até a ponta da caixa da ferramenta do
perfil em uso, em `seguranca.yaml` (com a célula FA7155, **67,7 mm** até a face da ponteira).

**Onde é usada:**
- **Segurança:** antes de cada movimento, o caminho é percorrido em passos pequenos. Em cada
  passo, a direta posiciona todas as peças, e a distância delas até a mesa, o chão, as
  paredes, os armários e as outras peças do braço é conferida (ver
  [Segurança](../README.md#segurança)). A direta também dá o **alcance de cada junta** (o
  quanto a ponta anda por grau), que define o tamanho do passo, para nenhum ponto do braço
  andar mais que 10 mm entre duas verificações.
- **Ponteira:** a posição da face da ponteira em cada toque do Alinhamento da Mesa.

---

## Jacobiano

Matriz **6×6** que diz quanto a ponteira **anda** (3 linhas, em m) e **gira** (3 linhas, em
rad) quando cada junta (6 colunas) se mexe um pouquinho.

É calculado **numericamente, por diferenças finitas** (`Cinematica._jacobiano`): cada junta é
mexida **0,001°**, a direta é calculada de novo, e a diferença de posição e de orientação,
dividida pelo passo, vira a coluna daquela junta. A diferença de orientação é o vetor de
rotação (eixo × ângulo) entre as duas poses.

**Peso da orientação:** posição e giro têm unidades diferentes. Nas linhas de giro, 1 rad
"vale" **0,1 m** (`PESO_ORIENTACAO_M`), para os dois erros pesarem parecido na conta.

---

## Cinemática inversa (posição → juntas)

`Cinematica.inversa(T_alvo, q_semente)` acha as 6 juntas que levam a ponteira à pose
`T_alvo` (posição **e** orientação). É **numérica e iterativa**: parte de `q_semente` (a
posição atual) e corrige aos poucos.

Em cada iteração:
1. calcula a pose atual e o erro até o alvo: `e = [Δposição, Δorientação · 0,1 m]`;
2. calcula o Jacobiano `J` nessa pose;
3. calcula a correção das juntas por **mínimos quadrados amortecidos** (*damped least
   squares*, ou Levenberg–Marquardt):

   ```
   Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e,      λ = 10⁻³
   ```

   O amortecimento `λ` impede que a correção exploda perto de uma **singularidade**, onde o
   Jacobiano quase não tem inversa;
4. limita a correção a **2° por junta** por iteração (`PASSO_MAX_GRAUS`) e soma em `q`.

Para quando o erro fica abaixo de **2 µm** de posição e **0,002°** de orientação, ou depois
de **60 iterações**. No fim, só aceita a solução se o erro estiver abaixo de 10 µm e 0,01°.

Como parte da posição atual e anda em passos pequenos, a inversa devolve **a solução mais
próxima** da configuração em que o braço já está. Ela nunca "pula" para outra configuração
(por exemplo, cotovelo para cima ↔ para baixo).

---

## Ponteira em linha reta

`Cinematica.linha_reta(q_inicio, deslocamento)` é o que usa a inversa na prática:
1. calcula a pose atual da ponteira (direta);
2. divide o deslocamento em passos de **1 mm**, mantendo a **orientação fixa**;
3. em cada passo, resolve a inversa partindo da solução do passo anterior;
4. recusa a linha se algum ponto não for alcançável com aquela orientação, ou se perto de uma
   **singularidade**: quando um passo de 1 mm da ponteira exigiria **mais de 5°** de alguma
   junta.

O servidor (ação `MoverPonta`) confere cada trecho entre dois pontos da linha com a mesma
verificação de segurança dos outros movimentos, valida a trajetória (velocidade e trancos) e
envia os pontos ao robô por `ServoJ`, um a um, acompanhando a força da célula de carga para
**parar ao encostar**. É assim que o **Alinhamento da Mesa** desce a ponteira reta até a mesa.

---

## O que não é usado

- Inversa **analítica** (fórmula fechada) e Jacobiano **analítico**.
- Controle de velocidade da ponteira pelo Jacobiano, medida de manipulabilidade e escolha
  entre as várias soluções da inversa.
- Os outros movimentos (Controle Juntas, ir para a home, Robô 3D) são feitos **no espaço das
  juntas**: os ângulos vão direto ao robô (`MovJ`), sem inversa. Neles, a cinemática entra
  só pela **direta**, na verificação de segurança. Nas Coletas, quem calcula a trajetória da
  ponteira é o explorador do cr10twin, com a cinemática dele.

## Parâmetros

Em [`cinematica.py`](../src/dobot_control/dobot_control/cinematica.py):

| Constante | Valor | O que é |
|---|---|---|
| `PASSO_JACOBIANO_GRAUS` | 0,001° | Passo das diferenças finitas do Jacobiano |
| `LAMBDA_DLS` | 10⁻³ | Amortecimento dos mínimos quadrados (perto de singularidades) |
| `PASSO_MAX_GRAUS` | 2° | Correção máxima de cada junta por iteração da inversa |
| `PESO_ORIENTACAO_M` | 0,1 m | Quanto 1 rad de erro de orientação "vale" em posição |
| `TOL_POSICAO_M` | 2 µm | Erro de posição para parar a inversa |
| `TOL_ORIENTACAO_RAD` | 0,002° | Erro de orientação para parar a inversa |
