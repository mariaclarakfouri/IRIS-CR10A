"""Cinemática da ponta da ferramenta, para mover a ponteira em linha reta.

Usa o MESMO modelo do braço que a verificação de segurança (seguranca.py lê o
xacro do gêmeo): posição e orientação da face da ponteira a partir das 6
juntas (direta) e as juntas para uma pose da ponteira (inversa, numérica).

Frame: o do robô, origem no centro da base, na superfície onde ele está
parafusado; x para a frente da base, z para cima. Metros e graus (Dobot).
"""
import math

import numpy as np

PASSO_JACOBIANO_GRAUS = 1e-3
LAMBDA_DLS = 1e-3          # amortecimento (perto de singularidades)
PASSO_MAX_GRAUS = 2.0      # por iteração da inversa
PESO_ORIENTACAO_M = 0.1    # 1 rad de erro de orientação "vale" 0,1 m
TOL_POSICAO_M = 2e-6
TOL_ORIENTACAO_RAD = math.radians(0.002)


def _rotvec(R):
    """Vetor de rotação (eixo × ângulo, rad) de uma matriz de rotação."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    ang = math.acos(c)
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = math.sin(ang)
    if abs(s) < 1e-9:            # ~180°: não acontece em passos pequenos
        return np.zeros(3)
    return v * (ang / (2.0 * s))


class Cinematica:

    def __init__(self, modelo, comprimento_ferramenta_m):
        """`modelo`: seguranca.ModeloBraco; `comprimento_ferramenta_m`: do
        flange até a face da ponteira, ao longo do z do flange."""
        self.modelo = modelo
        self.tcp = float(comprimento_ferramenta_m)

    def direta(self, q_deg):
        """Pose 4x4 da face da ponteira."""
        Ts, _ = self.modelo.transformadas(q_deg)
        T = Ts['Link6'].copy()
        T[:3, 3] = T[:3, 3] + T[:3, 2] * self.tcp
        return T

    def _erro(self, T, T_alvo):
        dp = T_alvo[:3, 3] - T[:3, 3]
        dr = _rotvec(T_alvo[:3, :3] @ T[:3, :3].T)
        return dp, dr

    def _jacobiano(self, q, T):
        J = np.zeros((6, 6))
        for i in range(6):
            qi = np.array(q, dtype=float)
            qi[i] += PASSO_JACOBIANO_GRAUS
            Ti = self.direta(qi)
            J[:3, i] = (Ti[:3, 3] - T[:3, 3]) / PASSO_JACOBIANO_GRAUS
            J[3:, i] = _rotvec(Ti[:3, :3] @ T[:3, :3].T) / PASSO_JACOBIANO_GRAUS * PESO_ORIENTACAO_M
        return J

    def inversa(self, T_alvo, q_semente, iteracoes=60):
        """Juntas (graus) que levam a ponteira a `T_alvo`, partindo de
        `q_semente` (a solução mais próxima dela). Devolve (q, ok)."""
        q = np.array(q_semente, dtype=float)
        for _ in range(iteracoes):
            T = self.direta(q)
            dp, dr = self._erro(T, T_alvo)
            if np.linalg.norm(dp) < TOL_POSICAO_M and np.linalg.norm(dr) < TOL_ORIENTACAO_RAD:
                return q, True
            e = np.concatenate([dp, dr * PESO_ORIENTACAO_M])
            J = self._jacobiano(q, T)
            dq = J.T @ np.linalg.solve(J @ J.T + (LAMBDA_DLS ** 2) * np.eye(6), e)
            maior = float(np.max(np.abs(dq)))
            if maior > PASSO_MAX_GRAUS:
                dq *= PASSO_MAX_GRAUS / maior
            q = q + dq
        T = self.direta(q)
        dp, dr = self._erro(T, T_alvo)
        return q, bool(np.linalg.norm(dp) < 1e-5 and np.linalg.norm(dr) < math.radians(0.01))

    def linha_reta(self, q_inicio, deslocamento_m, passo_m=0.001):
        """Juntas ao longo de uma linha reta da ponteira (orientação fixa), uma
        a cada `passo_m`. Devolve (lista de q, motivo); motivo vazio se ok."""
        T0 = self.direta(q_inicio)
        d = np.asarray(deslocamento_m, dtype=float)
        n = max(1, int(math.ceil(float(np.linalg.norm(d)) / passo_m)))
        qs, q = [np.array(q_inicio, dtype=float)], np.array(q_inicio, dtype=float)
        for k in range(1, n + 1):
            T = T0.copy()
            T[:3, 3] = T0[:3, 3] + d * (k / n)
            q_novo, ok = self.inversa(T, q)
            if not ok:
                return None, (f'a ponteira não alcança o ponto a {k * passo_m * 1000:.0f} mm do '
                              'começo da linha mantendo a orientação')
            if float(np.max(np.abs(q_novo - q))) > 5.0:
                return None, ('perto de uma singularidade do braço: as juntas girariam demais '
                              'para um passo pequeno da ponteira')
            qs.append(q_novo)
            q = q_novo
        return qs, ''
