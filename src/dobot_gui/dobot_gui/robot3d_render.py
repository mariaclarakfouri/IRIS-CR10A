# Derivado de manip3d.py do projeto cr10twin (Martins-Lucaas, Apache-2.0):
# https://github.com/Martins-Lucaas/cr10twin — só a câmera, a projeção e o
# sombreamento (numpy puro). A viewport Tk e a IK foram deixadas de fora.
"""Câmera orbital, projeção e sombreamento dos triângulos do robô (numpy)."""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

_WORLD_UP = np.array([0.0, 0.0, 1.0])

# Direção da luz no frame da câmera (headlight deslocado p/ dar relevo).
_LIGHT_CAM = np.array([-0.35, 0.55, -0.75])
_AMBIENT = 0.90
_DIFFUSE = 0.20


@dataclass
class Camera:
    """Câmera orbital com projeção perspectiva."""
    az: float = math.radians(-125.0)
    el: float = math.radians(20.0)
    dist: float = 2.3
    target: np.ndarray = field(
        default_factory=lambda: np.array([0.0, 0.0, 0.45]))
    fov_deg: float = 45.0
    near: float = 0.05

    # Limites da órbita: |el| < 89° evita o gimbal com o world-up.
    EL_LIM = math.radians(89.0)
    DIST_MIN = 0.6
    DIST_MAX = 8.0

    def eye(self) -> np.ndarray:
        ce, se = math.cos(self.el), math.sin(self.el)
        return self.target + self.dist * np.array(
            [math.cos(self.az) * ce, math.sin(self.az) * ce, se])

    def basis(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Retorna (right, up, forward) unitários no frame do mundo."""
        f = self.target - self.eye()
        f = f / (np.linalg.norm(f) + 1e-12)
        r = np.cross(f, _WORLD_UP)
        n = float(np.linalg.norm(r))
        if n < 1e-9:      # olhando reto para cima/baixo — escolhe um right
            r = np.array([1.0, 0.0, 0.0])
        else:
            r = r / n
        u = np.cross(r, f)
        return r, u, f

    def focal_px(self, height_px: int) -> float:
        return 0.5 * float(height_px) / math.tan(math.radians(self.fov_deg) / 2)

    def project(self, pts: np.ndarray, w: int, h: int
                ) -> tuple[np.ndarray, np.ndarray]:
        """Projeta pontos do mundo (N,3) → (uv (N,2) px, depth (N,) m)."""
        pts = np.atleast_2d(np.asarray(pts, dtype=float))
        r, u, f = self.basis()
        rel = pts - self.eye()
        x = rel @ r
        y = rel @ u
        z = rel @ f
        fpx = self.focal_px(h)
        safe = np.where(np.abs(z) < 1e-6, 1e-6, z)
        uv = np.empty((pts.shape[0], 2), dtype=float)
        uv[:, 0] = 0.5 * w + fpx * x / safe
        uv[:, 1] = 0.5 * h - fpx * y / safe
        return uv, z

    def unproject_delta(self, du: float, dv: float, depth: float,
                        h: int) -> np.ndarray:
        """Converte um deslocamento de MOUSE (px) num deslocamento no mundo
        (m), sobre o plano paralelo à tela que está a `depth` da câmera.
        """
        r, u, _f = self.basis()
        fpx = self.focal_px(h)
        s = float(depth) / fpx
        return (du * s) * r + (-dv * s) * u

    def orbit(self, d_az: float, d_el: float) -> None:
        self.az = (self.az + d_az + math.pi) % (2 * math.pi) - math.pi
        self.el = max(-self.EL_LIM, min(self.EL_LIM, self.el + d_el))

    def zoom(self, factor: float) -> None:
        self.dist = max(self.DIST_MIN, min(self.DIST_MAX,
                                           self.dist * float(factor)))

    def pan(self, du: float, dv: float, h: int) -> None:
        self.target = self.target - self.unproject_delta(du, dv, self.dist, h)


# Geometria do braço para desenho


def scene_triangles(scene, transforms: dict) -> tuple:
    """Aplica a FK do quadro → (tris (M,3,3) no mundo, cores (M,3))."""
    if getattr(scene, 'flat_tris', None) is None:
        tri_blocks, col_blocks = [], []
        for part in scene.parts:
            T = transforms.get(part.link)
            if T is None:
                continue
            tri_blocks.append(part.tris @ T[:3, :3].T + T[:3, 3])
            col_blocks.append(np.repeat(
                np.array(part.color, dtype=np.float64)[None, :],
                part.tris.shape[0], axis=0))
        if not tri_blocks:
            return np.zeros((0, 3, 3)), np.zeros((0, 3))
        return np.concatenate(tri_blocks), np.concatenate(col_blocks)

    eye4 = np.eye(4)
    mats = np.stack([transforms.get(link, eye4)
                     for link in scene.part_links])          # (P, 4, 4)
    idx = scene.flat_part
    R = mats[idx, :3, :3]                                    # (M, 3, 3)
    t = mats[idx, :3, 3]                                     # (M, 3)
    world = np.einsum('mij,mkj->mki', R, scene.flat_tris) + t[:, None, :]
    return world, scene.flat_colors


def shade_and_sort(tris: np.ndarray, colors: np.ndarray, cam: Camera,
                   w: int, h: int, com_indices: bool = False) -> tuple:
    """Projeta, descarta o que não se vê e devolve tudo pronto para pintar.

    Returns:
        (uv (K,3,2) px, rgb (K,3) uint8) já ordenados do mais distante para
        o mais próximo — é só percorrer e preencher. Com `com_indices`, também
        o índice (K,) de cada triângulo em `tris` (para saber de quem ele é).
    """
    vazio = (np.zeros((0, 3, 2)), np.zeros((0, 3), dtype=np.uint8))
    if com_indices:
        vazio = vazio + (np.zeros(0, dtype=np.int64),)
    if tris.shape[0] == 0:
        return vazio

    eye = cam.eye()
    r, u, f = cam.basis()

    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    normals = np.cross(v1 - v0, v2 - v0)
    nn = np.linalg.norm(normals, axis=1)
    ok = nn > 1e-15
    normals = np.where(ok[:, None], normals / np.where(nn[:, None] < 1e-15,
                                                       1.0, nn[:, None]), 0.0)
    centroid = tris.mean(axis=1)
    view = eye - centroid

    # Backface culling pela normal do STL (winding CCW para fora).
    facing = np.einsum('ij,ij->i', normals, view)
    keep = ok & (facing > 0.0)
    if not np.any(keep):
        return vazio

    indices = np.flatnonzero(keep)
    tris, colors, normals = tris[keep], colors[keep], normals[keep]
    centroid = centroid[keep]

    rel = tris.reshape(-1, 3) - eye
    z = rel @ f
    # Um único vértice atrás do plano de corte invalida o triângulo: sem
    # clipping real, projetá-lo geraria um polígono espelhado atravessando
    # a tela.
    zt = z.reshape(-1, 3)
    front = np.all(zt > cam.near, axis=1)
    if not np.any(front):
        return vazio

    fpx = cam.focal_px(h)
    uv = np.empty((tris.shape[0] * 3, 2))
    uv[:, 0] = 0.5 * w + fpx * (rel @ r) / z
    uv[:, 1] = 0.5 * h - fpx * (rel @ u) / z
    uv = uv.reshape(-1, 3, 2)[front]
    colors, normals, indices = colors[front], normals[front], indices[front]
    depth = zt[front].mean(axis=1)

    # Lambert com a luz fixa NA CÂMERA: gira junto com o observador, então
    # a peça nunca fica totalmente escura ao orbitar.
    light = _LIGHT_CAM[0] * r + _LIGHT_CAM[1] * u + _LIGHT_CAM[2] * f
    light = light / (np.linalg.norm(light) + 1e-12)
    lam = np.clip(normals @ (-light), 0.0, 1.0)
    shade = (_AMBIENT + _DIFFUSE * lam)[:, None]
    rgb = np.clip(colors * shade, 0, 255).astype(np.uint8)

    order = np.argsort(-depth)          # do mais longe para o mais perto
    if com_indices:
        return uv[order], rgb[order], indices[order]
    return uv[order], rgb[order]


