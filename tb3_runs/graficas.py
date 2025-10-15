# -*- coding: utf-8 -*-
"""
Overlay de trayectorias por controlador + referencia cuadrada (1.75 m por lado).
Lee todos los CSV en DATA_DIR, detecta columnas x,y por alias e infiere el controlador
desde el nombre del archivo. Dibuja la referencia (cuadrado) y superpone hasta
LIMIT_TRAJ_OVERLAYS trayectorias por controlador.

Requisitos: pandas, numpy, matplotlib
"""

import os
import re
import glob
from typing import Dict, List, Optional

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ============================
# CONFIGURACIÓN
# ============================
DATA_DIR = r"C:\Users\Azziz Youssef\OneDrive\Documentos\Trabajo de grado\turtlebot3_trayectory\tb3_runs"
SAVE_FIGS = True
FIG_DIR = "./figs"
LIMIT_TRAJ_OVERLAYS = 30  # Aumenta para mostrar más corridas por controlador

# Inferencia del controlador desde el nombre del archivo
CONTROLLER_HINTS = {
    r"pid": "PID",
    r"lyapunov|lyap": "Lyapunov",
    r"pure[_\- ]?pursuit|pp|persecucion": "PurePursuit",
    r"\bmpc\b|model.*predictive": "MPC",
}

# ============================
# ALIAS Y LECTURA
# ============================
def _norm(s: str) -> str:
    s = s.strip().lower()
    s = (s
         .replace("á","a").replace("é","e").replace("í","i")
         .replace("ó","o").replace("ú","u").replace("ñ","n"))
    s = re.sub(r"[\s\-]+", "_", s)
    s = re.sub(r"[^a-z0-9_]", "", s)
    return s

ALIASES = {
    "x": ["x","posx","x_m","position_x","pose_x","robot_x"],
    "y": ["y","posy","y_m","position_y","pose_y","robot_y"],
}

def prepare_dataframe(df: pd.DataFrame) -> pd.DataFrame:
    if df is None or df.empty:
        return pd.DataFrame()
    # normaliza encabezados
    new_cols = {}
    used = set()
    for c in df.columns:
        nc = _norm(c)
        base = nc; k = 2
        while nc in used:
            nc = f"{base}_{k}"; k += 1
        used.add(nc)
        new_cols[c] = nc
    df2 = df.rename(columns=new_cols)

    # mapea a x,y
    def map_one(key):
        for cand in ALIASES[key]:
            nc = _norm(cand)
            for col in df2.columns:
                if nc == col or (nc in col and len(nc) >= 2):
                    return col
        return None

    xcol, ycol = map_one("x"), map_one("y")
    keep = {}
    if xcol: keep["x"] = df2[xcol]
    if ycol: keep["y"] = df2[ycol]
    return pd.DataFrame(keep)

def load_csv_auto(path: str) -> pd.DataFrame:
    try:
        df = pd.read_csv(path, sep=None, engine="python")
    except Exception:
        df = pd.read_csv(path, sep=";")
    return prepare_dataframe(df)

def infer_controller_from_name(fname: str) -> Optional[str]:
    base = os.path.basename(fname).lower()
    for pat, lab in CONTROLLER_HINTS.items():
        if re.search(pat, base):
            return lab
    return "UNKNOWN"

# ============================
# REFERENCIA: CUADRADO 1.75 m
# ============================
def square_reference(side: float = 1.75, start=(0.0, 0.0), pts_per_edge: int = 200) -> pd.DataFrame:
    """Genera un polilíneo cuadrado (cerrado) de lado 'side' empezando en 'start'."""
    x0, y0 = start
    corners = np.array([
        [x0,         y0        ],
        [x0 + side,  y0        ],
        [x0 + side,  y0 + side ],
        [x0,         y0 + side ],
        [x0,         y0        ]  # cierre
    ], dtype=float)

    # interpolación lineal en cada lado para un trazo suave
    xs, ys = [], []
    for i in range(len(corners)-1):
        p0, p1 = corners[i], corners[i+1]
        t = np.linspace(0.0, 1.0, pts_per_edge, endpoint=True)
        xs.append(p0[0] + (p1[0]-p0[0]) * t)
        ys.append(p0[1] + (p1[1]-p0[1]) * t)
    x = np.concatenate(xs); y = np.concatenate(ys)
    return pd.DataFrame({"x": x, "y": y})

# ============================
# PLOT OVERLAY
# ============================
def plot_trajectories_overlay(data_dir: str):
    files = sorted(glob.glob(os.path.join(data_dir, "*.csv")))
    if not files:
        print(f"[AVISO] No se encontraron CSV en {data_dir}")
        return

    # agrupa archivos por controlador (según nombre)
    by_ctrl: Dict[str, List[str]] = {}
    for f in files:
        ctrl = infer_controller_from_name(f)
        by_ctrl.setdefault(ctrl, []).append(f)

    # referencia cuadrada
    ref_df = square_reference(side=1.75, start=(0.0, 0.0), pts_per_edge=250)

    # un plot por controlador
    os.makedirs(FIG_DIR, exist_ok=True)
    for ctrl, filelist in by_ctrl.items():
        if not filelist:
            continue
        plt.figure(figsize=(6, 6))

        # referencia en negro discontinuo
        plt.plot(ref_df["x"], ref_df["y"], "k--", linewidth=2, label="Referencia (cuadrado 1.75 m)")

        # superpone hasta LIMIT_TRAJ_OVERLAYS trayectorias
        count = 0
        for f in filelist[:LIMIT_TRAJ_OVERLAYS]:
            df = load_csv_auto(f)
            if {"x","y"}.issubset(df.columns) and not df.empty:
                plt.plot(df["x"], df["y"], alpha=0.6)
                count += 1

        plt.axis("equal")
        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title(f"Trayectorias superpuestas - {ctrl} (n={count})")
        plt.grid(True, alpha=0.3)
        plt.legend(loc="best")

        if SAVE_FIGS:
            safe = re.sub(r"[^a-zA-Z0-9_]+", "_", ctrl)
            plt.savefig(os.path.join(FIG_DIR, f"traj_overlay_{safe}.png"), dpi=180, bbox_inches="tight")

    plt.show()
    print(f"[LISTO] Figuras guardadas en: {FIG_DIR}")

# ============================
# MAIN
# ============================
if __name__ == "__main__":
    plot_trajectories_overlay(DATA_DIR)
