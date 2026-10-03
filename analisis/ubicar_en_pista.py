"""
Ubica la trayectoria registrada por el robot (datos.txt) sobre la pista real.

1. Extrae la linea central de la pista del plano en vista superior.
2. Escala el plano con las medidas reales (99,4 cm x 198,5 cm). El dibujo NO
   esta a escala, asi que cada eje tiene su propia escala.
3. Busca el punto de salida y el sentido comparando el perfil de rumbo de los
   datos con el de la pista (todas las posiciones de salida posibles).
4. Emparejamiento con el mapa (map matching) con DTW sobre el rumbo: cada punto
   medido se lleva a su posicion sobre la linea central, conservando la
   oscilacion lateral del seguidor.

Uso:  python ubicar_en_pista.py  [datos.txt] [plano.png]
Salidas: datos_reubicados_pista.csv, datos_reubicados.txt, figuras PNG.
Requiere: numpy, scipy, opencv-python, matplotlib
"""
import sys
import numpy as np
import cv2
from scipy.ndimage import gaussian_filter1d, median_filter
from scipy.spatial import cKDTree
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ANCHO_CM, LARGO_CM = 99.4, 198.5
DATOS = sys.argv[1] if len(sys.argv) > 1 else "../datos/datos.txt"
PLANO = sys.argv[2] if len(sys.argv) > 2 else "pista_vista_superior.png"


def remuestrear(p, paso, cerrada=False):
    q = np.vstack([p, p[:1]]) if cerrada else p
    s = np.r_[0, np.cumsum(np.hypot(*np.diff(q, axis=0).T))]
    ss = np.arange(0, s[-1], paso)
    return np.c_[np.interp(ss, s, q[:, 0]), np.interp(ss, s, q[:, 1])], ss, s[-1]


def rumbo(p, sigma):
    dp = np.gradient(p, axis=0)
    return gaussian_filter1d(np.unwrap(np.arctan2(dp[:, 1], dp[:, 0])), sigma, mode="nearest")


def rot(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


# --- 1. linea central del plano -------------------------------------------------
im = cv2.imread(PLANO, 0)
b = (im < 128).astype(np.uint8) * 255
cnts, _ = cv2.findContours(b, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
ext, intr = cnts[0][:, 0, :].astype(float), cnts[1][:, 0, :].astype(float)
_, i = cKDTree(intr).query(ext)
c = (ext + intr[i]) / 2
c = np.c_[gaussian_filter1d(c[:, 0], 3, mode="wrap"), gaussian_filter1d(c[:, 1], 3, mode="wrap")]
c[:, 1] = -c[:, 1]                                  # y hacia arriba
area = 0.5 * np.sum(c[:, 0] * np.roll(c[:, 1], -1) - np.roll(c[:, 0], -1) * c[:, 1])
if area < 0:
    c = c[::-1]                                     # antihorario

# --- 2. escala real por eje -------------------------------------------------------
mn, mx = c.min(0), c.max(0)
esc = np.array([ANCHO_CM / (mx[0] - mn[0]), LARGO_CM / (mx[1] - mn[1])])
pista = ((c - mn) * esc)[::-1]                      # cm, sentido horario
P1, _, L_pista = remuestrear(pista, 1.0, True)
print(f"escala: {esc[0]:.4f} cm/px (ancho), {esc[1]:.4f} cm/px (largo); perimetro {L_pista:.0f} cm")

# --- 3. salida y sentido por perfil de rumbo -----------------------------------
d = np.loadtxt(DATOS, delimiter=",", skiprows=1)
r, ss, L_datos = remuestrear(d, 1.0)
hd = rumbo(r, 5); hd -= hd[0]
ph = rumbo(np.vstack([P1, P1]), 5)
mejor = None
for k in np.arange(0.95, 1.25, 0.01):
    for i0 in range(0, len(P1), 2):
        th = np.interp(i0 + k * ss, np.arange(len(ph)), ph); th -= th[0]
        e = np.sqrt(np.mean((hd - th) ** 2))
        if mejor is None or e < mejor[0]:
            mejor = (e, k, i0)
_, k, i0 = mejor
S = P1[i0]
print(f"salida en ({S[0]:.1f}, {S[1]:.1f}) cm del plano; rumbo RMS {np.degrees(mejor[0]):.1f} grados")
# pista en el marco del robot: origen en la salida, eje x en la direccion inicial (norte)
T = (np.roll(P1, -i0, axis=0) - S) @ rot(-np.pi / 2).T

# --- 4. map matching con DTW ----------------------------------------------------
t, su, Lt = remuestrear(T, 1.0, True)
ht = rumbo(np.vstack([t, t[:1]]), 5)[: len(t)]; ht -= ht[0]
n, m = len(hd), len(ht)
D = np.full((n + 1, m + 1), np.inf); D[0, 0] = 0
B = np.zeros((n + 1, m + 1), np.int8)
pen = 0.4
for a in range(1, n + 1):
    lo, hi = max(1, int(a * m / n) - 120), min(m, int(a * m / n) + 120)
    for j in range(lo, hi + 1):
        opts = (D[a - 1, j - 1], D[a - 1, j] + pen, D[a, j - 1] + pen)
        kk = int(np.argmin(opts)); D[a, j] = abs(hd[a - 1] - ht[j - 1]) + opts[kk]; B[a, j] = kk
a, j, camino = n, m, []
while a > 0 and j > 0:
    camino.append((a - 1, j - 1)); kk = B[a, j]
    a, j = (a - 1, j - 1) if kk == 0 else (a - 1, j) if kk == 1 else (a, j - 1)
camino = np.array(camino[::-1])
u = np.array([su[camino[camino[:, 0] == q, 1]].mean() for q in range(n)])
u = np.maximum.accumulate(gaussian_filter1d(u, 3, mode="nearest"))

Tc = np.vstack([T, T[:1]]); st = np.r_[0, np.cumsum(np.hypot(*np.diff(Tc, axis=0).T))]
def punto_pista(uu):
    uu = np.clip(uu, 0, st[-1] - 1e-6)
    f = lambda v: np.c_[np.interp(v, st, Tc[:, 0]), np.interp(v, st, Tc[:, 1])]
    tg = f(uu + 0.5) - f(uu - 0.5); tg /= np.linalg.norm(tg, axis=1)[:, None]
    return f(uu), np.c_[-tg[:, 1], tg[:, 0]]

s = np.r_[0, np.cumsum(np.hypot(*np.diff(d, axis=0).T))]
uu = np.interp(s, ss, u)
rs = np.c_[gaussian_filter1d(r[:, 0], 4, mode="nearest"), gaussian_filter1d(r[:, 1], 4, mode="nearest")]
sm = np.c_[np.interp(s, ss, rs[:, 0]), np.interp(s, ss, rs[:, 1])]
tg = np.gradient(sm, axis=0); tg /= np.linalg.norm(tg, axis=1)[:, None] + 1e-9
lat = np.sum((d - sm) * np.c_[-tg[:, 1], tg[:, 0]], axis=1)
paso = np.r_[0, np.hypot(*np.diff(d, axis=0).T)]
malo = np.zeros(len(d), bool)
for q in np.nonzero(paso > 4)[0]:
    malo[max(0, q - 8): q + 8] = True                # saltos por correccion del LiDAR
lat = median_filter(lat, 5); lat[malo] = 0; lat = np.clip(lat, -2.5, 2.5)
p, nrm = punto_pista(uu)
corr = p + nrm * lat[:, None]; corr[0] = p[0]
err = np.linalg.norm(d - p, axis=1)
print(f"error medio {err.mean():.1f} cm, maximo {err.max():.1f} cm, cierre {np.linalg.norm(d[-1]):.1f} cm")

np.savetxt("datos_reubicados_pista.csv", np.c_[np.arange(len(d)), s, d, corr, err], delimiter=",",
           header="n,s_cm,x_ekf,y_ekf,x_pista,y_pista,error_cm", comments="", fmt=["%d", "%.2f"] + ["%.4f"] * 4 + ["%.2f"])
np.savetxt("datos_reubicados.txt", corr, delimiter=",", header="x,y", comments="", fmt="%.4f")
for nombre, P, col in (("datos_sobre_pista.png", d, "#2a78d6"), ("datos_reubicados.png", corr, "#eb6834")):
    fig, ax = plt.subplots(figsize=(9, 5.5))
    ax.plot(*np.vstack([T, T[:1]]).T, color="#3a3a38", lw=7, alpha=.2, label="Pista (linea central)")
    ax.plot(P[:, 0], P[:, 1], ".-", color=col, ms=2, lw=.6, label="Trayectoria")
    ax.plot(0, 0, "o", color="#008300", ms=8, label="Salida")
    ax.set_aspect("equal"); ax.grid(alpha=.3); ax.set_xlabel("x (cm)"); ax.set_ylabel("y (cm)"); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(nombre, dpi=160); plt.close(fig)
print("listo: datos_reubicados_pista.csv, datos_reubicados.txt, datos_sobre_pista.png, datos_reubicados.png")
