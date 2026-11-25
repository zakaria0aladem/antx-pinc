import numpy as np, pandas as pd, matplotlib
matplotlib.use("Agg"); import matplotlib.pyplot as plt
from scipy.signal import find_peaks
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 12, "axes.spines.top": False, "axes.spines.right": False})
GRAY, BLUE, RED, INK = "#79828E", "#1F4E78", "#C0392B", "#1B1F24"
# ---------- check 1: known answer
T = pd.read_csv("known_answer_true.csv", index_col=0).iloc[:, 0]; K = pd.read_csv("known_answer_results.csv")
fig, axes = plt.subplots(1, 3, figsize=(11.5, 3.9), sharey=False)
fig.subplots_adjust(left=0.07, right=0.98, top=0.86, bottom=0.2, wspace=0.32)
lv = [0, 0.5, 1, 2]; rng = np.random.default_rng(1)
for ax, k, name in zip(axes, ["wn", "xi", "mu"], ["ωn (stiffness)", "ξ (damping)", "μ (motor gain)"]):
    for i, n in enumerate(lv):
        e = (K[K.noise == n][k] / T[k] - 1) * 100
        ax.scatter(i + rng.uniform(-0.12, 0.12, len(e)), e, s=22, color=BLUE if n != 1 else RED, alpha=0.8, zorder=3)
        ax.text(i, max(e.max(), 0) + 4, f"worst {np.abs(e).max():.0f}%", ha="center", fontsize=10.5)
    ax.axhline(0, color=INK, lw=1); ax.set_xticks(range(4), ["0", "½×", "1×\n(rig)", "2×"])
    ax.set_title(name, fontsize=12.5); ax.set_ylim(-60, 200 if k == "xi" else 40); ax.grid(axis="y", alpha=0.3); ax.set_axisbelow(True)
axes[0].set_ylabel("error vs chosen value [%]")
fig.text(0.53, 0.03, "noise added to the fake flight (× the rig's real noise)", ha="center", fontsize=11.5)
plt.savefig("check1_known_answer.png", dpi=220)
# ---------- check 2: free swing
D = '../../data/clean/1dof/'
S = pd.read_csv("swing_results.csv")
f = pd.read_csv(D + "impulse_1.csv"); tend = f.t[f.M > 0].iloc[-1]; g = f[f.t > tend + 0.02]
th0 = g.theta.iloc[-200:].mean(); y = (g.theta - th0).values * 57.3; t = g.t.values - tend
pk, _ = find_peaks(y, prominence=0.5); tr, _ = find_peaks(-y, prominence=0.5); ex = np.sort(np.r_[pk, tr])
fig = plt.figure(figsize=(11.5, 4.0)); gs = fig.add_gridspec(1, 3, width_ratios=[1.35, 1, 1], wspace=0.15)
fig.subplots_adjust(left=0.065, right=0.98, top=0.86, bottom=0.17)
a = fig.add_subplot(gs[0]); m = t < 4
a.plot(t[m], y[m], color=INK, lw=1.6); a.plot(t[ex[:3]], y[ex[:3]], "o", color=RED, ms=7)
a.annotate("", xy=(t[ex[2]], y[ex[0]] + 3), xytext=(t[ex[0]], y[ex[0]] + 3), arrowprops=dict(arrowstyle="<->", color=RED))
a.text((t[ex[0]] + t[ex[2]]) / 2, y[ex[0]] + 4.2, f"one swing = {t[ex[2]] - t[ex[0]]:.2f} s  →  ωn", ha="center", color=RED, fontsize=10.5)
a.text(t[ex[2]] + 0.1, y[ex[2]] + 2, f"{y[ex[0]]:.0f}° → {y[ex[2]]:.1f}°  →  ξ", color=RED, fontsize=10.5)
a.axhline(0, color=GRAY, lw=0.8); a.set_xlabel("time after the push [s]"); a.set_ylabel("pitch [deg]")
a.set_title("motors off, impulse_1: let it swing", fontsize=12); a.set_ylim(-16, 24)
rows = [("Free swing\n(no fitting, 5 flights)", S.wn.mean(), S.wn.std(), S.xi.mean(), S.xi.std(), BLUE),
        ("Simulation fit", 4.075, 0, 0.213, 0, GRAY),
        ("Inverse PINN", 4.035, 0.012, 0.161, 0.005, GRAY),
        ("Hand method\n(settling-time rule)", 4.08, 0, 0.396, 0, GRAY)]
yy = np.arange(len(rows))[::-1]
for j, (col, sc, lim, ttl) in enumerate([(1, 2, 5.2, "ωn [rad/s]"), (3, 4, 0.52, "ξ")]):
    b = fig.add_subplot(gs[j + 1]); v = [r[col] for r in rows]; e = [r[sc] for r in rows]
    b.barh(yy, v, xerr=e, color=[r[5] for r in rows], height=0.6, ecolor=INK, capsize=2.5)
    for yi, vi, ei in zip(yy, v, e): b.text(vi + ei + lim * 0.02, yi, f"{vi:.2f}" if col == 1 else f"{vi:.3f}", va="center", fontsize=10.5)
    b.set_xlim(0, lim); b.set_title(ttl, fontsize=12.5); b.grid(axis="x", alpha=0.3); b.set_axisbelow(True)
    b.set_yticks(yy, [r[0] for r in rows] if j == 0 else [""] * 4, fontsize=10.5)
plt.savefig("check2_free_swing.png", dpi=220)
