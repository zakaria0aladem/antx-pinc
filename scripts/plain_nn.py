"""
Plain neural network (no physics) for the ANT-X 2-DOF drone project (Amr's part).

Run from anywhere:   python scripts/plain_nn.py      (after scripts/baselines.py, which makes data/clean/)
Needs: numpy, pandas, matplotlib, torch (PyTorch).

Idea
  A small network learns ONE step (0.02 s) of the dynamics from examples on the train flights only.
  To predict a whole test flight it is chained: each prediction is fed back as the next input (rollout),
  exactly like the classical / SI models are simulated. No physics equation is used anywhere.

  1-DOF:  input [theta_k, q_k, M_k, M_(k-1)]  ->  output [theta_(k+1) - theta_k, q_(k+1) - q_k]
          (M_(k-1) is the command 0.02 s ago = the motor delay tau)
  2-DOF:  input [v_k, theta_k]                ->  output [v_(k+1) - v_k];  x is added up from v
          (same inputs as the SI model: pitch drives the cart)

  Data at 50 Hz (every 5th row of the 250 Hz clean files), network 2 hidden layers x 32 neurons, tanh,
  Adam (learning rate 0.003), 3000 full-batch epochs, inputs/outputs scaled to mean 0, std 1.
  Results depend a little on the random starting weights, so 5 networks (seeds 0-4) are trained and the
  mean and spread are reported; plots show seed 0.

Outputs
  results/plain_nn_scores.csv     every seed, every test flight
  results/plain_nn_summary.csv    mean and std over seeds (read by baselines.py for the report tables)
  figures/plain_nn/*.png
"""
import os, time
import numpy as np, pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import torch
from scipy.integrate import solve_ivp

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLEAN = os.path.join(ROOT, 'data', 'clean')
RES, FIG = os.path.join(ROOT, 'results'), os.path.join(ROOT, 'figures', 'plain_nn')
os.makedirs(RES, exist_ok=True); os.makedirs(FIG, exist_ok=True)
torch.set_num_threads(4)

STEP, DT = 5, 0.02            # 250 Hz -> 50 Hz; one step = 0.02 s = motor delay
SEEDS = [0, 1, 2, 3, 4]
EPOCHS, LR, WIDTH = 3000, 0.003, 32

TRAIN_1DOF = ['impulse_1', 'impulse_2', 'impulse_3', 'impulse_4']
TEST_1DOF = ['impulse_5', 'doublet_5', 'doublet_6']
TRAIN_2DOF = ['velocity_doublet_2', 'velocity_doublet_3', 'position_1', 'position_3']
TEST_2DOF = ['velocity_doublet_4', 'position_2']
EXTRA_2DOF = ['attitude_doublet_1', 'attitude_doublet_3', 'attitude_doublet_4', 'attitude_doublet_5']

# classical 1-DOF constants (hand method, from scripts/baselines.py) - only for the comparison plots
CLASSICAL = dict(wn=4.080, xi=0.396, mu=81.3, tau=0.02)


def vaf(y, yhat):
    return max(1 - np.var(np.asarray(y) - np.asarray(yhat)) / np.var(y), 0) * 100


def rmse(y, yhat):
    return float(np.sqrt(np.mean((np.asarray(y) - np.asarray(yhat)) ** 2)))


def load(kind, name):
    return pd.read_csv(os.path.join(CLEAN, kind, name + '.csv')).iloc[::STEP].reset_index(drop=True)


# ---------------------------------------------------------------- examples
def pairs_1dof(f):
    th, q, M = f.theta.values, f.q.values, f.M.values
    X = np.column_stack([th[1:-1], q[1:-1], M[1:-1], M[:-2]])
    Y = np.column_stack([th[2:] - th[1:-1], q[2:] - q[1:-1]])
    return X, Y


def pairs_2dof(f):
    v, th = f.v.values, f.theta.values
    return np.column_stack([v[:-1], th[:-1]]), (v[1:] - v[:-1])[:, None]


class Scaled:
    """Network + the scaling numbers, so predict() takes and returns real units."""
    def __init__(self, X, Y, seed):
        self.Xm, self.Xs, self.Ym, self.Ys = X.mean(0), X.std(0), Y.mean(0), Y.std(0)
        torch.manual_seed(seed)
        self.net = torch.nn.Sequential(torch.nn.Linear(X.shape[1], WIDTH), torch.nn.Tanh(),
                                       torch.nn.Linear(WIDTH, WIDTH), torch.nn.Tanh(),
                                       torch.nn.Linear(WIDTH, Y.shape[1]))
        Xt = torch.tensor((X - self.Xm) / self.Xs, dtype=torch.float32)
        Yt = torch.tensor((Y - self.Ym) / self.Ys, dtype=torch.float32)
        opt = torch.optim.Adam(self.net.parameters(), lr=LR)
        self.losses = []
        for _ in range(EPOCHS):                       # the training loop
            loss = ((self.net(Xt) - Yt) ** 2).mean()  # mean squared error on the (scaled) changes
            opt.zero_grad(); loss.backward(); opt.step()
            self.losses.append(loss.item())
        self.n_weights = sum(p.numel() for p in self.net.parameters())

    def predict(self, x):
        with torch.no_grad():
            out = self.net(torch.tensor((np.asarray(x) - self.Xm) / self.Xs, dtype=torch.float32)).numpy()
        return out * self.Ys + self.Ym


# ---------------------------------------------------------------- rollouts
def rollout_1dof(m, f):
    th, q, M = f.theta.values, f.q.values, f.M.values
    tp, qp = [th[0], th[1]], [q[0], q[1]]
    for k in range(1, len(f) - 1):
        d = m.predict([tp[k], qp[k], M[k], M[k - 1]])
        tp.append(tp[k] + d[0]); qp.append(qp[k] + d[1])
    return np.array(tp), np.array(qp)


def rollout_2dof(m, f, restart=None):
    """restart = every how many steps to reset to the measured state (None = whole flight)."""
    v, th, x = f.v.values, f.theta.values, f.x.values
    vp, xp = np.empty(len(f)), np.empty(len(f))
    for k in range(len(f)):
        if k == 0 or (restart and k % restart == 0):
            vp[k], xp[k] = v[k], x[k]; continue
        vp[k] = vp[k - 1] + m.predict([vp[k - 1], th[k - 1]])[0]
        xp[k] = xp[k - 1] + 0.5 * (vp[k - 1] + vp[k]) * DT       # position = sum of speed x time
    return xp, vp


def classical_1dof(f):
    t, M, p = f.t.values, f.M.values, CLASSICAL
    rhs = lambda tt, z: [z[1], -2 * p['xi'] * p['wn'] * z[1] - p['wn'] ** 2 * z[0]
                         + p['mu'] * np.interp(tt - p['tau'], t, M, left=0)]
    return solve_ivp(rhs, (t[0], t[-1]), [f.theta[0], f.q[0]], t_eval=t, max_step=0.004).y[0]


# ---------------------------------------------------------------- main
def main():
    rows = []
    F1 = {n: load('1dof', n) for n in TRAIN_1DOF + TEST_1DOF}
    F2 = {n: load('2dof', n) for n in TRAIN_2DOF + TEST_2DOF + EXTRA_2DOF}
    X1 = np.vstack([pairs_1dof(F1[n])[0] for n in TRAIN_1DOF]); Y1 = np.vstack([pairs_1dof(F1[n])[1] for n in TRAIN_1DOF])
    X2 = np.vstack([pairs_2dof(F2[n])[0] for n in TRAIN_2DOF]); Y2 = np.vstack([pairs_2dof(F2[n])[1] for n in TRAIN_2DOF])
    print(f'examples: 1-DOF {len(X1)}, 2-DOF {len(X2)}')

    for seed in SEEDS:
        t0 = time.time()
        m1 = Scaled(X1, Y1, seed)
        for n in TEST_1DOF:
            f = F1[n]; th, q = rollout_1dof(m1, f)
            rows.append(dict(case='1-DOF', test_flight=n, seed=seed, VAF_pitch=vaf(f.theta, th), VAF_pitch_rate=vaf(f.q, q),
                             RMSE_pitch_deg=np.degrees(rmse(f.theta, th))))
            if seed == 0:
                th_c = classical_1dof(f)
                plt.figure(figsize=(10, 3.8))
                plt.plot(f.t, np.degrees(f.theta), 'k', lw=2, label='measured')
                plt.plot(f.t, np.degrees(th_c), label=f'classical hand method (VAF {vaf(f.theta, th_c):.1f} %)')
                plt.plot(f.t, np.degrees(th), '--', label=f'plain NN (VAF {vaf(f.theta, th):.1f} %)')
                plt.xlabel('t [s]'); plt.ylabel('pitch theta [deg]'); plt.legend(fontsize=8)
                plt.title(f'1-DOF test flight {n}: rollout, never seen in training (50 Hz)')
                plt.tight_layout(); plt.savefig(os.path.join(FIG, f'1dof_{n}.png'), dpi=120); plt.close()
        m2 = Scaled(X2, Y2, seed)
        for n in TEST_2DOF + EXTRA_2DOF:
            f = F2[n]; xp, vp = rollout_2dof(m2, f); xs, vs = rollout_2dof(m2, f, restart=int(round(5 / DT)))
            rows.append(dict(case='2-DOF' + (' (extra test)' if n in EXTRA_2DOF else ''), test_flight=n, seed=seed,
                             VAF_velocity=vaf(f.v, vp), VAF_position=vaf(f.x, xp), VAF_position_5s_restart=vaf(f.x, xs),
                             RMSE_position_m=rmse(f.x, xp), RMSE_velocity=rmse(f.v, vp)))
            if seed == 0 and n in TEST_2DOF:
                fig, ax = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
                ax[0].plot(f.t, f.x, 'k', lw=2, label='measured')
                ax[0].plot(f.t, xp, '--', label=f'plain NN (VAF {vaf(f.x, xp):.1f} %)')
                ax[0].plot(f.t, xs, ':', label=f'plain NN, restarted every 5 s (VAF {vaf(f.x, xs):.1f} %)')
                ax[1].plot(f.t, f.v, 'k', lw=2); ax[1].plot(f.t, vp, '--', label=f'plain NN (VAF {vaf(f.v, vp):.1f} %)')
                ax[0].set(ylabel='x [m]', title=f'2-DOF test flight {n}: plain NN rollout from measured pitch')
                ax[1].set(ylabel='v [m/s]', xlabel='t [s]'); ax[0].legend(fontsize=8); ax[1].legend(fontsize=8)
                plt.tight_layout(); plt.savefig(os.path.join(FIG, f'2dof_{n}.png'), dpi=120); plt.close()
        if seed == 0:
            plt.figure(figsize=(7, 3.5))
            plt.semilogy(m1.losses, label=f'1-DOF ({m1.n_weights} weights)'); plt.semilogy(m2.losses, label=f'2-DOF ({m2.n_weights} weights)')
            plt.xlabel('epoch'); plt.ylabel('training loss (scaled MSE)'); plt.legend(); plt.title('Plain NN training, seed 0')
            plt.tight_layout(); plt.savefig(os.path.join(FIG, 'training_loss.png'), dpi=120); plt.close()
        print(f'seed {seed} done ({time.time() - t0:.0f} s)')

    S = pd.DataFrame(rows)
    S.to_csv(os.path.join(RES, 'plain_nn_scores.csv'), index=False, float_format='%.3f')
    cols = [c for c in S.columns if c.startswith(('VAF', 'RMSE'))]
    G = S.groupby(['case', 'test_flight'], sort=False)[cols].agg(['mean', 'std'])
    G.columns = [f'{a}_{b}' for a, b in G.columns]
    G = G.dropna(axis=1, how='all').reset_index()
    G.to_csv(os.path.join(RES, 'plain_nn_summary.csv'), index=False, float_format='%.3f')
    pd.set_option('display.width', 220)
    print(G.round(2).to_string())


if __name__ == '__main__':
    main()
