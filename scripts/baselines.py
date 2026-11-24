"""
Baselines for the ANT-X 2-DOF drone project (Amr's part).

Run from anywhere:   python scripts/baselines.py
Needs only: numpy, scipy, pandas, matplotlib.

What it does
  B  clean     cut each flight to the part we model, remove angle offsets, check NaN / time jumps
               -> data/clean/1dof/*.csv, data/clean/2dof/*.csv, data/clean/clean_summary.csv
  C  split     which flights train, which test -> split.txt
  D  models    1-DOF classical model (ANT-X published constants, and the ANT-X hand method
               applied to our flights); 2-DOF SI fit (kbar, cbar by solve_ivp + least_squares)
  E  test      rollout (whole flight from the first state, no correction) and one-step-ahead
               on the unseen test flights -> figures/baselines/*.png
  F  scores    VAF, RMSE -> results/baseline_vaf.csv, results/baseline_constants.csv,
               results/baseline_rmse.csv, results/baselines_summary.md

Models
  1-DOF (Panza 2020):  theta' = q,  q' = -2 xi wn q - wn^2 theta + mu M(t - tau)
  2-DOF (Panza 2021):  x' = v,      v' = -cbar v - kbar theta      (theta = measured pitch)
"""
import os, json
import numpy as np, pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares
from scipy.signal import find_peaks, savgol_filter

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RAW1, RAW2 = os.path.join(ROOT, 'data', '1dof'), os.path.join(ROOT, 'data', '2dof')
CLEAN = os.path.join(ROOT, 'data', 'clean')
RES, FIG = os.path.join(ROOT, 'results'), os.path.join(ROOT, 'figures', 'baselines')
for d in [os.path.join(CLEAN, '1dof'), os.path.join(CLEAN, '2dof'), RES, FIG]:
    os.makedirs(d, exist_ok=True)

DT = 0.004                  # 250 Hz grid of the csv files
TAU = 0.020                 # command -> motor delay [s] (ANT-X)
IMP_WIN = 6.0               # impulse window: 0.1 s before the push + 6 s (same as the PINN notebook)
DBL_WIN = 1.5               # doublet window: 0.1 s before + 1.4 s of open loop (autopilot takes over after)

# published ANT-X constants
ANTX_1DOF = dict(wn=3.437, xi=0.0836, mu=358.5)       # Panza 2020, 1-DOF pitch
ANTX_2DOF = dict(kbar=4.748, cbar=1.168)              # Panza 2021, v1.6 material (normalised thrust 0.35)

# ---------------------------------------------------------------- split (step C)
SPLIT = {
    '1dof_impulse': dict(train=['impulse_1', 'impulse_2', 'impulse_3', 'impulse_4'], test=['impulse_5']),
    '1dof_doublet': dict(train=['doublet_1', 'doublet_2', 'doublet_3', 'doublet_4'], test=['doublet_5', 'doublet_6']),
    '2dof':         dict(train=['velocity_doublet_2', 'velocity_doublet_3', 'position_1', 'position_3'],
                         test=['velocity_doublet_4', 'position_2'],
                         extra_test=['attitude_doublet_1', 'attitude_doublet_3', 'attitude_doublet_4', 'attitude_doublet_5']),
}
NOT_USED = {'attitude_doublet_2': 'x does not move', 'velocity_doublet_1': 'no flight',
            'prbs_1': 'x does not move (pitch only)', 'prbs_2': 'x does not move (pitch only)',
            'prbs_3': 'x does not move (pitch only)'}


# ---------------------------------------------------------------- scores
def vaf(y, yhat):
    """Variance Accounted For: 100 % = perfect match, 0 % = no better than a flat line."""
    y, yhat = np.asarray(y), np.asarray(yhat)
    return max(1 - np.var(y - yhat) / np.var(y), 0) * 100


def rmse(y, yhat):
    return float(np.sqrt(np.mean((np.asarray(y) - np.asarray(yhat)) ** 2)))


def smooth_derivative(y, dt=DT, window=25):
    """Derivative of a noisy signal: Savitzky-Golay filter (fits a small polynomial in a 0.1 s window)."""
    window = min(window, len(y) - (1 - len(y) % 2))
    return savgol_filter(y, window, 3, deriv=1, delta=dt)


# ================================================================ step B: clean
def check(df, name):
    """NaN count and biggest time step (a jump > 1.5 dt means missing samples)."""
    return dict(file=name, nan=int(df.isna().sum().sum()), max_dt=round(float(np.diff(df.t).max()), 4),
                time_jump=bool(np.diff(df.t).max() > 1.5 * DT))


def clean_1dof():
    FL = pd.read_csv(os.path.join(RAW1, 'flights_1dof.csv'))
    FL['name'] = FL.file.str.replace('.csv', '', regex=False)
    flights, rows = {}, []
    for _, r in FL.iterrows():
        df = pd.read_csv(os.path.join(RAW1, r.file))
        rows.append(check(df, r['name']))
        t0 = r.inj_start_s - 0.1
        win = IMP_WIN if r.kind == 'impulse' else DBL_WIN
        pre = df[(df.t >= r.inj_start_s - 0.5) & (df.t < r.inj_start_s)]          # 0.5 s before the push
        f = df[(df.t >= t0) & (df.t < t0 + win)].reset_index(drop=True).copy()
        f['t'] = f.t - f.t[0]
        # offsets: rest angle and trim moment before the push (motors off -> M trim is 0)
        theta0, q0 = pre.theta.mean(), pre.q.mean()
        M0 = 0.0 if r.kind == 'impulse' else pre.M.mean()
        f['theta'] = f.theta - theta0
        f['q'] = f.q - q0
        f['M'] = f.M - M0
        f = f[['t', 'theta', 'q', 'M', 'T', 'm_inj', 'openloop']]
        f.to_csv(os.path.join(CLEAN, '1dof', r.file), index=False, float_format='%.6g')
        rows[-1].update(kind=r.kind, window_start_s=round(t0, 3), window_s=win, theta_offset_deg=round(np.degrees(theta0), 3),
                        M_trim=round(M0, 4), samples=len(f))
        flights[r['name']] = f
    return flights, rows


def clean_2dof():
    flights, rows = {}, []
    for name in SPLIT['2dof']['train'] + SPLIT['2dof']['test'] + SPLIT['2dof']['extra_test']:
        df = pd.read_csv(os.path.join(RAW2, name + '.csv'))
        rows.append(check(df, name))
        fly = np.nonzero(df['T'].values > 0.2)[0]                                   # motors at flight thrust
        t_on, t_off = df.t[fly[0]] + 1.0, df.t[fly[-1]] - 1.0                       # skip 1 s of take-off / landing
        f = df[(df.t >= t_on) & (df.t <= t_off)].reset_index(drop=True).copy()
        f['t'] = f.t - f.t[0]
        # level offset: median pitch while the cart is standing still (|v| < 1 cm/s)
        still = np.abs(f.v) < 0.01
        theta0 = f.theta[still].median()
        f['theta'] = f.theta - theta0
        f['theta_sp'] = f.theta_sp - theta0
        f = f[['t', 'theta', 'q', 'x', 'v', 'M', 'T', 'theta_sp', 'x_sp', 'v_sp']]
        f.to_csv(os.path.join(CLEAN, '2dof', name + '.csv'), index=False, float_format='%.6g')
        rows[-1].update(kind=name.rsplit('_', 1)[0], window_start_s=round(t_on, 2), window_s=round(t_off - t_on, 1),
                        theta_offset_deg=round(np.degrees(theta0), 3), M_trim='', samples=len(f))
        flights[name] = f
    return flights, rows


def write_split(clean_rows):
    dur = {r['file']: r['window_s'] for r in clean_rows}
    L = ['# Train / test split, shared with Zakaria (PINC and plain NN train on exactly the same flights).',
         '# Clean files: data/clean/1dof/<name>.csv and data/clean/2dof/<name>.csv (made by scripts/baselines.py).',
         '# Windows: impulse = 0.1 s before the push + 6 s; doublet = 0.1 s before + 1.4 s open loop;',
         '#          2-DOF = whole flight at flight thrust minus 1 s at each end.', '']
    for key, s in SPLIT.items():
        for part in ['train', 'test', 'extra_test']:
            if part in s:
                L.append(f'{key:13s} {part:10s} ' + ', '.join(f'{n} ({dur[n]} s)' for n in s[part]))
    L += ['', '# Not used:'] + [f'#   {k}: {v}' for k, v in NOT_USED.items()]
    open(os.path.join(ROOT, 'split.txt'), 'w').write('\n'.join(L) + '\n')


# ================================================================ step D: 1-DOF classical model
def rhs_1dof(p, t_in, M_in):
    def f(t, z):
        M = np.interp(t - TAU, t_in, M_in, left=0.0)        # delayed input, read from the flight
        return [z[1], -2 * p['xi'] * p['wn'] * z[1] - p['wn'] ** 2 * z[0] + p['mu'] * M]
    return f


def simulate_1dof(p, f):
    """Rollout: start from the first measured state, feed the measured M, no correction. -> theta, q"""
    t = f.t.values
    sol = solve_ivp(rhs_1dof(p, t, f.M.values), (t[0], t[-1]), [f.theta[0], f.q[0]], t_eval=t, max_step=DT)
    return sol.y[0], sol.y[1]


def rk4_step(fun, t, z, h):
    k1 = np.array(fun(t, z)); k2 = np.array(fun(t + h / 2, z + h / 2 * k1))
    k3 = np.array(fun(t + h / 2, z + h / 2 * k2)); k4 = np.array(fun(t + h, z + h * k3))
    return z + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def one_step_1dof(p, f):
    """One step ahead: from the MEASURED state at each sample, predict the next sample (4 ms later)."""
    t, Z = f.t.values, f[['theta', 'q']].values
    fun = rhs_1dof(p, t, f.M.values)
    pred = np.array([rk4_step(fun, t[k], Z[k], DT) for k in range(len(t) - 1)])
    return pred[:, 0], pred[:, 1]                           # predictions for samples 1..N-1


def free_response(f):
    """ANT-X hand method, step 1: period and settling time of the free swing after the push."""
    t, th = f.t.values, f.theta.values
    push_end = t[np.nonzero(f.M.values > 0.05)[0][-1]] + TAU
    after = t > push_end
    pk, _ = find_peaks(th[after], prominence=np.radians(0.5))       # maxima of the swing
    tp = t[after][pk]
    T_osc = float(np.mean(np.diff(tp[:3]))) if len(tp) >= 2 else np.nan   # period from the first maxima
    # settling time: last time theta is more than 1 % of the swing away from where it finally rests
    # (4.6 / (xi wn) is the 1 % envelope). Bearing friction stops the swing at a slightly different
    # angle than it started from, so the final rest angle is the mean of the last second of the window.
    rest = th[t > t[-1] - 1.0].mean()
    amp = np.abs(th[after] - rest).max()
    above = np.nonzero(np.abs(th[after] - rest) > 0.01 * amp)[0]
    Ts = t[after][above[-1]] - push_end
    wn = 2 * np.pi / T_osc          # ANT-X: TP = 2 pi / (wn sqrt(1 - xi^2)) ~ 2 pi / wn (damping neglected)
    xi = 4.6 / (Ts * wn)            # ANT-X: TS ~ 4.6 / (xi wn)
    # cross-check: logarithmic decrement of successive maxima
    xi_ld = np.nan
    if len(pk) >= 2:
        d = np.log(th[after][pk[0]] / th[after][pk[1]])
        xi_ld = d / np.sqrt(4 * np.pi ** 2 + d ** 2)
    return dict(T_osc=T_osc, Ts=Ts, wn=wn, xi=xi, xi_logdec=xi_ld, peaks_t=tp, push_end=push_end, rest=rest)


def gain_from_peak(f, wn, xi):
    """ANT-X hand method, step 2: simulate with mu = 1, then mu = measured peak q / simulated peak q.

    The unit-gain simulation starts from rest (zero state), so only the response to the push is
    compared. Starting it from the measured state lets the tiny initial wobble (about as large as
    the mu = 1 response) distort the ratio: per-flight mu then spread from 40 to 101 instead of 78-86.
    """
    t = f.t.values
    sol = solve_ivp(rhs_1dof(dict(wn=wn, xi=xi, mu=1.0), t, f.M.values), (t[0], t[-1]), [0.0, 0.0],
                    t_eval=t, max_step=DT)
    q1 = sol.y[1]
    first = np.nonzero(np.abs(f.M.values) > 1e-3)[0][0]
    sign = np.sign(f.M.values[first])
    i_end = min(len(q1), first + int(0.6 / DT))                   # first lobe of the response
    if sign > 0:
        return f.q.values[:i_end].max() / q1[:i_end].max()
    return f.q.values[:i_end].min() / q1[:i_end].min()


# ================================================================ step D: 2-DOF SI fit
def simulate_2dof(p, f):
    """Rollout of x, v with the measured (offset-free) theta as input. -> x, v"""
    t, th = f.t.values, f.theta.values
    def rhs(tt, z):
        return [z[1], -p['cbar'] * z[1] - p['kbar'] * np.interp(tt, t, th)]
    sol = solve_ivp(rhs, (t[0], t[-1]), [f.x[0], f.v[0]], t_eval=t, max_step=DT)
    return sol.y[0], sol.y[1]


def segment_rollout_2dof(p, f, seg=5.0):
    """Rollout restarted from the measured state every `seg` seconds (removes the slow creep). -> x, v"""
    n, x, v = int(round(seg / DT)), np.empty(len(f)), np.empty(len(f))
    for i0 in range(0, len(f), n):
        s = f.iloc[i0:i0 + n].reset_index(drop=True)
        if len(s) < 2:
            x[i0:], v[i0:] = s.x.values, s.v.values; continue
        x[i0:i0 + len(s)], v[i0:i0 + len(s)] = simulate_2dof(p, s)
    return x, v


def one_step_2dof(p, f):
    t, th, Z = f.t.values, f.theta.values, f[['x', 'v']].values
    fun = lambda tt, z: np.array([z[1], -p['cbar'] * z[1] - p['kbar'] * np.interp(tt, t, th)])
    pred = np.array([rk4_step(fun, t[k], Z[k], DT) for k in range(len(t) - 1)])
    return pred[:, 0], pred[:, 1]


def fit_2dof(flights, names):
    """kbar, cbar so that the simulated velocity matches the measured velocity on all train flights at once.

    Why velocity and not position: the ANT-X model is pitch -> velocity (Gv = -kbar / (s + cbar)).
    When the drone holds position the cart is stuck by static friction although pitch is 1-3 deg off
    level; the linear model cannot stop, so the simulated x creeps away over a 25-45 s flight.
    Fitting on x lets that creep dominate and pushes kbar towards 0 (we tried: kbar 1.25, cbar 3.29,
    test VAF on v only 24 %). Fitting on v gives a model that reproduces the motion (test VAF on v 64-82 %).
    """
    def residual(v):
        p = dict(kbar=v[0], cbar=v[1])
        return np.concatenate([simulate_2dof(p, flights[n])[1] - flights[n].v.values for n in names])
    r = least_squares(residual, [ANTX_2DOF['kbar'], ANTX_2DOF['cbar']], bounds=([0, 0], [50, 20]))
    J = r.jac; s2 = 2 * r.cost / max(1, len(r.fun) - 2)
    std = np.sqrt(np.diag(np.linalg.inv(J.T @ J) * s2))        # rough 1-sigma from the fit
    return dict(kbar=r.x[0], cbar=r.x[1], kbar_std=std[0], cbar_std=std[1])


# ================================================================ report tables
def write_summary(V, R, fr, wn_h, xi_h, mu_imp, mu_dbl_each, fit):
    g = lambda case, flight, model, col: V[(V.case == case) & (V.test_flight == flight) & (V.model == model)][col].iloc[0]
    xi_ld = np.mean([v['xi_logdec'] for v in fr.values()])
    L = ['# Baseline results (made by scripts/baselines.py)', '',
         'All scores are on flights the models never saw. VAF = max(1 - var(y - y_hat)/var(y), 0) x 100 %, 100 % = perfect.',
         'Rollout = simulate the whole test window from its first measured state, with the measured input, no correction.', '',
         '## Table 1a. 1-DOF pitch, test flights (rollout VAF %; RMSE of pitch in deg over the whole window)', '',
         '| Model | impulse_5: pitch VAF | pitch rate VAF | pitch RMSE [deg] | doublet_5: pitch VAF | doublet_6: pitch VAF |',
         '|---|---|---|---|---|---|']
    rm = lambda flight, m: R[(R.case == '1-DOF') & (R.test_flight == flight) & (R.model == m)]['RMSE_whole_flight'].iloc[0]
    for m in ['classical, ANT-X published', 'classical, hand method (ours)']:
        L.append(f"| {m} | {g('1-DOF','impulse_5',m,'VAF_pitch'):.1f} | {g('1-DOF','impulse_5',m,'VAF_pitch_rate'):.1f} | "
                 f"{rm('impulse_5', m):.2f} | {g('1-DOF','doublet_5',m,'VAF_pitch'):.1f} | {g('1-DOF','doublet_6',m,'VAF_pitch'):.1f} |")
    NN = None
    nn_file = os.path.join(RES, 'plain_nn_summary.csv')
    if os.path.exists(nn_file):
        NN = pd.read_csv(nn_file).set_index('test_flight')
        pm = lambda fl, c: f"{NN.loc[fl, c + '_mean']:.1f} +- {NN.loc[fl, c + '_std']:.1f}"
        L.append(f"| plain NN (PyTorch, mean +- std of 5 seeds) | {pm('impulse_5','VAF_pitch')} | {pm('impulse_5','VAF_pitch_rate')} | "
                 f"{NN.loc['impulse_5','RMSE_pitch_deg_mean']:.2f} | {pm('doublet_5','VAF_pitch')} | {pm('doublet_6','VAF_pitch')} |")
    else:
        L.append('| plain NN | | | | | |')
    L += ['| simulation fit (Zakaria, notebook batch 1-2) | 97.0 | | | | |',
          '| inverse PINN (Zakaria, notebook batch 1-2) | 94.7 | | | | |',
          '| PINC (Zakaria) | | | | | |', '',
          '## Table 1b. 2-DOF position, test flights (VAF %)', '',
          '| Model | vel_doublet_4: x | x (5 s restart) | v | acc. | position_2: x | x (5 s restart) | v | acc. |', '|---|---|---|---|---|---|---|---|---|']
    for m in ['ANT-X published', 'SI fit (ours)']:
        c = []
        for fl in ['velocity_doublet_4', 'position_2']:
            c += [g('2-DOF', fl, m, k) for k in ['VAF_position', 'VAF_position_5s_restart', 'VAF_velocity', 'VAF_acc']]
        L.append(f'| {m} | ' + ' | '.join(f'{x:.1f}' for x in c) + ' |')
    if NN is not None:
        c = []
        for fl in ['velocity_doublet_4', 'position_2']:
            c += [f"{NN.loc[fl, k + '_mean']:.1f} +- {NN.loc[fl, k + '_std']:.1f}" for k in ['VAF_position', 'VAF_position_5s_restart', 'VAF_velocity']] + ['-']
        L.append('| plain NN (PyTorch, mean +- std of 5 seeds) | ' + ' | '.join(c) + ' |')
    else:
        L.append('| plain NN | | | | | | | | |')
    L += ['| PINC (Zakaria) | | | | | | | | |', '',
          'Extra 2-DOF check on the 4 attitude-doublet flights (not in the split, never used for fitting), mean VAF:', '']
    ex = V[V.case == '2-DOF (extra test)'].groupby('model')[['VAF_position', 'VAF_position_5s_restart', 'VAF_velocity', 'VAF_acc']].mean()
    L += ['| Model | x | x (5 s restart) | v | acc. |', '|---|---|---|---|---|']
    L += [f'| {m} | ' + ' | '.join(f'{x:.1f}' for x in r) + ' |' for m, r in ex.iterrows()]
    if NN is not None:
        e = NN.loc[['attitude_doublet_1', 'attitude_doublet_3', 'attitude_doublet_4', 'attitude_doublet_5']]
        L.append('| plain NN (mean of 5 seeds) | ' + ' | '.join(f"{e[k + '_mean'].mean():.1f}" for k in
                 ['VAF_position', 'VAF_position_5s_restart', 'VAF_velocity']) + ' | - |')
    L += ['', '## Table 2. Constants', '',
          '| Constant | ANT-X published | Classical hand method (ours) | Simulation fit (Zakaria) | Inverse PINN (Zakaria) | PINC (Zakaria) |',
          '|---|---|---|---|---|---|',
          f'| wn [rad/s] | 3.437 | {wn_h:.3f} | 4.075 | 4.035 | |',
          f'| xi [-] | 0.0836 | {xi_h:.3f} (log-decrement check: {xi_ld:.3f}) | 0.213 | 0.161 | |',
          f'| mu [rad/s^2] | 358.5 | {mu_imp:.1f} | 78.3 | 65.5 | |',
          f'| tau [s] | 0.020 | 0.020 (fixed) | 0.020 | 0.020 | |', '',
          '| Constant | ANT-X published | SI fit (ours) | PINC (Zakaria) |', '|---|---|---|---|',
          f"| kbar [m/s^2/rad] | 4.748 | {fit['kbar']:.3f} (+-{fit['kbar_std']:.3f}) | |",
          f"| cbar [1/s] | 1.168 | {fit['cbar']:.3f} (+-{fit['cbar_std']:.3f}) | |", '',
          '## RMSE of the rollout over its first 1, 5, 10 s', '',
          '| Case | Test flight | Model | signal | first 1 s | first 5 s | first 10 s | whole window |', '|---|---|---|---|---|---|---|---|']
    for _, r in R[R.case.isin(['1-DOF', '2-DOF'])].iterrows():
        L.append(f"| {r.case} | {r.test_flight} | {r.model} | {r.signal} | " +
                 ' | '.join('-' if pd.isna(r[f'RMSE_first_{h}s']) else f"{r[f'RMSE_first_{h}s']:.3f}" for h in [1, 5, 10]) + f" | {r['RMSE_whole_flight']:.3f} |")
    L += ['', '## Notes for the report', '',
          '- ANT-X published constants give 0 % VAF on our rig: they come from another ANT-X drone, with motors armed, '
          'and use a different scaling of the moment command M (their mu is about 5x ours). Our rig needs its own constants.',
          '- Hand method: the swing period gives wn almost exactly equal to the full simulation fit. The settling-time rule '
          'overestimates damping (bearing friction stops the swing early, at a slightly different angle), which is why the '
          'classical model under-predicts the second swing.',
          '- Hand method, mu: peak matching on the impulse train flights gives a consistent mu (per flight values in '
          'results/baseline_constants.csv). The same peak matching on the hover doublets gives estimates from about 90 to 500 '
          f"({', '.join(f'{m:.0f}' for m in mu_dbl_each)}), so it is not used: one classical model, all constants from the "
          'impulse flights, is tested on every flight.',
          '- 1-DOF doublets are hovering flights (motors at hover thrust). They react differently from the motors-off impulse '
          'flights, so all models score lower there.',
          '- 2-DOF: the model is fitted on velocity, because it is a pitch -> velocity model (Gv = -kbar/(s + cbar)). When the '
          'drone holds position, static friction holds the cart although pitch is 1-3 deg off level; the linear model cannot '
          'stop, so simulated position creeps over a 25-45 s flight. That is why whole-flight position VAF is low and the '
          '"5 s restart" column (simulation restarted from the measured state every 5 s) is also given.',
          '- One-step-ahead predictions (4 ms) are almost perfect for every model (VAF > 99 %), so they do not separate the models; '
          'see results/baseline_vaf.csv.',
          '- Plain NN (scripts/plain_nn.py): PyTorch, 2 x 32 tanh, learns one 0.02 s step from the train flights only and is '
          'rolled out by feeding its predictions back. Scored at 50 Hz (the classical rows at 250 Hz; that changes VAF by < 1 point). '
          'It beats the classical model on the impulse flight (it also learns the bearing friction the linear equation lacks), '
          'but on the hover doublets, a situation it never saw, its first peak is about twice too high and results vary between '
          'seeds. VAF hides such steady offsets (it ignores a constant error), so check the plots and the RMSE too.',
          '- 2-DOF plain NN: on position_2 it learns the stiction (the cart stays put while pitch is slightly off level), so '
          'velocity and position are much better than the SI fit. On velocity_doublet_4 it over-predicts one speed block, '
          'and whole-flight position drifts like the SI model.',
          '- Published kbar is for normalised hover thrust 0.35; our flights hover at about 0.41, which partly explains the '
          'difference (kbar is proportional to thrust).']
    open(os.path.join(RES, 'baselines_summary.md'), 'w').write('\n'.join(L) + '\n')


# ================================================================ main
def main():
    # ---------- B, C
    F1, rows1 = clean_1dof()
    F2, rows2 = clean_2dof()
    summary = pd.DataFrame(rows1 + rows2)
    summary.to_csv(os.path.join(CLEAN, 'clean_summary.csv'), index=False)
    write_split(rows1 + rows2)
    print('clean:', len(F1), '1-DOF and', len(F2), '2-DOF flights; NaN total', summary.nan.sum(),
          '; time jumps', int(summary.time_jump.sum()))

    vaf_rows, rmse_rows, const_rows = [], [], []

    # ---------- D: 1-DOF classical, hand method on our train flights
    fr = {n: free_response(F1[n]) for n in SPLIT['1dof_impulse']['train']}
    wn_h = float(np.mean([v['wn'] for v in fr.values()]))
    xi_h = float(np.mean([v['xi'] for v in fr.values()]))
    mu_each = [gain_from_peak(F1[n], wn_h, xi_h) for n in SPLIT['1dof_impulse']['train']]
    mu_imp = float(np.mean(mu_each))
    # the same peak matching on the hover doublets is not usable: the 4 estimates spread from about
    # 90 to 500, because the hovering drone does not follow the motors-off wn, xi. So ONE classical
    # model (wn, xi, mu all from the impulse train flights) is used on every test flight.
    mu_dbl_each = [gain_from_peak(F1[n], wn_h, xi_h) for n in SPLIT['1dof_doublet']['train']]
    hand_imp = dict(wn=wn_h, xi=xi_h, mu=mu_imp)
    hand_dbl = hand_imp
    print('free response per train flight:')
    for (n, v), m in zip(fr.items(), mu_each):
        print(f"  {n}: period {v['T_osc']:.3f} s, settling {v['Ts']:.2f} s -> wn {v['wn']:.3f}, xi {v['xi']:.3f} "
              f"(log-dec xi {v['xi_logdec']:.3f}), peak-matched mu {m:.1f}")
    print(f'hand method: wn {wn_h:.3f} rad/s, xi {xi_h:.3f}, mu {mu_imp:.1f}')
    print('doublet peak-matched mu (not used):', [round(m, 1) for m in mu_dbl_each])

    const_rows += [
        dict(model='1-DOF classical, ANT-X published', wn=ANTX_1DOF['wn'], xi=ANTX_1DOF['xi'], mu=ANTX_1DOF['mu'],
             source='Panza 2020 (ANT-X rig, armed motors, their M scaling)'),
        dict(model='1-DOF classical, hand method on our flights', wn=wn_h, xi=xi_h, mu=mu_imp,
             source='period + settling time of impulse_1..4; mu from first peak of impulse_1..4 '
                    f"(per flight {', '.join(f'{m:.1f}' for m in mu_each)})"),
    ]

    # ---------- E, F: 1-DOF test flights
    tests_1dof = [('impulse', n, [('classical, ANT-X published', ANTX_1DOF), ('classical, hand method (ours)', hand_imp)])
                  for n in SPLIT['1dof_impulse']['test']] + \
                 [('doublet', n, [('classical, ANT-X published', ANTX_1DOF), ('classical, hand method (ours)', hand_dbl)])
                  for n in SPLIT['1dof_doublet']['test']]
    for kind, n, models in tests_1dof:
        f = F1[n]
        acc_meas = smooth_derivative(f.q.values)
        fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        ax[0].plot(f.t, np.degrees(f.theta), 'k', lw=2, label='measured')
        ax[1].plot(f.t, np.degrees(f.q), 'k', lw=2, label='measured')
        for label, p in models:
            th, q = simulate_1dof(p, f)
            th1, q1 = one_step_1dof(p, f)
            acc = -2 * p['xi'] * p['wn'] * q - p['wn'] ** 2 * th + p['mu'] * np.interp(f.t - TAU, f.t, f.M, left=0)
            vaf_rows.append(dict(case='1-DOF', test_flight=n, model=label,
                                 VAF_pitch=vaf(f.theta, th), VAF_pitch_rate=vaf(f.q, q), VAF_pitch_acc=vaf(acc_meas, acc),
                                 VAF_position=np.nan, VAF_velocity=np.nan, VAF_acc=np.nan,
                                 onestep_VAF_pitch_rate=vaf(f.q.values[1:], q1)))
            r = dict(case='1-DOF', test_flight=n, model=label, signal='theta [deg]')
            for h in [1, 5, 10]:
                k = int(round(h / DT))
                r[f'RMSE_first_{h}s'] = rmse(np.degrees(f.theta[:k]), np.degrees(th[:k])) if k <= len(f) else np.nan
            r['RMSE_whole_flight'] = rmse(np.degrees(f.theta), np.degrees(th))
            r['RMSE_onestep'] = rmse(np.degrees(f.theta.values[1:]), np.degrees(th1))
            rmse_rows.append(r)
            ax[0].plot(f.t, np.degrees(th), label=f'{label} (VAF {vaf(f.theta, th):.1f} %)')
            ax[1].plot(f.t, np.degrees(q), label=label)
        ax[0].set(ylabel='pitch theta [deg]', title=f'1-DOF test flight {n} ({kind}): rollout, never seen in training')
        ax[1].set(ylabel='pitch rate q [deg/s]', xlabel='t [s]')
        if kind == 'doublet':
            ax[0].set_ylim(-40, 40); ax[1].set_ylim(-150, 150)
        else:
            ax[0].set_ylim(-30, 30); ax[1].set_ylim(-120, 120)
        ax[0].legend(fontsize=8); plt.tight_layout()
        plt.savefig(os.path.join(FIG, f'1dof_test_{n}.png'), dpi=120); plt.close()

    # free-response identification figure (how the hand method reads the swing)
    n0 = SPLIT['1dof_impulse']['train'][0]; f, v = F1[n0], fr[n0]
    plt.figure(figsize=(10, 3.5))
    plt.plot(f.t, np.degrees(f.theta), 'k', label=f'{n0} measured')
    plt.plot(v['peaks_t'], np.degrees(np.interp(v['peaks_t'], f.t, f.theta)), 'r*', ms=10, label=f"maxima: period {v['T_osc']:.2f} s")
    plt.axvline(v['push_end'], color='g', ls='--', label='push ends')
    plt.axvline(v['push_end'] + v['Ts'], color='b', ls='--', label=f"settled (1 %): {v['Ts']:.2f} s")
    plt.xlabel('t [s]'); plt.ylabel('theta [deg]'); plt.title('ANT-X hand method, step 1: read period and settling time of the free swing')
    plt.legend(fontsize=8); plt.tight_layout(); plt.savefig(os.path.join(FIG, '1dof_hand_method_free_response.png'), dpi=120); plt.close()

    # ---------- D: 2-DOF SI fit
    fit = fit_2dof(F2, SPLIT['2dof']['train'])
    si = dict(kbar=fit['kbar'], cbar=fit['cbar'])
    print(f"2-DOF SI fit: kbar {fit['kbar']:.3f} (+-{fit['kbar_std']:.3f}), cbar {fit['cbar']:.3f} (+-{fit['cbar_std']:.3f})")
    const_rows += [
        dict(model='2-DOF, ANT-X published', kbar=ANTX_2DOF['kbar'], cbar=ANTX_2DOF['cbar'], source='Panza 2021, v1.6 (normalised thrust 0.35)'),
        dict(model='2-DOF, SI fit on our flights', kbar=fit['kbar'], cbar=fit['cbar'],
             source=f"fit on {', '.join(SPLIT['2dof']['train'])}; 1-sigma kbar {fit['kbar_std']:.3f}, cbar {fit['cbar_std']:.3f}"),
    ]

    # ---------- E, F: 2-DOF test flights
    for part in ['test', 'extra_test']:
        for n in SPLIT['2dof'][part]:
            f = F2[n]
            a_meas = smooth_derivative(f.v.values)
            fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
            ax[0].plot(f.t, f.x, 'k', lw=2, label='measured'); ax[1].plot(f.t, f.v, 'k', lw=2); ax[2].plot(f.t, np.degrees(f.theta), 'k', lw=1)
            for label, p in [('ANT-X published', ANTX_2DOF), ('SI fit (ours)', si)]:
                x, v_ = simulate_2dof(p, f)
                xs, vs = segment_rollout_2dof(p, f, 5.0)
                x1, v1 = one_step_2dof(p, f)
                acc = -p['cbar'] * v_ - p['kbar'] * f.theta.values
                vaf_rows.append(dict(case='2-DOF' + (' (extra test)' if part == 'extra_test' else ''), test_flight=n, model=label,
                                     VAF_pitch=np.nan, VAF_pitch_rate=np.nan, VAF_pitch_acc=np.nan,
                                     VAF_position=vaf(f.x, x), VAF_velocity=vaf(f.v, v_), VAF_acc=vaf(a_meas, acc),
                                     VAF_position_5s_restart=vaf(f.x, xs), VAF_velocity_5s_restart=vaf(f.v, vs),
                                     onestep_VAF_velocity=vaf(f.v.values[1:], v1)))
                if label.startswith('SI'):
                    ax[0].plot(f.t, xs, ':', color='tab:green', label=f'SI fit, restarted every 5 s (VAF {vaf(f.x, xs):.1f} %)')
                r = dict(case='2-DOF' + (' (extra test)' if part == 'extra_test' else ''), test_flight=n, model=label, signal='x [m]')
                for h in [1, 5, 10]:
                    k = int(round(h / DT)); r[f'RMSE_first_{h}s'] = rmse(f.x[:k], x[:k])
                r['RMSE_whole_flight'] = rmse(f.x, x)
                r['RMSE_onestep'] = rmse(f.x.values[1:], x1)
                rmse_rows.append(r)
                ax[0].plot(f.t, x, label=f'{label} (VAF {vaf(f.x, x):.1f} %)'); ax[1].plot(f.t, v_)
            ax[0].set(ylabel='x [m]', title=f'2-DOF test flight {n}: rollout from measured pitch, never seen in fitting')
            ax[1].set(ylabel='v [m/s]'); ax[2].set(ylabel='theta (input) [deg]', xlabel='t [s]')
            ax[0].legend(fontsize=8); plt.tight_layout()
            plt.savefig(os.path.join(FIG, f'2dof_test_{n}.png'), dpi=120); plt.close()

    # ---------- F: tables
    V = pd.DataFrame(vaf_rows); R = pd.DataFrame(rmse_rows); C = pd.DataFrame(const_rows)
    V.to_csv(os.path.join(RES, 'baseline_vaf.csv'), index=False, float_format='%.2f')
    R.to_csv(os.path.join(RES, 'baseline_rmse.csv'), index=False, float_format='%.4f')
    C.to_csv(os.path.join(RES, 'baseline_constants.csv'), index=False, float_format='%.4f')
    json.dump(dict(hand_method_1dof=dict(wn=wn_h, xi=xi_h, mu=mu_imp, mu_doublet_peak_estimates_not_used=mu_dbl_each,
                                         per_flight={n: dict(period=v['T_osc'], settling=v['Ts'], wn=v['wn'], xi=v['xi'],
                                                             xi_logdec=v['xi_logdec']) for n, v in fr.items()}),
                   si_fit_2dof=fit, tau=TAU),
              open(os.path.join(RES, 'baseline_constants.json'), 'w'), indent=2)
    write_summary(V, R, fr, wn_h, xi_h, mu_imp, mu_dbl_each, fit)
    pd.set_option('display.width', 200)
    print(V.round(1).to_string()); print(R.round(3).to_string()); print(C.to_string())


if __name__ == '__main__':
    main()
