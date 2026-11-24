# Baseline results (made by scripts/baselines.py)

All scores are on flights the models never saw. VAF = max(1 - var(y - y_hat)/var(y), 0) x 100 %, 100 % = perfect.
Rollout = simulate the whole test window from its first measured state, with the measured input, no correction.

## Table 1a. 1-DOF pitch, test flights (rollout VAF %; RMSE of pitch in deg over the whole window)

| Model | impulse_5: pitch VAF | pitch rate VAF | pitch RMSE [deg] | doublet_5: pitch VAF | doublet_6: pitch VAF |
|---|---|---|---|---|---|
| classical, ANT-X published | 0.0 | 0.0 | 43.41 | 0.0 | 0.0 |
| classical, hand method (ours) | 83.1 | 81.5 | 2.07 | 59.5 | 43.6 |
| plain NN (PyTorch, mean +- std of 5 seeds) | 99.8 +- 0.0 | 99.6 +- 0.0 | 0.45 | 89.8 +- 2.6 | 79.9 +- 8.9 |
| simulation fit (Zakaria, notebook batch 1-2) | 97.0 | | | | |
| inverse PINN (Zakaria, notebook batch 1-2) | 94.7 | | | | |
| PINC (Zakaria) | | | | | |

## Table 1b. 2-DOF position, test flights (VAF %)

| Model | vel_doublet_4: x | x (5 s restart) | v | acc. | position_2: x | x (5 s restart) | v | acc. |
|---|---|---|---|---|---|---|---|---|
| ANT-X published | 0.0 | 66.2 | 82.8 | 81.3 | 0.0 | 2.8 | 9.8 | 70.8 |
| SI fit (ours) | 23.7 | 65.6 | 86.5 | 93.8 | 23.2 | 58.0 | 60.6 | 88.0 |
| plain NN (PyTorch, mean +- std of 5 seeds) | 0.0 +- 0.0 | 40.1 +- 28.3 | 87.1 +- 3.2 | - | 68.5 +- 22.7 | 98.2 +- 0.4 | 93.9 +- 2.0 | - |
| PINC (Zakaria) | | | | | | | | |

Extra 2-DOF check on the 4 attitude-doublet flights (not in the split, never used for fitting), mean VAF:

| Model | x | x (5 s restart) | v | acc. |
|---|---|---|---|---|
| ANT-X published | 14.8 | 26.9 | 42.4 | 56.9 |
| SI fit (ours) | 30.6 | 62.5 | 76.4 | 76.7 |
| plain NN (mean of 5 seeds) | 67.7 | 88.3 | 79.3 | - |

## Table 2. Constants

| Constant | ANT-X published | Classical hand method (ours) | Simulation fit (Zakaria) | Inverse PINN (Zakaria) | PINC (Zakaria) |
|---|---|---|---|---|---|
| wn [rad/s] | 3.437 | 4.080 | 4.075 | 4.035 | |
| xi [-] | 0.0836 | 0.396 (log-decrement check: 0.191) | 0.213 | 0.161 | |
| mu [rad/s^2] | 358.5 | 81.3 | 78.3 | 65.5 | |
| tau [s] | 0.020 | 0.020 (fixed) | 0.020 | 0.020 | |

| Constant | ANT-X published | SI fit (ours) | PINC (Zakaria) |
|---|---|---|---|
| kbar [m/s^2/rad] | 4.748 | 3.381 (+-0.018) | |
| cbar [1/s] | 1.168 | 1.345 (+-0.010) | |

## RMSE of the rollout over its first 1, 5, 10 s

| Case | Test flight | Model | signal | first 1 s | first 5 s | first 10 s | whole window |
|---|---|---|---|---|---|---|---|
| 1-DOF | impulse_5 | classical, ANT-X published | theta [deg] | 65.277 | 46.688 | - | 43.414 |
| 1-DOF | impulse_5 | classical, hand method (ours) | theta [deg] | 0.931 | 2.262 | - | 2.069 |
| 1-DOF | doublet_5 | classical, ANT-X published | theta [deg] | 10.135 | - | - | 16.226 |
| 1-DOF | doublet_5 | classical, hand method (ours) | theta [deg] | 3.210 | - | - | 4.863 |
| 1-DOF | doublet_6 | classical, ANT-X published | theta [deg] | 11.228 | - | - | 12.299 |
| 1-DOF | doublet_6 | classical, hand method (ours) | theta [deg] | 3.541 | - | - | 7.471 |
| 2-DOF | velocity_doublet_4 | ANT-X published | x [m] | 0.007 | 0.018 | 0.086 | 0.284 |
| 2-DOF | velocity_doublet_4 | SI fit (ours) | x [m] | 0.006 | 0.033 | 0.119 | 0.226 |
| 2-DOF | position_2 | ANT-X published | x [m] | 0.012 | 0.210 | 0.242 | 0.323 |
| 2-DOF | position_2 | SI fit (ours) | x [m] | 0.008 | 0.133 | 0.175 | 0.181 |

## Notes for the report

- ANT-X published constants give 0 % VAF on our rig: they come from another ANT-X drone, with motors armed, and use a different scaling of the moment command M (their mu is about 5x ours). Our rig needs its own constants.
- Hand method: the swing period gives wn almost exactly equal to the full simulation fit. The settling-time rule overestimates damping (bearing friction stops the swing early, at a slightly different angle), which is why the classical model under-predicts the second swing.
- Hand method, mu: peak matching on the impulse train flights gives a consistent mu (per flight values in results/baseline_constants.csv). The same peak matching on the hover doublets gives estimates from about 90 to 500 (239, 126, 497, 90), so it is not used: one classical model, all constants from the impulse flights, is tested on every flight.
- 1-DOF doublets are hovering flights (motors at hover thrust). They react differently from the motors-off impulse flights, so all models score lower there.
- 2-DOF: the model is fitted on velocity, because it is a pitch -> velocity model (Gv = -kbar/(s + cbar)). When the drone holds position, static friction holds the cart although pitch is 1-3 deg off level; the linear model cannot stop, so simulated position creeps over a 25-45 s flight. That is why whole-flight position VAF is low and the "5 s restart" column (simulation restarted from the measured state every 5 s) is also given.
- One-step-ahead predictions (4 ms) are almost perfect for every model (VAF > 99 %), so they do not separate the models; see results/baseline_vaf.csv.
- Plain NN (scripts/plain_nn.py): PyTorch, 2 x 32 tanh, learns one 0.02 s step from the train flights only and is rolled out by feeding its predictions back. Scored at 50 Hz (the classical rows at 250 Hz; that changes VAF by < 1 point). It beats the classical model on the impulse flight (it also learns the bearing friction the linear equation lacks), but on the hover doublets, a situation it never saw, its first peak is about twice too high and results vary between seeds. VAF hides such steady offsets (it ignores a constant error), so check the plots and the RMSE too.
- 2-DOF plain NN: on position_2 it learns the stiction (the cart stays put while pitch is slightly off level), so velocity and position are much better than the SI fit. On velocity_doublet_4 it over-predicts one speed block, and whole-flight position drifts like the SI model.
- Published kbar is for normalised hover thrust 0.35; our flights hover at about 0.41, which partly explains the difference (kbar is proportional to thrust).
