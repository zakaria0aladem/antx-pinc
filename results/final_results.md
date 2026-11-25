# Final results (1-DOF pitch, 2-DOF position)

All scores: output-error rollout (start from the first measured state, feed the measured command, never correct), VAF on pitch theta (1-DOF) or position x / velocity v (2-DOF), on flights never used in training. 100 % = perfect.

## Table 1: VAF on unseen test flights

### 1-DOF, motors off (train impulse_1-4, test impulse_5, 6 s rollout)
| Model | VAF [%] | Source |
|---|---|---|
| Classical, ANT-X published constants | 0.0 | baseline_vaf.csv |
| Classical, hand method | 83.1 | baseline_vaf.csv |
| Equation, simulation fit | 96.9 | batch6 |
| Equation, inverse-PINN constants | 94.7 | batch1_2 |
| Plain NN (5 seeds) | 99.8 ± 0.0 | batch6 |
| PINC (5 seeds) | 99.2 ± 0.1 | batch6 |

### 1-DOF, hover (train doublet_1-4, test doublet_5 and doublet_6, 1.46 s open-loop windows; Batch 8)
Networks: Batch 3 design (output = next state), physics constants from the simulation fit on doublet_1-4.
| Model | doublet_5 | doublet_6 | Source |
|---|---|---|---|
| Classical, ANT-X published constants | 0.0 | 0.0 | baseline_vaf.csv |
| Classical, hand method (impulse constants) | 59.5 | 43.6 | baseline_vaf.csv |
| Equation, simulation fit (doublet_1-4) | 56.1 | 97.4 | batch8 |
| Plain NN (5 seeds) | 58.9 ± 2.2 | 97.9 ± 0.3 | batch8 |
| PINC (5 seeds) | 65.6 ± 1.7 | 98.9 ± 0.2 | batch8 |

doublet_5 is a hard flight for every model (the equation itself reaches only 56).

### 1-DOF, hover, new inputs never flown (reference = equation with sim-fit constants; Batch 8)
| Input | PINC | Plain NN |
|---|---|---|
| doublet x1.5 | 99.71 ± 0.09 | 97.42 ± 0.55 |
| hold M = 0.03 for 1 s | 95.3 ± 1.4 | 76.9 ± 5.4 |

### 2-DOF, position from measured pitch (train velocity_doublet_2,3 + position_1,3)
| Test flight | ANT-X published (x / v) | SI fit (x / v) | Plain NN (x / v) |
|---|---|---|---|
| velocity_doublet_4 | 0.0 / 82.8 | 23.7 / 86.5 | 0.0 / 87.1 |
| position_2 | 0.0 / 9.8 | 23.2 / 60.6 | 68.5 / 93.9 |

## Table 2: constants (1-DOF)
| | ANT-X published | Motors off, sim fit | Motors off, inverse PINN | Hover, sim fit | Hover, inverse PINN |
|---|---|---|---|---|---|
| wn [rad/s] | 3.437 | 4.075 | 4.035 | 3.79 | 3.84 |
| xi [-] | 0.0836 | 0.213 | 0.161 | 0.157 | 0.105 |
| mu [rad/s^2] | 358.5 | 78.3 | 65.5 | 186.8 | 159.9 |

2-DOF: kbar / cbar = 4.748 / 1.168 (ANT-X), 3.381 / 1.345 (SI fit, ± 0.018 / 0.010).

## Notes for the report
- Motors-off networks use one design (Amr's: output = change per step); hover networks use the Batch 3 design (output = next state) on the shared split (Batch 8). Same design within each comparison.
- Hover physics needs the windows that stop just before the motor kick-back: the fixed 1.5 s clean windows give xi = 0 (no damping) in the fit (Batch 8 B), which is why Batch 6 hover was poor.
- PINC wins where data is scarce or the input is new (hover: best on both test flights; new inputs 95.3 vs 76.9 on the hold). With plenty of data (motors off, closed-loop position, attitude) the plain NN fits measured flights slightly better, but on closed-loop position 1 of its 5 seeds diverges while all 5 PINC seeds work.

## Table 3: direct comparison with Ghezzi's thesis (2-DOF, closed loop; Batch 7, 7b)
Same models, method (output error) and score (VAF) as the thesis. Train attitude_doublet_1, 3 / test attitude_doublet_4, 5 (doublet windows); train position_1, 3 / test position_2.

### Closed-loop position (x setpoint -> x), test position_2
| Model | VAF x | VAF speed | VAF acceleration |
|---|---|---|---|
| Thesis, reported on its own drones | 99.45 | - | 58.72 |
| Thesis constants on our flight | 92.9 | 27.5 | 0.0 |
| Thesis model refit on our flights | 99.3 | 91.8 | 68.5 |
| Inverse-PINN constants | 97.0 | 80.0 | - |
| PINC (5 seeds) | 99.35 ± 0.03 | 95.95 ± 0.15 | 86.75 ± 0.57 |
| Plain NN (5 seeds; 4 work, 1 diverged) | 4 good seeds: 99.6 | 98.3 | 96.4 |

Acceleration VAF depends on how speed is smoothed before differentiating (Batch 8): smoothing over 0.22 / 0.50 / 1.02 s gives refit 51 / 69 / 81, PINC 80 / 87 / 91, plain NN (4 good seeds) 95 / 96 / 97. Ours is computed from the estimated speed; the thesis's measured acceleration may come from the accelerometer, so compare with its 58.7 with care.

### Closed-loop attitude (pitch setpoint -> pitch), test attitude_doublet_4 / 5
| Model | VAF pitch | VAF rate | VAF acceleration |
|---|---|---|---|
| Thesis, reported | 99.86 | 97.27 | - |
| Thesis constants | 98.3 / 98.5 | 84.2 / 87.6 | 93.7 / 94.8 |
| Refit | 99.5 / 99.6 | 86.3 / 89.7 | 97.4 / 97.5 |
| Inverse-PINN constants | 99.5 / 99.6 | 87.8 / 91.1 | - |
| PINC (5 seeds) | 99.31 / 99.40 | 87.6 / 90.4 | 97.0 / 96.9 |
| Plain NN (5 seeds) | 99.66 / 99.59 | 96.2 / 95.9 | 98.9 / 98.4 |

### Constants
| Model | Thesis | Refit (± from fit) | Inverse PINN (± over 2 flights) |
|---|---|---|---|
| CL attitude wn [rad/s] | 16.16 | 27.4 ± 1.6 | 26.4 ± 2.8 |
| CL attitude xi | 0.56 | 0.95 ± 0.04 | 0.89 ± 0.01 |
| CL attitude delay [s] | - | 0.022 ± 0.002 | (fixed 0.022) |
| CL position wn [rad/s] | 3.47 | 2.27 ± 0.10 | 1.27 ± 0.23 |
| CL position xi | 0.84 | 1.13 ± 0.04 | 0.55 ± 0.14 |

Open-loop longitudinal model (Xu, Xq, Mq, Xdelta, Mdelta): thesis constants give VAF 0 on our logs; refit reaches x VAF 77 only on 2 s restarted windows (does not transfer; our M differs from the thesis delta and our rig has pitch stiffness).
Inverse PINN on position: NN(t) over a 35 s flight cannot follow the sudden jumps in acceleration at each setpoint step, so its constants come out low; on the short attitude windows it matches the refit.

## Table 4: Batch 9 improvements (notebooks/batch9_improvements.ipynb)
One design for every case: output = change per step, 5 seeds, 10k epochs. Hover here uses the kick-back windows with this design, so hover numbers differ from Batch 8 (Batch 3 design); keep Batch 8 for the hover comparison.

### Learned constants, physics weight, ensembles (VAF of y; median of 5 seeds, seeds with VAF > 50 in brackets)
| Model | Motors off, impulse_5 | Hover, doublet_6 | CL position, position_2 (x / acc) |
|---|---|---|---|
| Equation, sim fit | 96.9 | 97.3 | 99.3 / 68.1 |
| Plain NN | 99.8 (5/5) | 92.3 (4/5) | 99.6 / 96.4 (4/5) |
| PINC, fixed constants, w 1 | 99.3 (5/5) | 99.3 (5/5) | 99.3 / 86.7 (5/5) |
| PINC, learned constants, w 1 | 99.3 (5/5) | 99.3 (5/5) | 99.2 / 94.1 (5/5) |
| PINC, learned constants, w 0.1 | 99.5 (5/5) | 98.9 (5/5) | 95.6 / 61.9 (4/5) |
| Equation with PINC-learned constants | 88.8 | 98.3 | 98.9 / 84.3 |

- Learned constants: PINC acceleration on position 86.7 -> 94.1, close to the plain NN (96.4) with all 5 seeds working. Small or no change elsewhere.
- Physics weight 0.1: slightly better motors off (99.5), clearly worse on position (acc 62, 1 seed fails). Keep w = 1.
- Ensemble (average of 5 seeds): little change, except the plain NN on hover doublet_5 (45 -> 67).
- Hover doublet_5 stays hard for every model (best 77, equation with PINC-learned constants).
- Learned constants (mean of 5): motors off wn 3.97, xi 0.139, mu 65.8 (sim fit 4.07 / 0.212 / 76.9); hover 3.80 / 0.074 / 146 (sim fit 3.73 / 0.138 / 204); position wn 1.54, xi 0.72 (sim fit 2.25 / 1.13).

### Inverse PINN on position, one fit per setpoint step
| Model | wn | xi | VAF x | VAF v |
|---|---|---|---|---|
| Inverse PINN, whole flight (Batch 7b) | 1.27 | 0.55 | 97.0 | 80.0 |
| Inverse PINN, one fit per step | 1.41 | 0.63 | 98.2 | 88.7 |
| Simulation fit | 2.25 | 1.13 | 99.3 | 91.7 |

Per-step fits move toward the simulation fit but stay far from it; single segments scatter widely (wn 0.3 to 2.8).

### Better equations
- Friction + level offset in the cart model (v' = -cbar v - kbar (theta - theta_b) - Fc tanh(v/0.01)): x VAF 23.2 -> 90.7 on position_2, 23.6 -> 56.2 on velocity_doublet_4 (v 60.6 -> 95.0 and 86.5 -> 91.5). Fitted kbar 3.55, cbar 0.35, Fc 0.15.
- sin(theta) instead of theta in the pitch equation: no change (96.9 vs 96.9 motors off).

### ANT-X check (2026-11-25, three independent checks)
- Same push (M 0.1 for 0.2 s, motors off), raw logs: ANT-X lab rig (log X0038) peak 33.5 deg, swing 1.8 s, each swing keeps 74 % of the last; our rig 17.3 deg, 1.5 s, 34 %. Our rig is faster and far more damped.
- Our simulation fit on the ANT-X log: wn 3.445 (published 3.437), xi 0.042, mu 85, VAF 98.4. The ANT-X equation and our method agree; the constants belong to each rig.
- Published set (3.437 / 0.0836 / 358.5) scores 0 even on the ANT-X log: mu 358.5 is a hover gain, wn and xi come from a motors-off push.
