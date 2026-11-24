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

### 1-DOF, hover (test doublet_6, 1.46 s open-loop window)
Networks: Batch 3 design (output = next state), trained on doublet_1-5. Classical rows use Amr's split (train doublet_1-4).
| Model | VAF [%] | Source |
|---|---|---|
| Classical, ANT-X published constants | 0.0 | baseline_vaf.csv |
| Classical, hand method (impulse constants) | 43.6 | baseline_vaf.csv |
| Equation, simulation fit | 98.8 | batch1_2b |
| Equation, inverse-PINN constants | 99.0 | batch1_2b |
| Plain NN (5 seeds) | 99.0 ± 0.1 | batch3 |
| PINC (5 seeds) | 99.1 ± 0.1 | batch3 |

### 1-DOF, hover, new inputs never flown (reference = equation with sim-fit constants)
| Input | PINC | Plain NN | Source |
|---|---|---|---|
| doublet x1.5 | 99.75 ± 0.01 | 98.67 ± 0.41 | batch3 |
| hold M = 0.03 for 1 s | 93.5 ± 3.1 | 78.3 ± 6.5 | batch3 |

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
- Motors-off networks use one design (Amr's: output = change per step); hover networks use the Batch 3 design (output = next state), which handled the short hover windows better. Same design within each comparison.
- PINC wins where data is missing (new inputs, short hover windows: Batch 6 hover PINC 90.8 vs plain NN 9.5 on doublet_6); with plenty of data (motors off) the plain NN fits slightly better.
