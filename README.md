# PINC vs classical / SI fit / plain NN on the ANT-X 2-DOF drone

MTR615 course project (Zakaria Aladem, Amr Saleh). Due Wed 25 Nov 2026.

## Folders
- `data/1dof/` 1-DOF pitch flights at 250 Hz. `impulse_log22,24-27` (motors off, 0.1 pulse 0.2 s), `doublet_log30-35` (hover, open loop). Columns: `t, theta [rad], q [rad/s], M (front-back motor), T (mean motor), m_inj (injected), openloop`. Summary: `flights_1dof.csv`.
- `data/2dof/` 2-DOF flights (closed loop) at 250 Hz. Columns add `x [m], v [m/s], dist (raw laser), theta_sp, x_sp, v_sp`. Summary + usable flag: `flights_2dof.csv`.
- `scripts/` `.ulg` -> csv converters (`pip install pyulog`). Raw `.ulg` logs stay on OneDrive.
- `figures/` overview plots of every flight.
- `docs/` WBS and Amr's task prompt.
- `notebooks/` modelling notebooks.

## Split (suggested)
- 1-DOF: train impulse 22, 24, 25, 26; test impulse 27.
- 2-DOF: train velocity-doublet 10, 11 + position 2, 39; test velocity-doublet 12 + position 40.
- Delay tau = 20 ms in all models.
