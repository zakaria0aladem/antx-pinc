# PINC vs classical / SI fit / plain NN on the ANT-X 2-DOF drone

MTR615 course project (Zakaria Aladem, Amr Saleh). Due Wed 25 Nov 2026.

## Folders
- `data/1dof/` 1-DOF pitch flights at 250 Hz. `impulse_1..5` (motors off, 0.1 pulse 0.2 s), `doublet_1..6` (hover, open loop). Files are named signal type + number in time order; `original_log` in the summary gives the PX4 log number. Columns: `t, theta [rad], q [rad/s], M (front-back motor), T (mean motor), m_inj (injected), openloop`. Summary: `flights_1dof.csv`.
- `data/2dof/` 2-DOF flights (closed loop) at 250 Hz. Columns add `x [m], v [m/s], dist (raw laser), theta_sp, x_sp, v_sp`. Summary + usable flag: `flights_2dof.csv`.
- `scripts/` `.ulg` -> csv converters (`pip install pyulog`). Raw `.ulg` logs stay on OneDrive.
- `figures/` overview plots of every flight.
- `docs/` WBS and Amr's task prompt.
- `notebooks/` modelling notebooks.

## Split (suggested)
- 1-DOF: train impulse_1..4; test impulse_5.
- 2-DOF: train velocity_doublet_2, 3 + position_1, 3; test velocity_doublet_4 + position_2.
- Not usable: attitude_doublet_2, velocity_doublet_1 (no flight); prbs_1..3 (x does not move, pitch only).
- Delay tau = 20 ms in all models.
