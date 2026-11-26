# Final code: ANT-X 2-DOF rig, PINC vs equation vs plain NN

`final_code.ipynb` makes every result on slides 12 to 16 of the talk from the flight files in `../data/`.
The saved notebook already shows all its outputs. To run it again:

    pip install numpy pandas scipy torch matplotlib jupyter
    jupyter notebook final_code.ipynb

A full run takes about 1 hour on a 4-core CPU. Almost all of that time is spent training the networks: 5 random starts (seeds) per network.

| Slide | Section of the notebook | Output |
|---|---|---|
| 12 motors off | 5 (constants), 6 (VAF) | `final_vaf.csv` |
| 13 hover, unseen flights | 7 | `final_hover_vaf.csv` |
| 14 hover, commands never flown | 7 | `final_hover_vaf.csv` |
| 15 closed-loop position | 6 | `final_vaf.csv` |
| 16 cart friction | 8 | table in the notebook |

Data used: `data/clean/1dof`, `data/clean/2dof` (cleaned flights, 250 Hz) and `data/1dof` (raw hover flights and the flight summary).
