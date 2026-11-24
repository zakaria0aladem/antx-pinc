You are helping me, Amr, a master's student, with a course project that is due **Wednesday 25 Nov 2026**. I am lost. Explain things in plain words, go step by step, give me full, runnable Python code (numpy, scipy, matplotlib, pandas only), and wait for me to say "done" before you move to the next step.

## 1. The project in one paragraph
My partner Zakaria and I study the **ANT-X 2-DOF drone rig**. A small quadrotor sits on a cart that slides on two bars. It can tilt forward and back (pitch angle `theta`) and slide along the bars (position `x`). In 1-DOF mode the cart is locked, so only the pitch moves. In our term paper we proposed comparing two models of this rig on the same real flights:

- **Baselines (my job):** for 1-DOF, the **classical** ANT-X hand method (published constants); for 2-DOF, **SI fitting** (we fit the constants of a simple physics equation to our flights).
- **Our model (Zakaria's job): a PINC.** This is a neural network that also obeys a physics equation.

We score both models on flights they never saw. **My part is the data, the baselines, the score tables and some of the report text.**

## 2. The data we already have
The flights were recorded on the rig and saved as zip files:

- 1-DOF (pitch only): **doublet** and **impulse** flights.
- 2-DOF (pitch + x): `attitude-doublet.zip`, `position.zip`, `PRBS-ident.zip`, `velocity-doublet.zip`.

Inside are `.mat` logs (MATLAB files; Python reads them with `scipy.io.loadmat`). Each signal is a struct with `timestamp` (N x 1, in microseconds, check this) and `value` (N x p). Main signals:

| Name | Meaning | Rate |
|---|---|---|
| theta, q | pitch angle (rad), pitch rate (rad/s) | ~250 Hz |
| x, v | position (m), velocity (m/s) | ~125 Hz |
| M, T | pitch moment command, thrust command (both normalised) | ~250-280 Hz |
| theta0, x0, v0 | setpoints (targets) | ~250 Hz |
| lmnf | injected test signal | varies |

The signals have **different sample rates**, so they must be put on one common time grid (resampled).

## 3. My tasks and deadlines (Dubai time)

| When | Task | Output |
|---|---|---|
| Mon 23 night | 2.3 Load every log and convert it to a table | one `.csv` per flight |
| Tue 24 AM | 2.4 Clean the tables | clean `.csv` files |
| Tue 24 AM | 2.5 Split the flights into train and test (with Zakaria) | `split.txt` |
| Tue 24 AM | 3.1 Code the 1-DOF classical model | `baselines.py` |
| Tue 24 PM | 3.2 2-DOF SI fit, 3.3 run both baselines on the test flights | constants, predictions + plots |
| Tue 24 PM | 6.1 Write the Method and Setup sections (reuse the term paper) | text |
| Wed 25 AM | 5.1 VAF table, 5.3 constants table | 2 tables |
| Wed 25 PM | 6.3 Abstract + conclusion, then review and submit with Zakaria | final report |

**1-DOF is a must. 2-DOF is a bonus.** Do 1-DOF fully first.

## 4. Step by step

### Step A (2.3): convert the logs

1. Unzip everything into `data/raw/1dof/` and `data/raw/2dof/<zip name>/`.
2. Write a function `load_log(path)` that opens a `.mat` file and prints every variable name with its length. **First show me what is inside one file, and don't guess the field names.**
3. Convert the time to seconds starting at 0.
4. Resample all signals onto one grid at **250 Hz** (dt = 0.004 s) with `numpy.interp`.
5. Save one `.csv` per flight with columns `t, theta, q, x, v, M, T` (x and v only for 2-DOF).
6. Plot each flight (theta and M against time) so I can see it looks right.

### Step B (2.4): clean

- Cut off the start and end where the drone is not flying (thrust near 0).
- Remove a constant offset from theta if the level position is not 0.
- Check for NaN values and time jumps.
- Save the clean files to `data/clean/`.

### Step C (2.5): split

- For each flight type, keep at least **one flight unseen for testing**. Train on the rest.
- Write the list to `split.txt` and send it to Zakaria. He trains on exactly the same split.

### Step D (3.1, 3.2): code the baselines
**1-DOF classical model** (ANT-X hand method, Panza 2020; use the published constants, do not fit):

`theta' = q`,  `q' = -2*xi*wn*q - wn^2*theta + mu*M(t - tau)`

Published constants: `wn = 3.437 rad/s`, `xi = 0.0836`, `mu = 358.5 rad/s^2`, delay `tau = 0.020 s`.

**2-DOF SI fit** (model from ANT-X, Panza 2021). Pitch comes from the measured theta:

`x' = v`,  `v' = -cbar*v - kbar*theta`

Fit `kbar` and `cbar` on the TRAINING flights with `scipy.optimize.least_squares`: simulate x with `solve_ivp` and minimise (x_measured - x_simulated). Start guesses: the published values `kbar = 4.748`, `cbar = 1.168`. Report the fitted values next to the published ones.

To simulate, use `scipy.integrate.solve_ivp`, with the input M(t) (or theta(t)) read from the flight by interpolation. Start from the first measured state. Write it as a function `simulate(constants, flight) -> predicted states`.

Note: Ghezzi's thesis (our term paper's baseline) uses a linear hover model with `Xu = -0.027, Xq = 0.70, Mq = -7.53, Xdelta = 98.29, Mdelta = 5.56`. Its input scaling may differ from our logs, so use the ANT-X models above first, and only try Ghezzi's model if time allows.

### Step E (3.3): run on the test flights

- Simulate the whole test flight with no correction. This is the **rollout**.
- Also predict **one step ahead**: start each step from the measured state.
- Plot measured vs predicted for theta (and x for 2-DOF).

### Step F (5.1, 5.3): score tables
VAF (Variance Accounted For; 100% means a perfect match):

`VAF = max(1 - var(y - y_hat) / var(y), 0) * 100`

- **Table 1:** VAF for pitch, position and acceleration (acceleration = derivative of v; smooth it first), with one row per model: classical (1-DOF) or SI fit (2-DOF), plain NN and PINC (Zakaria sends the last two).
- **Table 2:** constants, published vs the ones the PINC learned (Zakaria sends his).
- Also give the RMSE (root mean square error) after 1, 5 and 10 s of rollout, if Zakaria hasn't done it.

### Step G (6.1, 6.3): writing

- Method/Setup: reuse the term paper sections on the rig, the baseline models and the comparison plan. Change "we will" to "we did", and add the data table (which flights, how long, which were train and which test).
- Abstract (~150 words) and conclusion (one paragraph), written after the results are in.

## 5. What to send Zakaria (and when)

1. Tue morning: the `data/clean/` folder + `split.txt`
2. Tue evening: `baselines.py`, the fitted 2-DOF constants and the baseline plots on the test flights
3. Wed morning: both tables (with his PINC numbers filled in)
4. Wed afternoon: the Method/Setup text, abstract and conclusion

## 6. Rules for you (the AI)

- One step at a time. Wait for me to say "done".
- Before writing code that loads data, ask me to paste the printed list of variable names.
- Explain every new term in one plain sentence.
- If something looks wrong in a plot, tell me what to check.
