# Power model from telemetry (branch `surge-model`)

Based on the paper "Data-driven parametrization of an energy-propulsive model for an electric
racing vessel" (unpublished draft, Oct 2026, **don't share**).

## Now vs new

| | **Now** (strategy repo) | **New** (`src/surge_model.py`) |
|---|---|---|
| Power curve | ShipFlow resistance table ÷ `eta_grid` (per-speed medians from the July data) | `P = a·u^b + c_turn·r²·u`, fitted on telemetry |
| Where it comes from | CFD for a 315 kg hull + a correction per speed bin | Steady-power stretches (≥ 30 s) from the race logs, least squares |
| Turns | Not modelled (the Seg-MPC learns them per segment) | `c_turn·r²·u`: extra power for the sideways push in a turn |
| Battery | SOC % only | Also `V = OCV(SOC) − R0·I` and capacity per pack |
| Checked on an unseen day | No | Yes: fit on one day, test on another |

## What we found (fit: endurance 10/07, blind: laps test 08/07)

- **Our boat isn't cubic.** The paper assumes `P = k·u³`. For us it's `P ≈ 179·u^1.80`, because
  P/u³ falls from about 50 at 3 m/s to about 20 at 6 m/s. The cubic fits badly (R² 0.65 vs 0.89).
- **Turn cost isn't stable yet:** `c_turn` came out at 3,600 on one day and 10,900 on the other.
  The yaw rate on the endurance day had to come from 2 s GPS positions (no heading logged).
- **Battery:** R0 ≈ 8 mΩ per pack, capacity ≈ 89–96 Ah per pack (≈ 10 kWh total ✔).
- **No drift with SOC** over the race (a = 178 → 171). The paper saw +10 %.

| Model | Fit day: energy error | Blind day: energy error | Blind: power RMSE |
|---|---|---|---|
| New `179·u^1.80 + turns` | **−0.1 %** | −11.8 % (safe side) | 432 W |
| Now (ShipFlow / eta_grid)* | +13.2 % | −3.4 % | 357 W |
| `88·u^2.29` (strategy notes) | +0.5 % | −10.5 % | 507 W |

\* `eta_grid` was fitted on all the July data, so it has already seen the "blind" day.

Fitting the other way round (on the laps test, then testing on the endurance race) gives +1.4 %
energy error. **Conclusion:** at race speeds the new curve agrees with the current one (within
1–7 %). Its benefits are that it's checked on unseen data and gives a turn term, but it isn't
more accurate yet. The two days differ by about 10 %, as in the paper.

## Use

```sh
python src/surge_model.py                       # fit + validation -> config/surge_model_2026.json
python src/surge_model.py --fit lapstest --test endurance
python src/strategy_service.py ... --surge-model config/surge_model_2026.json   # opt-in
python -m unittest tests.test_surge_model       # 8 tests on synthetic days
```

`--surge-model` replaces `eta_grid` at 3–6 m/s (the fitted range) so the strategy's curve equals
`a·u^b`. It's off by default. Only the Seg-MPC and Kalman use `eta_grid`; the `instant`
controller uses the single `eta` and isn't affected.

## Changed from the paper

- The plateau band is 300 W or 10 % on a 5 s average, not 50 W: our power comes in ~110 W
  steps and wobbles ±200 W at a steady throttle.
- The exponent is free (searched 1.0–3.5) as well as fixed at 3; the version with the lower
  error on the blind day is used.
- Gaps of up to 12 s are interpolated, because the 08/07 export loses ~10 s every 4 samples.

## Next

1. **Yaw rate from the IMU gyro** (100 Hz, being tested now) instead of GPS → a stable `c_turn`.
2. **More days:** refit on new sessions, and keep only what holds on a blind day.
3. **Load cell** (once calibrated) → split `a` into drag and efficiency, which the paper couldn't do.
4. Pass the turn term to the Seg-MPC (cost per segment from the lap's turn radius).
