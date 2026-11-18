# Strategy service: suggested improvements

The strategy service (`src/strategy_service.py`) runs the strategy team's code from
`Strategy_EnduranceRace`. These are the changes I'd make, from replaying the 10 July 2026
endurance race (last 37 min, CAN data) and the afternoon test log. Items 1–4 are in the
strategy team's code; 5–9 are in ff_server.

## Strategy model (Strategy_EnduranceRace)

1. **Replace the placeholder boat model.** *Done (13 Nov) from the 2026 Mechanical Final Report
   and the July race data: mass 318 kg, efficiency 0.37 with a per-speed `eta_grid` (median
   0.24–0.41; the old one was 0.38–0.76), pre-race plan 3167 W / 4.91 m/s, 10 kWh, 24 kW, 600 m
   lap. In the strategy repo (old values kept as comments) and in `config/boat_2026.json`. The
   disturbance estimate's median dropped from 179 N to 7 N. Still to do: the ShipFlow resistance
   table itself, once the Phase 1–5 sea trials exist.* Original note: `eta = 0.35`, `mass = 315 kg`, `PRERACE` and the
   ShipFlow resistance table are marked PLACEHOLDER/UPDATE and don't match the 2026 boat. At
   9.6 m/s it measured about 16.7 kW, while the model predicts about 7–8 kW. As a result the
   speed target is "BELOW" 76 % of the time and gets stuck at the table's 10.29 m/s limit. Fill
   them from the Phase 1–5 sea-trial scripts in `planning/Phases`.
2. **Smooth the Seg-MPC plan.** On real data, `endurance_mpc_solve` gives plans like
   0 / 17 / 0 / 0 kW per segment, which a pilot can't follow. Add a penalty on the power
   difference between segments, or a minimum and maximum power around the average.
3. **Check the lap and segment geometry.** *Lap set to the 2026 endurance course: 600 m, with
   segments 267 / 33 / 267 / 33 m (turns are ~11 % of the distance in the July data).* Before, the
   Monaco lap was 1,222 m in 4 segments. The lap is
   counted by adding up speed × time, so it drifts. Find the segment from GPS position
   (`simulation/track.py` already has the track) instead of from distance travelled.
4. **Bound the disturbance estimate.** `D_hat` had a median of 179 N with the placeholder
   model. `AlgebraicDisturbanceObserver` clips to ±300 N and low-pass filters, but the live
   Seg-MPC path does neither. Use that observer, or the same clip.

## Inputs (ff_server)

5. **Calibrate the load cell before `--use-loadcell`.** Logs show 9,440 N and −11,300 N with
   the boat stopped. Zero it at rest, check it against a known pull, and send the corrected value.
6. **Make the IMU usable for acceleration.** IMU `ax` is in m/s², x points forward and gravity
   is included. Underway the boat trims about 1.6° nose-up, so `ax` reads about 0.3 m/s² high
   (about 94 N in `D_hat`), and the GPS pitch can't correct it. Subtract a slow moving average
   of `ax − dv/dt` (a complementary filter), then make `--accel-source imu` the default.
7. **Field names in the planning scripts.** Phases 1–5 read the InfluxDB field `thrust`; the
   bridge writes `thrust_loadcell_n`. Rename one of them.
8. *Done:* the strategy now uses the **lowest pack's SOC**. The motor needs both packs, and in the
   2026 endurance race one pack reached 1.8 % while the other was at 11 %.
9. **SOC gaps.** About 9 % of steps in the afternoon log had no recommendation because the
   battery SOC was more than 10 s old. Hold the last value for longer, or estimate SOC from
   battery power between messages.
10. **End of race.** In the last seconds, energy ÷ time left jumps to `p_max` (25 kW). Cap the
   increase over the last few minutes, or stop recommending in the last minute.

## Train on simulated data, check on the water

The plan is to tune the strategy on simulated but realistic data, then check it on real data
in one or two sessions on the water. This works if each part is tuned on the right data:

| Part | Learned from | Simulated data? |
|---|---|---|
| Speed → power curve, `eta`, mass | the boat | **No.** The simulation copies whatever curve you put in, so it can't find the real one. Needs real runs (Phase 1–5) |
| Controller settings (ramp, `recency`, Kalman `q`/`r`, smoothing, reserve) | how the controller behaves | **Yes.** This is what simulation is good for |
| Disturbance per segment (`D`, `alpha`) | the race day | **No need.** The Seg-MPC learns them live, lap by lap |
| Robustness (wrong battery size, waves, sensor gaps, pilot overshoot) | scenarios | **Yes** |

Suggested order:

1. **Simulator from real data.** Build it on the 2026 boat's speed → power curve (the July CAN
   logs give `P ≈ 88 · v^2.29`), not on the ShipFlow placeholders. Use
   `simulation/generate_planner_telemetry.py` with real noise and disturbance levels.
2. **Tune the controller settings in simulation.** Run many simulated races, including the
   scenarios above.
3. **Session 1 on the water (about 1 h): calibrate.** Straight runs in opposite directions at
   5–6 steady powers, 1–2 min each (Phases 1, 3–5). Zero the load cell at rest first. This
   gives the real resistance curve, `eta` and the load-cell offset. Rebuild the simulator with
   them and repeat step 2.
4. **Session 2 (30–60 min of laps): check.** Run the service in shadow mode while the pilot
   drives laps, then replay the log and compare predicted with actual energy use per lap.
   Only then try `--mode auto`.

**Caveat:** the sea trials in Denmark use a different hull from Monaco, so the curve from
session 1 has to be measured again on the race hull, or at least checked in the first laps.
