# 🚢 Float Forward Telemetry Server  
### DTU Boat Telemetry Pipeline

A modern telemetry backend for the Float Forward (DTU) boat.  
This system replaces last year's Raspberry Pi with a more reliable, cloud-integrated stack:

- **HiveMQ Cloud** – secure MQTT broker  
- **Python MQTT Bridge** – receives telemetry & writes to InfluxDB  
- **InfluxDB** – time-series storage  
- **Grafana** – real-time dashboards  
- **Cloudflare Tunnel** – remote access  
- **ESP32 Simulator** – realistic telemetry generator for development  

Designed to run on the **DTU mini-PC telemetry server**.

---

# ⚡ System Architecture

## Bidirectional Communication Flow

### **Uplink** (Boat → Server: Telemetry Data)
```
ESP32 / Simulator
    │ (publishes telemetry)
    ▼
HiveMQ Cloud (boat/telemetry/#)
    │
    ▼
mqtt_subscriber_real_test.py
    │ (receives & processes)
    ▼
Your Application / InfluxDB
```

### **Downlink** (Server → Boat: Commands)
```
Your API
    │ (publishes commands)
    ▼
HiveMQ Cloud (boat/message/#)
    │
    ▼
downlink_listener.py
    │ (receives & forwards)
    ▼
ESP32 (via serial/wireless)
```

---

# 📂 Project Structure

```
DTU-ff-telemetry-server/
│
├── src/
│   ├── downlink_listener.py              # Receives API commands from HiveMQ → forwards to ESP32
│   ├── mqtt_subscriber_real_test.py      # Uplink: receives telemetry FROM boat
│   ├── mqtt_subscriber_basic_test.py     # Basic test version (no real credentials)
│   ├── esp32_simulator_real.py           # Realistic ESP32 simulator (publishes telemetry)
│   ├── esp32_simulator_basic.py          # Basic ESP32 simulator
│   └── __init__.py
│
├── config/
│   └── .env                    # Environment variables (not in Git)
│
├── tools/
│   └── cloudflared.deb         # Optional Cloudflare installer
│
├── old_scripts/                # Previous scripts (legacy)
│
├── docker-compose.yml          # InfluxDB + Grafana containers
├── requirements.txt
└── README.md
```

---

# 🚀 Running the Telemetry Pipeline

## Setup

### 1️⃣ Activate Python Virtual Environment

```bash
source ff-env/bin/activate
```

### 2️⃣ Install dependencies

```bash
pip install -r requirements.txt
```

### 3️⃣ Configure Environment Variables

Create `config/.env` with your HiveMQ credentials:

```
HIVEMQ_HOST=your-broker.hivemq.cloud
HIVEMQ_PORT=8883
HIVEMQ_USER=your_username
HIVEMQ_PASSWORD=your_password
```

---

## Testing Uplink (Boat → Server)

**Test that telemetry flows from the boat to your server.**

### Option A: Using ESP32 Simulator

Terminal 1 - Start simulator:
```bash
python src/esp32_simulator_real.py
```

Terminal 2 - Subscribe to telemetry:
```bash
python src/mqtt_subscriber_real_test.py
```

Expected output in Terminal 2:
```
Connected: 0
boat/telemetry/gps/speed -> 3.87
boat/telemetry/battery/1/voltage -> 52.4
boat/telemetry/battery/2/voltage -> 51.9
```

**What's happening:**
- The simulator publishes telemetry to HiveMQ topics
- The subscriber receives and displays that data in real-time
- Data is also visible in HiveMQ web dashboard under `boat/telemetry/#`

### Option B: Using Real ESP32

Simply run the subscriber without the simulator:
```bash
python src/mqtt_subscriber_real_test.py
```

The real ESP32 will publish telemetry, and you'll see it displayed.

---

## Testing Downlink (Server → Boat)

**Test that commands flow from your API to the boat.**

```bash
python src/downlink_listener.py
```

Expected output:
```
Connected: 0
Subscribed to boat/message/#
Waiting for commands from API...
```

**What's happening:**
- The listener subscribes to `boat/message/#` topics
- When your API publishes a command (e.g., `boat/message/power: ON`), the listener receives it
- It processes the command and forwards it to the ESP32 via serial connection

---

## Full Integration Test

Run all three in separate terminals:

Terminal 1 (Simulator):
```bash
python src/esp32_simulator_real.py
```

Terminal 2 (Uplink):
```bash
python src/mqtt_subscriber_real_test.py
```

Terminal 3 (Downlink):
```bash
python src/downlink_listener.py
```

You should see:
- Simulator publishing telemetry
- Subscriber receiving telemetry
- Listener receiving the command and forwarding to "ESP32"

---
# 🧭 Strategy Service (target power for the pilot)

`src/strategy_service.py` reads the boat's telemetry from HiveMQ, works out the battery power the pilot
should hold to finish the endurance race on the SOC reserve, and sends it on the existing downlink
(`boat/message/power`, `{"powerMin": kW, "powerMax": kW}`), so the ESP32 and display need no change.
The strategy itself is the strategy team's code in the **Strategy_EnduranceRace** repo (packages
`strategy` and `boat`). `src/strategy_engine.py` feeds it the telemetry the same way that repo's
`backend/live_controller.py` does, but from HiveMQ instead of InfluxDB queries. Nothing is copied, so
pulling that repo updates the strategy. Its location is `STRATEGY_PATH` (in `config/.env` or
`--strategy-path`), by default `../../Strategy_EnduranceRace` next to ff_server. It needs `numpy`
and `scipy`.

- `--controller instant` (default): `instant_values.recommend`, the energy above the reserve spread
  over the time left, plus the matching speed and an ABOVE / ON TARGET / BELOW status.
- `--controller mpc`: Seg-MPC. One calibration lap at the average power, then each lap it learns the
  disturbance per track segment (`ThrustKalman`, `LapAlphaHistory`) and re-plans the power per
  segment (`endurance_mpc_solve`). Track: `--seg-lengths`, `--lap-len-m` (default from the boat config).
- Boat data comes from `config/boat_2026.json` (from the 2026 Mechanical Final Report and the July
  race data): 10 kWh, 3 h, 5 % reserve, 24 kW, 318 kg, measured efficiency 0.37 and per-speed
  `eta_grid`, pre-race plan 3167 W / 4.91 m/s, 600 m lap in 4 segments. The mass and `eta_grid`
  are also loaded into the strategy repo's physical model. Any option on the command line wins
  (`--eta`, `--mass-kg`, `--v-opt`, `--p-opt-kw`, `--battery-wh`, ...); `--boat-config ''` uses
  the strategy repo's own values.
- The SOC used is the **lowest pack's**, because the motor needs both packs (2026 endurance: back
  6 min early with one pack at 1.8 % and the other at 11 %).
- `--speed-unit ms` (default; the 2026 boat sends m/s) or `kmh` (the repo's 2025 CSVs).
- `--use-loadcell`: give `thrust/loadcell_n` to the thrust Kalman filter. Off until the load cell
  is calibrated. `--accel-source imu` uses the IMU batch `ax` (m/s², x forward, gravity included)
  instead of dv/dt. Not the default: underway the boat trims ~1.6° nose up, so `ax` reads
  ~0.3 m/s² high (~94 N in the disturbance estimate), and the GPS pitch can't correct it.
- `--mode shadow` (default): compute and log only. `--mode auto`: also publish to the boat.
- Every step is logged to InfluxDB, measurement `strategy`, including the speed target, status and
  the Seg-MPC state (`--no-influx` to turn off).
- A message on `boat/message/power` sent by anyone else (the downlink page) pauses auto mode for
  `--override-pause-min` minutes.
- No recommendation is given when the battery SOC is older than `--stale-s` seconds.

Replay a recorded bridge log (no network):
```bash
python src/strategy_service.py --replay logtest_afternoonRace.txt --duration-min 30 --controller mpc
```

Live, shadow mode, race starting now:
```bash
python src/strategy_service.py --race-start now --controller instant
```

Tests (need the Strategy_EnduranceRace repo, numpy and scipy):
```bash
python -m unittest discover -s tests
```

## What changed (13 Nov 2026)

**Strategy: from our Simple controller to the strategy team's code**
- `src/strategy_engine.py` no longer has its own `SimpleController`. It imports `strategy` and `boat`
  from Strategy_EnduranceRace and runs them as `backend/live_controller.py` does (`instant` or `mpc`),
  with telemetry from HiveMQ. MQTT, downlink, shadow/auto, override pause, stale check, InfluxDB
  logging and replay are unchanged.
- Differences from `live_controller.py`, on purpose: speed in m/s (the 2026 boat; their code divides
  by 3.6 for the 2025 CSVs), load cell off by default, acceleration from dv/dt by default, and after
  a restart the power ramp starts from the measured power instead of 0.
- The SOC used is the lowest pack's, not the mean.
- New options: `--controller`, `--strategy-path`, `--boat-config`, `--eta`, `--mass-kg`, `--v-opt`,
  `--p-opt-kw`, `--seg-lengths`, `--lap-len-m`, `--speed-unit`, `--use-loadcell`, `--accel-source`.
  `--ramp-kw-per-s` now defaults to 0.8 (their value). `--duration-min`, `--battery-wh`,
  `--soc-reserve` and `--p-max-kw` are no longer required: they come from the boat config.
- InfluxDB `strategy` also logs the speed target, statuses, `soc_mean` and the Seg-MPC state (lap,
  segment, disturbance, per-segment targets).
- `requirements.txt`: added `numpy`, `scipy`.
- `tests/test_strategy_engine.py` rewritten for the new controllers, the boat config and the
  lowest-pack SOC (17 tests).

**Boat data from the 2026 reports** (Mechanical Final Report Spring 2026, DTU Float Forward 02-09-2026)
- New `config/boat_2026.json`: 10 kWh, 3 h race, 5 % reserve, 24 kW, 318 kg, efficiency 0.37,
  `eta_grid` 0.24–0.41 (measured from the July race data; was 0.38–0.76), pre-race plan
  3167 W / 4.91 m/s, 600 m lap as 267 / 33 / 267 / 33 m.
- The same values were written into Strategy_EnduranceRace, with the old values kept as comments
  (not committed there): `boat/physical_model.py` (`eta_grid`, `M_BOAT`),
  `strategy/instant_values.py` (`PRERACE`), `boat/dynamics.py` (`E_nom`, `eta_sys`),
  `backend/live_controller.py` (`LiveParams` defaults), `backend/main.py` (`LiveStartRequest`
  defaults, new `ENDURANCE_2026_SEG_LENGTHS`).
- Effect on the 10 July endurance replay: the disturbance estimate's median went from 179 N to 7 N.

**Other files**
- `STRATEGY_IMPROVEMENTS.md`: what is left to improve, and the plan for tuning in simulation and
  checking on the water.
- `src/mqtt_subscriber_real_test.py` (separate change, from the IMU test): the IMU handler is
  switched back on and writes `imu_*` fields.

**Still open:** the ShipFlow resistance table (needs the sea-trial phases), the uneven Seg-MPC plan
(keep `--controller mpc` in shadow mode), and the race length (the report says 3 h; the July
telemetry shows about 3.5 h of racing).
