"""
Strategy service: recommends a battery power to the pilot during the endurance race.

Reads the boat's telemetry straight from HiveMQ (boat/telemetry/#), runs the strategy in
strategy_engine.py every --dt seconds, and in auto mode publishes the recommendation on
the existing downlink topic (boat/message/power, {"powerMin": kW, "powerMax": kW}, QoS 1),
so the ESP32 and the pilot display need no change.

Modes:
  shadow  compute and log only; nothing is sent to the boat (default)
  auto    also publish to the boat, at most every --publish-every s, or sooner when the
          recommendation moves by more than --publish-change-kw
In both modes every step is written to InfluxDB (measurement "strategy") unless --no-influx.

A message on the downlink topic that this service didn't send (someone used the downlink
page) pauses auto mode for --override-pause-min minutes, so the operator always wins.

Replay a recorded bridge log instead of listening to HiveMQ (no network used):
  python src/strategy_service.py --replay logtest_afternoonRace.txt \
      --duration-min 30 --battery-wh 5000 --soc-reserve 5 --p-max-kw 10

Live, shadow mode, race starting now:
  python src/strategy_service.py --race-start now --duration-min 60 \
      --battery-wh 5000 --soc-reserve 5 --p-max-kw 10
"""

import argparse
import json
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from strategy_engine import RaceConfig, StrategyEngine, TelemetryState, load_log, payload_text

BASE_DIR = Path(__file__).resolve().parent.parent
ENV_PATH = BASE_DIR / "config" / ".env"


def fmt_time(t: float) -> str:
    return datetime.fromtimestamp(t, timezone.utc).strftime("%H:%M:%S") + "Z"


def describe(rec) -> str:
    i = rec.inputs
    soc = f"{100 * i['soc']:.1f}%" if "soc" in i else "-"
    p_batt = f"{i['p_batt'] / 1000:.2f}kW" if "p_batt" in i else "-"
    if not rec.ok:
        return f"{fmt_time(rec.t)}  no recommendation ({rec.reason})  soc={soc} p_batt={p_batt}"
    return (f"{fmt_time(rec.t)}  {rec.controller:<6}  target {rec.p_w / 1000:6.2f} kW "
            f"[{rec.p_min_w / 1000:.2f}-{rec.p_max_w / 1000:.2f}]  soc={soc} p_batt={p_batt} "
            f"left={rec.t_left_s / 60:.1f}min")


def parse_start(value: str) -> float:
    """'now', an ISO time (2026-11-05T10:00:00+01:00), or HH:MM local time today."""
    if value == "now":
        return time.time()
    if len(value) == 5 and value[2] == ":":
        h, m = map(int, value.split(":"))
        return datetime.now().replace(hour=h, minute=m, second=0, microsecond=0).timestamp()
    t = datetime.fromisoformat(value.replace("Z", "+00:00"))
    return t.timestamp()


def make_config(args, start: float) -> RaceConfig:
    return RaceConfig(
        start=start,
        duration_s=args.duration_min * 60,
        battery_wh=args.battery_wh,
        soc_reserve=args.soc_reserve / 100,
        p_max_w=args.p_max_kw * 1000,
        ramp_w_per_s=args.ramp_kw_per_s * 1000,
        band_w=args.band_kw * 1000,
        stale_s=args.stale_s,
        packs=tuple(int(p) for p in args.packs.split(",")),
    )


# ==========================
# Replay
# ==========================
def replay(args):
    records = load_log(args.replay, args.replay_day)
    if not records:
        raise SystemExit(f"No telemetry lines found in {args.replay}")

    start = parse_start(args.race_start) if args.race_start else records[0][0]
    cfg = make_config(args, start)
    state = TelemetryState()
    engine = StrategyEngine(cfg, state)

    print(f"Replaying {len(records)} records from {args.replay}, "
          f"{fmt_time(records[0][0])} -> {fmt_time(records[-1][0])}")

    i, now, last_print = 0, records[0][0], None
    counts = {}
    while now <= records[-1][0]:
        while i < len(records) and records[i][0] <= now:
            state.update(records[i][1], records[i][2], records[i][0])
            i += 1
        rec = engine.step(now)
        counts[rec.controller] = counts.get(rec.controller, 0) + 1
        if last_print is None or now - last_print >= args.publish_every:
            print(describe(rec))
            last_print = now
        now += args.dt

    print("\nSteps per controller: " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())))


# ==========================
# Live
# ==========================
def live(args):
    import os
    import paho.mqtt.client as mqtt
    from dotenv import load_dotenv
    from paho.mqtt.subscribeoptions import SubscribeOptions

    load_dotenv(ENV_PATH)
    cfg = make_config(args, parse_start(args.race_start or "now"))
    state = TelemetryState()
    engine = StrategyEngine(cfg, state)
    lock = threading.Lock()
    paused_until = [0.0]

    write = None
    if not args.no_influx:
        from influxdb_client import InfluxDBClient, Point, WritePrecision
        from influxdb_client.client.write_api import WriteOptions

        bucket, org = os.getenv("INFLUX_BUCKET"), os.getenv("INFLUX_ORG")
        db = InfluxDBClient(url=os.getenv("INFLUX_URL"), token=os.getenv("INFLUX_TOKEN"), org=org)
        write_api = db.write_api(write_options=WriteOptions(batch_size=50, flush_interval=1000))
        print(f"Logging to InfluxDB {os.getenv('INFLUX_URL')}, bucket {bucket}")

        def write(rec, sent):
            p = (Point("strategy").tag("controller", rec.controller)
                 .field("sent", int(sent))
                 .field("t_left_s", float(rec.t_left_s))
                 .time(int(rec.t * 1e9), WritePrecision.NS))
            if rec.ok:
                p.field("p_target_w", rec.p_w).field("p_min_w", rec.p_min_w).field("p_max_w", rec.p_max_w)
            else:
                p.field("reason", rec.reason)
            for key, value in rec.inputs.items():
                p.field(f"in_{key}", float(value))
            try:
                write_api.write(bucket=bucket, org=org, record=p)
            except Exception as e:
                print(f"InfluxDB write failed: {e}")

    def on_connect(client, userdata, flags, reason_code, properties):
        print(f"Connected to HiveMQ: {reason_code}")
        client.subscribe("boat/telemetry/#", qos=0)
        # noLocal: our own publishes don't come back, so anything received here was
        # sent by someone else (the downlink page)
        client.subscribe(args.topic, options=SubscribeOptions(qos=1, noLocal=True))

    def on_message(client, userdata, msg):
        if msg.topic == args.topic:
            paused_until[0] = time.time() + args.override_pause_min * 60
            print(f"Manual message on {args.topic}: {msg.payload.decode(errors='replace')} "
                  f"-> auto paused for {args.override_pause_min} min")
            return
        group = msg.topic.removeprefix("boat/telemetry/")
        try:
            data = json.loads(msg.payload.decode())
        except (ValueError, UnicodeDecodeError):
            return  # the bridge logs malformed payloads; the strategy just skips them
        if isinstance(data, dict):
            with lock:
                state.update(group, data, time.time())

    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                         protocol=mqtt.MQTTv5)
    client.username_pw_set(os.getenv("HIVEMQ_USER"), os.getenv("HIVEMQ_PASSWORD"))
    client.tls_set()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(os.getenv("HIVEMQ_HOST"), int(os.getenv("HIVEMQ_PORT", "8883")))
    client.loop_start()

    print(f"Mode {args.mode}; race {fmt_time(cfg.start)} + {args.duration_min} min")
    last_pub_t, last_pub_w = 0.0, None
    try:
        while True:
            now = time.time()
            with lock:
                rec = engine.step(now)

            sent = False
            if args.mode == "auto" and rec.ok and now >= paused_until[0]:
                due = now - last_pub_t >= args.publish_every
                moved = last_pub_w is None or abs(rec.p_w - last_pub_w) >= args.publish_change_kw * 1000
                if due or moved:
                    info = client.publish(args.topic, payload_text(rec), qos=1)
                    sent = info.rc == mqtt.MQTT_ERR_SUCCESS
                    if sent:
                        last_pub_t, last_pub_w = now, rec.p_w

            if write:
                write(rec, sent)
            print(describe(rec) + ("  -> sent" if sent else ""))
            time.sleep(max(0.0, args.dt - (time.time() - now)))
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    race = ap.add_argument_group("race")
    race.add_argument("--race-start", help="'now', ISO time, or HH:MM local (replay: first record)")
    race.add_argument("--duration-min", type=float, required=True)
    race.add_argument("--battery-wh", type=float, required=True, help="all packs together at 100 %% SOC")
    race.add_argument("--soc-reserve", type=float, required=True, help="SOC %% to keep at the finish")
    race.add_argument("--p-max-kw", type=float, required=True)
    race.add_argument("--ramp-kw-per-s", type=float, default=0.0, help="0 = no ramp limit")
    race.add_argument("--band-kw", type=float, default=0.5, help="half-width of the range sent to the pilot")
    race.add_argument("--packs", default="1,2", help="battery/<n> groups of the main battery")
    race.add_argument("--stale-s", type=float, default=10.0, help="ignore inputs older than this")

    run = ap.add_argument_group("run")
    run.add_argument("--mode", choices=["shadow", "auto"], default="shadow")
    run.add_argument("--dt", type=float, default=1.0, help="seconds between steps")
    run.add_argument("--publish-every", type=float, default=5.0)
    run.add_argument("--publish-change-kw", type=float, default=0.5)
    run.add_argument("--override-pause-min", type=float, default=10.0)
    run.add_argument("--topic", default="boat/message/power")
    run.add_argument("--no-influx", action="store_true")
    run.add_argument("--replay", help="bridge log to replay instead of listening to HiveMQ")
    run.add_argument("--replay-day", default="2000-01-01", help="date for logs without ts")
    args = ap.parse_args()

    # print each line at once, also when the output goes to a file or a service log
    sys.stdout.reconfigure(line_buffering=True)

    if args.replay:
        replay(args)
    else:
        live(args)


if __name__ == "__main__":
    main()
