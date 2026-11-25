# IMU data sending

How the boat's IMU data gets from the ESP32 to InfluxDB, and what the overnight tests showed.
Branch `imu-data`, made from `timestamps-code` plus the IMU handler in
`src/mqtt_subscriber_real_test.py` (nothing else is changed).

## How it works

- The ESP32 takes an IMU sample every 10 ms (100 Hz). Every 10th sample is a 100 ms "tick", and
  the other telemetry groups of that tick carry exactly that sample's time.
- Every tick it publishes the 10 samples of the previous tick on `boat/telemetry/imu/batch`
  (about 490 B), QoS 0, with one array per key:
  ```json
  {"count":10,"seq":1,"ts":"2026-11-24T20:34:33.123Z","t":[0,10,...,90],"ax":[...],...,"gz":[...]}
  ```
  `ts` is the time of the first sample; `t` is each sample's offset in ms.
- The bridge writes each sample as `imu_ax` ... `imu_gz` in the `telemetry` measurement. It also
  still reads the old form (`"samples":[{...},...]`). A batch without a valid `ts` is not written.

The test data is synthetic (the firmware is the link test in the sibling folder
`esp32_link_test`, which is not in this repo).

## Results

| | 16-17 Nov 2026 | 24-25 Nov 2026 |
|---|---|---|
| IMU message | 100 samples once per s (~7.7 kB) | 10 samples per tick (~490 B) |
| Serial log length | about 9.4 h | 4.2 h |
| Phase B ticks | 332,940 | 137,715 |
| Over 100 ms late, all | 9.72 % | 1.60 % |
| Over 100 ms late, without connection trouble | 9.62 % | 0.30 % |

"Connection trouble" leaves out ticks from 2 s before to 15 s after a failed publish, a
disconnect or a tick over 1 s late. An earlier run without IMU had 0.17 % over 100 ms late. The
runs were on different nights, so the network was not identical.

24-25 Nov, late ticks per hour: 0.95 % (21h), **6.13 % (22h)**, 0.27 % (23h), 0.21 % (00h),
0.11 % (01h). The 22h value is the connection outages, mainly 22:34-22:48.

### IMU batches, 24-25 Nov (while the serial log ran)

| | Batches | Share |
|---|---|---|
| Expected (one per 100 ms) | about 151,207 | 100 % |
| Not in the ESP32's log (mostly publisher too far behind, probably) | about 4,678 | 3.1 % |
| Logged, publish failed | 3,727 | 2.5 % |
| Published, never reached the bridge | 124 | 0.08 % |
| **Received by the bridge** | **142,678** | **94.36 %** |

All received batches had 10 samples; there were no malformed-batch warnings and no duplicates.

## Observations

1. **Smaller batches fixed most of the lateness.** The 7.7 kB message blocked the publisher for
   about 200 ms every second, which made the next tick late. With 10 samples per tick, 99 % of
   publishes take under 50 ms.
2. **The remaining late ticks come from slow publishes, not from QoS 0.** A late tick is a
   message that goes out late because the publisher was still busy with an earlier tick. Of the
   2,207 late ticks, 2,196 came right after a tick whose delay plus publish time was over 100 ms,
   and 1,822 were in connection trouble. QoS 0 does not delay a message: it is written once to
   the TLS connection, and the publisher waits if that connection is stalled.
3. **QoS 0 explains losses, at most in part.** The MQTT client does not resend a QoS 0 message,
   so the 124 published-but-not-received batches fit it. The 3,727 failed publishes also depend
   on it: at QoS 0, esp-mqtt rejects a publish while disconnected, where QoS 1 would keep it in
   its outbox. The 4,678 batches missing from the log were never handed to MQTT, so QoS does not
   affect them.
4. **QoS 1 is untested here.** It might recover some batches after an outage, but it adds traffic
   and outbox memory, and it would not stop a stalled connection from blocking the publisher.
5. **Wi-Fi quality is not ruled out.** RSSI had a median of -64 dBm and a minimum of -89 dBm. The
   cause of the outages has not been confirmed.

## What this run does not show

- **InfluxDB:** the `dtufloatforward.com` domain did not resolve (registration on `clientHold`),
  so the bridge logged 2,054 write errors and 405 failed batches. The exact-timestamp and
  same-row checks could not be done. The bridge log has no IMU values, so it can't be backfilled.
- **The full night:** the Mac's battery ran out at 01:46 and it slept until 06:33, which cut the
  ESP32's USB power. The bridge received 99.58 % of the expected batches from 06:33 to 08:28, but
  there is no serial log for that part.
- **Real sensor behaviour:** the values are synthetic.

## Next

1. 1-2 h run once InfluxDB is reachable, then `compare_influx_ts.py`.
2. Full night with the Mac on its charger.
3. If losses in outages matter: same test with QoS 1 (`LT_QOS` in menuconfig), comparing late
   ticks and received batches.
