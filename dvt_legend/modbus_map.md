# Modbus map, DVT 510MR @ 192.168.10.242:502 (read-only, unit/device id 1)

Source: modbus_unit1.json, modbus_base_1..3.json, modbus_part_1..3.json, modbus_pre_yi.json, modbus_post_yi.json, step_taskB_timeseries.txt.

## Readable ranges (CONFIRMED)
FC1 coils, FC2 discrete inputs, FC3 holding registers, FC4 input registers: **every address 0..255 answers, no exception.** FC1==FC2 and FC3==FC4 (mirrored).

## Static at idle
bits 32, 52, 58, 73 = 1 | reg 2 = 256 (0x0100) | reg 4 = 2 | everything else 0.

## Live, but free-running (CONFIRMED by time series, ~6 Hz polling over 12 s)
reg 3 and bits 53, 54, 55, 59 cycle about once per second: reg 3 steps 4100 (0x1004) -> 36868 (0x9004) -> a burst value (20492 / 12300 / 45068 / 53260, i.e. 0x?00C) -> 4100 ...
Coil 55 = bit 15 of reg 3's pattern; 53/54/59 appear in the burst.
This pattern is the same:
- before and after a part was put under the lens (3 reads each),
- before and after one `#YI` at t=4 s (step_taskB_timeseries.txt: no change in rhythm or values at the trigger).
HYPOTHESIS: a heartbeat / rotating status field of the camera's I/O task, not an inspection result.

## Diff with a part under the camera
No address differs between baseline and part-present reads (only the free-running ones above).

## NOT found
No address that changes with the scene or with the trigger => **no Pass/Fail signal located** in 0..255 on unit 1. Not tried: addresses above 255, other unit ids.
