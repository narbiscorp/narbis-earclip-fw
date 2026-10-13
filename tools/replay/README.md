# replay — the firmware's beat detection, re-run on recorded data

Two settings can only be compared fairly on the **same** data: two people (or two placements)
differ by more than the settings do. These tools run the firmware's own C code — `narbis_core`
`nc_dsp` (DC removal + band-pass), `nc_gate` (artifact gate, accel energy, DC step) and
`nc_ibi` (beat detector) — over a dashboard recording, and score the result against the
Polar H10 recorded beside it.

## Build

```
cd test_host
/c/msys64/mingw64/bin/gcc.exe -std=c11 -O2 -Wall -Wextra -Werror \
  -I../firmware/components/narbis_core/include -Isupport \
  ../tools/replay/replay_ibi.c ../firmware/components/narbis_core/src/*.c support/*.c \
  -o out/replay_ibi.exe -lm
```

## Use

A guided recording from the dashboard (the session zip: `v2/ppg_raw.csv`, `v2/accel_raw.csv`,
`v2/ibi.csv`, `protocol/blocks.csv`, `reference_h10/h10_beats.csv`):

```
python tools/replay/replay_session.py session_....zip \
  --config live \
  --config old="--gate replay --knob 0x0708=1" \
  --config gentle="--gate replay"
```

Each `--config` is a label and `replay_ibi` arguments; `live` scores the beats the board
emitted while recording. `--knob 0xID=value` sets any knob (IDs in
`firmware/components/narbis_core/include/narbis/knob_list.h`), e.g. `0x060C` refract_frac_pct,
`0x0703` gate_acc_thr, `0x0708` gate_acc_hard_thr. Gate modes: `recorded` (the capture's own
GATE flags), `none` (never gated: an upper bound), `replay` (recomputed from the accelerometer).

Per block it prints the H10's beats, how many the earclip found, how many earclip beats matched
none (false), and the interval error where both ends of an interval line up with the H10.

## What it does not replay

- Ambient subtraction (recordings carry no ambient channel): the band-pass makes this small.
- AGC steps (a recording with LED / gain steps replays them as raw steps), and the AGC itself.
- The wear detector (the capture's WEAR_OFF flag is used as recorded).
- The board updates its accel energy in FIFO bursts, the replay at each sample's time: replayed
  and recorded gates agreed on 89 % of samples in the first recording checked.

On recording 2026-10-13 00:24 the replay's beats were within 20 ms of the board's live beats for
96 % of them (median 0.5 ms). Scored against the H10 with `--gate recorded`, it finds the beats
the board found (foot tap / chew / talk 65 / 36 / 54 % against the board's 65 / 34 / 53 %), but
its intervals are a few ms noisier (rest: median 7 ms against 4 ms live), most likely from the
missing ambient subtraction. So use it to compare settings with each other, and quote absolute
accuracy from live recordings.

## Results so far (2026-10-12, three guided runs, 100 sps)

Beats found, by block (H10 = 100 %). "old" = 1.3's single threshold (`--knob 0x0708=1`),
"gentle" = 1.4's defaults; both replayed on the same data:

| run | gate | rest | foot tap | chew | talk |
|---|---|---|---|---|---|
| 1 (v0.3.0-40) | old | 99 % | 55 % | 32 % | 51 % |
| 1 | gentle | 99 % | 100 % | 81 % | 91 % |
| 2 (v0.3.0-42) | old | 95 % | 63 % | 42 % | 51 % |
| 2 | gentle | 95 % | 98 % | 92 % | 87 % |
| 3 (v0.3.0-42, weak pulse) | old | 69 % | 68 % | 56 % | 37 % |
| 3 | gentle | 71 % | 75 % | 57 % | 50 % |

False beats (earclip beats with no H10 match) stayed about the same (chew 12 → 13, talk 10 → 7
on run 1). The beats found through motion are a little less precise: median interval error
7–13 ms, p90 18–32 ms, against 5–8 / 10–18 ms at rest (runs 1–2, replayed; the board's
own live rest beats on run 1 were median 3–4 ms, 95–97 % within 10 ms).
