# Narbis Earclip — Functional Test Instructions (for the assembly vendor)

**Purpose:** confirm each assembled V2.1 earclip board is functional before
shipment. This is a go/no-go check of every component and connection —
optical performance limits are **informational at this stage** (record the
numbers; Narbis sets binding limits after receiving first articles).

**You need:** a Windows/Mac PC with Google Chrome, a USB-C **data** cable,
the firmware file `narbis-earclip-functest.bin` (supplied), the test
dashboard (link supplied by Narbis), and for the optional sleep-current
step: a bench multimeter (µA range) in series with the battery.

---

## 1. Load the firmware (once per board, via USB)

1. Install the flasher once per PC: `pip install esptool`
   (Python from python.org if `pip` is missing.)
2. Plug the board's USB-C into the PC. A new serial port appears
   (Windows: check Device Manager → Ports; it is a "USB Serial Device").
   *If no port appears:* try another cable (must be a data cable), then
   hold the small **BOOT** button on the XIAO module while plugging in.
3. Flash (replace `COM5` with your port):

```
python -m esptool --chip esp32c6 -p COM5 -b 460800 write_flash 0x0 narbis-earclip-functest.bin
```

4. Success looks like `Wrote ... bytes ... Hash of data verified.` The board
   reboots by itself.

**After boot the red LED glows dimly and blinks once per second.** That is
the "powered on, waiting for connection" indicator (it is dim by design —
the LED runs at a 1% optical duty cycle). No blink = board fails, set aside.

## 2. Button check (built into normal use)

- **Power ON:** hold the side button **1 second** (from off).
- **Power OFF:** hold the side button **5 seconds** (LED stops).
- Short presses do nothing else in this firmware. The dashboard shows a
  live "button pressed" indicator — you will verify the button during the
  test sequence.

## 3. Run the functional test

1. Open the dashboard link in **Chrome** (Edge also works; not Safari/iOS).
2. Power the board on (LED blinking), click **Connect**, pick
   `Narbis Edge Earclip TEST` from the list. The LED goes **steady** when
   connected.
3. Type the board's **serial number** into the serial field.
4. Click **Start sequence** and follow the prompts. Automatic steps run by
   themselves; a few steps ask you to do something (place a fingertip on
   the sensor, press the button 5 times, plug/unplug USB, confirm you can
   see the red LED glow, rotate the board through six orientations).
5. When the sequence ends: click **Download JSON** (the report card) and,
   from the Dashboard tab, make a ~60 second **recording** with a fingertip
   on the sensor and download the `.zip`.
6. **Ship gate:** every step PASS (optical steps report values — they
   cannot fail at this stage), report + recording saved per serial number.
   Send all files to Narbis with the shipment.

## 4. Optional: deep-sleep current (sample plan per Narbis PO)

1. Meter in µA range in series with the battery.
2. Dashboard → **Deep-sleep current test** → the board sleeps 10 s later.
3. Record the settled reading (expected ≤ 80 µA). Wake = hold button 1 s.

## 5. Do not update firmware

The board must ship with exactly the firmware you loaded in step 1.
Narbis installs production firmware later, wirelessly. If a board was
flashed with anything else, re-run step 1.

## Troubleshooting

| Symptom | Do |
|---|---|
| No serial port on USB | Data cable? Different port? Hold BOOT while plugging. |
| Port exists, flash fails | Close other serial programs; retry at `-b 115200`. |
| No LED blink after boot | Re-flash once; still dark → **fail unit** (log it). |
| Not in Chrome's device list | Board on? (hold 1 s, look for blink) · within 1 m? · Chrome has Bluetooth permission? |
| Connect drops repeatedly | Move away from other 2.4 GHz gear; retry; note it in the report. |
| A step fails | Re-run the step once (Skip → rerun). Persistent fail → record and set the unit aside. |

Questions / results submission: dgreco@narbis.com
