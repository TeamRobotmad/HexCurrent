# HexCurrent

Standalone EEPROM-side app source for the Team RobotMad HexCurrent hexpansion.

This repository starts from the HexTest EEPROM app as a functional baseline, but the app itself has been refocused around one job: monitoring the INA226 current and voltage sensor on a HexCurrent board.

HexCurrent is intended for badge hexpansion developers who need to measure real-world current draw from a Hexpansion Under Test. The board sits between the badge and the device under test, passes through the normal hexpansion signals, and exposes an INA226 so the companion app can show live current and voltage and record time-series capture data.

## What This Repo Contains

- `hexcurrent.py`: the MicroPython app that is compiled to `.mpy`, copied onto a hexpansion EEPROM, and run by BadgeOS as `app.mpy`.

The exported runtime class is `HexCurrentApp`.

## Usage Modes

HexCurrent supports two deployment models:

- Installed on the HexCurrent EEPROM itself, in which case the app runs directly from that port.
- Installed permanently on the badge, in which case it scans all hexpansion slots and uses the first port that exposes an INA226.

If the Hexpansion Under Test includes its own EEPROM, the badge-installed mode must be used so the HexCurrent EEPROM is not present on the shared I2C bus.

## Current Scope

The current implementation provides:

- Live monitor mode showing current and voltage from the INA226.
- Timed capture mode with a persisted timeout setting.
- Manual stop of an in-progress capture using the confirm button.
- Post-capture charting of current and voltage over time.
- CSV export of recorded samples.
- Optional save-to-hexpansion-filesystem support when the app is running from the HexCurrent EEPROM.

The app intentionally does not retain the HexTest wheel-speed, pulse counter, motor-control, or HexDrive integration paths.

## Building

Compile the EEPROM app with `mpy-cross`:

```bash
mpy-cross -v hexcurrent.py -o hexcurrent.mpy
```

The resulting `hexcurrent.mpy` should then be copied into the consuming app's `EEPROM/` directory. When a host app writes that file to a hexpansion EEPROM it is renamed to `app.mpy` on the EEPROM so BadgeOS will discover it automatically.

## Hardware Notes

- HexCurrent uses VID `0xCBCB`, PID `0x5000`.
- The onboard INA226 reports bus voltage in millivolts and current in milliamps.
- An optional flying lead can be used to probe a voltage point on the device under test while the app continues to log current and voltage samples.

## Intended Consumer

The first intended consumer is HexManager, which can vendor this repository as a git submodule and compile `hexcurrent.py` into `EEPROM/hexcurrent.mpy` for EEPROM programming.