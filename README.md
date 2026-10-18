# Chiller Diagnostics GUI

A standalone desktop application for communicating with **SMC industrial chillers** via **Modbus ASCII** over a serial connection.
The app lets you monitor, log, and control diagnostic parameters such as temperature, pressure, conductivity, run state, and setpoint in real time.

---

## Features

- **Multiple chiller models** — pick the register map from the *Model* drop-down
- **Real-time polling** of diagnostic registers, one request/response at a time with timeouts
- **Dynamic graphs** with multiple selectable data fields (Heat Transfer on its own right-hand axis)
- **Live system status indicator** (RUN / STOP bit) with confirmed, retried Run/Stop commands
- **Setpoint control** with °C/°F write-back and per-model range validation
- **Heat transfer calculation** (W) from flow rate, fluid type, and supply/return temperatures
- **Data logging** — on-screen table with raw TX/RX frames, plus CSV export (flushed every row)
- **Demo mode** for testing without hardware (`--demo`)
- **PyQt5 interface** with dark theme

---

## Supported models

| Model | Registers polled | Run cmd | Status (bit0) | Setpoint | Units |
|---|---|---|---|---|---|
| Standard | `0x0000–0x000B`, `0x5200–0x5201` | `0x000C` | `0x0004` | `0x000B` (5–40 °C) | 0.1 °C |
| HEC | `0x0040–0x0051` | `0x0050` | `0x0043` | `0x0051` (10–60 °C) | 0.01 °C |

To add a model, append a `ChillerModel` entry to `MODELS` in `ChillerDiagGui.py`.

---

## Running from source

Requires **Python 3.8** (the last version supporting Windows 7).

```
py -3.8 -m venv .venv38
.venv38\Scripts\pip install -r requirements.txt
.venv38\Scripts\python ChillerDiagGui.py          # real serial port
.venv38\Scripts\python ChillerDiagGui.py --demo   # simulated chiller
```

Default serial framing is 7E1 at 9600 baud; all settings can be changed in the UI before opening the port.

## Installing (end users)

Download `ChillerDiagGui-Setup-<version>.exe` from the GitHub Releases page, run it, and click through the wizard. No admin rights or other software are needed; it installs for the current user and adds a Start Menu entry (and optionally a desktop shortcut). Uninstall from *Settings → Apps*. Running a newer setup upgrades the existing install.

Requirements: Windows 7 SP1 or later (Windows 7/8.1 also need update KB2999226 — the Universal C Runtime), plus the driver for your USB-serial adapter.

IT can install for all users with `ChillerDiagGui-Setup-<version>.exe /ALLUSERS`, or silently with `/VERYSILENT`.

## Building a release

Requires the `.venv38` environment above and [Inno Setup 6](https://jrsoftware.org/isdl.php).

1. Bump the version in `version_info.txt` (both `filevers`/`prodvers` and the `FileVersion`/`ProductVersion` strings).
2. Run `build.bat`.

This builds the app into `dist\ChillerDiagGui\` with PyInstaller, then packages it as `dist\ChillerDiagGui-Setup-<version>.exe` using `installer\ChillerDiagGui.iss`. Upload the setup exe to a GitHub Release; don't commit build output.

### Antivirus / SmartScreen warnings

Unsigned PyInstaller executables are sometimes flagged on Windows 10/11:

- **"Windows protected your PC"** (blue SmartScreen screen) is a reputation check for unsigned downloads. Click *More info → Run anyway*; code-signing the exe removes it.
- **"Threat found"** (e.g. `Trojan:Win32/Wacatac`) is a heuristic false positive. Report it at https://www.microsoft.com/wdsi/filesubmission ("incorrectly detected"), or have IT allowlist the install folder.

The build already disables UPX compression and embeds version info, both of which reduce false positives.

---

## Technologies Used

- **Python 3.8** (Windows 7 compatible)
- **PyQt5** — GUI framework
- **pyqtgraph** — real-time plotting
- **pyserial** — serial communication
- **PyInstaller** — packaging
