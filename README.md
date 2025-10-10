# SMC Diagnostics GUI

A standalone desktop application for communicating with **SMC chillers** via **Modbus ASCII** over serial connection.  
The app allows users to monitor, log, and control diagnostic parameters such as temperature, pressure, conductivity, alarms, and setpoints in real time.

---

## Features

- **Real-time polling** of chiller diagnostic registers  
- **Dynamic graphs** with multiple selectable data fields  
- **Live chiller status indicator** (RUN / STOP bit)  
- **Setpoint control** with °C/°F write-back and validation  
- **Data logging** (on-screen table + CSV file export)  
- **Continuous polling** when the serial port is open  
- **Demo Mode** for testing without hardware (simulated data)  
- **Modern PyQt5 interface** with dark theme and responsive layout  

---

## How It Works

The application communicates with an SMC chiller using the **Modbus ASCII** protocol (7E1, 9600/19200 bps).  
When connected to a serial port:

1. The app continuously polls registers `0x0000` – `0x000B` for diagnostic data.  
2. Each checked diagnostic field appears in the live graph and log.  
3. The **RUN/STOP bit** at register `0x0004` updates the status lamp automatically.  
4. The **Fluid Set Temp** register (`0x000B`) can be read and written directly.  
5. Data can be saved to CSV in wide format (each field is a column).  

---

## Technologies Used

- **Python 3.8+** (Windows 7 compatible)
- **PyQt5** — GUI framework  
- **pyqtgraph** — real-time plotting  
- **pyserial** — serial communication  
- **dataclasses / typing** — structured field mapping  
- **CSV I/O** — data export  

---

## Running the Application

### Option 1 — Run from Source
```bash
# Clone this repo
git clone https://github.com/<your-username>/smc-diagnostics-gui.git
cd smc-diagnostics-gui

# (Recommended) create virtual environment
python -m venv .venv
.\.venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run
python smc_diag_gui.py
```
Demo Mode (No Hardware Needed)

To test the UI and plotting logic without connecting a real chiller:

Open the script smc_diag_gui.py.

Set the following line near the top:

DEMO_MODE = True


Launch the app — the demo will simulate live register data for temperature, pressure, alarms, and setpoint.

Future Roadmap

 Multi-platform build (macOS / Linux)

 Web or mobile dashboard (Wi-Fi bridge integration)

 BLE or Wi-Fi Modbus gateway support

 Trend saving and replay viewer

 User-editable register map
