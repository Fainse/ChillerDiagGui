#Chiller Diagnostics GUI

A standalone desktop application for communicating with **industrial chillers** via **Modbus ASCII** over serial connection.  
The app allows users to monitor, log, and control diagnostic parameters such as temperature, pressure, conductivity, alarms, and setpoints in real time.

---

## Features

- **Real-time polling** of diagnostic registers  
- **Dynamic graphs** with multiple selectable data fields  
- **Live system status indicator** (RUN / STOP bit)  
- **Setpoint control** with °C/°F write-back and validation  
- **Data logging** (on-screen table + CSV export)  
- **Continuous polling** when the serial port is open  
- **Demo Mode** for testing without hardware (simulated data)  
- **Modern PyQt5 interface** with dark theme and responsive layout  

---

## How It Works

The application communicates with a chiller using the **Modbus ASCII** protocol (7E1, 9600/19200 bps).  
When connected to a serial port:

1. The app continuously polls registers `0x0000` – `0x000B` for diagnostic data.  
2. Each selected diagnostic field appears in the live graph and data log.  
3. The **RUN/STOP bit** at register `0x0004` updates the status indicator automatically.  
4. The **Fluid Set Temp** register (`0x000B`) can be read and written directly.  
5. Data can be saved to CSV in wide format (each field as a column).  

---

## Technologies Used

- **Python 3.8+** (Windows 7 compatible)
- **PyQt5** — GUI framework  
- **pyqtgraph** — real-time plotting  
- **pyserial** — serial communication  
- **dataclasses / typing** — structured field mapping  
- **CSV I/O** — data export  

---

