# -*- coding: utf-8 -*-
# Windows 7 compatible build: Python 3.8.x + PyQt5 5.15.x
import sys, time, binascii, csv, math
from dataclasses import dataclass
from typing import Dict, Callable, List, Tuple
from PyQt5 import QtCore, QtWidgets
import serial, serial.tools.list_ports
import pyqtgraph as pg

# -------------------- Settings --------------------
SLAVE_ID = 1
POLL_BASE_ADDR = 0x0000
POLL_QTY = 12                 # 0x0000 .. 0x000B inclusive
DEFAULT_BAUD = 9600

# Serial: ASCII, 7-E-1 per manual
BYTESIZE = serial.SEVENBITS
PARITY   = serial.PARITY_EVEN
STOPBITS = serial.STOPBITS_ONE
READ_TIMEOUT  = 0.5
WRITE_TIMEOUT = 0.5

# Command register (run/stop)
RUN_CMD_ADDR = 0x000C
RUN_VALUE    = 0x0001   # 1 = Run start
STOP_VALUE   = 0x0000   # 0 = Run stop

# Status word (for lamp)
STATUS_ADDR = 0x0004     # bit0 = Run

# Setpoint register (R/W)
SETPOINT_ADDR = 0x000B   # "Fluid Set Temp"

DEMO_MODE = True  # True = simulate without hardware; False = real serial

# -------------------- Modbus ASCII helpers --------------------
def lrc(bs: bytes) -> int:
    s = sum(bs) & 0xFF
    return (-s) & 0xFF

def build_read_ascii(slave: int, start_addr: int, quantity: int) -> bytes:
    pdu = bytes([
        slave & 0xFF, 0x03,
        (start_addr >> 8) & 0xFF, start_addr & 0xFF,
        (quantity   >> 8) & 0xFF, quantity   & 0xFF
    ])
    chk = lrc(pdu)
    return b':' + binascii.hexlify(pdu + bytes([chk])).upper() + b'\r\n'

def build_write_single_register_ascii(slave: int, addr: int, value: int) -> bytes:
    pdu = bytes([
        slave & 0xFF, 0x06,
        (addr >> 8) & 0xFF, addr & 0xFF,
        (value >> 8) & 0xFF, value & 0xFF
    ])
    chk = lrc(pdu)
    return b':' + binascii.hexlify(pdu + bytes([chk])).upper() + b'\r\n'

def parse_ascii_frame(line_between_colon_and_cr: bytes):
    try:
        raw = binascii.unhexlify(line_between_colon_and_cr)
    except binascii.Error:
        return None
    if len(raw) < 3:
        return None
    addr, func = raw[0], raw[1]
    data, recv_lrc = raw[2:-1], raw[-1]
    if lrc(raw[:-1]) != recv_lrc:
        return None
    return addr, func, data

class ModbusAsciiParser:
    def __init__(self):
        self.buf = bytearray()
    def feed(self, chunk: bytes):
        out = []
        self.buf.extend(chunk)
        while True:
            try:
                i = self.buf.index(0x3A)  # ':'
            except ValueError:
                self.buf.clear()
                break
            if i > 0:
                del self.buf[:i]
            try:
                j = self.buf.index(0x0A, 1)  # LF
                if j < 2 or self.buf[j-1] != 0x0D:
                    del self.buf[:j+1]
                    continue
            except ValueError:
                break
            line = bytes(self.buf[1:j-1])  # between ':' and CR
            del self.buf[:j+1]
            f = parse_ascii_frame(line)
            if f:
                out.append(f)
        return out

def parse_0x03_registers(data_bytes: bytes) -> List[int]:
    if not data_bytes or len(data_bytes) < 1:
        return []
    count = data_bytes[0]
    regs = []
    for i in range(1, 1 + count, 2):
        if i + 1 < len(data_bytes):
            regs.append((data_bytes[i] << 8) | data_bytes[i+1])
    return regs

# -------------------- Register map & scaling --------------------
@dataclass
class FieldSpec:
    addr: int
    qty: int
    scale: Callable[[List[int]], float]
    unit: str

def s16(x: int) -> int:
    return x - 0x10000 if x >= 0x8000 else x

def scale_temp10_signed(regs: List[int]) -> float:    return s16(regs[0]) / 10.0
def scale_temp10_unsigned(regs: List[int]) -> float:  return regs[0] / 10.0
def scale_pressure_mpa(regs: List[int]) -> float:     return regs[0] * 0.01
def scale_pressure_psi(regs: List[int]) -> float:     return float(regs[0])
def scale_conductivity_uScm(regs: List[int]) -> float:return regs[0] * 0.1
def scale_resistivity_Mohm_cm(regs: List[int]) -> float: return regs[0] * 0.1

FIELD_MAP: Dict[str, FieldSpec] = {
    "Discharge Temp (°C)": FieldSpec(0x0000, 1, scale_temp10_signed, "°C"),
    "Discharge Temp (°F)": FieldSpec(0x0000, 1, scale_temp10_signed, "°F"),
    "Discharge Pressure (MPa)": FieldSpec(0x0002, 1, scale_pressure_mpa, "MPa"),
    "Discharge Pressure (PSI)": FieldSpec(0x0002, 1, scale_pressure_psi, "PSI"),
    "Conductivity (μS/cm)": FieldSpec(0x0003, 1, scale_conductivity_uScm, "μS/cm"),
    "Resistivity (MΩ·cm)":  FieldSpec(0x0003, 1, scale_resistivity_Mohm_cm, "MΩ·cm"),
    "Alarm Flag 1 (bits)": FieldSpec(0x0005, 1, lambda r: float(r[0]), "bits"),
    "Alarm Flag 2 (bits)": FieldSpec(0x0006, 1, lambda r: float(r[0]), "bits"),
    "Alarm Flag 3 (bits)": FieldSpec(0x0007, 1, lambda r: float(r[0]), "bits"),
    "Fluid Set Temp (°C)": FieldSpec(SETPOINT_ADDR, 1, scale_temp10_unsigned, "°C"),
}

# -------------------- Serial worker --------------------
class SerialWorker(QtCore.QThread):
    frameParsed = QtCore.pyqtSignal(int, int, bytes)
    status      = QtCore.pyqtSignal(str)
    error       = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.ser = None
        self._run = False
        self.parser = ModbusAsciiParser()
        self.tx_queue: List[bytes] = []

    def is_open(self) -> bool:
        try:
            return bool(self._run and self.ser and self.ser.is_open)
        except Exception:
            return False

    def open(self, port: str, baud: int):
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baud, bytesize=BYTESIZE,
                parity=PARITY, stopbits=STOPBITS,
                timeout=READ_TIMEOUT, write_timeout=WRITE_TIMEOUT
            )
            self._run = True
            if not self.isRunning():
                self.start()
            self.status.emit(f"Opened {port} @ {baud} (7E1)")
        except Exception as e:
            self.ser = None
            self.error.emit(f"Open failed: {e}")

    def close(self):
        self._run = False
        if self.ser:
            try: self.ser.close()
            except Exception: pass
            self.ser = None
            self.status.emit("Port closed")

    @QtCore.pyqtSlot(bytes)
    def send(self, frame: bytes):
        self.tx_queue.append(frame)

    def run(self):
        while True:
            if not self._run or not self.ser:
                time.sleep(0.05)
                if not self._run and not self.ser:
                    break
                continue

            while self.tx_queue:
                frame = self.tx_queue.pop(0)
                try:
                    self.ser.write(frame)
                except Exception as e:
                    self.error.emit(f"Write error: {e}")

            try:
                chunk = self.ser.read(256)
                if chunk:
                    for (addr, func, data) in self.parser.feed(chunk):
                        self.frameParsed.emit(addr, func, data)
            except Exception as e:
                self.error.emit(f"Read error: {e}")
                self.close()

# -------------------- Demo worker (no hardware needed) --------------------
class DemoWorker(QtCore.QThread):
    frameParsed = QtCore.pyqtSignal(int, int, bytes)
    status      = QtCore.pyqtSignal(str)
    error       = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._run = False
        self.tx_queue: List[bytes] = []
        self._t0 = time.time()
        self._timer = QtCore.QTimer()
        self._timer.setInterval(100)
        self._timer.timeout.connect(self._process)
        self._run_state = 0  # 0=OFF, 1=ON (changed by writes to 0x000C)
        self._setpoint_raw = int(round(22.0 * 10))  # default 22.0°C in raw 0.1°C

    def is_open(self) -> bool:
        return bool(self._run)

    def open(self, port: str, baud: int):
        self._run = True
        if not self.isRunning():
            self.start()
        self._timer.start()
        self.status.emit(f"(DEMO) Opened virtual device @ {baud} (7E1)")

    def close(self):
        self._run = False
        self._timer.stop()
        self.status.emit("(DEMO) Port closed")

    @QtCore.pyqtSlot(bytes)
    def send(self, frame: bytes):
        # Recognize 0x03 vs 0x06 so demo honors ON/OFF/SETPOINT commands and ACKs writes
        try:
            if not frame or frame[0:1] != b':' or not frame.endswith(b'\r\n'):
                return
            raw = binascii.unhexlify(frame[1:-2])
        except Exception:
            return
        if len(raw) < 3:
            return
        func = raw[1]
        if func == 0x03:
            self.tx_queue.append(b'R')
        elif func == 0x06:
            if len(raw) >= 7:
                reg_addr = (raw[2] << 8) | raw[3]
                reg_val  = (raw[4] << 8) | raw[5]
                if reg_addr == RUN_CMD_ADDR:
                    self._run_state = 1 if reg_val != 0 else 0
                elif reg_addr == SETPOINT_ADDR:
                    self._setpoint_raw = reg_val  # accept raw write as-is
            self.frameParsed.emit(SLAVE_ID, 0x06, b'')

    def run(self):
        loop = QtCore.QEventLoop()
        while self._run:
            loop.processEvents(QtCore.QEventLoop.AllEvents, 50)
            time.sleep(0.01)

    def _process(self):
        if not self._run or not self.tx_queue:
            return
        _ = self.tx_queue.pop(0)

        t = time.time() - self._t0
        regs = [0] * POLL_QTY

        # Demo signals
        regs[0] = int(round((25 + 5 * math.sin(t/6.0)) * 10))          # 0x0000 temp (0.1 units)
        regs[2] = int(round((1.05 + 0.25 * math.sin(t/8.0)) * 100))    # 0x0002 pressure (0.01 MPa)
        regs[3] = int(round((30 + 20 * math.sin(t/10.0)) * 10))        # 0x0003 conductivity (0.1)
        regs[4] = 1 if self._run_state else 0                          # 0x0004 status bit0
        regs[11]= self._setpoint_raw                                   # 0x000B setpoint (raw)

        bc = POLL_QTY * 2
        payload = bytearray([bc])
        for v in regs:
            payload.extend([(v >> 8) & 0xFF, v & 0xFF])

        self.frameParsed.emit(SLAVE_ID, 0x03, bytes(payload))

# -------------------- Main GUI --------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SMC Diagnostics (Win7/PyQt5)")

        # Worker
        self.worker = DemoWorker() if DEMO_MODE else SerialWorker()
        self.worker.status.connect(self.on_status)
        self.worker.error.connect(self.on_error)
        self.worker.frameParsed.connect(self.on_frame)

        # Top controls
        top = QtWidgets.QWidget()
        tl = QtWidgets.QGridLayout(top)
        tl.setHorizontalSpacing(10)
        tl.setVerticalSpacing(8)
        tl.setContentsMargins(8, 8, 8, 4)


        self.portBox = QtWidgets.QComboBox()
        self.refreshBtn = QtWidgets.QPushButton("Refresh")
        self.baudBox = QtWidgets.QComboBox()
        self.baudBox.addItems(["9600", "19200"])

        # Port toggle + status
        self.portToggleBtn = QtWidgets.QPushButton("Open Port")
        self.portToggleBtn.setCheckable(True)
        self.portToggleBtn.clicked.connect(self.toggle_port)
        self.portStatusDot = QtWidgets.QLabel("● Closed")
        self.portStatusDot.setStyleSheet("color:#cc0000; font-weight:600;")

        self.pollMs = QtWidgets.QSpinBox()
        self.pollMs.setRange(100, 10000)
        self.pollMs.setValue(1000)

        row = 0
        tl.addWidget(QtWidgets.QLabel("Port"), row, 0); tl.addWidget(self.portBox, row, 1)
        tl.addWidget(self.refreshBtn, row, 2)
        tl.addWidget(QtWidgets.QLabel("Baud"), row, 3); tl.addWidget(self.baudBox, row, 4)
        tl.addWidget(self.portToggleBtn, row, 5)
        tl.addWidget(self.portStatusDot, row, 6)
        tl.addItem(QtWidgets.QSpacerItem(20, 10,
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Minimum), row, 99)

        row += 1
        tl.addWidget(QtWidgets.QLabel("Poll (ms)"), row, 0); tl.addWidget(self.pollMs, row, 1)

        # Chiller toggle + status
        self.chillerToggleBtn = QtWidgets.QPushButton("Turn Chiller ON")
        self.chillerToggleBtn.setCheckable(True)
        self.chillerToggleBtn.toggled.connect(self.toggle_chiller)
        self.statusDot = QtWidgets.QLabel("● OFF")
        self.statusDot.setStyleSheet("color:#cc0000; font-weight:600;")
        tl.addWidget(QtWidgets.QLabel("Chiller Control"), row, 3)
        tl.addWidget(self.chillerToggleBtn, row, 4)
        tl.addWidget(self.statusDot, row, 5)
        tl.addItem(QtWidgets.QSpacerItem(20, 10,
            QtWidgets.QSizePolicy.Expanding,
            QtWidgets.QSizePolicy.Minimum), row, 99)


        # -------- Fluid Set Temp (write) --------
        row += 1
        tl.addWidget(QtWidgets.QLabel("Fluid Set Temp"), row, 0)
        self.setTempSpin = QtWidgets.QDoubleSpinBox()
        self.setTempSpin.setDecimals(1)
        self.setTempSpin.setSingleStep(0.1)
        self.setTempSpin.setKeyboardTracking(False)
        self.setTempSpin.setAccelerated(True)
        # Allow ANY value to be typed; we'll validate on Set click
        self.setTempSpin.setRange(-999.0, 999.0)
        self.setTempSpin.setValue(22.0)
        self.setTempSpin.setSuffix(" °C")

        self.setTempUnit = QtWidgets.QComboBox()
        self._settemp_unit = "°C"  # track previous unit for conversion on change
        self.setTempUnit.currentIndexChanged.connect(self._on_settemp_unit_changed)
        self.setTempUnit.addItems(["°C", "°F"])
        self.setTempUnit.currentIndexChanged.connect(self._on_settemp_unit_changed)
        self.setTempUnit.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        self.setTempUnit.setFixedWidth(58)  # ~ “°F” + padding

        self.writeSetTempBtn = QtWidgets.QPushButton("Set")
        self.writeSetTempBtn.clicked.connect(self.write_setpoint)

        # Last read (always °C)
        self.setTempReadLbl = QtWidgets.QLabel("Last read: --.- °C")
        self.setTempReadLbl.setStyleSheet("color:#BBBBBB;")

        tl.addWidget(self.setTempSpin, row, 1)
        tl.addWidget(self.setTempUnit, row, 2)
        tl.addWidget(self.writeSetTempBtn, row, 3)
        tl.addWidget(self.setTempReadLbl, row, 4, 1, 3)

        # Field list: checkboxes control logging + plotting (no selection)
        self.fieldList = QtWidgets.QListWidget()
        self.fieldList.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        for name in FIELD_MAP:
            it = QtWidgets.QListWidgetItem(name)
            it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
            it.setCheckState(QtCore.Qt.Unchecked)
            self.fieldList.addItem(it)
        self.fieldList.itemChanged.connect(self.on_field_check)

        # Plot: white background with black axes + anti-aliased lines
        pg.setConfigOptions(antialias=True)
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setBackground('w')
        for axis in ['bottom', 'left']:
            ax = self.plot.getPlotItem().getAxis(axis)
            ax.setPen('k'); ax.setTextPen('k')

        # Legend + per-field curves (distinct pens)
        self.legend = self.plot.addLegend(); self.legend.setVisible(True)
        self.curves: Dict[str, pg.PlotDataItem] = {}
        self._pens: Dict[str, pg.mkPen] = {}
        def _pen_for(name: str):
            idx = (abs(hash(name)) % 12)  # 12 distinct hues
            return pg.mkPen(pg.intColor(idx, hues=12), width=2)
        self._pen_for = _pen_for

        # History (name -> (xs, ys))
        self.history: Dict[str, Tuple[List[float], List[float]]] = {}
        self.t0 = time.time()

        # Table/log (on-screen)
        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["Timestamp", "Field", "Value"])

        # Start/Stop saving (wide CSV of selected fields)
        self.saveToggleBtn = QtWidgets.QPushButton("Start Saving")
        self.saveToggleBtn.setCheckable(True)
        self.saveToggleBtn.toggled.connect(self.toggle_saving)
        self.saving_active = False
        self.csv_file = None
        self.csv_writer = None
        self.csv_header_written = False
        self.save_fields: List[str] = []  # columns frozen at start

        # Layout middle
        mid = QtWidgets.QSplitter()
        left = QtWidgets.QWidget(); ll = QtWidgets.QVBoxLayout(left)
        ll.addWidget(QtWidgets.QLabel("Diagnostics (check to log & plot):"))
        ll.addWidget(self.fieldList)
        right = QtWidgets.QWidget(); rl = QtWidgets.QVBoxLayout(right)
        rl.addWidget(self.plot, 3); rl.addWidget(self.table, 2); rl.addWidget(self.saveToggleBtn)
        mid.addWidget(left); mid.addWidget(right); mid.setStretchFactor(1, 3)

        central = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(central)
        layout.addWidget(top); layout.addWidget(mid, 1)
        self.setCentralWidget(central)

        # Signals
        self.refreshBtn.clicked.connect(self.refresh_ports)
        self.pollMs.valueChanged.connect(self.on_poll_interval_changed)

        # Timers
        self.pollTimer = QtCore.QTimer()
        self.pollTimer.timeout.connect(self.poll_once)

        # Status bar
        self.statusBar().showMessage("Ready")

        # Dark UI theme (black app, white chart)
        self.setStyleSheet("""
            QWidget { background-color: #000000; color: #FFFFFF; }
            QLineEdit, QComboBox, QSpinBox, QTableWidget, QTextEdit {
                background-color: #1e1e1e; color: #FFFFFF;
            }
            QPushButton {
                background-color: #333333; color: #FFFFFF;
                border: 1px solid #555; padding: 4px;
            }
            QPushButton:hover { background-color: #444444; }
            QHeaderView::section {
                background-color: #222222; color: #FFFFFF;
            }
        """)
        self.refresh_ports()

    # --------- Unit helpers ----------
    @staticmethod
    def c_to_f(c: float) -> float:
        return c * 9.0/5.0 + 32.0
    @staticmethod
    def f_to_c(f: float) -> float:
        return (f - 32.0) * 5.0/9.0

    def _on_settemp_unit_changed(self, _idx: int):
    
        new_unit = self.setTempUnit.currentText()
        old_unit = getattr(self, "_settemp_unit", "°C")
        val = self.setTempSpin.value()

        # Convert only if the unit actually changed
        if old_unit != new_unit:
            if old_unit == "°C" and new_unit == "°F":
                val = round(self.c_to_f(val), 1)
            elif old_unit == "°F" and new_unit == "°C":
                val = round(self.f_to_c(val), 1)

        # Update suffix and keep a very wide range (no clamping here)
        self.setTempSpin.blockSignals(True)
        self.setTempSpin.setSuffix(" " + new_unit)
        self.setTempSpin.setRange(-999.0, 999.0)
        self.setTempSpin.setValue(val)
        self.setTempSpin.blockSignals(False)

        # Remember current unit for next switch
        self._settemp_unit = new_unit


    # --------- UI helpers ----------
    def refresh_ports(self):
        self.portBox.clear()
        if DEMO_MODE:
            self.portBox.addItem("DEMO (simulated)", userData="DEMO")
            self.refreshBtn.setEnabled(False)
        else:
            self.refreshBtn.setEnabled(True)
            for p in serial.tools.list_ports.comports():
                label = f"{p.device} – {p.description}"
                self.portBox.addItem(label, userData=p.device)

    def set_port_indicator(self, is_open: bool):
        if is_open:
            self.portToggleBtn.setText("Close Port")
            self.portToggleBtn.setChecked(True)
            self.portStatusDot.setText("● Open")
            self.portStatusDot.setStyleSheet("color:#1a7f37; font-weight:600;")
            self.portBox.setEnabled(False)
            self.refreshBtn.setEnabled(False)
        else:
            self.portToggleBtn.setText("Open Port")
            self.portToggleBtn.setChecked(False)
            self.portStatusDot.setText("● Closed")
            self.portStatusDot.setStyleSheet("color:#cc0000; font-weight:600;")
            self.portBox.setEnabled(True)
            self.refreshBtn.setEnabled(False if DEMO_MODE else True)

    # --------- Port open/close ---------
    def toggle_port(self):
        if hasattr(self.worker, "is_open") and self.worker.is_open():
            self.close_port()
        else:
            self.open_port()

    def open_port(self):
        if hasattr(self.worker, "is_open") and self.worker.is_open():
            self.on_status("Port already open"); return

        baud = int(self.baudBox.currentText()) if self.baudBox.currentText() else DEFAULT_BAUD

        if DEMO_MODE:
            self.worker.open("DEMO", baud)
            self.t0 = time.time()
            self.poll_once()
            self.pollTimer.start(self.pollMs.value())
            self.set_port_indicator(True)
            return

        idx = self.portBox.currentIndex()
        port = self.portBox.itemData(idx) if idx >= 0 else None
        if port:
            self.worker.open(port, baud)
            self.t0 = time.time()
            self.poll_once()
            self.pollTimer.start(self.pollMs.value())
            self.set_port_indicator(True)
        else:
            self.on_status("No port selected")

    def close_port(self):
        if self.pollTimer.isActive():
            self.pollTimer.stop()
        self.worker.close()
        self.set_port_indicator(False)
        # Also stop saving if active (closes file cleanly)
        if self.saving_active:
            self.saveToggleBtn.setChecked(False)  # will call toggle_saving(False)

    # --------- Status + chiller control ----------
    def on_status(self, s: str):
        self.statusBar().showMessage(s, 5000)
    def on_error(self, e: str):
        self.statusBar().showMessage(e, 8000)

    def set_run_indicator(self, running: bool):
        if running:
            self.statusDot.setText("● ON")
            self.statusDot.setStyleSheet("color:#1a7f37; font-weight:600;")
        else:
            self.statusDot.setText("● OFF")
            self.statusDot.setStyleSheet("color:#cc0000; font-weight:600;")
        # Mirror device state on the toggle (without re-triggering slot)
        self.chillerToggleBtn.blockSignals(True)
        self.chillerToggleBtn.setChecked(running)
        self.chillerToggleBtn.setText("Turn Chiller OFF" if running else "Turn Chiller ON")
        self.chillerToggleBtn.blockSignals(False)

    def toggle_chiller(self, want_run: bool):
        value = RUN_VALUE if want_run else STOP_VALUE
        self.worker.send(build_write_single_register_ascii(SLAVE_ID, RUN_CMD_ADDR, value))
        # Lamp flips on next poll when STATUS bit confirms.

    # --------- Setpoint write ----------
    def write_setpoint(self):
        unit = self.setTempUnit.currentText()
        val  = round(self.setTempSpin.value(), 1)

        # Validate ONLY on click
        if unit == "°C":
            if not (5.0 <= val <= 40.0):
                QtWidgets.QMessageBox.warning(self, "Invalid setpoint",
                    "°C setpoint must be between 5.0 and 40.0")
                return
            raw = int(round(val * 10.0))  # 0.1°C/dig
        else:
            if not (41.0 <= val <= 104.0):
                QtWidgets.QMessageBox.warning(self, "Invalid setpoint",
                    "°F setpoint must be between 41.0 and 104.0")
                return
            raw = int(round(val * 10.0))  # 0.1°F/dig per spec

        self.worker.send(build_write_single_register_ascii(SLAVE_ID, SETPOINT_ADDR, raw))
        self.on_status(f"Setpoint written: {val:.1f} {unit} (raw {raw:#06x})")

    # --------- Polling ----------
    def on_poll_interval_changed(self, _):
        if self.pollTimer.isActive():
            self.pollTimer.stop()
            self.poll_once()  # immediate tick
            self.pollTimer.start(self.pollMs.value())

    def poll_once(self):
        self.worker.send(build_read_ascii(SLAVE_ID, POLL_BASE_ADDR, POLL_QTY))

    # --------- CSV saving (wide format: only fields checked at start) ----------
    def toggle_saving(self, enable: bool):
        if enable:
            chosen = self.checked_fields()
            if not chosen:
                QtWidgets.QMessageBox.information(self, "Select fields",
                    "Check at least one diagnostic field before starting to save.")
                self.saveToggleBtn.blockSignals(True)
                self.saveToggleBtn.setChecked(False)
                self.saveToggleBtn.blockSignals(False)
                return

            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Start saving CSV", "diagnostics_log.csv", "CSV Files (*.csv)"
            )
            if not path:
                self.saveToggleBtn.blockSignals(True)
                self.saveToggleBtn.setChecked(False)
                self.saveToggleBtn.blockSignals(False)
                return
            try:
                self.csv_file = open(path, "w", newline="", encoding="utf-8")
                self.csv_writer = csv.writer(self.csv_file)
                self.save_fields = list(chosen)  # freeze columns
                header = ["Timestamp"] + self.save_fields
                self.csv_writer.writerow(header)
                self.csv_header_written = True
                self.saving_active = True
                self.saveToggleBtn.setText("Stop Saving")
                self.on_status(f"Saving to {path} (columns: {', '.join(self.save_fields)})")
            except Exception as e:
                self.on_status(f"Failed to open file: {e}")
                self.saveToggleBtn.blockSignals(True)
                self.saveToggleBtn.setChecked(False)
                self.saveToggleBtn.blockSignals(False)
        else:
            if self.csv_file:
                try: self.csv_file.close()
                except Exception: pass
            self.csv_file = None
            self.csv_writer = None
            self.csv_header_written = False
            self.saving_active = False
            self.save_fields = []
            self.saveToggleBtn.setText("Start Saving")
            self.on_status("Saving stopped")

    def write_csv_row(self, timestamp_str: str, snapshot: Dict[int, int]):
        if not (self.saving_active and self.csv_writer and self.csv_header_written):
            return
        row = [timestamp_str]
        for name in self.save_fields:
            spec = FIELD_MAP.get(name)
            if not spec:
                row.append("")
                continue
            if spec.addr in snapshot:
                try:
                    val = spec.scale([snapshot[spec.addr]])
                    row.append(f"{val:.3f}")
                except Exception:
                    row.append("")
            else:
                row.append("")
        self.csv_writer.writerow(row)

    # --------- Incoming frames ----------
    def on_frame(self, addr: int, func: int, data: bytes):
        if func == 0x06:
            self.on_status("Write ACK received")
            return
        if func != 0x03:
            return

        regs = parse_0x03_registers(data)
        if not regs:
            return

        # Snapshot: absolute address -> value
        snapshot = {POLL_BASE_ADDR + i: v for i, v in enumerate(regs)}

        # Status lamp from bit0 at 0x0004
        if STATUS_ADDR in snapshot:
            self.set_run_indicator(bool(snapshot[STATUS_ADDR] & 0x0001))

        now = time.time() - self.t0
        ts_str = time.strftime("%H:%M:%S")  # time-only

        # Keep a friendly "Last read" for setpoint (ALWAYS display in °C)
        if SETPOINT_ADDR in snapshot:
            try:
                sp_c = scale_temp10_unsigned([snapshot[SETPOINT_ADDR]])
                self.setTempReadLbl.setText(f"Last read: {sp_c:.1f} °C")
            except Exception:
                pass

        # 1) Always maintain history for all mapped fields present in snapshot
        for name, spec in FIELD_MAP.items():
            if spec.addr not in snapshot:
                continue
            try:
                val = spec.scale([snapshot[spec.addr]])
            except Exception:
                continue
            xs, ys = self.history.setdefault(name, ([], []))
            xs.append(now); ys.append(val)
            if len(xs) > 1000:
                del xs[:len(xs)-1000]; del ys[:len(ys)-1000]

        # 2) Log only checked fields to the on-screen table (no units)
        checked = self.checked_fields()
        for name in checked:
            spec = FIELD_MAP[name]
            if spec.addr not in snapshot:
                continue
            try:
                val = spec.scale([snapshot[spec.addr]])
            except Exception:
                continue
            row = self.table.rowCount(); self.table.insertRow(row)
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(ts_str))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(name))
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(f"{val:.3f}"))
            self.table.scrollToBottom()

        # 3) Plot all checked fields together (create/remove curves as needed)
        for name in list(self.curves.keys()):
            if name not in checked:
                self.plot.removeItem(self.curves[name])
                del self.curves[name]

        for name in checked:
            xs, ys = self.history.get(name, ([], []))
            if name not in self.curves:
                pen = self._pen_for(name)
                self.curves[name] = self.plot.plot(xs, ys, name=name, pen=pen)
            else:
                self.curves[name].setData(xs, ys)

        # Rebuild legend to stay in sync
        self.legend.clear()
        for n, c in self.curves.items():
            self.legend.addItem(c, n)

        # Optional: auto-range each tick; comment out if you prefer sticky zoom
        self.plot.enableAutoRange('x', True)
        self.plot.enableAutoRange('y', True)
        self.plot.setLabel('bottom', 'Time', 's')
        self.plot.setLabel('left', 'Value')

        # 4) If saving is active, write a wide CSV row for this tick (selected fields only)
        self.write_csv_row(ts_str, snapshot)

    # --------- Field list helpers ----------
    def checked_fields(self) -> List[str]:
        names = []
        for i in range(self.fieldList.count()):
            it = self.fieldList.item(i)
            if it.checkState() == QtCore.Qt.Checked:
                names.append(it.text())
        return names

    def on_field_check(self, item: QtWidgets.QListWidgetItem):
        name = item.text()
        checked = (item.checkState() == QtCore.Qt.Checked)
        if checked:
            if name not in self.curves:
                xs, ys = self.history.get(name, ([], []))
                pen = self._pen_for(name)
                self.curves[name] = self.plot.plot(xs, ys, name=name, pen=pen)
        else:
            if name in self.curves:
                self.plot.removeItem(self.curves[name])
                del self.curves[name]

        # Re-sync legend
        self.legend.clear()
        for n, c in self.curves.items():
            self.legend.addItem(c, n)

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(); w.resize(1200, 760); w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
