# -*- coding: utf-8 -*-
# Windows 7 compatible build: Python 3.8.x + PyQt5 5.15.x
import sys, time, binascii, csv, math, collections
from dataclasses import dataclass
from typing import Dict, Callable, List, Tuple, Optional
from PyQt5 import QtCore, QtWidgets
import serial, serial.tools.list_ports
import pyqtgraph as pg

# -------------------- Settings --------------------
SLAVE_ID = 1

# Older unit map blocks (read only what we need; reserved are ignored by not mapping)
POLL_BLOCKS: List[Tuple[int, int]] = [
    (0x0000, 0x000C),  # 0x0000 .. 0x000B (incl. setpoint at 0x000B)
    (0x5200, 0x0002),  # 0x5200 .. 0x5201 (supply/return temps)
]
DEFAULT_BAUD = 9600

# Serial defaults (user can override in UI)
BYTESIZE = serial.SEVENBITS
PARITY   = serial.PARITY_EVEN
STOPBITS = serial.STOPBITS_ONE
READ_TIMEOUT  = 0.5
WRITE_TIMEOUT = 0.5
INTER_REQUEST_DELAY_MS = 60     # small gap after each write

# Command/Status (older unit)
RUN_CMD_ADDR = 0x000C           # 0=stop, 1=run
RUN_VALUE    = 0x0001
STOP_VALUE   = 0x0000
STATUS_ADDR  = 0x0004           # bit0 = Run flag

# Setpoint register (R/W)
SETPOINT_ADDR = 0x000B          # 5.0..40.0°C (0.1°C units). 41..104°F

DEMO_MODE = False  # True = simulate without hardware; False = real serial

# Distinct, colorblind-friendly palette (24 unique)
PALETTE_HEX = [
    "#E15759", "#4E79A7", "#59A14F", "#F28E2B", "#B07AA1", "#9C755F",
    "#76B7B2", "#EDC948", "#FF9DA7", "#8CD17D", "#499894", "#FABFD2",
    "#B6992D", "#86BCB6", "#5F9ED1", "#AF7AA1", "#D4A6C8", "#FFBE7D",
    "#8C564B", "#1F77B4", "#2CA02C", "#D62728", "#9467BD", "#17BECF"
]

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

def build_response_ascii(addr: int, func: int, data_bytes: bytes) -> bytes:
    pdu = bytes([addr & 0xFF, func & 0xFF]) + data_bytes
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

def scale_temp10_signed(regs: List[int])   -> float: return s16(regs[0]) / 10.0
def scale_temp10_unsigned(regs: List[int]) -> float: return regs[0] / 10.0
def scale_pressure_01mpa(regs: List[int])  -> float: return regs[0] * 0.01
def scale_cond_01uScm(regs: List[int])     -> float: return regs[0] * 0.1
def scale_identity(regs: List[float])      -> float: return float(regs[0]) if regs else 0.0

FIELD_MAP: Dict[str, FieldSpec] = {
    # 0x0000 block
    "Discharge Temp (°C)":        FieldSpec(0x0000, 1, scale_temp10_signed, "°C"),
    "Discharge Pressure (MPa)":   FieldSpec(0x0002, 1, scale_pressure_01mpa, "MPa"),
    "Fluid Conductivity (µS/cm)": FieldSpec(0x0003, 1, scale_cond_01uScm, "µS/cm"),
    "Fluid Set Temp (°C)":        FieldSpec(SETPOINT_ADDR, 1, scale_temp10_unsigned, "°C"),

    # 0x5200 block
    "Supply Temp (°C)":           FieldSpec(0x5200, 1, scale_temp10_signed, "°C"),
    "Return Temp (°C)":           FieldSpec(0x5201, 1, scale_temp10_signed, "°C"),

    # Derived (virtual) channel
    "Heat Transfer (W)":          FieldSpec(-1, 0, scale_identity, "W"),
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

    def open(self, port: str, baud: int, bytesize, parity, stopbits):
        try:
            self.ser = serial.Serial(
                port=port, baudrate=baud,
                bytesize=bytesize, parity=parity, stopbits=stopbits,
                timeout=READ_TIMEOUT, write_timeout=WRITE_TIMEOUT
            )
            self._run = True
            if not self.isRunning():
                self.start()
            p = {serial.PARITY_NONE: "N", serial.PARITY_EVEN: "E", serial.PARITY_ODD: "O"}.get(parity, "?")
            db = 7 if bytesize == serial.SEVENBITS else 8
            sb = 1 if stopbits == serial.STOPBITS_ONE else 2
            self.status.emit(f"Opened {port} @ {baud} ({db}{p}{sb})")
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
                    self.ser.flush()
                    time.sleep(INTER_REQUEST_DELAY_MS / 1000.0)
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
        self._run_state = 0  # 0=OFF, 1=ON
        self._setpoint_raw = int(round(22.0 * 10))  # 22.0°C in 0.1°C
        self._press_raw = 12   # 0.12 MPa
        self._cond_raw  = 150  # 15.0 µS/cm

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
                    self._setpoint_raw = reg_val
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

        # Block 0: 0x0000..0x000B
        regs0 = [0]*0x000C
        regs0[0]  = int(round((25 + 5*math.sin(t/6.0))*10))  # discharge temp
        regs0[2]  = self._press_raw                          # 0.01 MPa/dig
        regs0[3]  = self._cond_raw                           # 0.1 µS/cm/dig
        regs0[4]  = 1 if self._run_state else 0              # status
        regs0[11] = self._setpoint_raw                       # setpoint

        bc0 = len(regs0)*2
        payload0 = bytearray([bc0])
        for v in regs0:
            payload0.extend([(v>>8)&0xFF, v&0xFF])
        self.frameParsed.emit(SLAVE_ID, 0x03, bytes(payload0))

        # Block 1: 0x5200..0x5201
        regs1 = [0]*0x0002
        supply = int(round((24 + 3*math.sin(t/7.0))*10))
        ret    = int(round((24.5 + 3*math.sin(t/7.0 + 1.0))*10))
        regs1[0] = supply
        regs1[1] = ret

        bc1 = len(regs1)*2
        payload1 = bytearray([bc1])
        for v in regs1:
            payload1.extend([(v>>8)&0xFF, v&0xFF])
        self.frameParsed.emit(SLAVE_ID, 0x03, bytes(payload1))

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

        # Per-block TX/RX frames + queue of outstanding blocks
        self._block_frames: Dict[Tuple[int,int], Tuple[bytes,bytes]] = {}
        self._awaiting: collections.deque[Tuple[int,int]] = collections.deque()

        # Pens / colors
        self._pen_cache: Dict[str, pg.mkPen] = {}
        self._palette_i = 0

        # ---- UI: Top bar ----
        top = QtWidgets.QWidget()
        tl = QtWidgets.QGridLayout(top)
        tl.setHorizontalSpacing(10)
        tl.setVerticalSpacing(8)
        tl.setContentsMargins(8, 8, 8, 4)

        self.portBox = QtWidgets.QComboBox()
        self.refreshBtn = QtWidgets.QPushButton("Refresh")
        self.baudBox = QtWidgets.QComboBox()
        self.baudBox.addItems(["600", "1200", "2400", "4800", "9600", "19200"])
        self.baudBox.setCurrentText("9600")

        self.dataBitsBox = QtWidgets.QComboBox(); self.dataBitsBox.addItems(["7","8"]); self.dataBitsBox.setCurrentText("7")
        self.parityBox   = QtWidgets.QComboBox(); self.parityBox.addItems(["Even","Odd","None"]); self.parityBox.setCurrentText("Even")
        self.stopBitsBox = QtWidgets.QComboBox(); self.stopBitsBox.addItems(["1","2"]); self.stopBitsBox.setCurrentText("1")

        self.portToggleBtn = QtWidgets.QPushButton("Open Port")
        self.portToggleBtn.setCheckable(True)
        self.portToggleBtn.clicked.connect(self.toggle_port)
        self.portStatusDot = QtWidgets.QLabel("● Closed")
        self.portStatusDot.setStyleSheet("color:#cc0000; font-weight:600;")

        self.pollMs = QtWidgets.QSpinBox(); self.pollMs.setRange(100,10000); self.pollMs.setValue(1000)

        row = 0
        tl.addWidget(QtWidgets.QLabel("Port"), row, 0); tl.addWidget(self.portBox, row, 1)
        tl.addWidget(self.refreshBtn, row, 2)
        tl.addWidget(QtWidgets.QLabel("Baud"), row, 3); tl.addWidget(self.baudBox, row, 4)
        tl.addWidget(QtWidgets.QLabel("Data bits"), row, 5); tl.addWidget(self.dataBitsBox, row, 6)
        tl.addWidget(QtWidgets.QLabel("Parity"), row, 7); tl.addWidget(self.parityBox, row, 8)
        tl.addWidget(QtWidgets.QLabel("Stop bits"), row, 9); tl.addWidget(self.stopBitsBox, row,10)
        tl.addWidget(self.portToggleBtn, row, 11); tl.addWidget(self.portStatusDot, row, 12)
        tl.addItem(QtWidgets.QSpacerItem(20,10,QtWidgets.QSizePolicy.Expanding,QtWidgets.QSizePolicy.Minimum), row, 99)

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
        tl.addItem(QtWidgets.QSpacerItem(20,10,QtWidgets.QSizePolicy.Expanding,QtWidgets.QSizePolicy.Minimum), row, 99)

        # -------- Fluid Set Temp (write) --------
        row += 1
        tl.addWidget(QtWidgets.QLabel("Fluid Set Temp"), row, 0)
        self.setTempSpin = QtWidgets.QDoubleSpinBox()
        self.setTempSpin.setDecimals(1)
        self.setTempSpin.setSingleStep(0.1)
        self.setTempSpin.setKeyboardTracking(False)
        self.setTempSpin.setAccelerated(True)
        self.setTempSpin.setRange(-999.0, 999.0)
        self.setTempSpin.setValue(22.0)
        self.setTempSpin.setSuffix(" °C")

        self.setTempUnit = QtWidgets.QComboBox(); self._settemp_unit = "°C"
        self.setTempUnit.addItems(["°C","°F"]); self.setTempUnit.setFixedWidth(58)
        self.setTempUnit.currentIndexChanged.connect(self._on_settemp_unit_changed)

        self.writeSetTempBtn = QtWidgets.QPushButton("Set")
        self.writeSetTempBtn.clicked.connect(self.write_setpoint)

        self.setTempReadLbl = QtWidgets.QLabel("Last read: --.- °C"); self.setTempReadLbl.setStyleSheet("color:#BBBBBB;")

        tl.addWidget(self.setTempSpin, row, 1)
        tl.addWidget(self.setTempUnit, row, 2)
        tl.addWidget(self.writeSetTempBtn, row, 3)
        tl.addWidget(self.setTempReadLbl, row, 4, 1, 3)

        # --- Heat Transfer inputs ---
        row += 1
        tl.addWidget(QtWidgets.QLabel("Flow Rate (L/min)"), row, 0)
        self.flowSpin = QtWidgets.QDoubleSpinBox()
        self.flowSpin.setDecimals(1)
        self.flowSpin.setMinimum(0.0)
        self.flowSpin.setMaximum(9999999)   # effectively unlimited
        self.flowSpin.setSingleStep(0.10)
        self.flowSpin.setValue(5.0)
        tl.addWidget(self.flowSpin, row, 1)

        tl.addWidget(QtWidgets.QLabel("Fluid Type"), row, 2)
        self.fluidTypeBox = QtWidgets.QComboBox()
        self.fluidTypeBox.addItems(["Water", "Fluorinated Fluid"])
        self.fluidTypeBox.setCurrentIndex(0)
        tl.addWidget(self.fluidTypeBox, row, 3)

        # Field list
        self.fieldList = QtWidgets.QListWidget()
        self.fieldList.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        for name in FIELD_MAP:
            it = QtWidgets.QListWidgetItem(name)
            it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
            it.setCheckState(QtCore.Qt.Unchecked)
            self.fieldList.addItem(it)
        self.fieldList.itemChanged.connect(self.on_field_check)

        # -------- Plot (dual Y axes) --------
        pg.setConfigOptions(antialias=True)
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setBackground('w')

        pi = self.plot.getPlotItem()
        for axis in ['bottom','left','right']:
            pi.showAxis(axis, True)

        for axis in ['bottom','left']:
            ax = pi.getAxis(axis)
            ax.setPen('k'); ax.setTextPen('k')
        self.plot.setLabel('bottom', 'Time', 's')
        self.plot.setLabel('left', 'Value')

        pi.getAxis('right').setPen('k'); pi.getAxis('right').setTextPen('k')
        pi.getAxis('right').setLabel('Heat Transfer', units='W')

        # Right ViewBox (for Heat Transfer), X-linked to left
        self.vb_right = pg.ViewBox()
        pi.scene().addItem(self.vb_right)
        pi.getAxis('right').linkToView(self.vb_right)
        self.vb_right.setXLink(pi.vb)

        def _update_views():
            self.vb_right.setGeometry(pi.vb.sceneBoundingRect())
            self.vb_right.linkedViewChanged(pi.vb, self.vb_right.XAxis)
        pi.vb.sigResized.connect(_update_views)
        _update_views()

        # Legend (auto-add for left curves; manual for right)
        self.legend = self.plot.addLegend()
        self.legend.setVisible(True)
        self.right_in_legend: set = set()  # names added manually

        # Curve registries
        self.left_curves: Dict[str, pg.PlotDataItem] = {}
        self.right_curves: Dict[str, pg.PlotDataItem] = {}

        # Pen factory
        def _pen_for(name: str) -> pg.mkPen:
            if name in self._pen_cache:
                return self._pen_cache[name]
            if self._palette_i < len(PALETTE_HEX):
                color = PALETTE_HEX[self._palette_i]; self._palette_i += 1
            else:
                i = self._palette_i; self._palette_i += 1
                color = pg.intColor((i * 7) % 36, hues=36)
            pen = pg.mkPen(color, width=2.5, cosmetic=True)
            self._pen_cache[name] = pen
            return pen
        self._pen_for = _pen_for

        self.history: Dict[str, Tuple[List[float], List[float]]] = {}
        self.t0 = time.time()

        # Table/log (on-screen) with TX/RX per-row
        self.table = QtWidgets.QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Timestamp","Field","Value","S (ASCII)","R (ASCII)"])
        self.table.setColumnWidth(4, 530)

        # Saving
        self.saveToggleBtn = QtWidgets.QPushButton("Start Saving")
        self.saveToggleBtn.setCheckable(True)
        self.saveToggleBtn.toggled.connect(self.toggle_saving)
        self.saving_active = False
        self.csv_file = None
        self.csv_writer = None
        self.csv_header_written = False
        self.save_fields: List[str] = []

        # Pending run/confirm/retry flow
        self._pending_run: Optional[bool] = None
        self._pending_attempts = 0
        self._confirm_timer = QtCore.QTimer(self)
        self._confirm_timer.setSingleShot(True)
        self._confirm_timer.timeout.connect(self._confirm_run_state)

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

        # Theme
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
    def c_to_f(c: float) -> float: return c * 9.0/5.0 + 32.0
    @staticmethod
    def f_to_c(f: float) -> float: return (f - 32.0) * 5.0/9.0

    def _on_settemp_unit_changed(self, _idx: int):
        new_unit = self.setTempUnit.currentText()
        old_unit = getattr(self, "_settemp_unit", "°C")
        val = self.setTempSpin.value()
        if old_unit != new_unit:
            if old_unit == "°C" and new_unit == "°F":
                val = round(self.c_to_f(val), 1)
            elif old_unit == "°F" and new_unit == "°C":
                val = round(self.f_to_c(val), 1)
        self.setTempSpin.blockSignals(True)
        self.setTempSpin.setSuffix(" " + new_unit)
        self.setTempSpin.setRange(-999.0, 999.0)
        self.setTempSpin.setValue(val)
        self.setTempSpin.blockSignals(False)
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

    def _selected_serial_params(self):
        bytesize = serial.SEVENBITS if self.dataBitsBox.currentText() == "7" else serial.EIGHTBITS
        parity_map = {"Even": serial.PARITY_EVEN, "Odd": serial.PARITY_ODD, "None": serial.PARITY_NONE}
        parity = parity_map[self.parityBox.currentText()]
        stopbits = serial.STOPBITS_ONE if self.stopBitsBox.currentText() == "1" else serial.STOPBITS_TWO
        return bytesize, parity, stopbits

    def set_port_indicator(self, is_open: bool):
        if is_open:
            self.portToggleBtn.setText("Close Port")
            self.portToggleBtn.setChecked(True)
            self.portStatusDot.setText("● Open")
            self.portStatusDot.setStyleSheet("color:#1a7f37; font-weight:600;")
            self.portBox.setEnabled(False); self.refreshBtn.setEnabled(False)
        else:
            self.portToggleBtn.setText("Open Port")
            self.portToggleBtn.setChecked(False)
            self.portStatusDot.setText("● Closed")
            self.portStatusDot.setStyleSheet("color:#cc0000; font-weight:600;")
            self.portBox.setEnabled(True); self.refreshBtn.setEnabled(False if DEMO_MODE else True)

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
            bytesize, parity, stopbits = self._selected_serial_params()
            self.worker.open(port, baud, bytesize, parity, stopbits)
            self.t0 = time.time()
            self.poll_once()
            self.pollTimer.start(self.pollMs.value())
            self.set_port_indicator(True)
        else:
            self.on_status("No port selected")

    def close_port(self):
        if self.pollTimer.isActive(): self.pollTimer.stop()
        self.worker.close()
        self.set_port_indicator(False)
        if self.saving_active:
            self.saveToggleBtn.setChecked(False)

    # --------- Status + chiller control ----------
    def on_status(self, s: str): self.statusBar().showMessage(s, 5000)
    def on_error(self, e: str):  self.statusBar().showMessage(e, 8000)

    def set_run_indicator(self, running: bool):
        if running:
            self.statusDot.setText("● ON");  self.statusDot.setStyleSheet("color:#1a7f37; font-weight:600;")
        else:
            self.statusDot.setText("● OFF"); self.statusDot.setStyleSheet("color:#cc0000; font-weight:600;")
        self.chillerToggleBtn.blockSignals(True)
        self.chillerToggleBtn.setChecked(running)
        self.chillerToggleBtn.setText("Turn Chiller OFF" if running else "Turn Chiller ON")
        self.chillerToggleBtn.blockSignals(False)

    # ---- Write→confirm→retry flow ----
    def _request_status_reads(self, delay_ms=180):
        def _send_all():
            for start, qty in POLL_BLOCKS:
                frame = build_read_ascii(SLAVE_ID, start, qty)
                self._block_frames[(start, qty)] = (frame, b"")  # reset RX for this block
                self._awaiting.append((start, qty))
                self.worker.send(frame)
        QtCore.QTimer.singleShot(delay_ms, _send_all)

    def _confirm_run_state(self):
        if self._pending_run is None:
            return
        if self._pending_attempts < 2:
            value = RUN_VALUE if self._pending_run else STOP_VALUE
            self.worker.send(build_write_single_register_ascii(SLAVE_ID, RUN_CMD_ADDR, value))
            self._pending_attempts += 1
            self._request_status_reads(220)
            self._confirm_timer.start(350)
        else:
            self.chillerToggleBtn.setEnabled(True)
            self._pending_run = None
            self._pending_attempts = 0
            self.on_status("Run/Stop did not confirm after retries.")

    def toggle_chiller(self, want_run: bool):
        value = RUN_VALUE if want_run else STOP_VALUE
        self.chillerToggleBtn.setEnabled(False)
        self._pending_run = want_run
        self._pending_attempts = 0
        self.worker.send(build_write_single_register_ascii(SLAVE_ID, RUN_CMD_ADDR, value))
        self._request_status_reads(200)
        self._confirm_timer.start(400)

    # --------- Setpoint write ----------
    def write_setpoint(self):
        unit = self.setTempUnit.currentText()
        val  = round(self.setTempSpin.value(), 1)
        if unit == "°C":
            if not (5.0 <= val <= 40.0):
                QtWidgets.QMessageBox.warning(self, "Invalid setpoint",
                    "°C setpoint must be between 5.0 and 40.0")
                return
            raw = int(round(val * 10.0))
        else:
            if not (41.0 <= val <= 104.0):
                QtWidgets.QMessageBox.warning(self, "Invalid setpoint",
                    "°F setpoint must be between 41.0 and 104.0")
                return
            raw = int(round(self.f_to_c(val) * 10.0))
        self.worker.send(build_write_single_register_ascii(SLAVE_ID, SETPOINT_ADDR, raw))
        self.on_status(f"Setpoint written: {val:.1f} {unit} (raw {raw:#06x})")

    # --------- Polling ----------
    def on_poll_interval_changed(self, _):
        if self.pollTimer.isActive():
            self.pollTimer.stop()
            self.poll_once()
            self.pollTimer.start(self.pollMs.value())

    def poll_once(self):
        self._awaiting.clear()
        for start, qty in POLL_BLOCKS:
            frame = build_read_ascii(SLAVE_ID, start, qty)
            self._block_frames[(start, qty)] = (frame, b"")
            self._awaiting.append((start, qty))
            self.worker.send(frame)

    # --------- CSV saving ----------
    def toggle_saving(self, enable: bool):
        if enable:
            chosen = self.checked_fields()
            if not chosen:
                QtWidgets.QMessageBox.information(self, "Select fields",
                    "Check at least one diagnostic field before starting to save.")
                self.saveToggleBtn.blockSignals(True); self.saveToggleBtn.setChecked(False); self.saveToggleBtn.blockSignals(False)
                return
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Start saving CSV", "diagnostics_log.csv", "CSV Files (*.csv)")
            if not path:
                self.saveToggleBtn.blockSignals(True); self.saveToggleBtn.setChecked(False); self.saveToggleBtn.blockSignals(False)
                return
            try:
                self.csv_file = open(path, "w", newline="", encoding="utf-8")
                self.csv_writer = csv.writer(self.csv_file)
                self.save_fields = list(chosen)
                header = ["Timestamp"] + self.save_fields
                self.csv_writer.writerow(header)
                self.csv_header_written = True
                self.saving_active = True
                self.saveToggleBtn.setText("Stop Saving")
                self.on_status(f"Saving to {path} (columns: {', '.join(self.save_fields)})")
            except Exception as e:
                self.on_status(f"Failed to open file: {e}")
                self.saveToggleBtn.blockSignals(True); self.saveToggleBtn.setChecked(False); self.saveToggleBtn.blockSignals(False)
        else:
            if self.csv_file:
                try: self.csv_file.close()
                except Exception: pass
            self.csv_file = None; self.csv_writer = None
            self.csv_header_written = False; self.saving_active = False
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
                row.append(""); continue
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

        block = self._awaiting.popleft() if self._awaiting else None
        if block:
            start, qty = block
            rx_full = build_response_ascii(addr, func, data)
            tx = self._block_frames.get(block, (b"", b""))[0]
            self._block_frames[block] = (tx, rx_full)

        regs = parse_0x03_registers(data)
        if not regs:
            return

        base_addr = block[0] if block else POLL_BLOCKS[0][0]
        snapshot_part = {base_addr + i: v for i, v in enumerate(regs)}

        if not hasattr(self, "_tick_snapshot"):
            self._tick_snapshot = {}
        self._tick_snapshot.update(snapshot_part)

        if self._awaiting:
            return  # wait for remaining block(s)

        snapshot = getattr(self, "_tick_snapshot", {})
        self._tick_snapshot = {}

        if STATUS_ADDR in snapshot:
            self.set_run_indicator(bool(snapshot[STATUS_ADDR] & 0x0001))

        now = time.time() - self.t0
        ts_str = time.strftime("%H:%M:%S")

        if SETPOINT_ADDR in snapshot:
            try:
                sp_c = scale_temp10_unsigned([snapshot[SETPOINT_ADDR]])
                self.setTempReadLbl.setText(f"Last read: {sp_c:.1f} °C")
            except Exception:
                pass

        if self._pending_run is not None and STATUS_ADDR in snapshot:
            current_running = bool(snapshot[STATUS_ADDR] & 0x0001)
            if current_running == self._pending_run:
                if self._confirm_timer.isActive():
                    self._confirm_timer.stop()
                self.chillerToggleBtn.setEnabled(True)
                self._pending_run = None
                self._pending_attempts = 0

        # --- Derived Heat Transfer (W) ---
        if 0x5201 in snapshot and 0x0000 in snapshot:
            T_return = snapshot[0x5201] / 10.0
            T_dis    = snapshot[0x0000] / 10.0
            dT = T_return - T_dis

            flow_L_min = self.flowSpin.value()
            if self.fluidTypeBox.currentText() == "Water":
                cp = 1.002   # cal/g°C
                rho = 1.0    # g/cm³
            else:
                cp = 0.23
                rho = 1.0

            try:
                heat_watts = ((dT) * flow_L_min * rho * 1000.0 * (cp * 1000.0 / 0.239)) / (60.0 * 1000.0)
            except Exception:
                heat_watts = 0.0

            snapshot[-1] = heat_watts

        # -------- Maintain history --------
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

        # -------- Log checked fields (with TX/RX) --------
        checked = self.checked_fields()
        for name in checked:
            spec = FIELD_MAP[name]
            if spec.addr not in snapshot:
                continue
            try:
                val = spec.scale([snapshot[spec.addr]])
            except Exception:
                continue

            blk_for_row = None
            if spec.addr == -1:
                blk_for_row = (0x5200, 0x0002)
            else:
                for b in POLL_BLOCKS:
                    s, q = b
                    if s <= spec.addr < s + q:
                        blk_for_row = b
                        break

            tx_str = rx_str = ""
            if blk_for_row and blk_for_row in self._block_frames:
                tx, rx = self._block_frames[blk_for_row]
                tx_str = tx.decode('ascii', errors='replace').strip()
                rx_str = rx.decode('ascii', errors='replace').strip()

            row = self.table.rowCount(); self.table.insertRow(row)
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(ts_str))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(name))
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(f"{val:.3f}"))
            tx_item = QtWidgets.QTableWidgetItem(tx_str); tx_item.setToolTip("Poll request (Modbus ASCII)")
            rx_item = QtWidgets.QTableWidgetItem(rx_str); rx_item.setToolTip("Poll response (Modbus ASCII)")
            self.table.setItem(row, 3, tx_item); self.table.setItem(row, 4, rx_item)
            self.table.scrollToBottom()

        # -------- Plot: left vs right axes --------
        # Remove curves unchecked (also clean legend by name)
        for name in list(self.left_curves.keys()):
            if name not in checked:
                self.plot.removeItem(self.left_curves[name])
                self.legend.removeItem(name)  # remove the auto-added legend row
                del self.left_curves[name]
        for name in list(self.right_curves.keys()):
            if name not in checked:
                self.vb_right.removeItem(self.right_curves[name])
                if name in self.right_in_legend:
                    self.legend.removeItem(name)
                    self.right_in_legend.discard(name)
                del self.right_curves[name]

        # Ensure Heat Transfer goes to right axis; others on left
        for name in checked:
            xs, ys = self.history.get(name, ([], []))
            if name == "Heat Transfer (W)":
                if name not in self.right_curves:
                    c = pg.PlotDataItem(xs, ys, name=name, pen=self._pen_for(name))
                    self.vb_right.addItem(c)
                    self.right_curves[name] = c
                    if name not in self.right_in_legend:
                        self.legend.addItem(c, name)  # manual add for right axis only
                        self.right_in_legend.add(name)
                else:
                    self.right_curves[name].setData(xs, ys)
            else:
                if name not in self.left_curves:
                    c = self.plot.plot(xs, ys, name=name, pen=self._pen_for(name))  # auto legend add
                    self.left_curves[name] = c
                else:
                    self.left_curves[name].setData(xs, ys)

        # ---- Unified X autorange ----
        xmins, xmaxs = [], []
        for d in (self.left_curves, self.right_curves):
            for c in d.values():
                xd, _ = c.getData()
                if xd is not None and len(xd):
                    xmins.append(xd[0]); xmaxs.append(xd[-1])
        if xmins and xmaxs:
            xmin, xmax = min(xmins), max(xmaxs)
            if xmax > xmin:
                self.plot.getPlotItem().vb.setXRange(xmin, xmax, padding=0.02)

        # Left & right Y autorange
        self.plot.enableAutoRange('y', True)
        self.vb_right.enableAutoRange(axis=pg.ViewBox.YAxis, enable=True)

        # Saving
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
        # legend handled when curves are added/removed
        pass

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(); w.resize(1400, 840); w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
