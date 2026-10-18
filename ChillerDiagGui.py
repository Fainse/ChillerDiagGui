# -*- coding: utf-8 -*-
# Windows 7 compatible build: Python 3.8.x + PyQt5 5.15.x
import sys, time, binascii, csv, math, queue, threading
from dataclasses import dataclass
from typing import Dict, Callable, List, Tuple, Optional
from PyQt5 import QtCore, QtWidgets
import serial, serial.tools.list_ports
import pyqtgraph as pg

# -------------------- Settings --------------------
SLAVE_ID = 1
DEFAULT_BAUD = 9600

READ_TIMEOUT  = 0.05            # serial read granularity; keeps the worker responsive
WRITE_TIMEOUT = 0.5
RESPONSE_TIMEOUT = 0.5          # wait for a reply, on top of the frame transmit time
INTER_REQUEST_DELAY_MS = 60     # small gap after each transaction

STATUS_CONFIRM_DELAY_MS = 200   # give the unit time to update its run flag before reading back
RUN_CONFIRM_RETRIES = 2         # extra Run/Stop writes if the status does not follow

MAX_HISTORY_POINTS = 1000       # per plotted field
MAX_TABLE_ROWS = 2000           # on-screen log; oldest rows are dropped

DEMO_MODE = "--demo" in sys.argv  # simulate without hardware (python ChillerDiagGui.py --demo)

# Distinct, colorblind-friendly palette (24 unique)
PALETTE_HEX = [
    "#E15759", "#4E79A7", "#59A14F", "#F28E2B", "#B07AA1", "#9C755F",
    "#76B7B2", "#EDC948", "#FF9DA7", "#8CD17D", "#499894", "#FABFD2",
    "#B6992D", "#86BCB6", "#5F9ED1", "#AF7AA1", "#D4A6C8", "#FFBE7D",
    "#8C564B", "#1F77B4", "#2CA02C", "#D62728", "#9467BD", "#17BECF"
]

# Heat transfer fluids: name -> (specific heat cal/g°C, density g/cm³)
FLUIDS: Dict[str, Tuple[float, float]] = {
    "Water":             (1.002, 1.00),
    "Fluorinated Fluid": (0.23,  1.80),  # typical Fluorinert/Galden; check your fluid's datasheet
}
J_PER_CAL = 4.184

# -------------------- Modbus ASCII helpers --------------------
def lrc(bs: bytes) -> int:
    s = sum(bs) & 0xFF
    return (-s) & 0xFF

def build_ascii_frame(pdu: bytes) -> bytes:
    return b':' + binascii.hexlify(pdu + bytes([lrc(pdu)])).upper() + b'\r\n'

def build_read_ascii(slave: int, start_addr: int, quantity: int) -> bytes:
    return build_ascii_frame(bytes([
        slave & 0xFF, 0x03,
        (start_addr >> 8) & 0xFF, start_addr & 0xFF,
        (quantity   >> 8) & 0xFF, quantity   & 0xFF
    ]))

def build_write_single_register_ascii(slave: int, addr: int, value: int) -> bytes:
    return build_ascii_frame(bytes([
        slave & 0xFF, 0x06,
        (addr  >> 8) & 0xFF, addr  & 0xFF,
        (value >> 8) & 0xFF, value & 0xFF
    ]))

def build_response_ascii(addr: int, func: int, data_bytes: bytes) -> bytes:
    return build_ascii_frame(bytes([addr & 0xFF, func & 0xFF]) + data_bytes)

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
    if not data_bytes:
        return []
    count = data_bytes[0]
    regs = []
    for i in range(1, 1 + count, 2):
        if i + 1 < len(data_bytes):
            regs.append((data_bytes[i] << 8) | data_bytes[i+1])
    return regs

# -------------------- Scaling --------------------
def s16(x: int) -> int:
    return x - 0x10000 if x >= 0x8000 else x

def scale_temp10_signed(raw: int)    -> float: return s16(raw) / 10.0
def scale_temp10_unsigned(raw: int)  -> float: return raw / 10.0
def scale_temp100_signed(raw: int)   -> float: return s16(raw) / 100.0
def scale_temp100_unsigned(raw: int) -> float: return raw / 100.0
def scale_pressure_01mpa(raw: int)   -> float: return raw * 0.01
def scale_cond_01uScm(raw: int)      -> float: return raw * 0.1

def to_raw(value: float, counts_per_unit: int) -> int:
    return int(round(value * counts_per_unit)) & 0xFFFF

# -------------------- Chiller models (register maps) --------------------
@dataclass
class FieldSpec:
    addr: int
    scale: Callable[[int], float]
    unit: str

@dataclass
class HeatTransferSpec:
    outlet_addr: int                # fluid leaving the chiller (to the load)
    return_addr: int                # fluid returning from the load
    scale: Callable[[int], float]   # raw -> °C

@dataclass
class ChillerModel:
    name: str
    poll_blocks: List[Tuple[int, int]]       # (start, quantity) read with 0x03
    run_cmd_addr: int                        # 0=stop, 1=run
    status_addr: int                         # bit0 = run flag
    setpoint_addr: int
    setpoint_counts_per_c: int               # 10 -> 0.1°C units, 100 -> 0.01°C units
    setpoint_range_c: Tuple[float, float]
    fields: Dict[str, FieldSpec]
    heat_transfer: Optional[HeatTransferSpec]
    demo_registers: Callable[[float, Dict[int, int]], Dict[int, int]]  # (t, written regs) -> regs

    @property
    def setpoint_decimals(self) -> int:
        return int(round(math.log10(self.setpoint_counts_per_c)))

HEAT_TRANSFER_FIELD = "Heat Transfer (W)"
RUN_VALUE, STOP_VALUE = 0x0001, 0x0000

def _demo_standard(t: float, written: Dict[int, int]) -> Dict[int, int]:
    return {
        0x0000: to_raw(25 + 5*math.sin(t/6.0), 10),        # discharge temp
        0x0002: 12,                                       # 0.12 MPa
        0x0003: 150,                                      # 15.0 µS/cm
        0x0004: 1 if written.get(0x000C, 0) else 0,       # status bit0
        0x000B: written.get(0x000B, to_raw(22.0, 10)),    # setpoint
        0x5200: to_raw(24 + 3*math.sin(t/7.0), 10),        # supply
        0x5201: to_raw(26.5 + 3*math.sin(t/7.0 + 1.0), 10),  # return
    }

def _demo_hec(t: float, written: Dict[int, int]) -> Dict[int, int]:
    internal = 25 + 5*math.sin(t/6.0)
    external = 25 + 3*math.sin(t/7.0)
    return {
        0x0040: to_raw(internal, 100),
        0x0041: to_raw(external, 100),
        0x0042: to_raw((internal + external) / 2.0, 100),
        0x0043: 1 if written.get(0x0050, 0) else 0,
        0x0051: written.get(0x0051, to_raw(22.0, 100)),
    }

MODELS: List[ChillerModel] = [
    ChillerModel(
        name="Standard (0x0000 / 0x5200 map)",
        poll_blocks=[
            (0x0000, 0x000C),  # 0x0000 .. 0x000B (incl. setpoint at 0x000B)
            (0x5200, 0x0002),  # 0x5200 .. 0x5201 (supply/return temps)
        ],
        run_cmd_addr=0x000C,
        status_addr=0x0004,
        setpoint_addr=0x000B,
        setpoint_counts_per_c=10,
        setpoint_range_c=(5.0, 40.0),
        fields={
            "Discharge Temp (°C)":        FieldSpec(0x0000, scale_temp10_signed, "°C"),
            "Discharge Pressure (MPa)":   FieldSpec(0x0002, scale_pressure_01mpa, "MPa"),
            "Fluid Conductivity (µS/cm)": FieldSpec(0x0003, scale_cond_01uScm, "µS/cm"),
            "Fluid Set Temp (°C)":        FieldSpec(0x000B, scale_temp10_unsigned, "°C"),
            "Supply Temp (°C)":           FieldSpec(0x5200, scale_temp10_signed, "°C"),
            "Return Temp (°C)":           FieldSpec(0x5201, scale_temp10_signed, "°C"),
        },
        heat_transfer=HeatTransferSpec(outlet_addr=0x0000, return_addr=0x5201,
                                       scale=scale_temp10_signed),
        demo_registers=_demo_standard,
    ),
    ChillerModel(
        name="HEC (0x0040 map)",
        poll_blocks=[(0x0040, 0x0012)],  # 0x0040 .. 0x0051
        run_cmd_addr=0x0050,
        status_addr=0x0043,
        setpoint_addr=0x0051,
        setpoint_counts_per_c=100,
        setpoint_range_c=(10.0, 60.0),
        fields={
            "Internal Temp (°C)":  FieldSpec(0x0040, scale_temp100_signed, "°C"),
            "External Temp (°C)":  FieldSpec(0x0041, scale_temp100_signed, "°C"),
            "Average Temp (°C)":   FieldSpec(0x0042, scale_temp100_signed, "°C"),
            "Fluid Set Temp (°C)": FieldSpec(0x0051, scale_temp100_unsigned, "°C"),
        },
        heat_transfer=None,
        demo_registers=_demo_hec,
    ),
]

# -------------------- Transactions --------------------
@dataclass
class Request:
    kind: str             # "poll", "run" or "setpoint"
    func: int             # 0x03 or 0x06
    addr: int
    qty: int              # register count (0x03) or value written (0x06)
    frame: bytes
    cycle: int = 0        # poll cycle this request belongs to
    note: str = ""        # status text shown when the request succeeds

    @property
    def reply_len(self) -> int:
        """Expected reply length in ASCII characters (':' + hex + CRLF)."""
        if self.func == 0x03:
            return 11 + 4 * self.qty
        return len(self.frame)  # 0x06 reply echoes the request

def read_request(start: int, qty: int, cycle: int) -> Request:
    return Request("poll", 0x03, start, qty, build_read_ascii(SLAVE_ID, start, qty), cycle)

def write_request(kind: str, addr: int, value: int, note: str = "") -> Request:
    return Request(kind, 0x06, addr, value,
                   build_write_single_register_ascii(SLAVE_ID, addr, value), note=note)

# -------------------- Serial worker --------------------
class SerialWorker(QtCore.QThread):
    """Owns the serial port and runs one request/reply transaction at a time.

    Every request gets exactly one replied or failed signal (unless the port closes
    first), so replies can never be matched to the wrong request.
    """
    replied = QtCore.pyqtSignal(object, bytes, bytes)  # request, data, rx frame
    failed  = QtCore.pyqtSignal(object, str)           # request, reason
    status  = QtCore.pyqtSignal(str)
    error   = QtCore.pyqtSignal(str)
    closed  = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.ser = None
        self._requests: "queue.Queue[Request]" = queue.Queue()
        self._stop = threading.Event()
        self._char_time = 0.0

    def is_open(self) -> bool:
        return self.isRunning() and not self._stop.is_set()

    def open(self, ser, char_time: float, label: str) -> bool:
        """Start a session on an already-opened port; the worker thread owns it from here."""
        if self.isRunning():
            self.error.emit("Port is still closing, try again")
            try: ser.close()
            except Exception: pass
            return False
        self.ser = ser
        self._char_time = char_time
        self._stop.clear()
        self._drain()
        self.start()
        self.status.emit(f"Opened {label}")
        return True

    def close(self):
        """Ask the worker thread to close the port and wait until it has."""
        self._stop.set()
        self.wait()

    def send(self, req: Request):
        self._requests.put(req)

    def _drain(self):
        while True:
            try: self._requests.get_nowait()
            except queue.Empty: break

    def run(self):
        try:
            while not self._stop.is_set():
                try:
                    req = self._requests.get(timeout=0.05)
                except queue.Empty:
                    continue
                self._transact(req)
        except Exception as e:
            self.error.emit(f"Serial error: {e}")
        finally:
            try: self.ser.close()
            except Exception: pass
            self.ser = None
            self._stop.set()
            self._drain()
            self.closed.emit()

    def _transact(self, req: Request):
        ser = self.ser
        try:
            ser.reset_input_buffer()  # drop late/stray bytes from earlier transactions
            try:
                ser.write(req.frame)
                ser.flush()
            except serial.SerialTimeoutException:
                self.failed.emit(req, "write timeout")
                return

            parser = ModbusAsciiParser()
            deadline = (time.monotonic() + RESPONSE_TIMEOUT
                        + (len(req.frame) + req.reply_len) * self._char_time)
            while not self._stop.is_set() and time.monotonic() < deadline:
                chunk = ser.read(ser.in_waiting or 1)
                for addr, func, data in parser.feed(chunk):
                    if addr != SLAVE_ID:
                        continue
                    if func == (req.func | 0x80):
                        code = data[0] if data else 0
                        self.failed.emit(req, f"Modbus exception {code:#04x}")
                        return
                    if func != req.func:
                        continue
                    # Ignore frames of the wrong size (e.g. our own request echoed on 2-wire RS-485)
                    if func == 0x03 and (not data or data[0] != 2 * req.qty or len(data) != 1 + 2 * req.qty):
                        continue
                    self.replied.emit(req, data, build_response_ascii(addr, func, data))
                    return
            if not self._stop.is_set():
                self.failed.emit(req, "no response (timeout)")
        finally:
            time.sleep(INTER_REQUEST_DELAY_MS / 1000.0)

# -------------------- Demo port (no hardware needed) --------------------
class DemoSerial:
    """Stands in for serial.Serial and answers like a chiller of the given model."""
    def __init__(self, model: ChillerModel):
        self.model = model
        self.is_open = True
        self._rx = bytearray()
        self._parser = ModbusAsciiParser()
        self._written: Dict[int, int] = {}
        self._t0 = time.time()

    @property
    def in_waiting(self) -> int:
        return len(self._rx)

    def reset_input_buffer(self):
        self._rx.clear()

    def write(self, frame: bytes):
        for addr, func, data in self._parser.feed(frame):
            if addr != SLAVE_ID or len(data) < 4:
                continue
            reg = (data[0] << 8) | data[1]
            arg = (data[2] << 8) | data[3]
            if func == 0x03:
                regs = self.model.demo_registers(time.time() - self._t0, self._written)
                payload = bytearray([2 * arg])
                for a in range(reg, reg + arg):
                    v = regs.get(a, 0)
                    payload.extend([(v >> 8) & 0xFF, v & 0xFF])
                self._rx.extend(build_response_ascii(addr, func, bytes(payload)))
            elif func == 0x06:
                self._written[reg] = arg
                self._rx.extend(build_response_ascii(addr, func, data[:4]))
        return len(frame)

    def flush(self):
        pass

    def read(self, n: int = 1) -> bytes:
        if not self._rx:
            time.sleep(READ_TIMEOUT)
            return b''
        out = bytes(self._rx[:n])
        del self._rx[:n]
        return out

    def close(self):
        self.is_open = False

# -------------------- Main GUI --------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SMC Chiller Diagnostics (Win7/PyQt5)" + (" — DEMO" if DEMO_MODE else ""))

        self.model: ChillerModel = MODELS[0]

        # Worker
        self.worker = SerialWorker()
        self.worker.status.connect(self.on_status)
        self.worker.error.connect(self.on_error)
        self.worker.replied.connect(self.on_reply)
        self.worker.failed.connect(self.on_request_failed)
        self.worker.closed.connect(self.on_worker_closed)

        # Poll cycle state
        self._cycle = 0
        self._cycle_outstanding = 0
        self._cycle_snapshot: Dict[int, int] = {}
        self._block_frames: Dict[Tuple[int, int], Tuple[bytes, bytes]] = {}  # last TX/RX per block

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
        self.baudBox.setCurrentText(str(DEFAULT_BAUD))

        self.dataBitsBox = QtWidgets.QComboBox(); self.dataBitsBox.addItems(["7","8"]); self.dataBitsBox.setCurrentText("7")
        self.parityBox   = QtWidgets.QComboBox(); self.parityBox.addItems(["Even","Odd","None"]); self.parityBox.setCurrentText("Even")
        self.stopBitsBox = QtWidgets.QComboBox(); self.stopBitsBox.addItems(["1","2"]); self.stopBitsBox.setCurrentText("1")

        self.portToggleBtn = QtWidgets.QPushButton("Open Port")
        self.portToggleBtn.setCheckable(True)
        self.portToggleBtn.clicked.connect(self.toggle_port)
        self.portStatusDot = QtWidgets.QLabel("● Closed")
        self.portStatusDot.setStyleSheet("color:#cc0000; font-weight:600;")

        self.pollMs = QtWidgets.QSpinBox(); self.pollMs.setRange(100,10000); self.pollMs.setValue(1000)

        self.modelBox = QtWidgets.QComboBox()
        self.modelBox.addItems([m.name for m in MODELS])

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
        tl.addWidget(QtWidgets.QLabel("Model"), row, 7); tl.addWidget(self.modelBox, row, 8, 1, 3)

        # -------- Fluid Set Temp (write) --------
        row += 1
        tl.addWidget(QtWidgets.QLabel("Fluid Set Temp"), row, 0)
        self.setTempSpin = QtWidgets.QDoubleSpinBox()
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

        self.setTempReadLbl = QtWidgets.QLabel(); self.setTempReadLbl.setStyleSheet("color:#BBBBBB;")
        self._last_setpoint_c: Optional[float] = None

        tl.addWidget(self.setTempSpin, row, 1)
        tl.addWidget(self.setTempUnit, row, 2)
        tl.addWidget(self.writeSetTempBtn, row, 3)
        tl.addWidget(self.setTempReadLbl, row, 4, 1, 3)

        # --- Heat Transfer inputs (models with supply/return temps only) ---
        row += 1
        flowLbl = QtWidgets.QLabel("Flow Rate (L/min)")
        self.flowSpin = QtWidgets.QDoubleSpinBox()
        self.flowSpin.setDecimals(1)
        self.flowSpin.setMinimum(0.0)
        self.flowSpin.setMaximum(9999999)   # effectively unlimited
        self.flowSpin.setSingleStep(0.10)
        self.flowSpin.setValue(5.0)
        fluidLbl = QtWidgets.QLabel("Fluid Type")
        self.fluidTypeBox = QtWidgets.QComboBox()
        self.fluidTypeBox.addItems(list(FLUIDS))
        tl.addWidget(flowLbl, row, 0); tl.addWidget(self.flowSpin, row, 1)
        tl.addWidget(fluidLbl, row, 2); tl.addWidget(self.fluidTypeBox, row, 3)
        self._heat_widgets = [flowLbl, self.flowSpin, fluidLbl, self.fluidTypeBox]

        # Field list (filled per model)
        self.fieldList = QtWidgets.QListWidget()
        self.fieldList.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.fieldList.itemChanged.connect(self.on_field_check)

        # -------- Plot (dual Y axes) --------
        pg.setConfigOptions(antialias=True)
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setBackground('w')

        pi = self.plot.getPlotItem()
        for axis in ['bottom','left','right']:
            pi.showAxis(axis, True)
            ax = pi.getAxis(axis)
            ax.setPen('k'); ax.setTextPen('k')
        self.plot.setLabel('bottom', 'Time', 's')
        self.plot.setLabel('left', 'Value')
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

        # Legend entries are managed manually (curves are created without a name)
        self.legend = self.plot.addLegend()

        # Curve registries
        self.left_curves: Dict[str, pg.PlotDataItem] = {}
        self.right_curves: Dict[str, pg.PlotDataItem] = {}

        self.history: Dict[str, Tuple[List[float], List[float]]] = {}
        self.t0 = time.time()

        # Table/log (on-screen) with TX/RX per-row
        self.table = QtWidgets.QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Timestamp","Field","Value","S (ASCII)","R (ASCII)"])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setColumnWidth(4, 530)

        # Saving
        self.saveToggleBtn = QtWidgets.QPushButton("Start Saving")
        self.saveToggleBtn.setCheckable(True)
        self.saveToggleBtn.toggled.connect(self.toggle_saving)
        self.saving_active = False
        self.csv_file = None
        self.csv_writer = None
        self.save_fields: List[str] = []

        # Pending run/confirm/retry flow
        self._pending_run: Optional[bool] = None
        self._pending_attempts = 0
        self._confirm_queued = False      # read back status once the current poll finishes
        self._confirm_cycle: Optional[int] = None

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
        self.modelBox.currentIndexChanged.connect(self.on_model_changed)

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
        self.apply_model(self.model)

    # --------- Model selection ----------
    def on_model_changed(self, idx: int):
        self.apply_model(MODELS[idx])

    def apply_model(self, model: ChillerModel):
        self.model = model

        # Field list
        self.fieldList.blockSignals(True)
        self.fieldList.clear()
        for name in self.field_names():
            it = QtWidgets.QListWidgetItem(name)
            it.setFlags(it.flags() | QtCore.Qt.ItemIsUserCheckable)
            it.setCheckState(QtCore.Qt.Unchecked)
            self.fieldList.addItem(it)
        self.fieldList.blockSignals(False)

        # Setpoint control
        self.setTempSpin.setDecimals(model.setpoint_decimals)
        self.setTempSpin.setSingleStep(1.0 / model.setpoint_counts_per_c if model.setpoint_decimals < 2 else 0.05)
        self._last_setpoint_c = None
        self._update_setpoint_label()

        # Heat transfer inputs + right axis
        has_heat = model.heat_transfer is not None
        for w in self._heat_widgets:
            w.setVisible(has_heat)
        self.plot.getPlotItem().showAxis('right', has_heat)

        # Reset data from the previous model
        self.history.clear()
        self._block_frames.clear()
        self.table.setRowCount(0)
        self.update_plot()

    def field_names(self) -> List[str]:
        names = list(self.model.fields)
        if self.model.heat_transfer:
            names.append(HEAT_TRANSFER_FIELD)
        return names

    # --------- Unit helpers ----------
    @staticmethod
    def c_to_f(c: float) -> float: return c * 9.0/5.0 + 32.0
    @staticmethod
    def f_to_c(f: float) -> float: return (f - 32.0) * 5.0/9.0

    def _on_settemp_unit_changed(self, _idx: int):
        new_unit = self.setTempUnit.currentText()
        old_unit = self._settemp_unit
        val = self.setTempSpin.value()
        decimals = self.model.setpoint_decimals
        if old_unit == "°C" and new_unit == "°F":
            val = round(self.c_to_f(val), decimals)
        elif old_unit == "°F" and new_unit == "°C":
            val = round(self.f_to_c(val), decimals)
        self.setTempSpin.blockSignals(True)
        self.setTempSpin.setSuffix(" " + new_unit)
        self.setTempSpin.setValue(val)
        self.setTempSpin.blockSignals(False)
        self._settemp_unit = new_unit
        self._update_setpoint_label()

    def _update_setpoint_label(self):
        decimals = self.model.setpoint_decimals
        unit = self.setTempUnit.currentText()
        if self._last_setpoint_c is None:
            text = "--." + "-" * decimals
        else:
            val = self._last_setpoint_c if unit == "°C" else self.c_to_f(self._last_setpoint_c)
            text = f"{val:.{decimals}f}"
        self.setTempReadLbl.setText(f"Last read: {text} {unit}")

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
            self.portStatusDot.setText("● Open")
            self.portStatusDot.setStyleSheet("color:#1a7f37; font-weight:600;")
        else:
            self.portToggleBtn.setText("Open Port")
            self.portStatusDot.setText("● Closed")
            self.portStatusDot.setStyleSheet("color:#cc0000; font-weight:600;")
        self.portToggleBtn.setChecked(is_open)
        for w in (self.portBox, self.modelBox, self.baudBox, self.dataBitsBox, self.parityBox, self.stopBitsBox):
            w.setEnabled(not is_open)
        self.refreshBtn.setEnabled(not is_open and not DEMO_MODE)

    # --------- Port open/close ---------
    def toggle_port(self):
        if self.worker.is_open():
            self.close_port()
        else:
            self.open_port()

    def open_port(self):
        if self.worker.is_open():
            self.on_status("Port already open"); return
        baud = int(self.baudBox.currentText()) if self.baudBox.currentText() else DEFAULT_BAUD
        bytesize, parity, stopbits = self._selected_serial_params()
        bits_per_char = (1 + (7 if bytesize == serial.SEVENBITS else 8)
                         + (0 if parity == serial.PARITY_NONE else 1)
                         + (1 if stopbits == serial.STOPBITS_ONE else 2))
        framing = "{}{}{}".format(7 if bytesize == serial.SEVENBITS else 8, parity,
                                  1 if stopbits == serial.STOPBITS_ONE else 2)

        idx = self.portBox.currentIndex()
        port = self.portBox.itemData(idx) if idx >= 0 else None
        if not port:
            self.on_status("No port selected")
            self.set_port_indicator(False)
            return
        try:
            if DEMO_MODE:
                ser = DemoSerial(self.model)
                label = f"(DEMO) virtual {self.model.name} @ {baud} ({framing})"
            else:
                ser = serial.Serial(
                    port=port, baudrate=baud,
                    bytesize=bytesize, parity=parity, stopbits=stopbits,
                    timeout=READ_TIMEOUT, write_timeout=WRITE_TIMEOUT
                )
                label = f"{port} @ {baud} ({framing})"
        except Exception as e:
            self.on_error(f"Open failed: {e}")
            self.set_port_indicator(False)
            return

        if not self.worker.open(ser, bits_per_char / baud, label):
            self.set_port_indicator(False)
            return
        self.t0 = time.time()
        self.set_port_indicator(True)
        self.poll_once()
        self.pollTimer.start(self.pollMs.value())

    def close_port(self):
        self.worker.close()  # on_worker_closed resets the UI
        self.on_status("Port closed")

    def on_worker_closed(self):
        if self.pollTimer.isActive(): self.pollTimer.stop()
        self._cycle_outstanding = 0
        self._cycle_snapshot = {}
        self._confirm_queued = False
        self._confirm_cycle = None
        if self._pending_run is not None:
            self._pending_run = None
            self.chillerToggleBtn.setEnabled(True)
        self.set_port_indicator(False)
        if self.saving_active:
            self.saveToggleBtn.setChecked(False)

    def closeEvent(self, event):
        if self.worker.isRunning():
            self.worker.close()
        if self.saving_active:
            self.toggle_saving(False)
        super().closeEvent(event)

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

    # ---- Write → read back status → retry flow ----
    def toggle_chiller(self, want_run: bool):
        if not self.worker.is_open():
            self.on_status("Open the port first")
            self.chillerToggleBtn.blockSignals(True)
            self.chillerToggleBtn.setChecked(not want_run)
            self.chillerToggleBtn.blockSignals(False)
            return
        self.chillerToggleBtn.setEnabled(False)
        self._pending_run = want_run
        self._pending_attempts = 0
        self._send_run_command()

    def _send_run_command(self):
        self._pending_attempts += 1
        value = RUN_VALUE if self._pending_run else STOP_VALUE
        self.worker.send(write_request("run", self.model.run_cmd_addr, value))

    def _request_confirm_poll(self):
        if self._pending_run is None:
            return
        if self._cycle_outstanding:
            self._confirm_queued = True  # that poll may predate the command; read again after it
        else:
            self.poll_once()
            self._confirm_cycle = self._cycle

    def _check_run_confirmation(self, snapshot: Dict[int, int]):
        self._confirm_cycle = None
        if self._pending_run is None:
            return
        status = snapshot.get(self.model.status_addr)
        if status is not None and bool(status & 0x0001) == self._pending_run:
            self.on_status("Chiller " + ("started" if self._pending_run else "stopped"))
        elif self._pending_attempts <= RUN_CONFIRM_RETRIES:
            self._send_run_command()
            return
        else:
            self.on_error("Run/Stop did not confirm after retries.")
        self._pending_run = None
        self._pending_attempts = 0
        self.chillerToggleBtn.setEnabled(True)

    # --------- Setpoint write ----------
    def write_setpoint(self):
        if not self.worker.is_open():
            self.on_status("Open the port first"); return
        unit = self.setTempUnit.currentText()
        decimals = self.model.setpoint_decimals
        val = round(self.setTempSpin.value(), decimals)
        lo_c, hi_c = self.model.setpoint_range_c
        lo, hi = (lo_c, hi_c) if unit == "°C" else (self.c_to_f(lo_c), self.c_to_f(hi_c))
        if not (lo <= val <= hi):
            QtWidgets.QMessageBox.warning(self, "Invalid setpoint",
                f"{unit} setpoint must be between {lo:.{decimals}f} and {hi:.{decimals}f}")
            return
        val_c = val if unit == "°C" else self.f_to_c(val)
        raw = to_raw(val_c, self.model.setpoint_counts_per_c)
        self.worker.send(write_request("setpoint", self.model.setpoint_addr, raw,
                                       note=f"Setpoint written: {val:.{decimals}f} {unit} (raw {raw:#06x})"))

    # --------- Polling ----------
    def on_poll_interval_changed(self, _):
        if self.pollTimer.isActive():
            self.pollTimer.start(self.pollMs.value())

    def poll_once(self):
        if not self.worker.is_open() or self._cycle_outstanding:
            return  # previous poll still running; skip rather than pile up requests
        self._cycle += 1
        self._cycle_snapshot = {}
        self._cycle_outstanding = len(self.model.poll_blocks)
        for start, qty in self.model.poll_blocks:
            self.worker.send(read_request(start, qty, self._cycle))

    # --------- Worker replies ----------
    def on_reply(self, req: Request, data: bytes, rx_frame: bytes):
        if req.kind == "poll":
            if req.cycle != self._cycle:
                return
            regs = parse_0x03_registers(data)
            self._cycle_snapshot.update({req.addr + i: v for i, v in enumerate(regs)})
            self._block_frames[(req.addr, req.qty)] = (req.frame, rx_frame)
            self._poll_request_done()
        elif req.kind == "run":
            QtCore.QTimer.singleShot(STATUS_CONFIRM_DELAY_MS, self._request_confirm_poll)
        elif req.kind == "setpoint":
            self.on_status(req.note)

    def on_request_failed(self, req: Request, reason: str):
        if req.kind == "poll":
            if req.cycle != self._cycle:
                return
            self.on_error(f"Read {req.addr:#06x} x{req.qty} failed: {reason}")
            self._poll_request_done()
        elif req.kind == "run":
            self.on_error(f"Run/Stop write failed: {reason}")
            QtCore.QTimer.singleShot(STATUS_CONFIRM_DELAY_MS, self._request_confirm_poll)
        elif req.kind == "setpoint":
            self.on_error(f"Setpoint write failed: {reason}")

    def _poll_request_done(self):
        self._cycle_outstanding -= 1
        if self._cycle_outstanding > 0:
            return
        snapshot, self._cycle_snapshot = self._cycle_snapshot, {}
        if snapshot:
            self.process_snapshot(snapshot)
        if self._confirm_cycle == self._cycle:
            self._check_run_confirmation(snapshot)
        if self._confirm_queued:
            self._confirm_queued = False
            self._request_confirm_poll()

    # --------- Snapshot processing ----------
    def heat_transfer_w(self, snapshot: Dict[int, int]) -> Optional[float]:
        ht = self.model.heat_transfer
        if not ht or ht.outlet_addr not in snapshot or ht.return_addr not in snapshot:
            return None
        dT = ht.scale(snapshot[ht.return_addr]) - ht.scale(snapshot[ht.outlet_addr])
        cp_cal, rho = FLUIDS[self.fluidTypeBox.currentText()]
        mass_kg_s = self.flowSpin.value() * rho / 60.0      # L/min × kg/L → kg/s
        cp_j_kg = cp_cal * J_PER_CAL * 1000.0                # cal/g°C → J/kg°C
        return mass_kg_s * cp_j_kg * dT

    def _frames_for_addr(self, addr: int) -> Tuple[str, str]:
        for (start, qty), (tx, rx) in self._block_frames.items():
            if start <= addr < start + qty:
                return (tx.decode('ascii', errors='replace').strip(),
                        rx.decode('ascii', errors='replace').strip())
        return "", ""

    def process_snapshot(self, snapshot: Dict[int, int]):
        m = self.model
        if m.status_addr in snapshot:
            self.set_run_indicator(bool(snapshot[m.status_addr] & 0x0001))
        if m.setpoint_addr in snapshot:
            self._last_setpoint_c = snapshot[m.setpoint_addr] / m.setpoint_counts_per_c
            self._update_setpoint_label()

        values: Dict[str, float] = {}
        for name, spec in m.fields.items():
            if spec.addr in snapshot:
                values[name] = spec.scale(snapshot[spec.addr])
        heat = self.heat_transfer_w(snapshot)
        if heat is not None:
            values[HEAT_TRANSFER_FIELD] = heat

        now = time.time() - self.t0
        ts_str = time.strftime("%H:%M:%S")

        # -------- Maintain history --------
        for name, val in values.items():
            xs, ys = self.history.setdefault(name, ([], []))
            xs.append(now); ys.append(val)
            if len(xs) > MAX_HISTORY_POINTS:
                del xs[:len(xs)-MAX_HISTORY_POINTS]; del ys[:len(ys)-MAX_HISTORY_POINTS]

        # -------- Log checked fields (with TX/RX) --------
        for name in self.checked_fields():
            if name not in values:
                continue
            frame_addr = m.heat_transfer.return_addr if name == HEAT_TRANSFER_FIELD else m.fields[name].addr
            tx_str, rx_str = self._frames_for_addr(frame_addr)
            row = self.table.rowCount(); self.table.insertRow(row)
            self.table.setItem(row, 0, QtWidgets.QTableWidgetItem(ts_str))
            self.table.setItem(row, 1, QtWidgets.QTableWidgetItem(name))
            self.table.setItem(row, 2, QtWidgets.QTableWidgetItem(f"{values[name]:.3f}"))
            tx_item = QtWidgets.QTableWidgetItem(tx_str); tx_item.setToolTip("Poll request (Modbus ASCII)")
            rx_item = QtWidgets.QTableWidgetItem(rx_str); rx_item.setToolTip("Poll response (Modbus ASCII)")
            self.table.setItem(row, 3, tx_item); self.table.setItem(row, 4, rx_item)
        excess = self.table.rowCount() - MAX_TABLE_ROWS
        for _ in range(max(0, excess)):
            self.table.removeRow(0)
        self.table.scrollToBottom()

        self.update_plot()
        self.write_csv_row(ts_str, values)

    # --------- Plot ----------
    def _pen_for(self, name: str) -> pg.mkPen:
        if name in self._pen_cache:
            return self._pen_cache[name]
        if self._palette_i < len(PALETTE_HEX):
            color = PALETTE_HEX[self._palette_i]
        else:
            color = pg.intColor((self._palette_i * 7) % 36, hues=36)
        self._palette_i += 1
        pen = pg.mkPen(color, width=2.5, cosmetic=True)
        self._pen_cache[name] = pen
        return pen

    def update_plot(self):
        checked = set(self.checked_fields())
        changed = False

        # Remove unchecked curves
        for curves, owner in ((self.left_curves, self.plot), (self.right_curves, self.vb_right)):
            for name in list(curves):
                if name not in checked:
                    owner.removeItem(curves.pop(name))
                    changed = True

        # Heat Transfer goes to the right axis; everything else on the left
        for name in self.field_names():
            if name not in checked:
                continue
            xs, ys = self.history.get(name, ([], []))
            curves = self.right_curves if name == HEAT_TRANSFER_FIELD else self.left_curves
            if name in curves:
                curves[name].setData(xs, ys)
                continue
            c = pg.PlotDataItem(xs, ys, pen=self._pen_for(name))
            (self.vb_right if curves is self.right_curves else self.plot).addItem(c)
            curves[name] = c
            changed = True

        if changed:
            self.legend.clear()
            for name in self.field_names():
                c = self.left_curves.get(name) or self.right_curves.get(name)
                if c is not None:
                    self.legend.addItem(c, name)

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

    # --------- CSV saving ----------
    def _reset_save_button(self):
        self.saveToggleBtn.blockSignals(True); self.saveToggleBtn.setChecked(False); self.saveToggleBtn.blockSignals(False)

    def toggle_saving(self, enable: bool):
        if enable:
            chosen = self.checked_fields()
            if not chosen:
                QtWidgets.QMessageBox.information(self, "Select fields",
                    "Check at least one diagnostic field before starting to save.")
                self._reset_save_button()
                return
            path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Start saving CSV", "diagnostics_log.csv", "CSV Files (*.csv)")
            if not path:
                self._reset_save_button()
                return
            try:
                self.csv_file = open(path, "w", newline="", encoding="utf-8")
                self.csv_writer = csv.writer(self.csv_file)
                self.save_fields = list(chosen)
                self.csv_writer.writerow(["Timestamp"] + self.save_fields)
                self.csv_file.flush()
                self.saving_active = True
                self.saveToggleBtn.setText("Stop Saving")
                self.on_status(f"Saving to {path} (columns: {', '.join(self.save_fields)})")
            except Exception as e:
                self.on_error(f"Failed to open file: {e}")
                self._reset_save_button()
        else:
            if self.csv_file:
                try: self.csv_file.close()
                except Exception: pass
            self.csv_file = None; self.csv_writer = None
            self.saving_active = False
            self.save_fields = []
            self.saveToggleBtn.setText("Start Saving")
            self.on_status("Saving stopped")

    def write_csv_row(self, timestamp_str: str, values: Dict[str, float]):
        if not (self.saving_active and self.csv_writer):
            return
        row = [timestamp_str]
        for name in self.save_fields:
            row.append(f"{values[name]:.3f}" if name in values else "")
        try:
            self.csv_writer.writerow(row)
            self.csv_file.flush()  # keep the log intact if the app or PC goes down
        except Exception as e:
            self.on_error(f"CSV write failed: {e}")
            self.saveToggleBtn.setChecked(False)

    # --------- Field list helpers ----------
    def checked_fields(self) -> List[str]:
        names = []
        for i in range(self.fieldList.count()):
            it = self.fieldList.item(i)
            if it.checkState() == QtCore.Qt.Checked:
                names.append(it.text())
        return names

    def on_field_check(self, _item: QtWidgets.QListWidgetItem):
        self.update_plot()

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(); w.resize(1400, 840); w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
