"""
dual_sine_controller.py
PySide6 desktop app for controlling the Arduino Due dual-sine DAC firmware.

Requirements:
    pip install PySide6 pyserial numpy pylsl pandas scipy

Usage:
    python dual_sine_controller.py
"""

import sys
import json
import re
import time
import threading
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
import serial.tools.list_ports

try:
    import pylsl
    from pylsl import StreamInlet, local_clock, resolve_streams

    LSL_IMPORT_ERROR = None
except Exception as e:  # pragma: no cover - depends on local installation
    pylsl = None
    StreamInlet = None
    local_clock = None
    resolve_streams = None
    LSL_IMPORT_ERROR = e

try:
    import pandas as pd

    PANDAS_IMPORT_ERROR = None
except Exception as e:  # pragma: no cover - depends on local installation
    pd = None
    PANDAS_IMPORT_ERROR = e

try:
    from scipy.signal import butter, filtfilt, iirnotch, lfilter, sosfilt, sosfiltfilt

    SCIPY_IMPORT_ERROR = None
except Exception as e:  # pragma: no cover - depends on local installation
    butter = filtfilt = iirnotch = lfilter = sosfilt = sosfiltfilt = None
    SCIPY_IMPORT_ERROR = e

from PySide6.QtCore import QSettings, QTimer, Signal, QObject, Slot, Qt
from PySide6.QtGui import QColor, QPalette, QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QDoubleSpinBox,
    QSpinBox,
    QPushButton,
    QComboBox,
    QGroupBox,
    QStatusBar,
    QFrame,
    QCheckBox,
    QTextEdit,
    QLineEdit,
    QTabWidget,
    QFileDialog,
    QProgressBar,
    QScrollArea,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QHeaderView,
    QAbstractItemView,
)

# ── Palette ───────────────────────────────────────────────────────────────────
COLORS = {
    "bg": "#0b0f14",
    "surface": "#121822",
    "surface2": "#19212d",
    "surface3": "#202938",
    "border": "#293445",
    "accent": "#5eead4",
    "accent2": "#f59e0b",
    "accent3": "#8b5cf6",
    "text": "#edf2f7",
    "text_dim": "#8a97ab",
    "green": "#34d399",
    "red": "#f87171",
    "yellow": "#fbbf24",
    "sine1": "#67e8f9",
    "sine2": "#fb923c",
    "combined": "#a78bfa",
}

STYLE = f"""
QMainWindow, QWidget {{
    background-color: {COLORS['bg']};
    color: {COLORS['text']};
    font-family: 'Segoe UI', 'SF Pro Text', 'Noto Sans', sans-serif;
    font-size: 13px;
}}
QGroupBox {{
    background-color: {COLORS['surface']};
    border: 1px solid {COLORS['border']};
    border-radius: 14px;
    margin-top: 16px;
    padding: 14px 12px 12px 12px;
    font-size: 12px;
    font-weight: 600;
    color: {COLORS['text_dim']};
}}
QGroupBox::title {{
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
}}
QLabel {{
    background-color: transparent;
}}
QFrame#card {{
    background-color: {COLORS['surface']};
    border: 1px solid {COLORS['border']};
    border-radius: 18px;
}}
QFrame#subcard {{
    background-color: {COLORS['surface2']};
    border: 1px solid {COLORS['border']};
    border-radius: 14px;
}}
QLabel#card_title {{
    color: {COLORS['text']};
    font-size: 15px;
    font-weight: 600;
}}
QLabel#card_hint {{
    color: {COLORS['text_dim']};
    font-size: 11px;
}}
QLabel#subcard_title {{
    color: {COLORS['text']};
    font-size: 12px;
    font-weight: 600;
}}
QLabel#section_label {{
    color: {COLORS['accent']};
    font-size: 11px;
    font-weight: 600;
}}
QLabel#field_label, QLabel#hint_label, QLabel#metric_label {{
    color: {COLORS['text_dim']};
    font-size: 11px;
}}
QLabel#metric_value, QLabel#warning_label, QLabel#status_value {{
    font-family: 'JetBrains Mono', 'Consolas', monospace;
}}
QLabel#metric_value {{
    color: {COLORS['text']};
    font-size: 12px;
    font-weight: 600;
}}
QLabel#warning_label {{
    color: {COLORS['red']};
    font-size: 11px;
}}
QLabel#status_value {{
    color: {COLORS['text_dim']};
    font-size: 11px;
}}
QDoubleSpinBox, QSpinBox, QComboBox, QLineEdit {{
    background-color: {COLORS['surface2']};
    border: 1px solid {COLORS['border']};
    border-radius: 10px;
    color: {COLORS['text']};
    padding: 7px 10px;
    min-height: 22px;
}}
QDoubleSpinBox, QSpinBox, QLabel#metric_value, QLabel#status_value, QTextEdit, QTableWidget, QStatusBar {{
    font-family: 'JetBrains Mono', 'Consolas', monospace;
}}
QDoubleSpinBox, QSpinBox {{
    min-width: 96px;
}}
QDoubleSpinBox:focus, QSpinBox:focus, QComboBox:focus, QLineEdit:focus {{
    border: 1px solid {COLORS['accent']};
}}
QComboBox {{
    min-width: 150px;
}}
QComboBox::drop-down {{ border: none; }}
QComboBox QAbstractItemView {{
    background-color: {COLORS['surface3']};
    color: {COLORS['text']};
    selection-background-color: {COLORS['surface2']};
}}
QPushButton {{
    background-color: {COLORS['surface2']};
    border: 1px solid {COLORS['border']};
    border-radius: 11px;
    color: {COLORS['text']};
    padding: 9px 16px;
    font-weight: 600;
}}
QPushButton:hover {{
    border-color: {COLORS['accent']};
    background-color: {COLORS['surface3']};
}}
QPushButton:pressed {{
    background-color: {COLORS['surface3']};
    border-color: {COLORS['accent']};
}}
QPushButton:disabled {{
    background-color: {COLORS['surface']};
    border-color: {COLORS['border']};
    color: {COLORS['text_dim']};
}}
QPushButton#ghost_btn {{
    background-color: transparent;
}}
QPushButton#start_btn {{
    background-color: #143129;
    border-color: {COLORS['green']};
    color: #d7ffee;
    min-height: 42px;
    font-size: 14px;
}}
QPushButton#start_btn:hover {{ background-color: #184037; }}
QPushButton#start_btn:pressed {{ background-color: #102720; }}
QPushButton#start_btn:disabled {{
    background-color: {COLORS['surface']};
    border-color: {COLORS['border']};
    color: {COLORS['text_dim']};
}}
QPushButton#stop_btn {{
    background-color: #35171a;
    border-color: {COLORS['red']};
    color: #ffe0e0;
    min-height: 42px;
    font-size: 14px;
}}
QPushButton#stop_btn:hover {{ background-color: #452025; }}
QPushButton#stop_btn:pressed {{ background-color: #2a1216; }}
QPushButton#stop_btn:disabled {{
    background-color: {COLORS['surface']};
    border-color: {COLORS['border']};
    color: {COLORS['text_dim']};
}}
QPushButton#clear_btn {{
    background-color: transparent;
    border-color: {COLORS['border']};
    padding: 7px 12px;
    font-size: 11px;
}}
QPushButton#clear_btn:hover {{
    background-color: {COLORS['surface3']};
    border-color: {COLORS['accent']};
}}
QPushButton#clear_btn:pressed {{ background-color: {COLORS['surface2']}; }}
QStatusBar {{
    background-color: {COLORS['surface']};
    border-top: 1px solid {COLORS['border']};
    color: {COLORS['text_dim']};
    font-size: 11px;
}}
QTextEdit {{
    background-color: {COLORS['surface2']};
    border: 1px solid {COLORS['border']};
    border-radius: 14px;
    color: {COLORS['text']};
    font-size: 11px;
    selection-background-color: {COLORS['surface3']};
}}
QLineEdit {{
    font-size: 11px;
}}
QLineEdit#command_input {{
    font-family: 'JetBrains Mono', 'Consolas', monospace;
}}
QTabWidget::pane {{
    border: 1px solid {COLORS['border']};
    background-color: {COLORS['surface']};
    border-radius: 16px;
}}
QTabBar::tab {{
    background-color: {COLORS['surface2']};
    border: 1px solid {COLORS['border']};
    padding: 9px 16px;
    color: {COLORS['text_dim']};
    font-size: 11px;
    border-top-left-radius: 12px;
    border-top-right-radius: 12px;
}}
QTabBar::tab:selected {{
    background-color: {COLORS['surface']};
    color: {COLORS['accent']};
    border-bottom-color: {COLORS['surface']};
}}
QFrame#divider {{
    background-color: {COLORS['border']};
    max-height: 1px;
}}
QCheckBox {{
    background-color: transparent;
    color: {COLORS['text_dim']};
    font-size: 11px;
    spacing: 6px;
}}
QCheckBox::indicator {{
    width: 14px; height: 14px;
    border: 1px solid {COLORS['border']};
    border-radius: 4px;
    background: {COLORS['surface2']};
}}
QCheckBox::indicator:checked {{
    background: {COLORS['accent']};
    border-color: {COLORS['accent']};
}}
QCheckBox#serial_status, QCheckBox#lsl_status {{
    spacing: 0px;
}}
QCheckBox#serial_status::indicator, QCheckBox#lsl_status::indicator {{
    width: 14px; height: 14px;
    border: 1px solid {COLORS['red']};
    border-radius: 3px;
    background: {COLORS['red']};
}}
QCheckBox#serial_status::indicator:checked, QCheckBox#lsl_status::indicator:checked {{
    background: {COLORS['green']};
    border-color: {COLORS['green']};
}}
QTableWidget {{
    background-color: {COLORS['surface2']};
    border: 1px solid {COLORS['border']};
    border-radius: 16px;
    color: {COLORS['text']};
    gridline-color: {COLORS['border']};
    selection-background-color: {COLORS['surface3']};
    selection-color: {COLORS['text']};
    font-size: 13px;
}}
QHeaderView::section {{
    background-color: {COLORS['surface3']};
    border: 1px solid {COLORS['border']};
    color: {COLORS['accent']};
    padding: 8px;
    font-weight: 600;
}}
QProgressBar {{
    background-color: {COLORS['surface2']};
    border: 1px solid {COLORS['border']};
    border-radius: 12px;
    color: {COLORS['text']};
    min-height: 38px;
    text-align: center;
    font-weight: 600;
}}
QProgressBar::chunk {{
    background-color: {COLORS['accent']};
    border-radius: 11px;
}}
"""


# ── Serial worker ─────────────────────────────────────────────────────────────
class SerialWorker(QObject):
    message_received = Signal(str)  # raw line from board
    error_occurred = Signal(str)
    connected = Signal(bool)

    def __init__(self):
        super().__init__()
        self._port: serial.Serial | None = None
        self._lock = threading.Lock()

    def open(self, port: str, baud: int = 115200) -> bool:
        try:
            with self._lock:
                if self._port and self._port.is_open:
                    self._port.close()
                self._port = serial.Serial(port, baud, timeout=0.1)
                time.sleep(2)
                self._port.reset_input_buffer()
            self.connected.emit(True)
            return True
        except Exception as e:
            self.error_occurred.emit(str(e))
            self.connected.emit(False)
            return False

    def close(self):
        with self._lock:
            if self._port and self._port.is_open:
                self._port.close()
        self.connected.emit(False)

    def send(self, cmd: str) -> str | None:
        """Send a command and return the first non-empty response line (≤300 ms)."""
        with self._lock:
            if not self._port or not self._port.is_open:
                return None
            try:
                self._port.reset_input_buffer()  # flush stale bytes
                self._port.write((cmd.strip() + "\n").encode())
                deadline = time.time() + 0.3
                while time.time() < deadline:
                    line = self._port.readline().decode(errors="replace").strip()
                    if line:
                        self.message_received.emit(f"← {line}")
                        return line
                return None
            except Exception as e:
                self.error_occurred.emit(str(e))
                return None

    def drain(self) -> list[str]:
        """Non-blocking read of all available lines (used for debug telemetry)."""
        lines = []
        with self._lock:
            if not self._port or not self._port.is_open:
                return lines
            try:
                while self._port.in_waiting:
                    line = self._port.readline().decode(errors="replace").strip()
                    if line:
                        lines.append(line)
            except Exception:
                pass
        return lines

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._port is not None and self._port.is_open


# ── Sequence model ────────────────────────────────────────────────────────────
@dataclass(frozen=True)
class SequenceStep:
    dac0_f1: float
    dac0_f2: float
    dac1_f1: float
    dac1_f2: float
    seconds: float


# ── Helpers ───────────────────────────────────────────────────────────────────
def make_spinbox(min_val, max_val, decimals, step, value, suffix="") -> QDoubleSpinBox:
    sb = QDoubleSpinBox()
    sb.setRange(min_val, max_val)
    sb.setDecimals(decimals)
    sb.setSingleStep(step)
    sb.setValue(value)
    if suffix:
        sb.setSuffix(f"  {suffix}")
    return sb


def safe_column_name(name: str, fallback: str) -> str:
    clean = re.sub(r"[^0-9A-Za-z_]+", "_", str(name).strip()).strip("_")
    return clean or fallback


def unique_column_names(names: list[str]) -> list[str]:
    seen: dict[str, int] = {}
    unique = []
    for name in names:
        count = seen.get(name, 0)
        seen[name] = count + 1
        unique.append(name if count == 0 else f"{name}_{count + 1}")
    return unique


class LSLRecordingWorker(threading.Thread):
    def __init__(self, inlet, sample_lock, samples, stim_getter, error_signal):
        super().__init__(daemon=True)
        self.inlet = inlet
        self.sample_lock = sample_lock
        self.samples = samples
        self.stim_getter = stim_getter
        self.error_signal = error_signal
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            try:
                chunk, timestamps = self.inlet.pull_chunk(timeout=0.05, max_samples=256)
            except Exception as e:
                self.error_signal.emit(str(e))
                break

            if not chunk:
                continue

            if len(timestamps) != len(chunk):
                timestamps = [np.nan] * len(chunk)

            rows = []
            for sample, lsl_ts in zip(chunk, timestamps):
                stim = self.stim_getter()
                rows.append(
                    {
                        "lsl_timestamp": float(lsl_ts) if lsl_ts else np.nan,
                        "app_timestamp": time.time(),
                        "sample": [float(v) for v in sample],
                        "stim_step_index": stim["step_index"],
                        "stim_dac0_f1_hz": stim["dac0_f1_hz"],
                        "stim_dac0_f2_hz": stim["dac0_f2_hz"],
                        "stim_dac1_f1_hz": stim["dac1_f1_hz"],
                        "stim_dac1_f2_hz": stim["dac1_f2_hz"],
                    }
                )

            with self.sample_lock:
                self.samples.extend(rows)


# ── Main window ───────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    # Signal so serial thread can safely append text to the console
    _console_append = Signal(str, str)  # (text, css_color)
    _lsl_worker_error = Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Контроллер двух синусов  //  Arduino Due  v3.0")
        self.resize(1100, 760)
        self.settings = QSettings("JUT SWO", "DualSineController")
        self.serial = SerialWorker()
        self.serial.error_occurred.connect(self._on_serial_error)
        self.serial.connected.connect(self._on_connected)
        self.serial.message_received.connect(self._log_rx)
        self._console_append.connect(self._append_console)
        self._lsl_worker_error.connect(self._on_lsl_worker_error)
        self._running = False
        self.sequence_steps: list[SequenceStep] = []
        self.sequence_file_path: str | None = None
        self._sequence_index = 0
        self._sequence_timer = QTimer(self)
        self._sequence_timer.setSingleShot(True)
        self._sequence_timer.timeout.connect(self._run_next_sequence_step)
        self._sequence_progress_timer = QTimer(self)
        self._sequence_progress_timer.timeout.connect(self._update_sequence_progress)
        self._auto_apply_timer = QTimer(self)
        self._auto_apply_timer.setSingleShot(True)
        self._auto_apply_timer.timeout.connect(self._auto_apply_params)
        self._sequence_total_duration = 0.0
        self._sequence_total_start_time: float | None = None
        self._sequence_stage_duration = 0.0
        self._sequence_stage_start_time: float | None = None
        self._sequence_stage_number = 0

        self.lsl_streams = []
        self.lsl_inlet = None
        self.lsl_stream_meta: dict = {}
        self.lsl_channel_names: list[str] = []
        self.lsl_worker: LSLRecordingWorker | None = None
        self.lsl_samples: list[dict] = []
        self.lsl_sample_lock = threading.Lock()
        self._record_sequence_steps: list[SequenceStep] = []
        self._stim_lock = threading.Lock()
        self._stim_events: list[dict] = []
        self._current_stim = {
            "step_index": -1,
            "dac0_f1_hz": np.nan,
            "dac0_f2_hz": np.nan,
            "dac1_f1_hz": np.nan,
            "dac1_f2_hz": np.nan,
        }
        self._recording_active = False
        self._record_start_app_time: float | None = None
        self._record_end_app_time: float | None = None
        self._build_ui()

        # Signal stats refresh timer (4 fps — purely local math)
        self._stats_timer = QTimer(self)
        self._stats_timer.timeout.connect(self._refresh_signal_stats)
        self._stats_timer.start(250)

        # Telemetry drain timer — poll serial for unsolicited debug JSON
        self._drain_timer = QTimer(self)
        self._drain_timer.timeout.connect(self._drain_serial)
        self._drain_timer.start(50)  # 20 Hz poll

        # Clip counters for stats panel
        self._total_clip_hi = [0, 0]
        self._total_clip_lo = [0, 0]
        QTimer.singleShot(0, self._attempt_initial_connections)

    def _on_lsl_combo_changed(self, idx: int):
        self._update_lsl_stream_preview()
        # Переподключаемся только если уже был подключён другой поток,
        # или если это первый выбор после refresh
        if self.lsl_inlet is not None:
            self._connect_selected_lsl_stream()

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(14)

        self._init_debug_support_widgets()

        left = QVBoxLayout()
        left.setSpacing(6)
        top_cards = QHBoxLayout()
        top_cards.setSpacing(6)
        top_cards.addWidget(self._build_session_card(), 3)
        top_cards.addWidget(self._build_transport_box(), 2)
        left.addLayout(top_cards)
        left.addWidget(self._build_signal_card())
        left.addWidget(self._build_experiment_card())
        left.addStretch()

        left_w = QWidget()
        left_w.setLayout(left)
        left_w.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Maximum)
        left_w.adjustSize()

        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        left_scroll.setFrameShape(QFrame.Shape.NoFrame)
        left_scroll.setWidget(left_w)
        panel_width = max(520, min(620, left_w.sizeHint().width() + 26))
        left_scroll.setFixedWidth(panel_width)
        left_scroll.setSizePolicy(QSizePolicy.Policy.Fixed, QSizePolicy.Policy.Expanding)

        right = QVBoxLayout()
        right.setSpacing(0)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_preview_tab(), "Мониторинг")
        self.tabs.addTab(self._build_lsl_tab(), "LSL")
        self.tabs.addTab(self._build_service_tab(), "Сервис")

        right.addWidget(self.tabs, 1)

        right_w = QWidget()
        right_w.setLayout(right)

        root.addWidget(left_scroll)
        root.addWidget(right_w, 1)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Отключено")
        self._refresh_lsl_streams()

    def _init_debug_support_widgets(self):
        def make_dbg_label(attr: str):
            label = QLabel("—")
            label.setObjectName("metric_value")
            setattr(self, attr, label)

        for attr in (
            "dstat_isr",
            "dstat_cliphi0",
            "dstat_cliplo0",
            "dstat_env0",
            "dstat_cliphi1",
            "dstat_cliplo1",
            "dstat_env1",
            "dstat_samps",
            "ir_state",
            "ir_raw",
            "ir_volts",
            "ir_span",
            "ir_th",
        ):
            make_dbg_label(attr)

        self.dbg_checkbox = QCheckBox("DBG телеметрия")
        self.dbg_checkbox.setChecked(False)
        self.dbg_checkbox.toggled.connect(self._toggle_dbg)

        self.ir_checkbox = QCheckBox("ИК-датчик (TCRT5000)")
        self.ir_checkbox.setChecked(False)
        self.ir_checkbox.toggled.connect(self._toggle_ir)

        self.autoscroll_cb = QCheckBox("Автопрокрутка")
        self.autoscroll_cb.setChecked(True)

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setPlaceholderText("Журнал команд и ответов Arduino")

        self.cmd_input = QLineEdit()
        self.cmd_input.setObjectName("command_input")
        self.cmd_input.setPlaceholderText("Введите команду Arduino вручную")
        self.cmd_input.returnPressed.connect(self._send_manual_cmd)

    def _make_card(self, title: str, hint: str | None = None) -> tuple[QFrame, QVBoxLayout]:
        card = QFrame()
        card.setObjectName("card")
        layout = QVBoxLayout(card)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        title_label = QLabel(title)
        title_label.setObjectName("card_title")
        layout.addWidget(title_label)

        if hint:
            hint_label = QLabel(hint)
            hint_label.setObjectName("card_hint")
            hint_label.setWordWrap(True)
            layout.addWidget(hint_label)

        return card, layout

    def _make_subcard(self, title: str, color: str | None = None) -> tuple[QFrame, QVBoxLayout]:
        box = QFrame()
        box.setObjectName("subcard")
        layout = QVBoxLayout(box)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        title_label = QLabel(title)
        title_label.setObjectName("subcard_title")
        if color:
            title_label.setStyleSheet(f"color:{color};")
        layout.addWidget(title_label)
        return box, layout

    def _make_field_label(self, text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("field_label")
        return label

    def _build_session_card(self) -> QFrame:
        card, layout = self._make_card("Сеанс")
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        grid.setColumnStretch(1, 1)

        serial_refresh_btn = QPushButton("Обновить")
        serial_refresh_btn.setObjectName("ghost_btn")
        serial_refresh_btn.setMinimumWidth(96)
        serial_refresh_btn.setToolTip("Обновить список портов")
        serial_refresh_btn.clicked.connect(self._refresh_ports)

        self.port_combo = QComboBox()
        self.port_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.port_combo.activated.connect(self._connect_selected_serial_port)
        self._refresh_ports()

        self.serial_connect_check = QCheckBox()
        self.serial_connect_check.setObjectName("serial_status")
        self.serial_connect_check.setAttribute(
            Qt.WidgetAttribute.WA_TransparentForMouseEvents, True
        )
        self.serial_connect_check.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.serial_connect_check.setToolTip("Устройство не подключено")
        self._set_serial_indicator("disconnected")

        grid.addWidget(self._make_field_label("Arduino"), 0, 0)
        grid.addWidget(self.port_combo, 0, 1)
        grid.addWidget(self.serial_connect_check, 0, 2)
        grid.addWidget(serial_refresh_btn, 0, 3)

        self.lsl_stream_combo = QComboBox()
        self.lsl_stream_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.lsl_stream_combo.currentIndexChanged.connect(self._on_lsl_combo_changed)

        self.lsl_connect_check = QCheckBox()
        self.lsl_connect_check.setObjectName("lsl_status")
        self.lsl_connect_check.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
        self.lsl_connect_check.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.lsl_connect_check.setToolTip("LSL не подключен")
        self._set_lsl_indicator("disconnected")

        self.lsl_refresh_btn = QPushButton("Обновить")
        self.lsl_refresh_btn.setObjectName("ghost_btn")
        self.lsl_refresh_btn.setMinimumWidth(96)
        self.lsl_refresh_btn.clicked.connect(self._refresh_lsl_streams)

        grid.addWidget(self._make_field_label("LSL"), 1, 0)
        grid.addWidget(self.lsl_stream_combo, 1, 1)
        grid.addWidget(self.lsl_connect_check, 1, 2)
        grid.addWidget(self.lsl_refresh_btn, 1, 3)

        self.lsl_status_label = QLabel("Отключено")
        self.lsl_status_label.setObjectName("status_value")

        layout.addLayout(grid)
        layout.addWidget(self.lsl_status_label)
        return card

    def _build_signal_card(self) -> QFrame:
        card, layout = self._make_card("Сигнал")
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(10)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.addWidget(self._build_sine_box("Синус 1", "1", COLORS["sine1"]), 0, 0)
        grid.addWidget(self._build_sine_box("Синус 2", "2", COLORS["sine2"]), 0, 1)
        grid.addWidget(
            self._build_dac_box("DAC0 (канал 1)", "dac0", COLORS["accent"]),
            1,
            0,
        )
        grid.addWidget(
            self._build_dac_box("DAC1 (канал 2)", "dac1", COLORS["combined"]),
            1,
            1,
        )
        layout.addLayout(grid)
        return card

    def _build_experiment_card(self) -> QFrame:
        card, layout = self._make_card("Эксперимент")
        layout.addWidget(self._build_global_box())
        layout.addWidget(self._build_sequence_box())
        return card

    def _build_preview_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(12, 12, 12, 12)
        v.setSpacing(12)

        lbl = QLabel("Этапы последовательности")
        lbl.setObjectName("section_label")
        v.addWidget(lbl)

        self.sequence_table = QTableWidget(0, 6)
        self.sequence_table.setHorizontalHeaderLabels(
            [
                "№ этапа",
                "DAC0 (канал 1) F1",
                "DAC0 (канал 1) F2",
                "DAC1 (канал 2) F1",
                "DAC1 (канал 2) F2",
                "Оставшееся время",
            ]
        )
        self.sequence_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.sequence_table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.sequence_table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.sequence_table.verticalHeader().setVisible(False)
        self.sequence_table.setAlternatingRowColors(False)
        header = self.sequence_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        v.addWidget(self.sequence_table, 1)

        progress_grid = QGridLayout()
        progress_grid.setHorizontalSpacing(12)
        progress_grid.setVerticalSpacing(8)

        self.stage_time_label = QLabel("Текущий этап")
        self.stage_time_label.setObjectName("field_label")
        progress_grid.addWidget(self.stage_time_label, 0, 0)

        self.total_time_label = QLabel("Весь эксперимент")
        self.total_time_label.setObjectName("field_label")
        progress_grid.addWidget(self.total_time_label, 0, 1)

        self.stage_time_bar = QProgressBar()
        self.stage_time_bar.setRange(0, 1000)
        self.stage_time_bar.setValue(0)
        self.stage_time_bar.setTextVisible(True)
        self.stage_time_bar.setFormat("Этап: ожидание")
        progress_grid.addWidget(self.stage_time_bar, 1, 0)

        self.total_time_bar = QProgressBar()
        self.total_time_bar.setRange(0, 1000)
        self.total_time_bar.setValue(0)
        self.total_time_bar.setTextVisible(True)
        self.total_time_bar.setFormat("Эксперимент: ожидание")
        progress_grid.addWidget(self.total_time_bar, 1, 1)

        v.addLayout(progress_grid)
        v.addWidget(self._build_stats_panel())
        return w

    def _build_lsl_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(12, 12, 12, 12)
        v.setSpacing(12)

        lsl_label = QLabel("Поток и запись LSL")
        lsl_label.setObjectName("section_label")
        v.addWidget(lsl_label)

        stream_box = QGroupBox("Информация о потоке")
        stream_grid = QGridLayout(stream_box)
        stream_grid.setHorizontalSpacing(10)
        stream_grid.setVerticalSpacing(10)

        lsl_hint_label = QLabel(
            "Выбор потока и подключение находятся в карточке «Сеанс» слева."
        )
        lsl_hint_label.setObjectName("hint_label")
        lsl_hint_label.setWordWrap(True)
        stream_grid.addWidget(lsl_hint_label, 0, 0, 1, 3)

        self.lsl_meta_text = QTextEdit()
        self.lsl_meta_text.setReadOnly(True)
        self.lsl_meta_text.setMinimumHeight(150)
        stream_grid.addWidget(self.lsl_meta_text, 1, 0, 1, 3)
        v.addWidget(stream_box)

        filter_label = QLabel("Фильтры записи")
        filter_label.setObjectName("section_label")
        v.addWidget(filter_label)

        filter_box = QGroupBox("Фильтрация и экспорт")
        filter_grid = QGridLayout(filter_box)
        filter_grid.setHorizontalSpacing(10)
        filter_grid.setVerticalSpacing(8)

        self.bandpass_enabled = QCheckBox("Полосовой")
        self.bandpass_enabled.setChecked(True)
        filter_grid.addWidget(self.bandpass_enabled, 0, 0)

        self.bandpass_low = make_spinbox(0.01, 5000.0, 2, 0.5, 0.5, "Гц")
        self.bandpass_high = make_spinbox(0.01, 5000.0, 2, 1.0, 40.0, "Гц")
        self.bandpass_order = QSpinBox()
        self.bandpass_order.setRange(1, 12)
        self.bandpass_order.setValue(4)
        filter_grid.addWidget(QLabel("Низ"), 1, 0)
        filter_grid.addWidget(self.bandpass_low, 1, 1)
        filter_grid.addWidget(QLabel("Верх"), 2, 0)
        filter_grid.addWidget(self.bandpass_high, 2, 1)
        filter_grid.addWidget(QLabel("Порядок"), 3, 0)
        filter_grid.addWidget(self.bandpass_order, 3, 1)

        self.notch_enabled = QCheckBox("Режекторный")
        self.notch_enabled.setChecked(True)
        filter_grid.addWidget(self.notch_enabled, 0, 2)

        self.notch_freq = make_spinbox(0.01, 5000.0, 2, 1.0, 50.0, "Гц")
        self.notch_q = make_spinbox(0.1, 500.0, 1, 1.0, 30.0, "Q")
        filter_grid.addWidget(QLabel("Частота"), 1, 2)
        filter_grid.addWidget(self.notch_freq, 1, 3)
        filter_grid.addWidget(QLabel("Q"), 2, 2)
        filter_grid.addWidget(self.notch_q, 2, 3)

        self.recording_status_label = QLabel("Запись: ожидание")
        self.recording_status_label.setObjectName("status_value")
        filter_grid.addWidget(self.recording_status_label, 4, 0, 1, 4)

        self.export_last_btn = QPushButton("Экспорт последней записи в TXT")
        self.export_last_btn.clicked.connect(self._export_lsl_recording)
        self.export_last_btn.setEnabled(False)
        filter_grid.addWidget(self.export_last_btn, 5, 0, 1, 4)
        v.addWidget(filter_box)
        v.addStretch()
        return w

    def _build_service_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(12, 12, 12, 12)
        v.setSpacing(12)

        toolbar = QHBoxLayout()
        toolbar.setSpacing(10)
        toolbar.addWidget(self.dbg_checkbox)
        toolbar.addWidget(self.ir_checkbox)
        toolbar.addWidget(self.autoscroll_cb)
        toolbar.addStretch()

        clear_btn = QPushButton("Очистить журнал")
        clear_btn.setObjectName("clear_btn")
        clear_btn.clicked.connect(self._clear_console)
        toolbar.addWidget(clear_btn)
        v.addLayout(toolbar)

        v.addWidget(self._build_debug_stats_panel())
        v.addWidget(self._build_ir_panel())

        cmd_box, cmd_layout = self._make_subcard("Ручная команда", COLORS["accent2"])
        cmd_row = QHBoxLayout()
        cmd_row.setSpacing(8)
        cmd_row.addWidget(self.cmd_input, 1)
        send_btn = QPushButton("Отправить")
        send_btn.setObjectName("ghost_btn")
        send_btn.clicked.connect(self._send_manual_cmd)
        cmd_row.addWidget(send_btn)
        cmd_layout.addLayout(cmd_row)
        v.addWidget(cmd_box)

        console_box, console_layout = self._make_subcard("Журнал", COLORS["accent"])
        console_layout.addWidget(self.console, 1)
        v.addWidget(console_box, 1)
        return w

    def _build_sine_box(self, title: str, idx: str, color: str) -> QFrame:
        box, layout = self._make_subcard(title, color)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)
        grid.setColumnStretch(1, 1)

        def row(label, widget):
            r = grid.rowCount()
            grid.addWidget(self._make_field_label(label), r, 0)
            grid.addWidget(widget, r, 1)

        if idx == "1":
            self.amp1 = make_spinbox(0, 1.65, 3, 0.05, 0.5, "В")
            self.phase1 = make_spinbox(-360, 360, 1, 5.0, 0.0, "°")
            for c in (self.amp1, self.phase1):
                c.valueChanged.connect(self._param_changed)
            row("Амплитуда", self.amp1)
            row("Фаза", self.phase1)
        else:
            self.amp2 = make_spinbox(0, 1.65, 3, 0.05, 0.5, "В")
            self.phase2 = make_spinbox(-360, 360, 1, 5.0, 0.0, "°")
            for c in (self.amp2, self.phase2):
                c.valueChanged.connect(self._param_changed)
            row("Амплитуда", self.amp2)
            row("Фаза", self.phase2)
        layout.addLayout(grid)
        return box

    def _build_dac_box(self, title: str, attr_prefix: str, color: str) -> QFrame:
        box, layout = self._make_subcard(title, color)
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)
        grid.setColumnStretch(1, 1)

        def row(label, widget):
            r = grid.rowCount()
            grid.addWidget(self._make_field_label(label), r, 0)
            grid.addWidget(widget, r, 1)

        freq1 = make_spinbox(0, 5000, 2, 10.0, 100.0, "Гц")
        freq2 = make_spinbox(0, 5000, 2, 10.0, 200.0, "Гц")
        freq1.valueChanged.connect(self._param_changed)
        freq2.valueChanged.connect(self._param_changed)
        setattr(self, f"{attr_prefix}_f1", freq1)
        setattr(self, f"{attr_prefix}_f2", freq2)
        row("Частота 1", freq1)
        row("Частота 2", freq2)
        layout.addLayout(grid)
        return box

    def _build_sequence_box(self) -> QFrame:
        box, layout = self._make_subcard("Последовательность частот", COLORS["accent2"])

        self.sequence_path_edit = QLineEdit()
        self.sequence_path_edit.setReadOnly(True)
        self.sequence_path_edit.setPlaceholderText("Файл последовательности не загружен")
        layout.addWidget(self.sequence_path_edit)

        self.sequence_status = QLabel("Формат: D0F1 D0F2 D1F1 D1F2 сек")
        self.sequence_status.setObjectName("hint_label")
        self.sequence_status.setWordWrap(True)
        layout.addWidget(self.sequence_status)

        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)
        self.sequence_load_btn = QPushButton("Загрузить")
        self.sequence_load_btn.clicked.connect(self._load_sequence_file)
        btn_row.addWidget(self.sequence_load_btn)

        self.sequence_clear_btn = QPushButton("Сброс")
        self.sequence_clear_btn.setObjectName("clear_btn")
        self.sequence_clear_btn.clicked.connect(self._clear_sequence_file)
        self.sequence_clear_btn.setEnabled(False)
        btn_row.addWidget(self.sequence_clear_btn)
        layout.addLayout(btn_row)
        return box

    def _build_global_box(self) -> QFrame:
        box, layout = self._make_subcard("Общие параметры", COLORS["accent"])
        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)

        self.dc_offset = make_spinbox(0, 3.3, 3, 0.05, 2.0, "В")
        self.sample_rate = QSpinBox()
        self.sample_rate.setRange(100, 10000)
        self.sample_rate.setValue(1000)
        self.sample_rate.setSuffix("  Гц")
        self.sample_rate.setSingleStep(1000)
        self.gain_max = make_spinbox(1.0, 20.0, 1, 0.5, 4.0, "×")

        for c in (self.dc_offset, self.sample_rate, self.gain_max):
            c.valueChanged.connect(self._param_changed)

        fields = (
            ("DC смещение", self.dc_offset),
            ("Частота дискр.", self.sample_rate),
            ("Макс. огиб.", self.gain_max),
        )
        for col, (label, widget) in enumerate(fields):
            grid.addWidget(self._make_field_label(label), 0, col)
            grid.addWidget(widget, 1, col)
            grid.setColumnStretch(col, 1)

        self.clip_label = QLabel("")
        self.clip_label.setObjectName("warning_label")
        self.clip_label.setWordWrap(True)
        layout.addLayout(grid)
        layout.addWidget(self.clip_label)
        return box

    def _build_transport_box(self) -> QFrame:
        box, layout = self._make_card("Запуск")
        lay = QVBoxLayout()
        lay.setSpacing(8)

        self.start_btn = QPushButton("▶ Старт")
        self.start_btn.setObjectName("start_btn")
        self.start_btn.clicked.connect(self._start)
        self.start_btn.setEnabled(False)

        self.stop_btn = QPushButton("■ Стоп")
        self.stop_btn.setObjectName("stop_btn")
        self.stop_btn.clicked.connect(self._stop)
        self.stop_btn.setEnabled(False)

        lay.addWidget(self.start_btn)
        lay.addWidget(self.stop_btn)
        layout.addLayout(lay)
        return box

    def _build_stats_panel(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("subcard")
        grid = QGridLayout(frame)
        grid.setContentsMargins(12, 12, 12, 12)
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(8)

        def stat(label, attr, col):
            lb = QLabel(label)
            lb.setObjectName("metric_label")
            vl = QLabel("—")
            vl.setObjectName("metric_value")
            setattr(self, attr, vl)
            grid.addWidget(lb, 0, col)
            grid.addWidget(vl, 1, col)

        stat("DAC0 (канал 1) пик", "stat_dac0_peak", 0)
        stat("DAC0 (канал 1) минимум", "stat_dac0_trough", 1)
        stat("DAC0 (канал 1) размах", "stat_dac0_range", 2)
        stat("DAC1 (канал 2) пик", "stat_dac1_peak", 3)
        stat("DAC1 (канал 2) минимум", "stat_dac1_trough", 4)
        stat("DAC1 (канал 2) размах", "stat_dac1_range", 5)
        stat("Найквист", "stat_nyquist", 6)
        return frame

    def _build_debug_stats_panel(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("subcard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        title = QLabel("Диагностика")
        title.setObjectName("subcard_title")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(8)

        metrics = (
            ("ISR", self.dstat_isr),
            ("Samples", self.dstat_samps),
            ("DAC0 (канал 1) clip+", self.dstat_cliphi0),
            ("DAC0 (канал 1) clip-", self.dstat_cliplo0),
            ("DAC0 (канал 1) env", self.dstat_env0),
            ("DAC1 (канал 2) clip+", self.dstat_cliphi1),
            ("DAC1 (канал 2) clip-", self.dstat_cliplo1),
            ("DAC1 (канал 2) env", self.dstat_env1),
        )
        for idx, (label, value) in enumerate(metrics):
            row = idx // 4
            col = (idx % 4) * 2
            caption = QLabel(label)
            caption.setObjectName("metric_label")
            value.setObjectName("metric_value")
            grid.addWidget(caption, row, col)
            grid.addWidget(value, row, col + 1)
            grid.setColumnStretch(col + 1, 1)

        layout.addLayout(grid)
        return frame

    def _build_ir_panel(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("subcard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        title = QLabel("ИК-датчик TCRT5000 (аналоговый вход A0)")
        title.setObjectName("subcard_title")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(8)

        metrics = (
            ("Состояние", self.ir_state),
            ("RAW (0…4095)", self.ir_raw),
            ("Напряжение, В", self.ir_volts),
            ("Мин / Макс за 100 мс", self.ir_span),
            ("Порог срабатывания", self.ir_th),
        )
        for idx, (label, value) in enumerate(metrics):
            row = idx // 3
            col = (idx % 3) * 2
            caption = QLabel(label)
            caption.setObjectName("metric_label")
            value.setObjectName("metric_value")
            grid.addWidget(caption, row, col)
            grid.addWidget(value, row, col + 1)
            grid.setColumnStretch(col + 1, 1)

        layout.addLayout(grid)
        return frame

    def _set_status_indicator(self, widget: QCheckBox, state: str, tooltip: str):
        color = {
            "disconnected": COLORS["red"],
            "connecting": COLORS["yellow"],
            "connected": COLORS["green"],
        }[state]
        widget.blockSignals(True)
        widget.setChecked(state == "connected")
        widget.setToolTip(tooltip)
        widget.blockSignals(False)
        widget.setStyleSheet(
            f"QCheckBox#{widget.objectName()} {{ spacing: 0px; }}"
            f"QCheckBox#{widget.objectName()}::indicator {{"
            f" width: 14px; height: 14px;"
            f" border: 1px solid {color};"
            f" border-radius: 3px;"
            f" background: {color};"
            f" }}"
        )

    def _set_serial_indicator(self, state: str):
        tooltips = {
            "disconnected": "Устройство не подключено",
            "connecting": "Подключение к устройству...",
            "connected": "Устройство подключено",
        }
        self._set_status_indicator(self.serial_connect_check, state, tooltips[state])

    def _set_lsl_indicator(self, state: str):
        tooltips = {
            "disconnected": "LSL не подключен",
            "connecting": "Подключение к LSL...",
            "connected": "LSL подключен",
        }
        self._set_status_indicator(self.lsl_connect_check, state, tooltips[state])

    def _valid_serial_port_selected(self) -> bool:
        port = self.port_combo.currentText()
        return bool(port) and "no ports" not in port and "порты не найдены" not in port

    def _valid_lsl_stream_selected(self) -> bool:
        idx = self.lsl_stream_combo.currentIndex()
        return 0 <= idx < len(self.lsl_streams)

    def _save_serial_selection(self, port: str):
        self.settings.setValue("serial/port", port)

    def _restore_serial_selection(self):
        saved_port = str(self.settings.value("serial/port", "", type=str))
        if not saved_port:
            return
        idx = self.port_combo.findText(saved_port)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)

    def _lsl_stream_key_from_meta(self, meta: dict) -> str:
        return meta.get("source_id") or meta.get("uid") or "|".join(
            [
                str(meta.get("name", "")),
                str(meta.get("type", "")),
                str(meta.get("channel_count", 0)),
                str(meta.get("nominal_srate_hz", 0.0)),
            ]
        )

    def _save_lsl_selection(self, idx: int):
        if idx < 0 or idx >= len(self.lsl_streams):
            return
        meta = self._read_lsl_stream_meta(self.lsl_streams[idx])
        self.settings.setValue("lsl/stream_key", self._lsl_stream_key_from_meta(meta))

    def _restore_lsl_selection(self):
        saved_key = str(self.settings.value("lsl/stream_key", "", type=str))
        if not saved_key:
            return
        for idx, info in enumerate(self.lsl_streams):
            meta = self._read_lsl_stream_meta(info)
            if self._lsl_stream_key_from_meta(meta) == saved_key:
                self.lsl_stream_combo.setCurrentIndex(idx)
                return

    def _attempt_initial_connections(self):
        if not self.serial.is_open and self._valid_serial_port_selected():
            self._connect_selected_serial_port()
        if self.lsl_inlet is None and self._valid_lsl_stream_selected():
            self._connect_selected_lsl_stream()

    # ── Port helpers ──────────────────────────────────────────────────────────
    def _refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)
        if not ports:
            self.port_combo.addItem("(порты не найдены)")
        self._restore_serial_selection()

    def _connect_selected_serial_port(self, idx: int | None = None):
        port = self.port_combo.currentText()
        if not port or "no ports" in port or "порты не найдены" in port:
            self._log("Serial-порт не выбран.", COLORS["yellow"])
            self._set_serial_checkbox(False)
            return

        if self.serial.is_open:
            if self._running or self._recording_active:
                self._stop(reason="Serial переключен")
            else:
                self.serial.close()

        self._save_serial_selection(port)
        self._set_serial_indicator("connecting")
        self.status_bar.showMessage(f"Подключение к {port} …")
        self._log(f"Подключение к {port} @ 115200 …", COLORS["accent"])
        QApplication.processEvents()
        ok = self.serial.open(port)
        if ok:
            self.status_bar.showMessage(f"Подключено  ·  {port}  @  115200 бод")
            self._log(f"Подключено к {port}.", COLORS["green"])
        else:
            self._set_serial_checkbox(False)

    # ── LSL helpers ──────────────────────────────────────────────────────────
    def _refresh_lsl_streams(self):
        self.lsl_stream_combo.clear()
        self.lsl_streams = []

        if resolve_streams is None:
            msg = f"pylsl недоступен: {LSL_IMPORT_ERROR}"
            self.lsl_stream_combo.addItem("(pylsl недоступен)")
            self.lsl_status_label.setText(msg)
            self._set_lsl_checkbox(False, enabled=False)
            self.lsl_meta_text.setPlainText(msg)
            return

        try:
            self.lsl_status_label.setText("Поиск LSL-потоков...")
            QApplication.processEvents()
            self.lsl_streams = resolve_streams(wait_time=1.0)
        except Exception as e:
            self.lsl_status_label.setText(f"Ошибка обновления LSL: {e}")
            self._log(f"Ошибка обновления LSL: {e}", COLORS["red"])
            self._set_lsl_checkbox(False, enabled=False)
            return

        if not self.lsl_streams:
            self.lsl_stream_combo.addItem("(LSL-потоки не найдены)")
            self.lsl_status_label.setText("LSL-потоки не найдены")
            self._set_lsl_checkbox(False, enabled=False)
            self.lsl_meta_text.clear()
            return

        for info in self.lsl_streams:
            meta = self._read_lsl_stream_meta(info)
            self.lsl_stream_combo.addItem(
                f"{meta['name']} | {meta['type']} | "
                f"{meta['channel_count']} кан. | {meta['nominal_srate_hz']} Гц"
            )
        self._restore_lsl_selection()
        self.lsl_status_label.setText(f"Найдено LSL-потоков: {len(self.lsl_streams)}")
        connected = self.lsl_inlet is not None
        self._set_lsl_checkbox(connected, enabled=True)
        if not connected:
            self.lsl_meta_text.clear()
        self._update_lsl_stream_preview()

    def _update_lsl_stream_preview(self):
        idx = self.lsl_stream_combo.currentIndex()
        if idx < 0 or idx >= len(self.lsl_streams):
            return
        meta = self._read_lsl_stream_meta(self.lsl_streams[idx])
        names = self._read_lsl_channel_names(self.lsl_streams[idx], meta["channel_count"])
        self.lsl_meta_text.setPlainText(
            "\n".join(
                [
                    f"имя: {meta['name']}",
                    f"тип: {meta['type']}",
                    f"source_id: {meta['source_id']}",
                    f"uid: {meta['uid']}",
                    f"каналов: {meta['channel_count']}",
                    f"частота_дискретизации_Гц: {meta['nominal_srate_hz']}",
                    f"формат_канала: {meta['channel_format']}",
                    f"имена_каналов: {', '.join(names)}",
                ]
            )
        )

    def _connect_lsl_stream(self):
        if StreamInlet is None:
            self._log(f"Не удалось подключить LSL: {LSL_IMPORT_ERROR}", COLORS["red"])
            self._set_lsl_checkbox(False, enabled=False)
            return
        idx = self.lsl_stream_combo.currentIndex()
        if idx < 0 or idx >= len(self.lsl_streams):
            self._log("Сначала выберите LSL-поток.", COLORS["yellow"])
            self._set_lsl_checkbox(False, enabled=bool(self.lsl_streams))
            return

        self._save_lsl_selection(idx)
        self._set_lsl_indicator("connecting")
        self.lsl_status_label.setText("Подключение к LSL-потоку...")
        QApplication.processEvents()
        try:
            info = self.lsl_streams[idx]
            self.lsl_inlet = StreamInlet(info, max_buflen=60)
            self.lsl_stream_meta = self._read_lsl_stream_meta(info)
            self.lsl_channel_names = self._read_lsl_channel_names(
                info, self.lsl_stream_meta["channel_count"]
            )
        except Exception as e:
            self.lsl_inlet = None
            self.lsl_stream_meta = {}
            self.lsl_channel_names = []
            self._log(f"Ошибка подключения LSL: {e}", COLORS["red"])
            self.lsl_status_label.setText(f"Ошибка подключения LSL: {e}")
            self._set_lsl_checkbox(False, enabled=True)
            return

        self._set_lsl_checkbox(True, enabled=True)
        self.lsl_refresh_btn.setEnabled(False)
        self.lsl_status_label.setText(
            f"Подключено: {self.lsl_stream_meta['name']} | "
            f"{self.lsl_stream_meta['channel_count']} кан. | "
            f"{self.lsl_stream_meta['nominal_srate_hz']} Гц"
        )
        self._log(f"LSL-поток подключен: {self.lsl_stream_meta['name']}", COLORS["green"])

    def _connect_selected_lsl_stream(self, idx: int | None = None):
        if self.lsl_inlet is not None:
            self._disconnect_lsl_stream()
        self._connect_lsl_stream()

    def _disconnect_lsl_stream(self):
        if self._recording_active or self._running:
            self._log("LSL отключен во время эксперимента.", COLORS["red"])
            self._stop(reason="LSL отключен")

        if self.lsl_inlet is not None:
            try:
                self.lsl_inlet.close_stream()
            except Exception:
                pass
        self.lsl_inlet = None
        self.lsl_stream_meta = {}
        self.lsl_channel_names = []
        self._set_lsl_checkbox(False, enabled=bool(self.lsl_streams))
        self.lsl_refresh_btn.setEnabled(True)
        self.lsl_status_label.setText("Отключено")
        self._log("LSL-поток отключен.", COLORS["text_dim"])

    def _toggle_lsl_connection(self, checked: bool):
        if checked:
            self._connect_lsl_stream()
        else:
            self._disconnect_lsl_stream()

    def _set_lsl_checkbox(self, checked: bool, enabled: bool = True):
        self.lsl_connect_check.setEnabled(enabled)
        self._set_lsl_indicator("connected" if checked else "disconnected")

    def _read_lsl_stream_meta(self, info) -> dict:
        def read(method_name: str, default):
            try:
                return getattr(info, method_name)()
            except Exception:
                return default

        protocol_version = ""
        library_version = ""
        if pylsl is not None:
            for attr, target in (
                ("protocol_version", "protocol_version"),
                ("library_version", "library_version"),
            ):
                try:
                    val = getattr(pylsl, attr)
                    value = val() if callable(val) else val
                except Exception:
                    value = ""
                if target == "protocol_version":
                    protocol_version = value
                else:
                    library_version = value

        return {
            "name": read("name", ""),
            "type": read("type", ""),
            "source_id": read("source_id", ""),
            "uid": read("uid", ""),
            "channel_count": int(read("channel_count", 0) or 0),
            "nominal_srate_hz": float(read("nominal_srate", 0.0) or 0.0),
            "channel_format": read("channel_format", ""),
            "lsl_protocol_version": protocol_version,
            "lsl_library_version": library_version,
        }

    def _read_lsl_channel_names(self, info, channel_count: int) -> list[str]:
        names = []
        try:
            ch = info.desc().child("channels").child("channel")
            while ch is not None and not ch.empty():
                label = ch.child_value("label") or ch.child_value("name")
                if label:
                    names.append(label)
                ch = ch.next_sibling()
        except Exception:
            names = []

        while len(names) < channel_count:
            names.append(f"ch_{len(names) + 1}")
        return names[:channel_count]

    def _get_params(self) -> dict:
        return {
            "a1": self.amp1.value(),
            "p1": self.phase1.value(),
            "a2": self.amp2.value(),
            "p2": self.phase2.value(),
            "dac0_f1": self.dac0_f1.value(),
            "dac0_f2": self.dac0_f2.value(),
            "dac1_f1": self.dac1_f1.value(),
            "dac1_f2": self.dac1_f2.value(),
            "dc": self.dc_offset.value(),
            "sr": self.sample_rate.value(),
            "gain_max": self.gain_max.value(),
        }

    def _load_sequence_file(self):
        if self._running:
            self._log("Остановите генерацию перед загрузкой файла частот.", COLORS["yellow"])
            return

        path, _ = QFileDialog.getOpenFileName(
            self,
            "Загрузить последовательность частот",
            "",
            "Текстовые файлы (*.txt *.csv);;Все файлы (*)",
        )
        if not path:
            return

        try:
            steps = self._parse_sequence_file(path)
        except ValueError as e:
            self._log(f"Ошибка файла частот: {e}", COLORS["red"])
            self.status_bar.showMessage("Ошибка файла частот")
            return

        self.sequence_steps = steps
        self.sequence_file_path = path
        self._sequence_index = 0
        total_seconds = self._sequence_total_seconds()
        self.sequence_path_edit.setText(Path(path).name)
        self.sequence_path_edit.setToolTip(path)
        self.sequence_status.setText(
            f"Шагов: {len(steps)} | всего: {total_seconds:.3f} с | частоты DAC0/DAC1 игнорируются"
        )
        self.sequence_clear_btn.setEnabled(True)
        self._set_frequency_controls_enabled(False)
        self._populate_sequence_table()
        self._reset_sequence_progress(total_seconds)
        self.tabs.setCurrentIndex(0)
        self._log(
            f"Файл частот загружен: {path} ({len(steps)} шагов, {total_seconds:.3f} с)",
            COLORS["green"],
        )

    def _clear_sequence_file(self):
        if self._running:
            self._log("Остановите генерацию перед сбросом файла частот.", COLORS["yellow"])
            return
        self.sequence_steps = []
        self.sequence_file_path = None
        self._sequence_index = 0
        self.sequence_path_edit.clear()
        self.sequence_path_edit.setToolTip("")
        self.sequence_status.setText("Формат: D0F1 D0F2 D1F1 D1F2 сек")
        self.sequence_clear_btn.setEnabled(False)
        self._set_frequency_controls_enabled(True)
        self._populate_sequence_table()
        self._reset_sequence_progress()
        self._log("Файл частот сброшен. Поля DAC0/DAC1 снова активны.", COLORS["text_dim"])

    def _parse_sequence_file(self, path: str) -> list[SequenceStep]:
        steps = []
        errors = []
        with open(path, "r", encoding="utf-8-sig") as f:
            for line_no, raw in enumerate(f, 1):
                line = raw.strip()
                if not line or line.startswith("#") or line.startswith("//"):
                    continue
                line = re.split(r"\s+#|\s+//", line, maxsplit=1)[0].strip()
                if not line:
                    continue

                parts = [p for p in re.split(r"[\s,;]+", line) if p]
                if len(parts) != 5:
                    errors.append(
                        "строка "
                        f"{line_no}: ожидалось 5 значений "
                        "(DAC0_F1 DAC0_F2 DAC1_F1 DAC1_F2 секунды), "
                        f"получено {len(parts)}"
                    )
                    continue

                try:
                    values = [float(p) for p in parts]
                except ValueError:
                    errors.append(f"строка {line_no}: значения должны быть числами")
                    continue

                dac0_f1, dac0_f2, dac1_f1, dac1_f2, seconds = values
                freqs = (dac0_f1, dac0_f2, dac1_f1, dac1_f2)
                if any(not (0 <= freq <= 5000) for freq in freqs):
                    errors.append(f"строка {line_no}: частоты должны быть в диапазоне 0..5000 Гц")
                    continue
                if seconds <= 0:
                    errors.append(f"строка {line_no}: длительность должна быть больше 0")
                    continue
                steps.append(
                    SequenceStep(
                        dac0_f1=dac0_f1,
                        dac0_f2=dac0_f2,
                        dac1_f1=dac1_f1,
                        dac1_f2=dac1_f2,
                        seconds=seconds,
                    )
                )

        if errors:
            preview = "; ".join(errors[:5])
            if len(errors) > 5:
                preview += f"; ... и еще {len(errors) - 5}"
            raise ValueError(preview)
        if not steps:
            raise ValueError("в файле нет строк последовательности")
        return steps

    def _set_frequency_controls_enabled(self, enabled: bool):
        for widget in (self.dac0_f1, self.dac0_f2, self.dac1_f1, self.dac1_f2):
            widget.setEnabled(enabled)

    def _sequence_total_seconds(self) -> float:
        return sum(step.seconds for step in self.sequence_steps)

    def _format_seconds(self, seconds: float) -> str:
        seconds = max(0.0, float(seconds))
        total = int(round(seconds))
        hours, rem = divmod(total, 3600)
        minutes, secs = divmod(rem, 60)
        if hours:
            return f"{hours:d}:{minutes:02d}:{secs:02d}"
        return f"{minutes:02d}:{secs:02d}"

    def _format_step_frequency(self, value: float | None) -> str:
        if value is None:
            return "выкл."
        return f"{value:.4f} Гц"

    def _make_sequence_table_item(self, text: str) -> QTableWidgetItem:
        item = QTableWidgetItem(text)
        item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        item.setFlags(Qt.ItemFlag.ItemIsEnabled)
        return item

    def _populate_sequence_table(self):
        self.sequence_table.setRowCount(len(self.sequence_steps))
        for row, step in enumerate(self.sequence_steps):
            values = [
                str(row + 1),
                self._format_step_frequency(step.dac0_f1),
                self._format_step_frequency(step.dac0_f2),
                self._format_step_frequency(step.dac1_f1),
                self._format_step_frequency(step.dac1_f2),
                self._format_seconds(step.seconds),
            ]
            for col, value in enumerate(values):
                self.sequence_table.setItem(row, col, self._make_sequence_table_item(value))
            self.sequence_table.setRowHeight(row, 38)
        self._style_sequence_table()

    def _set_sequence_remaining(self, step_no: int, text: str):
        row = step_no - 1
        if row < 0 or row >= self.sequence_table.rowCount():
            return
        item = self.sequence_table.item(row, 5)
        if item is not None:
            item.setText(text)

    def _reset_sequence_table_remaining(self):
        for row, step in enumerate(self.sequence_steps):
            item = self.sequence_table.item(row, 5)
            if item is not None:
                item.setText(self._format_seconds(step.seconds))

    def _style_sequence_table(
        self,
        current_step_no: int | None = None,
        completed_until: int = 0,
    ):
        for row in range(self.sequence_table.rowCount()):
            step_no = row + 1
            if current_step_no == step_no:
                bg = QColor(COLORS["accent"])
                fg = QColor(COLORS["bg"])
            elif step_no <= completed_until:
                bg = QColor(COLORS["surface2"])
                fg = QColor(COLORS["text_dim"])
            else:
                bg = QColor(COLORS["surface"])
                fg = QColor(COLORS["text"])
            for col in range(self.sequence_table.columnCount()):
                item = self.sequence_table.item(row, col)
                if item is not None:
                    item.setBackground(bg)
                    item.setForeground(fg)

    def _reset_sequence_progress(self, total_seconds: float = 0.0):
        self._sequence_progress_timer.stop()
        self._sequence_total_duration = float(total_seconds)
        self._sequence_total_start_time = None
        self._sequence_stage_duration = 0.0
        self._sequence_stage_start_time = None
        self._sequence_stage_number = 0
        self.stage_time_bar.setValue(0)
        self.stage_time_bar.setFormat("Этап: ожидание")
        self._reset_sequence_table_remaining()
        self._style_sequence_table()
        if total_seconds > 0:
            self.total_time_bar.setValue(0)
            self.total_time_bar.setFormat(
                f"Эксперимент: всего {self._format_seconds(total_seconds)}"
            )
        else:
            self.total_time_bar.setValue(0)
            self.total_time_bar.setFormat("Эксперимент: ожидание")

    def _begin_sequence_stage_progress(self, step_no: int, total_steps: int, seconds: float):
        now = time.monotonic()
        if self._sequence_total_start_time is None:
            self._sequence_total_start_time = now
        self._sequence_stage_start_time = now
        self._sequence_stage_duration = float(seconds)
        self._sequence_stage_number = step_no
        self._set_sequence_remaining(step_no - 1, "00:00")
        self._style_sequence_table(current_step_no=step_no, completed_until=step_no - 1)
        self._sequence_progress_timer.start(200)
        self._update_sequence_progress(total_steps=total_steps)

    def _update_sequence_progress(self, total_steps: int | None = None):
        if self._sequence_stage_start_time is None or self._sequence_total_start_time is None:
            return

        now = time.monotonic()
        stage_elapsed = now - self._sequence_stage_start_time
        stage_remaining = max(0.0, self._sequence_stage_duration - stage_elapsed)
        total_elapsed = now - self._sequence_total_start_time
        total_remaining = max(0.0, self._sequence_total_duration - total_elapsed)
        total_steps = total_steps or len(self.sequence_steps)

        stage_value = 0
        if self._sequence_stage_duration > 0:
            stage_value = int(
                (min(stage_elapsed, self._sequence_stage_duration) / self._sequence_stage_duration)
                * 1000
            )
        total_value = 0
        if self._sequence_total_duration > 0:
            total_value = int(
                (min(total_elapsed, self._sequence_total_duration) / self._sequence_total_duration)
                * 1000
            )

        self.stage_time_bar.setValue(max(0, min(1000, stage_value)))
        self.total_time_bar.setValue(max(0, min(1000, total_value)))
        self.stage_time_bar.setFormat(
            f"Этап {self._sequence_stage_number}/{total_steps}: "
            f"осталось {self._format_seconds(stage_remaining)}"
        )
        self.total_time_bar.setFormat(
            f"Эксперимент: осталось {self._format_seconds(total_remaining)}"
        )
        self._set_sequence_remaining(
            self._sequence_stage_number,
            self._format_seconds(stage_remaining),
        )

    def _finish_sequence_progress(self):
        self._sequence_progress_timer.stop()
        self.stage_time_bar.setValue(1000)
        self.total_time_bar.setValue(1000)
        self.stage_time_bar.setFormat("Этап: завершен")
        self.total_time_bar.setFormat("Эксперимент: завершен")
        for row in range(self.sequence_table.rowCount()):
            item = self.sequence_table.item(row, 5)
            if item is not None:
                item.setText("00:00")
        self._style_sequence_table(completed_until=len(self.sequence_steps))
        self._sequence_stage_start_time = None
        self._sequence_total_start_time = None

    def _stop_sequence_progress(self):
        self._sequence_progress_timer.stop()
        if self._sequence_stage_start_time is not None:
            self._update_sequence_progress()
            self.stage_time_bar.setFormat(self.stage_time_bar.format() + " (остановлено)")
            self.total_time_bar.setFormat(self.total_time_bar.format() + " (остановлено)")
            self._sequence_stage_start_time = None
            self._sequence_total_start_time = None

    def _get_filter_config(self) -> dict:
        return {
            "bandpass_enabled": self.bandpass_enabled.isChecked(),
            "bandpass_low_hz": self.bandpass_low.value(),
            "bandpass_high_hz": self.bandpass_high.value(),
            "bandpass_order": self.bandpass_order.value(),
            "notch_enabled": self.notch_enabled.isChecked(),
            "notch_hz": self.notch_freq.value(),
            "notch_q": self.notch_q.value(),
        }

    def _validate_lsl_recording_ready(self) -> bool:
        if self.lsl_inlet is None:
            self.status_bar.showMessage("Подключите LSL-поток перед стартом")
            self._log("Подключите LSL-поток перед стартом.", COLORS["red"])
            return False

        missing = []
        for name, err in (
            ("pandas", PANDAS_IMPORT_ERROR),
            ("scipy", SCIPY_IMPORT_ERROR),
        ):
            if err is not None:
                missing.append(f"{name}: {err}")
        if missing:
            self.status_bar.showMessage("Не установлены зависимости экспорта/фильтров")
            self._log("Не установлены зависимости: " + " | ".join(missing), COLORS["red"])
            return False

        cfg = self._get_filter_config()
        fs = float(self.lsl_stream_meta.get("nominal_srate_hz", 0.0) or 0.0)
        if fs <= 0:
            self._log("LSL-поток должен иметь положительную частоту дискретизации.", COLORS["red"])
            self.status_bar.showMessage("Некорректная частота дискретизации LSL")
            return False

        nyquist = fs / 2.0
        if cfg["bandpass_enabled"]:
            if cfg["bandpass_low_hz"] <= 0:
                self._log("Нижняя частота полосового фильтра должна быть больше 0.", COLORS["red"])
                return False
            if cfg["bandpass_low_hz"] >= cfg["bandpass_high_hz"]:
                self._log("Нижняя частота полосового фильтра должна быть меньше верхней.", COLORS["red"])
                return False
            if cfg["bandpass_high_hz"] >= nyquist:
                self._log(
                    f"Верхняя частота полосового фильтра должна быть ниже Найквиста ({nyquist:.3f} Гц).",
                    COLORS["red"],
                )
                return False

        if cfg["notch_enabled"]:
            if cfg["notch_hz"] <= 0:
                self._log("Частота режекторного фильтра должна быть больше 0.", COLORS["red"])
                return False
            if cfg["notch_hz"] >= nyquist:
                self._log(
                    f"Частота режекторного фильтра должна быть ниже Найквиста ({nyquist:.3f} Гц).",
                    COLORS["red"],
                )
                return False
            if cfg["notch_q"] <= 0:
                self._log("Q режекторного фильтра должен быть больше 0.", COLORS["red"])
                return False
        return True

    def _set_current_stim(
        self,
        step_index: int,
        dac0_f1: float,
        dac0_f2: float,
        dac1_f1: float,
        dac1_f2: float,
    ):
        event = {
            "step_index": int(step_index),
            "dac0_f1_hz": float(dac0_f1),
            "dac0_f2_hz": float(dac0_f2),
            "dac1_f1_hz": float(dac1_f1),
            "dac1_f2_hz": float(dac1_f2),
            "app_timestamp": time.time(),
            "lsl_timestamp": float(local_clock()) if local_clock else np.nan,
        }
        with self._stim_lock:
            self._current_stim = {
                "step_index": event["step_index"],
                "dac0_f1_hz": event["dac0_f1_hz"],
                "dac0_f2_hz": event["dac0_f2_hz"],
                "dac1_f1_hz": event["dac1_f1_hz"],
                "dac1_f2_hz": event["dac1_f2_hz"],
            }
            self._stim_events.append(event)

    def _get_current_stim(self) -> dict:
        with self._stim_lock:
            return dict(self._current_stim)

    def _begin_lsl_recording(
        self,
        step_index: int,
        dac0_f1: float,
        dac0_f2: float,
        dac1_f1: float,
        dac1_f2: float,
    ) -> bool:
        if self.lsl_inlet is None:
            self._log("Не удалось начать запись: LSL-поток не подключен.", COLORS["red"])
            return False

        if self.lsl_worker and self.lsl_worker.is_alive():
            self.lsl_worker.stop()
            self.lsl_worker.join(timeout=1.0)

        if self.lsl_samples:
            self._log("Предыдущая LSL-запись сброшена перед новым экспериментом.", COLORS["yellow"])

        with self.lsl_sample_lock:
            self.lsl_samples.clear()
        self.export_last_btn.setEnabled(False)
        with self._stim_lock:
            self._stim_events = []

        self._record_sequence_steps = list(self.sequence_steps)
        self._record_start_app_time = time.time()
        self._record_end_app_time = None
        self._set_current_stim(step_index, dac0_f1, dac0_f2, dac1_f1, dac1_f2)

        try:
            self.lsl_inlet.flush()
        except Exception:
            pass

        self.lsl_worker = LSLRecordingWorker(
            inlet=self.lsl_inlet,
            sample_lock=self.lsl_sample_lock,
            samples=self.lsl_samples,
            stim_getter=self._get_current_stim,
            error_signal=self._lsl_worker_error,
        )
        self._recording_active = True
        self.recording_status_label.setText("Запись: идет")
        self.lsl_worker.start()
        self._log("LSL-запись начата.", COLORS["green"])
        return True

    def _stop_lsl_recording(self, export: bool = True):
        worker = self.lsl_worker
        if worker is not None:
            worker.stop()
            worker.join(timeout=1.5)
            self.lsl_worker = None

        if self._recording_active:
            self._record_end_app_time = time.time()
        self._recording_active = False

        with self.lsl_sample_lock:
            count = len(self.lsl_samples)
        self.recording_status_label.setText(f"Запись: остановлена | отсчетов: {count}")
        self.export_last_btn.setEnabled(bool(count))
        if count:
            self._log(f"LSL-запись остановлена: отсчетов {count}.", COLORS["green"])
        else:
            self._log("LSL-запись остановлена: отсчеты не получены.", COLORS["yellow"])

        if export and count:
            self._export_lsl_recording()

    @Slot(str)
    def _on_lsl_worker_error(self, msg: str):
        self._log(f"Ошибка LSL-записи: {msg}", COLORS["red"])
        self.status_bar.showMessage(f"Ошибка LSL-записи: {msg}")
        if self._running:
            self._stop(reason="ошибка LSL-записи")

    def _export_lsl_recording(self):
        with self.lsl_sample_lock:
            records = list(self.lsl_samples)
        if not records:
            self._log("Нет LSL-данных для экспорта.", COLORS["yellow"])
            return

        default_name = f"experiment_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        dialog_title = (
            "Сохранить базовое имя экспортов по этапам"
            if self._record_sequence_steps
            else "Сохранить экспорт эксперимента"
        )
        path, _ = QFileDialog.getSaveFileName(
            self,
            dialog_title,
            str(Path.cwd() / default_name),
            "Текстовый файл (*.txt)",
        )
        if not path:
            self._log("Экспорт отменен. Запись остается в памяти до следующего старта.", COLORS["yellow"])
            self.status_bar.showMessage("Экспорт отменен")
            return
        if not path.lower().endswith(".txt"):
            path += ".txt"

        try:
            if self._record_sequence_steps:
                saved_paths = self._export_sequence_step_logs(path, records)
            else:
                data_df, metadata_df = self._build_export_frames(records)
                self._write_txt_export(path, data_df, metadata_df)
                saved_paths = [path]
        except Exception as e:
            self._log(f"Ошибка экспорта: {e}", COLORS["red"])
            self.status_bar.showMessage(f"Ошибка экспорта: {e}")
            return

        if len(saved_paths) == 1:
            self._log(f"Экспорт сохранен: {saved_paths[0]}", COLORS["green"])
            self.status_bar.showMessage(f"Экспорт сохранен: {saved_paths[0]}")
        else:
            self._log(f"Экспорт сохранен по этапам: {len(saved_paths)} файлов.", COLORS["green"])
            for saved_path in saved_paths:
                self._log(f"  {saved_path}", COLORS["text_dim"])
            self.status_bar.showMessage(f"Экспорт сохранен: {len(saved_paths)} файлов")

    def _export_sequence_step_logs(self, base_path: str, records: list[dict]) -> list[str]:
        saved_paths = []
        total_steps = len(self._record_sequence_steps)
        records_by_step: dict[int, list[dict]] = {idx: [] for idx in range(1, total_steps + 1)}
        for rec in records:
            try:
                step_idx = int(rec.get("stim_step_index", -1))
            except (TypeError, ValueError):
                step_idx = -1
            if step_idx in records_by_step:
                records_by_step[step_idx].append(rec)

        for step_no, step in enumerate(self._record_sequence_steps, 1):
            step_records = records_by_step.get(step_no, [])
            metadata_extra = {
                "export_split_by_step": True,
                "export_step_index": step_no,
                "export_step_total": total_steps,
                "export_step_dac0_f1_hz": step.dac0_f1,
                "export_step_dac0_f2_hz": step.dac0_f2,
                "export_step_dac1_f1_hz": step.dac1_f1,
                "export_step_dac1_f2_hz": step.dac1_f2,
                "export_step_duration_s": step.seconds,
                "export_step_records": len(step_records),
            }
            data_df, metadata_df = self._build_export_frames(step_records, metadata_extra)
            step_path = self._step_export_path(base_path, step_no, total_steps)
            self._write_txt_export(step_path, data_df, metadata_df)
            saved_paths.append(step_path)
        return saved_paths

    def _step_export_path(self, base_path: str, step_no: int, total_steps: int) -> str:
        path = Path(base_path)
        suffix = path.suffix or ".txt"
        digits = max(2, len(str(total_steps)))
        return str(path.with_name(f"{path.stem}_step_{step_no:0{digits}d}{suffix}"))

    def _build_export_frames(self, records: list[dict], metadata_extra: dict | None = None):
        cfg = self._get_filter_config()
        stream_meta = dict(self.lsl_stream_meta)
        nominal_fs = float(stream_meta.get("nominal_srate_hz", 0.0) or 0.0)
        max_sample_len = max((len(r["sample"]) for r in records), default=0)
        channel_count = max(int(stream_meta.get("channel_count", 0) or 0), max_sample_len)
        channel_names = list(self.lsl_channel_names)
        while len(channel_names) < channel_count:
            channel_names.append(f"ch_{len(channel_names) + 1}")
        channel_names = channel_names[:channel_count]
        safe_names = unique_column_names(
            [safe_column_name(name, f"ch_{i + 1}") for i, name in enumerate(channel_names)]
        )

        raw = np.full((len(records), channel_count), np.nan, dtype=np.float64)
        for row_idx, rec in enumerate(records):
            sample = rec["sample"]
            n = min(len(sample), channel_count)
            if n:
                raw[row_idx, :n] = sample[:n]

        filtered, filter_notes = self._apply_recording_filters(raw, nominal_fs, cfg)
        effective_fs = self._compute_effective_srate(records, nominal_fs)
        start_app = self._record_start_app_time
        if start_app is None:
            start_app = records[0]["app_timestamp"] if records else np.nan
        row_count = len(records)

        def repeat(value):
            return [value] * row_count

        data = {
            "record_elapsed_s": [r["app_timestamp"] - start_app for r in records],
            "lsl_timestamp": [r["lsl_timestamp"] for r in records],
            "app_timestamp": [r["app_timestamp"] for r in records],
            "stim_step_index": [r["stim_step_index"] for r in records],
            "stim_dac0_f1_hz": [r["stim_dac0_f1_hz"] for r in records],
            "stim_dac0_f2_hz": [r["stim_dac0_f2_hz"] for r in records],
            "stim_dac1_f1_hz": [r["stim_dac1_f1_hz"] for r in records],
            "stim_dac1_f2_hz": [r["stim_dac1_f2_hz"] for r in records],
            "lsl_stream_name": repeat(stream_meta.get("name", "")),
            "lsl_stream_type": repeat(stream_meta.get("type", "")),
            "lsl_source_id": repeat(stream_meta.get("source_id", "")),
            "lsl_nominal_srate_hz": repeat(nominal_fs),
            "lsl_effective_srate_hz": repeat(effective_fs),
            "bandpass_enabled": repeat(cfg["bandpass_enabled"]),
            "bandpass_low_hz": repeat(cfg["bandpass_low_hz"]),
            "bandpass_high_hz": repeat(cfg["bandpass_high_hz"]),
            "bandpass_order": repeat(cfg["bandpass_order"]),
            "notch_enabled": repeat(cfg["notch_enabled"]),
            "notch_hz": repeat(cfg["notch_hz"]),
            "notch_q": repeat(cfg["notch_q"]),
        }
        for idx, name in enumerate(safe_names):
            data[f"raw_{name}"] = raw[:, idx]
            data[f"filtered_{name}"] = filtered[:, idx]
        data_df = pd.DataFrame(data)

        p = self._get_params()
        metadata = {
            "export_created_at": datetime.now().isoformat(timespec="seconds"),
            "record_start_app_timestamp": self._record_start_app_time,
            "record_end_app_timestamp": self._record_end_app_time,
            "record_samples": len(records),
            "record_duration_s": (
                ((self._record_end_app_time or records[-1]["app_timestamp"]) - start_app)
                if records and np.isfinite(start_app)
                else 0.0
            ),
            "stim_sequence_file": self.sequence_file_path or "",
            "record_sequence_steps_json": self._json_value(
                self._sequence_steps_to_metadata(self._record_sequence_steps)
            ),
            "stim_events_json": self._json_value(self._stim_events),
            "lsl_stream_meta_json": self._json_value(stream_meta),
            "lsl_channel_names_json": self._json_value(channel_names),
            "filter_config_json": self._json_value(cfg),
            "filter_notes_json": self._json_value(filter_notes),
            "arduino_params_json": self._json_value(p),
            "serial_port": self.port_combo.currentText(),
            "serial_baud": 115200,
        }
        if metadata_extra:
            metadata.update(metadata_extra)
        metadata_df = pd.DataFrame(
            [{"key": key, "value": value} for key, value in metadata.items()]
        )
        return data_df, metadata_df

    def _sequence_steps_to_metadata(self, steps: list[SequenceStep]) -> list[dict]:
        return [
            {
                "step_index": idx,
                "dac0_f1_hz": step.dac0_f1,
                "dac0_f2_hz": step.dac0_f2,
                "dac1_f1_hz": step.dac1_f1,
                "dac1_f2_hz": step.dac1_f2,
                "duration_s": step.seconds,
            }
            for idx, step in enumerate(steps, 1)
        ]

    def _json_value(self, value) -> str:
        return json.dumps(value, ensure_ascii=False, default=str)

    def _apply_recording_filters(self, raw: np.ndarray, fs: float, cfg: dict):
        notes = []
        filtered = np.nan_to_num(raw.astype(np.float64, copy=True), nan=0.0, posinf=0.0, neginf=0.0)
        if raw.shape[0] == 0:
            notes.append("no samples for this export segment; filtered columns are empty")
            return filtered, notes
        if np.isnan(raw).any():
            notes.append("missing raw samples were filled with 0 for filtering")

        if cfg["bandpass_enabled"]:
            sos = butter(
                int(cfg["bandpass_order"]),
                [cfg["bandpass_low_hz"], cfg["bandpass_high_hz"]],
                btype="bandpass",
                fs=fs,
                output="sos",
            )
            try:
                filtered = sosfiltfilt(sos, filtered, axis=0)
                notes.append("bandpass applied with sosfiltfilt")
            except ValueError as e:
                filtered = sosfilt(sos, filtered, axis=0)
                notes.append(f"bandpass fallback to causal sosfilt: {e}")

        if cfg["notch_enabled"]:
            b, a = iirnotch(cfg["notch_hz"], cfg["notch_q"], fs=fs)
            try:
                filtered = filtfilt(b, a, filtered, axis=0)
                notes.append("notch applied with filtfilt")
            except ValueError as e:
                filtered = lfilter(b, a, filtered, axis=0)
                notes.append(f"notch fallback to causal lfilter: {e}")

        if not cfg["bandpass_enabled"] and not cfg["notch_enabled"]:
            notes.append("filters disabled; filtered columns equal raw columns with missing values filled")
        return filtered, notes

    def _compute_effective_srate(self, records: list[dict], nominal_fs: float) -> float:
        stamps = [
            r["lsl_timestamp"]
            for r in records
            if isinstance(r.get("lsl_timestamp"), float) and np.isfinite(r["lsl_timestamp"])
        ]
        if len(stamps) >= 2 and stamps[-1] > stamps[0]:
            return (len(stamps) - 1) / (stamps[-1] - stamps[0])
        return nominal_fs

    def _write_txt_export(self, path: str, data_df, metadata_df):
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write("# METADATA\n")
            metadata_df.to_csv(f, sep="\t", index=False, lineterminator="\n")
            f.write("\n# DATA\n")
            data_df.to_csv(f, sep="\t", index=False, lineterminator="\n")

    def _build_channel_signal(self, p: dict, f1: float, f2: float, t: np.ndarray):
        denom = np.sin(np.pi * (f2 - f1) * t + np.pi / 2)
        with np.errstate(divide="ignore", invalid="ignore"):
            env = np.where(
                np.abs(denom) < 1e-9,
                p["gain_max"],
                np.clip(np.abs(1.0 / denom), 0, p["gain_max"]),
            )
        raw = (
            p["dc"]
            + env
            * (
                p["a1"] * np.sin(2 * np.pi * f1 * t + np.radians(p["p1"]))
                + p["a2"] * np.sin(2 * np.pi * f2 * t + np.radians(p["p2"]))
            )
        )
        return raw, np.clip(raw, 0, 3.3)

    def _check_clipping(self, raw0: np.ndarray, raw1: np.ndarray) -> str:
        warnings = []
        for label, raw in (("DAC0", raw0), ("DAC1", raw1)):
            parts = []
            hi = float(np.max(raw))
            lo = float(np.min(raw))
            if hi > 3.3:
                parts.append(f"пик до +{hi:.3f} В")
            if lo < 0.0:
                parts.append(f"минимум до {lo:.3f} В")
            if parts:
                warnings.append(f"{label}: " + ", ".join(parts))
        return "  |  ".join(warnings)

    def _refresh_signal_stats(self):
        p = self._get_params()
        t = np.linspace(0, 0.1, 50000)
        raw0, sig0 = self._build_channel_signal(p, p["dac0_f1"], p["dac0_f2"], t)
        raw1, sig1 = self._build_channel_signal(p, p["dac1_f1"], p["dac1_f2"], t)
        self.clip_label.setText(self._check_clipping(raw0, raw1))

        self.stat_dac0_peak.setText(f"{np.max(sig0):.3f} В")
        self.stat_dac0_trough.setText(f"{np.min(sig0):.3f} В")
        self.stat_dac0_range.setText(f"{np.max(sig0) - np.min(sig0):.3f} В")
        self.stat_dac1_peak.setText(f"{np.max(sig1):.3f} В")
        self.stat_dac1_trough.setText(f"{np.min(sig1):.3f} В")
        self.stat_dac1_range.setText(f"{np.max(sig1) - np.min(sig1):.3f} В")
        self.stat_nyquist.setText(f"{p['sr']/2:.0f} Гц")

    # ── Serial telemetry drain (20 Hz) ────────────────────────────────────────
    def _drain_serial(self):
        for raw in self.serial.drain():
            self._process_incoming(raw)

    def _process_incoming(self, raw: str):
        """Route an unsolicited line to the console and debug stat labels."""
        try:
            d = json.loads(raw)
        except json.JSONDecodeError:
            self._log(raw, COLORS["text_dim"])
            return

        if d.get("dbg"):
            self._total_clip_hi[0] += int(d.get("clipHi0", 0))
            self._total_clip_hi[1] += int(d.get("clipHi1", 0))
            self._total_clip_lo[0] += int(d.get("clipLo0", 0))
            self._total_clip_lo[1] += int(d.get("clipLo1", 0))
            self.dstat_isr.setText(f"{d.get('isrUs', '?')} мкс")
            self.dstat_cliphi0.setText(str(self._total_clip_hi[0]))
            self.dstat_cliplo0.setText(str(self._total_clip_lo[0]))
            self.dstat_cliphi1.setText(str(self._total_clip_hi[1]))
            self.dstat_cliplo1.setText(str(self._total_clip_lo[1]))
            self.dstat_samps.setText(str(d.get("samps", "?")))
            self.dstat_env0.setText(f"{float(d.get('env0', 0)):.3f}")
            self.dstat_env1.setText(f"{float(d.get('env1', 0)):.3f}")

            for clip_hi, clip_lo, labels in (
                (
                    int(d.get("clipHi0", 0)),
                    int(d.get("clipLo0", 0)),
                    (self.dstat_cliphi0, self.dstat_cliplo0),
                ),
                (
                    int(d.get("clipHi1", 0)),
                    int(d.get("clipLo1", 0)),
                    (self.dstat_cliphi1, self.dstat_cliplo1),
                ),
            ):
                clip_color = COLORS["red"] if (clip_hi or clip_lo) else COLORS["text_dim"]
                for lbl in labels:
                    lbl.setStyleSheet(
                        f"color:{clip_color};font-family:'JetBrains Mono';font-size:11px;"
                    )

            self._log(
                f"[DBG] DAC0 V={d['v0']:.3f}В  огиб={d['env0']:.3f}  "
                f"пик={d['peak0']:.3f}  мин={d['trough0']:.3f}  "
                f"клип↑={d['clipHi0']}  клип↓={d['clipLo0']}  "
                f"DAC1 V={d['v1']:.3f}В  огиб={d['env1']:.3f}  "
                f"пик={d['peak1']:.3f}  мин={d['trough1']:.3f}  "
                f"клип↑={d['clipHi1']}  клип↓={d['clipLo1']}  "
                f"ISR={d['isrUs']}мкс  n={d['samps']}",
                COLORS["text_dim"],
            )

        elif d.get("ir"):
            raw = d.get("raw", "?")
            volts = float(d.get("v", 0.0))
            refl = bool(d.get("ref"))
            state = "ЕСТЬ отражение" if refl else "нет отражения"
            self.ir_raw.setText(str(raw))
            self.ir_volts.setText(f"{volts:.3f}")
            self.ir_span.setText(f"{d.get('min', '?')} / {d.get('max', '?')}")
            self.ir_th.setText(str(d.get("th", "?")))
            self.ir_state.setText(state)
            self.ir_state.setStyleSheet(
                f"color:{COLORS['green'] if refl else COLORS['text_dim']};"
                f"font-family:'JetBrains Mono';font-size:11px;font-weight:600;"
            )
            self._log(
                f"[IR] raw={raw}  U={volts:.3f}В  "
                f"мин/макс={d.get('min', '?')}/{d.get('max', '?')}  "
                f"{state}  (порог {d.get('th', '?')})",
                COLORS["accent2"] if refl else COLORS["text_dim"],
            )

        elif d.get("status"):
            self._log(f"[STATUS] {json.dumps(d)}", COLORS["accent"])
        else:
            self._log(f"[JSON] {raw}", COLORS["text_dim"])

    # ── Console helpers ───────────────────────────────────────────────────────
    def _log(self, text: str, color: str = COLORS["text"]):
        ts = time.strftime("%H:%M:%S")
        self._console_append.emit(f"[{ts}] {text}", color)

    def _log_rx(self, text: str):
        # Fired by SerialWorker.message_received (responses to commands we sent)
        self._log(text, COLORS["green"])

    @Slot(str, str)
    def _append_console(self, text: str, color: str):
        cursor = self.console.textCursor()

        # Sliding window: trim oldest lines when over 70
        doc = self.console.document()
        if doc.blockCount() >= 70:
            trim_cursor = QTextCursor(doc.begin())
            trim_cursor.select(QTextCursor.BlockUnderCursor)
            trim_cursor.movePosition(QTextCursor.EndOfBlock, QTextCursor.KeepAnchor)
            trim_cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor)
            trim_cursor.removeSelectedText()

        cursor.movePosition(QTextCursor.End)
        fmt = cursor.charFormat()
        fmt.setForeground(QColor(color))
        cursor.setCharFormat(fmt)
        cursor.insertText(text + "\n")
        if self.autoscroll_cb.isChecked():
            self.console.setTextCursor(cursor)
            self.console.ensureCursorVisible()

    def _clear_console(self):
        self.console.clear()
        self._total_clip_hi = [0, 0]
        self._total_clip_lo = [0, 0]
        for attr in (
            "dstat_isr",
            "dstat_cliphi0",
            "dstat_cliplo0",
            "dstat_env0",
            "dstat_cliphi1",
            "dstat_cliplo1",
            "dstat_env1",
            "dstat_samps",
            "ir_state",
            "ir_raw",
            "ir_volts",
            "ir_span",
            "ir_th",
        ):
            getattr(self, attr).setText("—")
        for attr in (
            "dstat_cliphi0",
            "dstat_cliplo0",
            "dstat_cliphi1",
            "dstat_cliplo1",
            "ir_state",
        ):
            getattr(self, attr).setStyleSheet(
                f"color:{COLORS['text_dim']};font-family:'JetBrains Mono';font-size:11px;"
            )

    def _send_manual_cmd(self):
        cmd = self.cmd_input.text().strip()
        if not cmd:
            return
        self._log(f"→ {cmd}", COLORS["accent2"])
        resp = self._send(cmd)
        if resp is None:
            self._log("(нет ответа / не подключено)", COLORS["red"])
        self.cmd_input.clear()

    def _toggle_dbg(self, enabled: bool):
        cmd = "DBG ON" if enabled else "DBG OFF"
        self._log(f"→ {cmd}", COLORS["accent2"])
        self._send(cmd)

    def _toggle_ir(self, enabled: bool):
        cmd = "IR ON" if enabled else "IR OFF"
        self._log(f"→ {cmd}", COLORS["accent2"])
        self._send(cmd)

    # ── Serial actions ────────────────────────────────────────────────────────
    def _toggle_connect(self, checked: bool):
        if not checked:
            if not self.serial.is_open:
                self._set_serial_checkbox(False)
                return
            self._stop()
            self.serial.close()
            self._log("Отключено.", COLORS["text_dim"])
        else:
            port = self.port_combo.currentText()
            if port and "no ports" not in port and "порты не найдены" not in port:
                self.status_bar.showMessage(f"Подключение к {port} …")
                self._log(f"Подключение к {port} @ 115200 …", COLORS["accent"])
                ok = self.serial.open(port)
                if ok:
                    self.status_bar.showMessage(f"Подключено  ·  {port}  @  115200 бод")
                    self._log(f"Подключено к {port}.", COLORS["green"])
                else:
                    self._set_serial_checkbox(False)
            else:
                self._log("Serial-порт не выбран.", COLORS["yellow"])
                self._set_serial_checkbox(False)

    def _param_changed(self):
        if self._running and self.sequence_steps:
            return
        if self.serial.is_open:
            self._auto_apply_timer.start(350)

    def _auto_apply_params(self):
        if not self.serial.is_open:
            return
        self._apply_params(
            include_frequencies=not bool(self.sequence_steps),
            header="Автоприменение параметров…",
        )

    def _set_serial_checkbox(self, checked: bool):
        self._set_serial_indicator("connected" if checked else "disconnected")

    def _send(self, cmd: str) -> str | None:
        resp = self.serial.send(cmd)
        if resp:
            self.status_bar.showMessage(f"← {resp}")
        return resp

    def _apply_all(self):
        self._apply_params(include_frequencies=not bool(self.sequence_steps))
        if self.sequence_steps:
            self._log(
                "Поля частот DAC0/DAC1 игнорируются, потому что загружен файл последовательности.",
                COLORS["text_dim"],
            )

    def _frequency_commands(
        self,
        dac0_f1: float,
        dac0_f2: float,
        dac1_f1: float,
        dac1_f2: float,
    ) -> list[str]:
        return [
            f"SET D0F1 {dac0_f1:.4f}",
            f"SET D0F2 {dac0_f2:.4f}",
            f"SET D1F1 {dac1_f1:.4f}",
            f"SET D1F2 {dac1_f2:.4f}",
        ]

    def _apply_params(
        self,
        include_frequencies: bool = True,
        dac0_f1: float | None = None,
        dac0_f2: float | None = None,
        dac1_f1: float | None = None,
        dac1_f2: float | None = None,
        header: str = "Применение параметров…",
    ) -> bool:
        if not self.serial.is_open:
            return False
        p = self._get_params()
        cmds = [
            f"SET A1 {p['a1']:.4f}",
            f"SET P1 {p['p1']:.2f}",
            f"SET A2 {p['a2']:.4f}",
            f"SET P2 {p['p2']:.2f}",
            f"SET DC {p['dc']:.4f}",
            f"SET GM {p['gain_max']:.2f}",
            f"SET SR {int(p['sr'])}",
        ]
        if include_frequencies:
            cmds.extend(
                self._frequency_commands(
                    p["dac0_f1"] if dac0_f1 is None else dac0_f1,
                    p["dac0_f2"] if dac0_f2 is None else dac0_f2,
                    p["dac1_f1"] if dac1_f1 is None else dac1_f1,
                    p["dac1_f2"] if dac1_f2 is None else dac1_f2,
                )
            )

        self._log(header, COLORS["accent"])
        for cmd in cmds:
            self._log(f"  → {cmd}", COLORS["accent2"])
            self._send(cmd)
        self.status_bar.showMessage("Параметры применены.")
        self._log("Параметры применены.", COLORS["green"])
        return True

    def _apply_sequence_step_params(
        self,
        step: SequenceStep,
        include_static: bool,
        header: str | None = None,
    ) -> bool:
        if not self.serial.is_open:
            return False

        p = self._get_params()
        if include_static:
            cmds = [
                f"SET A1 {p['a1']:.4f}",
                f"SET P1 {p['p1']:.2f}",
                f"SET A2 {p['a2']:.4f}",
                f"SET P2 {p['p2']:.2f}",
                f"SET DC {p['dc']:.4f}",
                f"SET GM {p['gain_max']:.2f}",
                f"SET SR {int(p['sr'])}",
            ] + self._frequency_commands(
                step.dac0_f1,
                step.dac0_f2,
                step.dac1_f1,
                step.dac1_f2,
            )
        else:
            cmds = self._frequency_commands(
                step.dac0_f1,
                step.dac0_f2,
                step.dac1_f1,
                step.dac1_f2,
            )

        if header:
            self._log(header, COLORS["accent"])
        for cmd in cmds:
            self._log(f"  → {cmd}", COLORS["accent2"])
            self._send(cmd)
        return True

    def _start(self):
        if self.sequence_steps:
            self._start_sequence()
            return

        if not self._validate_lsl_recording_ready():
            return

        self._auto_apply_timer.stop()
        if not self._apply_params():
            return
        time.sleep(0.05)  # let Due finish ACKing all SET commands
        self._log("→ START", COLORS["green"])
        resp = self._send("START")
        if resp and "OK" in resp:
            p = self._get_params()
            if not self._begin_lsl_recording(
                step_index=0,
                dac0_f1=p["dac0_f1"],
                dac0_f2=p["dac0_f2"],
                dac1_f1=p["dac1_f1"],
                dac1_f2=p["dac1_f2"],
            ):
                self._send("STOP")
                return
            self._running = True
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.status_bar.showMessage("▶ Выполняется")
            self._log("▶ Выполняется", COLORS["green"])
            # Auto-enable debug telemetry if checkbox is on
            if self.dbg_checkbox.isChecked():
                self._send("DBG ON")

    def _start_sequence(self):
        if not self.serial.is_open or not self.sequence_steps:
            return
        if not self._validate_lsl_recording_ready():
            return

        self._auto_apply_timer.stop()
        self._sequence_index = 0
        self._reset_sequence_progress(self._sequence_total_seconds())
        self._running = True
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.sequence_load_btn.setEnabled(False)
        self.sequence_clear_btn.setEnabled(False)
        name = Path(self.sequence_file_path).name if self.sequence_file_path else "sequence"
        self.status_bar.showMessage(f"▶ Выполняется последовательность: {name}")
        self._log(
            f"Старт последовательности: {name} ({len(self.sequence_steps)} шагов)",
            COLORS["green"],
        )
        self._run_next_sequence_step()

    def _run_next_sequence_step(self):
        if not self._running:
            return
        if not self.serial.is_open:
            self._abort_sequence("serial-порт отключен")
            return

        if self._sequence_index >= len(self.sequence_steps):
            self._finish_sequence()
            return

        step = self.sequence_steps[self._sequence_index]
        step_no = self._sequence_index + 1
        total = len(self.sequence_steps)
        self._log(
            f"[SEQ {step_no}/{total}] "
            f"DAC0=({step.dac0_f1:.4f}, {step.dac0_f2:.4f}) Гц  "
            f"DAC1=({step.dac1_f1:.4f}, {step.dac1_f2:.4f}) Гц  "
            f"длительность={step.seconds:.3f} с",
            COLORS["accent"],
        )

        if self._sequence_index == 0:
            ok = self._apply_sequence_step_params(
                step,
                include_static=True,
                header="Применение параметров последовательности…",
            )
            if not ok:
                self._abort_sequence("serial-порт не подключен")
                return
            time.sleep(0.05)  # let Due finish ACKing all SET commands
            self._log("→ START", COLORS["green"])
            resp = self._send("START")
            if not resp or "OK" not in resp:
                self._abort_sequence("команда START не подтверждена")
                return
            if not self._begin_lsl_recording(
                step_index=step_no,
                dac0_f1=step.dac0_f1,
                dac0_f2=step.dac0_f2,
                dac1_f1=step.dac1_f1,
                dac1_f2=step.dac1_f2,
            ):
                self._abort_sequence("LSL-запись не началась")
                return
            if self.dbg_checkbox.isChecked():
                self._send("DBG ON")
        else:
            if not self._apply_sequence_step_params(step, include_static=False):
                self._abort_sequence("serial-порт не подключен")
                return
            self._set_current_stim(
                step_no,
                step.dac0_f1,
                step.dac0_f2,
                step.dac1_f1,
                step.dac1_f2,
            )

        self._begin_sequence_stage_progress(step_no, total, step.seconds)
        self._sequence_index += 1
        self._sequence_timer.start(max(1, int(step.seconds * 1000)))

    def _finish_sequence(self):
        self._log("Последовательность завершена.", COLORS["green"])
        self._stop()
        self._finish_sequence_progress()
        self.status_bar.showMessage("Последовательность завершена")

    def _abort_sequence(self, reason: str):
        self._log(f"Последовательность прервана: {reason}.", COLORS["red"])
        self._sequence_timer.stop()
        self._stop_sequence_progress()
        if self.serial.is_open:
            self._log("→ STOP", COLORS["red"])
            self._send("STOP")
        if self._recording_active:
            self._stop_lsl_recording(export=True)
        self._running = False
        self._sequence_index = 0
        self.start_btn.setEnabled(self.serial.is_open)
        self.stop_btn.setEnabled(False)
        self.sequence_load_btn.setEnabled(True)
        self.sequence_clear_btn.setEnabled(bool(self.sequence_steps))
        self.status_bar.showMessage(f"Последовательность прервана: {reason}")

    def _stop(self, checked=False, export: bool = True, reason: str = "Остановлено"):
        self._sequence_timer.stop()
        self._auto_apply_timer.stop()
        if self.sequence_steps:
            self._stop_sequence_progress()
        self._log("→ STOP", COLORS["red"])
        if self.serial.is_open:
            self._send("STOP")
        if self._recording_active or self.lsl_worker is not None:
            self._stop_lsl_recording(export=export)
        self._running = False
        self._sequence_index = 0
        self.start_btn.setEnabled(self.serial.is_open)
        self.stop_btn.setEnabled(False)
        self.sequence_load_btn.setEnabled(True)
        self.sequence_clear_btn.setEnabled(bool(self.sequence_steps))
        self.status_bar.showMessage(f"■ {reason}")
        self._log(f"■ {reason}", COLORS["red"])
        if self.dbg_checkbox.isChecked() and self.serial.is_open:
            self._send("DBG OFF")

    # ── Connected slot ────────────────────────────────────────────────────────
    @Slot(bool)
    def _on_connected(self, connected: bool):
        if connected:
            self._set_serial_checkbox(True)
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            self._auto_apply_timer.start(200)
        else:
            if self._running or self._recording_active:
                self._stop(reason="Serial отключен")
            self._sequence_timer.stop()
            self._set_serial_checkbox(False)
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self.sequence_load_btn.setEnabled(True)
            self.sequence_clear_btn.setEnabled(bool(self.sequence_steps))
            self._running = False

    @Slot(str)
    def _on_serial_error(self, msg: str):
        self.status_bar.showMessage(f"Ошибка Serial: {msg}")
        self._log(f"Ошибка Serial: {msg}", COLORS["red"])

    def closeEvent(self, event):
        if self.serial.is_open:
            self._send("DBG OFF")
            self._send("IR OFF")
        self._stop(export=False)
        if self.lsl_inlet is not None:
            try:
                self.lsl_inlet.close_stream()
            except Exception:
                pass
        self.serial.close()
        super().closeEvent(event)


# ── Entry point ───────────────────────────────────────────────────────────────
def _show_main_window(win: MainWindow, app: QApplication):
    if not sys.platform.startswith("linux"):
        win.showMaximized()
        return

    def apply_linux_maximized():
        screen = win.screen() or app.primaryScreen()
        if screen is not None:
            win.setGeometry(screen.availableGeometry())
        win.setWindowState(win.windowState() | Qt.WindowState.WindowMaximized)
        win.showMaximized()

    screen = app.primaryScreen()
    if screen is not None:
        win.setGeometry(screen.availableGeometry())

    win.show()
    apply_linux_maximized()
    QTimer.singleShot(0, apply_linux_maximized)
    QTimer.singleShot(150, apply_linux_maximized)
    QTimer.singleShot(500, apply_linux_maximized)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(COLORS["bg"]))
    palette.setColor(QPalette.WindowText, QColor(COLORS["text"]))
    palette.setColor(QPalette.Base, QColor(COLORS["surface"]))
    palette.setColor(QPalette.AlternateBase, QColor(COLORS["surface2"]))
    palette.setColor(QPalette.Text, QColor(COLORS["text"]))
    palette.setColor(QPalette.Button, QColor(COLORS["surface2"]))
    palette.setColor(QPalette.ButtonText, QColor(COLORS["text"]))
    palette.setColor(QPalette.Highlight, QColor(COLORS["accent"]))
    palette.setColor(QPalette.HighlightedText, QColor(COLORS["bg"]))
    app.setPalette(palette)
    app.setStyleSheet(STYLE)

    win = MainWindow()
    _show_main_window(win, app)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
