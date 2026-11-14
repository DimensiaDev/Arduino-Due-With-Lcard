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
import math
import re
import time
import threading
from collections import deque
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

from PySide6.QtCore import (
    QPointF,
    QRectF,
    QSettings,
    QSize,
    QTimer,
    QObject,
    Signal,
    Slot,
    Qt,
)
from PySide6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QLinearGradient,
    QPainter,
    QPainterPath,
    QPalette,
    QPen,
    QRadialGradient,
    QTextCursor,
)
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
    QMessageBox,
)

# ── Sensor inversion / ADC constants ─────────────────────────────────────────
# ADC на Arduino Due — 12-битный (0…4095).
ADC_MAX_VALUE = 4095

# Физическая схема с TCRT5000 и подтяжкой к земле даёт:
#   - зрачок (чёрный) → НИЗКИЙ raw
#   - белок/склера (белый) → ВЫСОКИЙ raw
# Прошивка же вычисляет ref по логике "raw < threshold = белок", рассчитанной
# на противоположный вариант. Чтобы вся внутренняя логика (калибровка,
# классификация, отображение, экспорт) работала в привычной нотации
# (низкий raw = белок = ОК), инвертируем сигнал на входе:
#     raw_inverted = ADC_MAX_VALUE - raw_firmware
# После инверсии: белок = низкий raw, зрачок = высокий.
# Если когда-нибудь поменяете схему — поставьте False.
INVERT_SENSOR_RAW = True

# Классификация взгляда по нормированным проекциям (при наличии калибровки).
# s = (raw − forward) / (ref_стороны − forward): 0 в центре, 1 в опорной точке.
GAZE_PROJECTION_ON = 0.5       # уход из центра: пройдено ≥50% пути к стороне
GAZE_PROJECTION_OFF = 0.25     # возврат в центр: путь сократился до ≤25%
GAZE_PROJECTION_MIN_SEP_RAW = 40.0   # минимальное смещение стороны, отсчёты
# Минимальное смещение стороны в σ датчика: ниже — сторона неразличима.
GAZE_PROJECTION_MIN_SEP_SIGMA = 2.5
MIN_CALIBRATION_SEPARATION = 80.0

# ── Устойчивость классификации (антидребезг) ──────────────────────────────────
# Скользящая медиана по последним отсчётам — гасит шум АЦП и моргания.
IR_MEDIAN_WINDOW = 5
# Новое состояние глаза должно продержаться указанное число свежих отсчётов
# подряд, прежде чем сменить текущее (при 10 Гц это ~200 мс задержки).
IR_DEBOUNCE_COUNT = 3
# Нижняя граница нормировки: не усиливаем шумы «идеально тихого» датчика.
MIN_SENSOR_SCALE = 30.0

# ── Качество фазы калибровки ──────────────────────────────────────────────────
CALIBRATION_MIN_SAMPLES = 8    # после отбрасывания первых CALIBRATION_SETTLE_SECONDS с фазы
CALIBRATION_MAX_STD = 250.0    # σ (устойчивая, MAD) фазы, выше — «дрожит»
CALIBRATION_MAX_RETRIES = 2    # сколько раз проблемная фаза повторяется
# Первые CALIBRATION_SETTLE_SECONDS с фазы не собираем: взгляд ещё
# перемещается к цели. Перевод взгляда занимает ~0,3–0,5 с, плюс время на
# реакцию пользователя — 0,5 с мало: в опорную точку попадали «хвосты» перехода.
CALIBRATION_SETTLE_SECONDS = 1.0
# Допуск, насколько опорная точка «вперёд» может отличаться от середины
# между «влево» и «вправо» (в долях расстояния left↔right).
CALIBRATION_FORWARD_TOLERANCE = 0.3
# Насколько далеко от опорной точки «вперёд» (в σ датчика) взгляд считается
# смещённым, когда по калибровке сторону различить нельзя.
GAZE_FALLBACK_SIGMA = 3.0

# Датчик считается отключённым, если от него давно нет телеметрии (с).
# Прошивка шлёт данные каждые 100 мс, поэтому 3 с — это ~30 пропущенных
# пакетов: надёжно ловит отключение и не срабатывает от кратких подвисаний UI.
IR_STALE_SECONDS = 3.0

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
    message_received = Signal(str)
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

    def send(self, cmd: str, timeout: float = 0.3) -> str | None:
        """Отправляет команду и возвращает строку-ответ прошивки.

        Ответом считаются только строки ``OK …``/``ERR …``. Телеметрия
        ``{"dbg":…}``, ``{"ir":…}`` и её «хвосты» после частичного чтения НЕ
        являются ответом — иначе команда «START» ложно считается
        неподтверждённой и эксперимент прерывается.
        """
        with self._lock:
            if not self._port or not self._port.is_open:
                return None
            try:
                self._port.reset_input_buffer()
                self._port.write((cmd.strip() + "\n").encode())
                deadline = time.time() + timeout
                while time.time() < deadline:
                    line = self._port.readline().decode(errors="replace").strip()
                    if not line:
                        continue
                    self.message_received.emit(f"← {line}")
                    if line.startswith("OK") or line.startswith("ERR"):
                        return line
                    # Телеметрия (или её остаток после частичного чтения) —
                    # показываем в журнале и продолжаем ждать ответ.
                    continue
                return None
            except Exception as e:
                self.error_occurred.emit(str(e))
                return None

    def status(self, timeout: float = 0.6) -> dict | None:
        """Спрашивает у прошивки состояние (STATUS) и возвращает JSON.

        Ответ может прийти обрезанным (порт отдаёт построчно), поэтому
        строки склеиваются до успешного разбора JSON.
        """
        with self._lock:
            if not self._port or not self._port.is_open:
                return None
            try:
                self._port.reset_input_buffer()
                self._port.write(b"STATUS\n")
                deadline = time.time() + timeout
                buf = ""
                while time.time() < deadline:
                    chunk = self._port.readline().decode(errors="replace")
                    if not chunk:
                        continue
                    self.message_received.emit(f"← {chunk.strip()}")
                    if '"status"' not in chunk:
                        continue
                    buf += chunk
                    try:
                        data = json.loads(buf.strip())
                    except Exception:
                        continue  # строка обрезана — ждём остаток
                    if isinstance(data, dict):
                        return data
                return None
            except Exception as e:
                self.error_occurred.emit(str(e))
                return None

    def drain(self) -> list[str]:
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


def gaze_position_label(pos: str) -> str:
    return {
        "forward": "вперёд",
        "left": "влево",
        "right": "вправо",
        "off": "в сторону",
    }.get(pos, pos)


def invert_adc(value: float) -> float:
    """Инвертирует значение АЦП, если включён INVERT_SENSOR_RAW."""
    if not np.isfinite(value):
        return value
    if INVERT_SENSOR_RAW:
        return float(ADC_MAX_VALUE) - float(value)
    return float(value)


def _robust_sigma(values) -> float | None:
    """Устойчивая σ по MAD (1.4826·MAD): не раздувается от одиночных выбросов."""
    vals = np.asarray([float(v) for v in values if np.isfinite(v)], dtype=float)
    if vals.size < 2:
        return None
    med = float(np.median(vals))
    mad = float(np.median(np.abs(vals - med)))
    return float(1.4826 * mad)


def _robust_median(values, trim_sigma: float = 3.0, min_abs: float = 30.0):
    """Медиана с отбрасыванием выбросов → (median, sigma, n).

    Отбрасываются отсчёты дальше ``max(trim_sigma·σ, min_abs)`` от медианы
    (например, моменты моргания или неполного перевода взгляда), затем
    медиана и σ пересчитываются по оставшимся.
    """
    vals = np.asarray([float(v) for v in values if np.isfinite(v)], dtype=float)
    if vals.size == 0:
        return None, None, 0
    med = float(np.median(vals))
    sigma = _robust_sigma(vals)
    if vals.size >= 5 and sigma is not None:
        limit = max(float(trim_sigma) * sigma, float(min_abs))
        kept = vals[np.abs(vals - med) <= limit]
        if kept.size >= 3:
            vals = kept
            med = float(np.median(vals))
            sigma = _robust_sigma(vals)
    return med, sigma, int(vals.size)


class LSLRecordingWorker(threading.Thread):
    def __init__(self, inlet, sample_lock, samples, stim_getter, ir_getter, error_signal):
        super().__init__(daemon=True)
        self.inlet = inlet
        self.sample_lock = sample_lock
        self.samples = samples
        self.stim_getter = stim_getter
        self.ir_getter = ir_getter
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
                ir = self.ir_getter()
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
                        "ir_raw": ir.get("raw", np.nan),
                        "ir_volts": ir.get("volts", np.nan),
                        "ir_reflect": ir.get("reflect", False),
                        "ir_threshold": ir.get("threshold", np.nan),
                        "ir_min": ir.get("min", np.nan),
                        "ir_max": ir.get("max", np.nan),
                        "ir_available": bool(ir.get("available", False)),
                        "ir_gaze_position": ir.get("gaze_position", "forward"),
                        "ir_raw0": ir.get("raw0", np.nan),
                        "ir_raw1": ir.get("raw1", np.nan),
                        "ir_eye0": ir.get("eye0", "forward"),
                        "ir_eye1": ir.get("eye1", "forward"),
                    }
                )

            with self.sample_lock:
                self.samples.extend(rows)


# ── Gaze eyes widget ──────────────────────────────────────────────────────────
class GazeEyesWidget(QWidget):
    """Наглядная индикация направления взгляда двумя «глазами».

    Глаза показаны «лицом к лицу», как реальные датчики расположены
    относительно пользователя: ИК-датчик 0 отображается СПРАВА, датчик 1 —
    СЛЕВА. Зрачки смещаются в ту же сторону, куда физически движутся зрачки
    при взгляде влево/вправо (то есть взгляд «влево» на экране смещает
    зрачки вправо). Датчики фиксируют ТОЛЬКО горизонтальное движение —
    вертикальных смещений зрачка в индикаторе не бывает. Состояния:
      • датчик не подключён   — серо-белый глаз, зрачок по центру;
      • взгляд по центру      — зелёный глаз, зрачок по центру;
      • взгляд смещён         — красный глаз, зрачок сдвинут влево/вправо
                                по отклонению; если направление определить
                                не удалось — зрачок по центру, глаз красный.
    Переходы (положение зрачка, цвет, свечение) плавно анимируются, а
    отклонённый глаз мягко пульсирует, чтобы привлекать внимание.
    """

    _COLOR_CENTER = "#34d399"   # зелёный «взгляд в центре»
    _COLOR_ALERT = "#f87171"    # красный «взгляд смещён»
    _COLOR_IDLE = "#c3ccd6"     # серо-белый «датчик не подключён»
    _PUPIL_RATIO = 0.32         # радиус зрачка относительно глаза
    _IRIS_RATIO = 0.645
    _SHIFT_RATIO = 0.30         # максимальный сдвиг зрачка (в пределах радужки)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(198)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setToolTip(
            "Индикация взгляда: зелёный — взгляд в центре, "
            "красный — смещён (влево/вправо), серо-белый — датчик "
            "не подключён. Датчики фиксируют только горизонтальное движение."
        )

        self._target = [self._blank_state(), self._blank_state()]
        self._current = [self._blank_state(), self._blank_state()]
        self._phase = 0.0
        self._last_tick = time.monotonic()

        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    @staticmethod
    def _blank_state() -> dict:
        return {
            "enabled": 0.0,
            "mix": 0.0,
            "dx": 0.0,
            "dy": 0.0,
            "label": "нет сигнала",
        }

    def sizeHint(self):
        return QSize(420, 198)

    # ── Внешний интерфейс ────────────────────────────────────────────────────
    def set_eyes(self, eyes: list) -> None:
        """Обновляет целевое состояние глаз.

        ``eyes`` — список из двух словарей: ``available`` (датчик подключён),
        ``deviated`` (взгляд смещён) и ``direction`` (left/right/off/center).
        """
        for i in range(2):
            state = eyes[i] if i < len(eyes) and isinstance(eyes[i], dict) else {}
            target = self._target[i]
            available = bool(state.get("available"))
            deviated = bool(state.get("deviated")) and available
            target["enabled"] = 1.0 if available else 0.0
            target["mix"] = 1.0 if deviated else 0.0

            direction = state.get("direction", "center")
            if not available:
                target["dx"] = target["dy"] = 0.0
                target["label"] = "нет сигнала"
            elif not deviated:
                target["dx"] = target["dy"] = 0.0
                target["label"] = "центр"
            elif direction == "left":
                # Взгляд влево (в сторону левой руки пользователя): на экране
                # глаза показаны «лицом к лицу», поэтому зрачки уходят ВПРАВО.
                target["dx"], target["dy"] = 1.0, 0.0
                target["label"] = "влево"
            elif direction == "right":
                # Зеркально: взгляд вправо → зрачки на экране уходят влево.
                target["dx"], target["dy"] = -1.0, 0.0
                target["label"] = "вправо"
            else:
                # Датчики фиксируют только движение влево/вправо — вертикальных
                # смещений в индикаторе не бывает. Если отклонение есть, а
                # направление определить не удалось, зрачок остаётся по центру.
                target["dx"] = target["dy"] = 0.0
                target["label"] = "смещён"

    # ── Анимация ─────────────────────────────────────────────────────────────
    def _tick(self) -> None:
        # Сглаживание привязано к реальному времени, поэтому анимация
        # остаётся плавной и доходит до цели даже при редких перерисовках.
        now = time.monotonic()
        dt = min(0.25, max(0.001, now - self._last_tick))
        self._last_tick = now
        alpha = 1.0 - math.exp(-dt / 0.075)

        changed = False
        for i in range(2):
            target, current = self._target[i], self._current[i]
            for key in ("enabled", "mix", "dx", "dy"):
                delta = target[key] - current[key]
                if abs(delta) > 0.002:
                    current[key] += delta * alpha
                    changed = True
                elif delta:
                    current[key] = target[key]
                    changed = True
            if target["label"] != current["label"]:
                current["label"] = target["label"]
                changed = True

        alerting = self._target[0]["mix"] > 0.01 or self._target[1]["mix"] > 0.01
        if alerting:
            self._phase = (self._phase + dt * 5.2) % (2.0 * math.pi)
            changed = True

        if changed:
            self.update()

    # ── Отрисовка ────────────────────────────────────────────────────────────
    def paintEvent(self, event):  # noqa: N802 (Qt API)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing, True)

        rect = QRectF(0.5, 0.5, self.width() - 1.0, self.height() - 1.0)
        self._paint_panel(painter, rect)

        header_h = 24.0
        labels_h = 36.0
        content = rect.adjusted(14.0, 10.0, -14.0, -10.0)
        self._paint_header(painter, content, header_h)

        eyes_top = content.top() + header_h
        eyes_h = max(20.0, content.height() - header_h - labels_h)
        gap = max(26.0, content.width() * 0.07)
        radius = min(
            52.0,
            eyes_h / 2.0 - 2.0,
            (content.width() - gap) / 4.0 - 2.0,
        )
        radius = max(16.0, radius)
        center_y = eyes_top + eyes_h / 2.0
        center_x = content.center().x()
        # Вид «лицом к лицу» (как датчики расположены в реальности):
        # датчик 0 — справа на экране, датчик 1 — слева.
        eye0_x = center_x + gap / 2.0 + radius
        eye1_x = center_x - gap / 2.0 - radius

        for idx, eye_x in enumerate((eye0_x, eye1_x)):
            self._paint_eye(painter, eye_x, center_y, radius, self._current[idx])
            self._paint_eye_caption(
                painter, eye_x, center_y + radius, idx, self._current[idx]
            )

    def _paint_panel(self, painter: QPainter, rect: QRectF) -> None:
        path = QPainterPath()
        path.addRoundedRect(rect, 18.0, 18.0)

        body = QLinearGradient(rect.topLeft(), rect.bottomRight())
        body.setColorAt(0.0, QColor("#18212e"))
        body.setColorAt(0.55, QColor("#121a24"))
        body.setColorAt(1.0, QColor("#0d141c"))
        painter.fillPath(path, QBrush(body))

        inner = QPainterPath()
        inner.addRoundedRect(rect.adjusted(1.0, 1.0, -1.0, -1.0), 17.0, 17.0)
        sheen = QLinearGradient(rect.topLeft(), QPointF(rect.left(), rect.top() + 30.0))
        sheen.setColorAt(0.0, QColor(255, 255, 255, 16))
        sheen.setColorAt(1.0, QColor(255, 255, 255, 0))
        painter.fillPath(inner, QBrush(sheen))

        painter.setPen(QPen(QColor(COLORS["border"]), 1.0))
        painter.drawPath(path)

    def _paint_header(self, painter: QPainter, content: QRectF, header_h: float) -> None:
        caption_font = QFont(self.font())
        caption_font.setPointSizeF(8.2)
        caption_font.setBold(True)
        caption_font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.6)
        painter.setFont(caption_font)
        painter.setPen(QColor(COLORS["text_dim"]))
        painter.drawText(
            QRectF(content.left() + 2.0, content.top(), content.width() * 0.6, header_h),
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            "НАПРАВЛЕНИЕ ВЗГЛЯДА",
        )

        chip_text, chip_color = self._overall_status()
        chip_font = QFont(self.font())
        chip_font.setPointSizeF(8.4)
        chip_font.setBold(True)
        chip_font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 0.6)
        painter.setFont(chip_font)
        metrics = painter.fontMetrics()
        text_w = float(metrics.horizontalAdvance(chip_text))
        chip_h = 20.0
        chip_w = text_w + 22.0
        chip = QRectF(content.right() - chip_w, content.top() + 2.0, chip_w, chip_h)

        chip_path = QPainterPath()
        chip_path.addRoundedRect(chip, chip_h / 2.0, chip_h / 2.0)
        fill = QColor(chip_color)
        fill.setAlpha(34)
        edge = QColor(chip_color)
        edge.setAlpha(150)
        painter.fillPath(chip_path, QBrush(fill))
        painter.setPen(QPen(edge, 1.0))
        painter.drawPath(chip_path)
        painter.setPen(QColor(chip_color))
        painter.drawText(chip, int(Qt.AlignmentFlag.AlignCenter), chip_text)

    def _overall_status(self) -> tuple:
        available = [c["enabled"] > 0.5 for c in self._target]
        deviated = [
            available[i] and self._target[i]["mix"] > 0.5 for i in range(2)
        ]
        if not any(available):
            return "НЕТ СИГНАЛА", self._COLOR_IDLE
        if any(deviated):
            if all(deviated):
                return "ВЗГЛЯД СМЕЩЁН", self._COLOR_ALERT
            # Датчик 0 — правый глаз, датчик 1 — левый.
            side = "ПРАВО"
            for i in range(2):
                if deviated[i]:
                    side = "ПРАВО" if i == 0 else "ЛЕВО"
            return f"СМЕЩЁН · {side}", self._COLOR_ALERT
        return "ВЗГЛЯД В ЦЕНТРЕ", self._COLOR_CENTER

    def _paint_eye(
        self, painter: QPainter, cx: float, cy: float, radius: float, current: dict
    ) -> None:
        enabled = current["enabled"]
        mix = current["mix"] * enabled
        color = self._lerp_color(
            self._lerp_color(QColor(self._COLOR_IDLE), QColor(self._COLOR_CENTER), enabled),
            QColor(self._COLOR_ALERT),
            mix,
        )

        pupil_dx = current["dx"] * radius * self._SHIFT_RATIO * enabled
        pupil_dy = current["dy"] * radius * self._SHIFT_RATIO * enabled
        pulse = 0.5 + 0.5 * math.sin(self._phase)

        # Мягкая тень под глазом — глубина.
        shadow = QRadialGradient(QPointF(cx, cy + radius * 0.86), radius * 1.05)
        shadow.setColorAt(0.0, QColor(0, 0, 0, 110))
        shadow.setColorAt(1.0, QColor(0, 0, 0, 0))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(shadow))
        painter.drawEllipse(
            QPointF(cx, cy + radius * 0.86), radius * 1.05, radius * 0.36
        )

        # Свечение состояния (для отклонения — пульсирующее).
        glow_alpha = int(round(22 + 34 * enabled))
        if mix > 0.01:
            glow_alpha = int(round(46 + (72 + 46 * pulse) * mix))
        glow = QRadialGradient(QPointF(cx, cy), radius * 2.0)
        glow_core = QColor(color)
        glow_core.setAlpha(glow_alpha)
        glow_mid = QColor(color)
        glow_mid.setAlpha(max(0, int(glow_alpha * 0.45)))
        glow.setColorAt(0.0, glow_core)
        glow.setColorAt(0.40, glow_mid)
        glow.setColorAt(1.0, QColor(color.red(), color.green(), color.blue(), 0))
        painter.setBrush(QBrush(glow))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(cx, cy), radius * 2.0, radius * 2.0)

        # Ободок-оправа состояния.
        rim = QColor(color)
        rim.setAlpha(215)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(rim, 2.2))
        painter.drawEllipse(QPointF(cx, cy), radius - 1.0, radius - 1.0)
        halo = QColor(color)
        halo.setAlpha(60)
        painter.setPen(QPen(halo, 1.0))
        painter.drawEllipse(QPointF(cx, cy), radius + 2.6, radius + 2.6)

        # Склера — объёмный шар.
        sclera = QRadialGradient(
            QPointF(cx - radius * 0.30, cy - radius * 0.34), radius * 1.9
        )
        sclera.setColorAt(0.0, QColor("#fbfdff"))
        sclera.setColorAt(0.55, QColor("#e6edf4"))
        sclera.setColorAt(1.0, QColor("#c4ceda") if enabled > 0.5 else QColor("#c9cfd6"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(sclera))
        painter.drawEllipse(QPointF(cx, cy), radius - 2.2, radius - 2.2)

        # Тень верхнего века.
        painter.save()
        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), radius - 2.2, radius - 2.2)
        painter.setClipPath(clip)
        lid = QLinearGradient(QPointF(cx, cy - radius), QPointF(cx, cy + radius * 0.15))
        lid.setColorAt(0.0, QColor(9, 12, 18, 120))
        lid.setColorAt(1.0, QColor(9, 12, 18, 0))
        painter.setBrush(QBrush(lid))
        painter.drawRect(QRectF(cx - radius, cy - radius, radius * 2.0, radius * 1.2))
        painter.restore()

        # Радужка и зрачок, смещённые по направлению взгляда.
        iris_cx, iris_cy = cx + pupil_dx, cy + pupil_dy
        iris_r = radius * self._IRIS_RATIO
        iris = QRadialGradient(
            QPointF(iris_cx - iris_r * 0.25, iris_cy - iris_r * 0.30), iris_r * 1.6
        )
        iris.setColorAt(0.0, color.lighter(126) if enabled > 0.5 else color.lighter(110))
        iris.setColorAt(0.68, color)
        iris.setColorAt(1.0, color.darker(158))
        painter.setBrush(QBrush(iris))
        painter.drawEllipse(QPointF(iris_cx, iris_cy), iris_r, iris_r)

        iris_ring = color.darker(165)
        iris_ring.setAlpha(190)
        painter.setPen(QPen(iris_ring, 1.1))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(iris_cx, iris_cy), iris_r, iris_r)

        pupil_r = radius * self._PUPIL_RATIO
        pupil = QRadialGradient(
            QPointF(iris_cx - pupil_r * 0.2, iris_cy - pupil_r * 0.25), pupil_r * 1.8
        )
        pupil.setColorAt(0.0, QColor("#131c26"))
        pupil.setColorAt(1.0, QColor("#05080c"))
        painter.setBrush(QBrush(pupil))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(iris_cx, iris_cy), pupil_r, pupil_r)

        # Блики — «мокрый» глянец.
        painter.setBrush(QBrush(QColor(255, 255, 255, 200)))
        painter.drawEllipse(
            QPointF(iris_cx - iris_r * 0.32, iris_cy - iris_r * 0.36), radius * 0.115, radius * 0.10
        )
        painter.setBrush(QBrush(QColor(255, 255, 255, 80)))
        painter.drawEllipse(
            QPointF(iris_cx + iris_r * 0.26, iris_cy + iris_r * 0.30), radius * 0.05, radius * 0.045
        )

    def _paint_eye_caption(
        self, painter: QPainter, cx: float, bottom: float, idx: int, current: dict
    ) -> None:
        name_font = QFont(self.font())
        name_font.setPointSizeF(7.6)
        name_font.setBold(True)
        name_font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.2)
        painter.setFont(name_font)
        painter.setPen(QColor(COLORS["text_dim"]))
        painter.drawText(
            QRectF(cx - 70.0, bottom + 7.0, 140.0, 14.0),
            int(Qt.AlignmentFlag.AlignCenter),
            # Датчик 0 — правый глаз, датчик 1 — левый.
            "ПРАВО" if idx == 0 else "ЛЕВО",
        )

        state_font = QFont(self.font())
        state_font.setPointSizeF(8.8)
        state_font.setBold(True)
        painter.setFont(state_font)
        enabled = current["enabled"]
        mix = current["mix"] * enabled
        color = self._lerp_color(
            self._lerp_color(QColor(self._COLOR_IDLE), QColor(self._COLOR_CENTER), enabled),
            QColor(self._COLOR_ALERT),
            mix,
        )
        painter.setPen(color)
        painter.drawText(
            QRectF(cx - 70.0, bottom + 21.0, 140.0, 15.0),
            int(Qt.AlignmentFlag.AlignCenter),
            current["label"],
        )

    @staticmethod
    def _lerp_color(c1: QColor, c2: QColor, t: float) -> QColor:
        t = max(0.0, min(1.0, t))
        return QColor(
            int(round(c1.red() + (c2.red() - c1.red()) * t)),
            int(round(c1.green() + (c2.green() - c1.green()) * t)),
            int(round(c1.blue() + (c2.blue() - c1.blue()) * t)),
        )


# ── Main window ───────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    _console_append = Signal(str, str)
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
        # Момент, когда этап должен закончиться (monotonic). Защита от
        # преждевременного перехода этапа, если таймер сработал раньше.
        self._sequence_stage_deadline: float | None = None

        # ── Pause-by-gaze state ─────────────────────────────────────────────
        self._paused_by_gaze = False
        # Эксперимент уже дошёл до конца: возобновление по взгляду не должно
        # перезапускать его (иначе приложение снова просит сохранить результаты).
        self._experiment_finished = False
        self._gaze_off = False
        self._current_gaze_pos = "forward"
        self._pause_step_index = -1
        self._pause_remaining_seconds = 0.0
        self._sequence_override_seconds = None
        self._resuming_after_gaze_pause = False
        # Момент начала паузы: на него сдвигается общий таймер эксперимента,
        # чтобы после долгой паузы не показывалось «Эксперимент: осталось 00:00».
        self._pause_started_at: float | None = None

        # ── IR sensors ──────────────────────────────────────────────────────
        self._ir_lock = threading.Lock()
        self._ir_sensors = [
            {"reflect": True, "raw": np.nan, "volts": np.nan,
             "threshold": np.nan, "min": np.nan, "max": np.nan,
             "available": False, "last_update": 0.0},
            {"reflect": True, "raw": np.nan, "volts": np.nan,
             "threshold": np.nan, "min": np.nan, "max": np.nan,
             "available": False, "last_update": 0.0},
        ]
        self._ir_events: list[dict] = []
        self._last_ir_reflect_state = [None, None]
        self._sensor_eye_pos = ["forward", "forward"]
        # Антидребезг: окно медианы, счётчики подтверждения состояния и
        # последний обработанный отсчёт каждого датчика.
        self._ir_raw_history = [
            deque(maxlen=IR_MEDIAN_WINDOW),
            deque(maxlen=IR_MEDIAN_WINDOW),
        ]
        self._sensor_eye_pending: list[tuple[str, int] | None] = [None, None]
        self._gaze_last_seen = [0.0, 0.0]
        self._ir_table_max_rows = 100

        # ── IR calibration (reference points + projection classifier) ────────
        self.ir_calibration: dict = {}
        self._calibration_in_progress = False
        self._calibration_phase = 0
        self._calibration_phase_collecting = False
        # Момент старта сбора фазы: первые CALIBRATION_SETTLE_SECONDS не
        # собираем — взгляд ещё перемещается к цели.
        self._calibration_phase_started_at = 0.0
        # Продолжение, которое запускается после успешной калибровки
        # (калибровка всегда выполняется перед стартом эксперимента).
        self._calibration_after = None
        # Повторы проблемной фазы калибровки (дрожание / мало отсчётов).
        self._calibration_retries = 0
        # Ожидание возврата взгляда в центр после калибровки: эксперимент
        # стартует автоматически, как только оба датчика увидят центр.
        self._waiting_gaze_center = False
        # Одноразовый обход проверки взгляда: пользователь нажал «Старт»
        # повторно, пока приложение ждало возврата взгляда в центр.
        self._start_now_once = False
        # Пауза по взгляду не срабатывает сразу после возобновления:
        # в окне медианы ещё могут быть старые отсчёты «смещённого» взгляда.
        self._gaze_pause_block_until = 0.0
        self._calibration_samples = [
            {"sensor0": [], "sensor1": []},
            {"sensor0": [], "sensor1": []},
            {"sensor0": [], "sensor1": []},
        ]
        self._calibration_instructions = [
            ("ВЛЕВО",  "Смотрите ВЛЕВО и держите взгляд неподвижно"),
            ("ВПЕРЁД", "Смотрите ВПЕРЁД, в центр экрана"),
            ("ВПРАВО", "Смотрите ВПРАВО и держите взгляд неподвижно"),
        ]

        self.lsl_streams = []
        self.lsl_inlet = None
        self.lsl_stream_meta: dict = {}
        self.lsl_channel_names: list[str] = []
        self.lsl_worker: LSLRecordingWorker | None = None
        self.lsl_samples: list[dict] = []
        self.lsl_sample_lock = threading.Lock()
        # Эксперимент полностью задаётся загруженной последовательностью частот,
        # поэтому отдельных полей амплитуды/фазы/частот в UI больше нет.
        self.amp1 = 0.5
        self.amp2 = 0.5
        self.phase1 = 0.0
        self.phase2 = 0.0
        self.dac0_f1 = 100.0
        self.dac0_f2 = 200.0
        self.dac1_f1 = 100.0
        self.dac1_f2 = 200.0
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

        self._load_calibration()
        self._update_calibration_status_label()
        self._update_gaze_indicator_text()

        self._stats_timer = QTimer(self)
        self._stats_timer.timeout.connect(self._refresh_signal_stats)
        self._stats_timer.start(250)

        self._drain_timer = QTimer(self)
        self._drain_timer.timeout.connect(self._drain_serial)
        self._drain_timer.start(50)

        self._total_clip_hi = [0, 0]
        self._total_clip_lo = [0, 0]
        QTimer.singleShot(0, self._attempt_initial_connections)

    def _on_lsl_combo_changed(self, idx: int):
        self._update_lsl_stream_preview()
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
        left.setSpacing(12)
        top_cards = QHBoxLayout()
        top_cards.setSpacing(6)
        top_cards.addWidget(self._build_session_card(), 3)
        top_cards.addWidget(self._build_transport_box(), 2)
        left.addLayout(top_cards)
        left.addWidget(self._build_global_box())
        left.addWidget(self._build_sequence_box())
        left.addWidget(self._build_gaze_eyes_card())
        # Баннер «Взгляд в норме / смещён» и строка датчиков — сразу под
        # панелью глаз, единым блоком индикации взгляда.
        left.addWidget(self.gaze_indicator)
        left.addWidget(self.gaze_detail)
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

        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setPlaceholderText("Журнал команд и ответов Arduino")

        self.gaze_indicator = QLabel("Взгляд в норме.")
        self.gaze_indicator.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.gaze_indicator.setWordWrap(True)
        self.gaze_indicator.setMinimumHeight(48)
        self._set_gaze_indicator_style("forward")

        self.gaze_detail = QLabel("Лево: —   |   Право: —   |   Позиция: —")
        self.gaze_detail.setObjectName("hint_label")
        self.gaze_detail.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.gaze_detail.setWordWrap(True)
        self.gaze_detail.setTextFormat(Qt.TextFormat.RichText)

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
        self.serial_connect_check.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, True)
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

    def _build_service_tab(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(12, 12, 12, 12)
        v.setSpacing(12)

        toolbar = QHBoxLayout()
        toolbar.setSpacing(10)
        toolbar.addStretch()

        clear_btn = QPushButton("Очистить журнал")
        clear_btn.setObjectName("clear_btn")
        clear_btn.clicked.connect(self._clear_console)
        toolbar.addWidget(clear_btn)
        v.addLayout(toolbar)

        v.addWidget(self._build_debug_stats_panel())
        v.addWidget(self._build_ir_panel())

        gaze_box, gaze_layout = self._make_subcard("Взгляд", COLORS["accent2"])
        self.ignore_gaze_check = QCheckBox("Игнорировать направление взгляда")
        self.ignore_gaze_check.setToolTip(
            "Отклонение взгляда не ставит эксперимент на паузу и не блокирует Старт.\n"
            "Индикатор глаз продолжает показывать реальное состояние датчиков."
        )
        gaze_layout.addWidget(self.ignore_gaze_check)

        self.skip_calibration_check = QCheckBox("Пропустить калибровку")
        self.skip_calibration_check.setToolTip(
            "«Старт» сразу запускает эксперимент без калибровки взгляда.\n"
            "Классификация использует сохранённую калибровку, а если её нет — "
            "пороги по умолчанию (порог АЦП прошивки и признак «виден зрачок»)."
        )
        gaze_layout.addWidget(self.skip_calibration_check)

        # Состояние флажков переживает перезапуск приложения.
        self.ignore_gaze_check.setChecked(
            bool(self.settings.value("service/ignore_gaze", False, type=bool))
        )
        self.skip_calibration_check.setChecked(
            bool(self.settings.value("service/skip_calibration", False, type=bool))
        )
        self.ignore_gaze_check.toggled.connect(self._on_ignore_gaze_toggled)
        self.skip_calibration_check.toggled.connect(self._on_skip_calibration_toggled)
        v.addWidget(gaze_box)

        console_box, console_layout = self._make_subcard("Журнал", COLORS["accent"])
        console_layout.addWidget(self.console, 1)
        v.addWidget(console_box, 1)
        return w

    def _ignore_gaze(self) -> bool:
        return hasattr(self, "ignore_gaze_check") and self.ignore_gaze_check.isChecked()

    def _skip_calibration(self) -> bool:
        return (
            hasattr(self, "skip_calibration_check")
            and self.skip_calibration_check.isChecked()
        )

    def _on_ignore_gaze_toggled(self, checked: bool):
        self.settings.setValue("service/ignore_gaze", bool(checked))
        # Если эксперимент уже стоит на паузе по взгляду — снимаем её сразу:
        # раз взгляд игнорируется, ждать возврата в центр бессмысленно.
        if checked and self._paused_by_gaze:
            self._log("Взгляд игнорируется — пауза по взгляду снимается.", COLORS["yellow"])
            self._resume_experiment()

    def _on_skip_calibration_toggled(self, checked: bool):
        self.settings.setValue("service/skip_calibration", bool(checked))
        if checked:
            self._log(
                "Калибровка будет пропущена при «Старте»: используются "
                "сохранённые пороги или значения по умолчанию.",
                COLORS["yellow"],
            )

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

    def _build_gaze_eyes_card(self) -> QWidget:
        self.gaze_eyes = GazeEyesWidget()
        # До подключения датчиков глаза серо-белые, зрачки по центру.
        self.gaze_eyes.set_eyes(
            [
                {"available": False, "deviated": False, "direction": "center"},
                {"available": False, "deviated": False, "direction": "center"},
            ]
        )
        return self.gaze_eyes

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

        title = QLabel("ИК-датчики TCRT5000 (положение зрачков)")
        title.setObjectName("subcard_title")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setHorizontalSpacing(16)
        grid.setVerticalSpacing(8)

        # Панель по-датчиковая: пакеты от сенсоров 0 и 1 чередуются, поэтому
        # общие поля «мигали» между двумя глазами и не соответствовали
        # индикатору направления. Теперь у каждого датчика своя колонка.
        grid.addWidget(QLabel(""), 0, 0)
        for col, header_text in ((1, "Датчик 0 — ПРАВО"), (2, "Датчик 1 — ЛЕВО")):
            header = QLabel(header_text)
            header.setObjectName("metric_label")
            grid.addWidget(header, 0, col)

        self.ir_metrics: dict[int, dict[str, QLabel]] = {0: {}, 1: {}}
        metric_rows = (
            ("raw", "RAW (0…4095)"),
            ("volts", "Напряжение, В"),
            ("span", "Мин / Макс за 100 мс"),
            ("th", "Порог срабатывания"),
            ("state", "Состояние"),
            ("dir", "Направление (датчик)"),
        )
        for row, (key, caption) in enumerate(metric_rows, start=1):
            caption_label = QLabel(caption)
            caption_label.setObjectName("metric_label")
            grid.addWidget(caption_label, row, 0)
            for col, sensor_id in ((1, 0), (2, 1)):
                value = QLabel("—")
                value.setObjectName("metric_value")
                grid.addWidget(value, row, col)
                grid.setColumnStretch(col, 1)
                self.ir_metrics[sensor_id][key] = value

        layout.addLayout(grid)

        # Общая классификация — ровно то же, что показывает главный экран.
        gaze_row = QHBoxLayout()
        gaze_row.setSpacing(8)
        gaze_caption = QLabel("Направление взгляда (общее):")
        gaze_caption.setObjectName("metric_label")
        gaze_row.addWidget(gaze_caption)
        self.ir_gaze_dir = QLabel("—")
        self.ir_gaze_dir.setObjectName("status_value")
        gaze_row.addWidget(self.ir_gaze_dir)
        gaze_row.addStretch()
        layout.addLayout(gaze_row)

        if INVERT_SENSOR_RAW:
            inv_hint = QLabel(
                "Показания приведены к шкале «меньше = белок (взгляд в норме), "
                "больше = зрачок»."
            )
            inv_hint.setObjectName("hint_label")
            inv_hint.setWordWrap(True)
            layout.addWidget(inv_hint)

        calib_label = QLabel("Калибровка положения взгляда")
        calib_label.setObjectName("section_label")
        layout.addWidget(calib_label)

        self.calibration_status_label = QLabel("Калибровка: не выполнена")
        self.calibration_status_label.setObjectName("status_value")
        self.calibration_status_label.setWordWrap(True)
        layout.addWidget(self.calibration_status_label)

        calib_hint = QLabel(
            "Калибровка выполняется автоматически перед каждым стартом "
            "эксперимента: ВЛЕВО → ВПЕРЁД → ВПРАВО, по 3 секунды на фазу."
        )
        calib_hint.setObjectName("hint_label")
        calib_hint.setWordWrap(True)
        layout.addWidget(calib_hint)

        hist_label = QLabel("История показаний (последние 100 замеров)")
        hist_label.setObjectName("section_label")
        layout.addWidget(hist_label)

        self.ir_table = QTableWidget(0, 6)
        self.ir_table.setHorizontalHeaderLabels(
            ["Датчик", "Время", "RAW", "U, В", "Состояние", "Мин / Макс"]
        )
        self.ir_table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.ir_table.setSelectionMode(QAbstractItemView.SelectionMode.NoSelection)
        self.ir_table.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.ir_table.verticalHeader().setVisible(False)
        self.ir_table.setAlternatingRowColors(False)
        _hdr = self.ir_table.horizontalHeader()
        _hdr.setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        for _c in range(5):
            _hdr.setSectionResizeMode(_c, QHeaderView.ResizeMode.ResizeToContents)
        self.ir_table.setMinimumHeight(210)
        layout.addWidget(self.ir_table)
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
        self._update_lsl_stream_preview()

    def _update_lsl_stream_preview(self):
        idx = self.lsl_stream_combo.currentIndex()
        if idx < 0 or idx >= len(self.lsl_streams):
            return
        meta = self._read_lsl_stream_meta(self.lsl_streams[idx])
        names = self._read_lsl_channel_names(self.lsl_streams[idx], meta["channel_count"])
        self.lsl_status_label.setToolTip(
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
            "a1": self.amp1,
            "p1": self.phase1,
            "a2": self.amp2,
            "p2": self.phase2,
            "dac0_f1": self.dac0_f1,
            "dac0_f2": self.dac0_f2,
            "dac1_f1": self.dac1_f1,
            "dac1_f2": self.dac1_f2,
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
            f"Шагов: {len(steps)} | всего: {total_seconds:.3f} с"
        )
        self.sequence_clear_btn.setEnabled(True)
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
        self._populate_sequence_table()
        self._reset_sequence_progress()
        self._log("Файл частот сброшен.", COLORS["text_dim"])

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
        # Фильтры экспорта заданы фиксированно — в UI полей настройки больше нет.
        return {
            "bandpass_enabled": True,
            "bandpass_low_hz": 0.5,
            "bandpass_high_hz": 40.0,
            "bandpass_order": 4,
            "notch_enabled": True,
            "notch_hz": 50.0,
            "notch_q": 30.0,
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

    # ── Gaze classification (nearest-neighbor) ───────────────────────────────
    def _set_gaze_indicator_style(self, position: str):
        if position == "forward":
            bg = "#143129"
            border = COLORS["green"]
            fg = "#d7ffee"
            text = "✓ Взгляд в норме."
        else:
            bg = "#35171a"
            border = COLORS["red"]
            fg = "#ffe0e0"
            # При «игнорировать направление взгляда» паузы не будет —
            # не пугаем пользователя словами про паузу.
            tail = (
                " (игнорируется)"
                if self._ignore_gaze()
                else "! Скорректируйте взгляд. Эксперимент на паузе."
            )
            if position == "left":
                text = f"⚠ Взгляд смещён ВЛЕВО{tail}"
            elif position == "right":
                text = f"⚠ Взгляд смещён ВПРАВО{tail}"
            else:
                text = f"⚠ Взгляд смещён{tail}"
        self.gaze_indicator.setText(text)
        self.gaze_indicator.setStyleSheet(
            f"background-color:{bg};"
            f"border:2px solid {border};"
            f"border-radius:14px;"
            f"color:{fg};"
            f"font-size:15px;"
            f"font-weight:700;"
            f"padding:10px;"
        )

    def _update_gaze_indicator_text(self):
        if not hasattr(self, "gaze_detail"):
            return
        states = self._gaze_eye_states()
        self._update_gaze_indicator_with_states(states)

    def _update_gaze_indicator_with_states(self, states: list[dict]):
        if not hasattr(self, "gaze_detail"):
            return
        with self._ir_lock:
            sensors = [dict(s) for s in self._ir_sensors]

        # Цвет каждого значения: зелёный — взгляд по центру (у этого датчика),
        # красный — смещён, серый — нет сигнала. Позиция — по общей
        # классификации: зелёная только при «вперёд».
        # Порядок в строке — как глаза на экране: сначала «Лево» (датчик 1),
        # затем «Право» (датчик 0). Числа — показания АЦП датчиков.
        sep = f'   <span style="color:{COLORS["text_dim"]};">|</span>   '
        parts = []
        for i in (1, 0):
            s = sensors[i]
            state = states[i] if i < len(states) else {}
            if not state.get("available"):
                color = COLORS["text_dim"]
            elif state.get("deviated"):
                color = COLORS["red"]
            else:
                color = COLORS["green"]
            raw_val = s.get("raw", np.nan)
            raw_str = f"{int(raw_val)}" if np.isfinite(raw_val) else "—"
            eye_name = "Лево" if i == 1 else "Право"
            parts.append(
                f'<span style="color:{color};">{eye_name}: {raw_str}</span>'
            )

        pos_color = COLORS["green"] if self._current_gaze_pos == "forward" else COLORS["red"]
        pos_text = gaze_position_label(self._current_gaze_pos)
        parts.append(f'<span style="color:{pos_color};">Позиция: {pos_text}</span>')

        html = sep.join(parts)
        if self.gaze_detail.text() != html:
            self.gaze_detail.setText(html)
        if hasattr(self, "gaze_eyes"):
            self.gaze_eyes.set_eyes(states)

        # Вкладка «Сервис»: та же классификация, что на главном экране —
        # поканально (по каждому датчику) и общая позиция.
        if hasattr(self, "ir_metrics"):
            dir_text = {
                "forward": "вперёд",
                "left": "влево",
                "right": "вправо",
                "off": "в сторону",
            }
            for i in (0, 1):
                direction = self._sensor_eye_pos[i]
                color = COLORS["green"] if direction == "forward" else COLORS["red"]
                label = self.ir_metrics[i]["dir"]
                label.setText(dir_text.get(direction, direction))
                label.setStyleSheet(
                    f"color:{color};font-family:'JetBrains Mono';font-size:11px;"
                    f"font-weight:600;"
                )
        if hasattr(self, "ir_gaze_dir"):
            pos_color = (
                COLORS["green"] if self._current_gaze_pos == "forward" else COLORS["red"]
            )
            self.ir_gaze_dir.setText(gaze_position_label(self._current_gaze_pos))
            self.ir_gaze_dir.setStyleSheet(
                f"color:{pos_color};font-family:'JetBrains Mono';font-size:12px;"
                f"font-weight:600;"
            )

    def _gaze_eye_states(self) -> list[dict]:
        """Состояние каждого «глаза» (датчика) для индикатора взгляда.

        Возвращает по датчику: ``available`` (есть свежая телеметрия),
        ``deviated`` (взгляд смещён) и ``direction`` (left/right/off/center).

        При наличии калибровки направление считается по **нормированным
        проекциям** отклонения от точки «вперёд»:
        ``s = (raw − forward) / (ref_стороны − forward)`` — доля пройденного
        пути к опорной точке стороны. В отличие от nearest-neighbor это не
        «притягивается» к ближней опорной точке при асимметричных смещениях.
        Без калибровки смещение определяется по признаку «виден зрачок».

        Устойчивость классификации:
          • вход — скользящая медиана последних отсчётов (не мгновенный raw);
          • сторона учитывается, только если её смещение от «вперёд» не меньше
            ``max(2.5·σ, 40)`` отсчётов — иначе датчик её не различает;
          • гистерезис ON/OFF (0.5 / 0.25 в долях пути) удерживает состояние
            у границы;
          • новое состояние должно продержаться IR_DEBOUNCE_COUNT свежих
            отсчёта подряд, прежде чем сменить текущее.

        Глаза человека сопряжены — смотреть врозь невозможно. Поэтому
        поканальные решения согласуются: лево + право = «в сторону»
        (противоречие датчиков), иначе берётся ненулевое направление.
        """
        now = time.time()
        with self._ir_lock:
            sensors = [dict(s) for s in self._ir_sensors]
        cal = self.ir_calibration if isinstance(self.ir_calibration, dict) else {}

        def ref_value(name: str, idx: int):
            ref = cal.get(name)
            if not isinstance(ref, dict):
                return np.nan
            value = ref.get(f"s{idx}", np.nan)
            if isinstance(value, (int, float)) and np.isfinite(value):
                return float(value)
            return np.nan

        states = []
        for idx, s in enumerate(sensors):
            last_update = float(s.get("last_update") or 0.0)
            fresh = bool(s.get("available")) and (now - last_update) <= IR_STALE_SECONDS
            if not fresh:
                self._sensor_eye_pos[idx] = "forward"
                self._sensor_eye_pending[idx] = None
                self._gaze_last_seen[idx] = 0.0
                states.append({"available": False, "deviated": False, "direction": "center"})
                continue

            # Классификация идёт по сглаженному значению (медиана окна),
            # а не по мгновенному отсчёту.
            raw_val = s.get("raw_f", s.get("raw", np.nan))
            f_ref = ref_value("forward", idx)
            calibrated = np.isfinite(raw_val) and np.isfinite(f_ref)

            if calibrated:
                sigma = self._sensor_scale(idx) or MIN_SENSOR_SCALE
                target = self._direction_from_projection(
                    float(raw_val),
                    f_ref,
                    ref_value("left", idx),
                    ref_value("right", idx),
                    float(sigma),
                    self._sensor_eye_pos[idx],
                )
                if target is None:
                    # Ни одна сторона этим датчиком не различается —
                    # деградируем к отклонению от точки «вперёд».
                    target = self._fallback_eye_target(idx, float(raw_val), s)
            else:
                target = self._fallback_eye_target(idx, float(raw_val), s)

            # Временной дебаунс: новое состояние должно продержаться
            # IR_DEBOUNCE_COUNT свежих отсчёта подряд, прежде чем сменить
            # текущее. Счётчик растёт только на новых отсчётах.
            current = self._sensor_eye_pos[idx]
            if target != current:
                pending = self._sensor_eye_pending[idx]
                has_new_data = last_update != self._gaze_last_seen[idx]
                if has_new_data:
                    count = pending[1] + 1 if pending and pending[0] == target else 1
                    self._sensor_eye_pending[idx] = (target, count)
                    if count >= IR_DEBOUNCE_COUNT:
                        self._sensor_eye_pos[idx] = target
                        self._sensor_eye_pending[idx] = None
            else:
                self._sensor_eye_pending[idx] = None
            self._gaze_last_seen[idx] = last_update

            states.append({"available": True, "deviated": False, "direction": "center"})

        # Совместное решение по обоим глазам (взгляд сопряжён): согласование
        # поканальных решений вместо «самый уверенный датчик навязывает
        # направление обоим» — один шумный датчик больше не перебивает второй.
        pos_list = [
            self._sensor_eye_pos[idx]
            for idx, st in enumerate(states)
            if st["available"]
        ]
        has_left = "left" in pos_list
        has_right = "right" in pos_list
        if has_left and has_right:
            joint = "off"          # датчики противоречат — направление неизвестно
        elif has_left:
            joint = "left"
        elif has_right:
            joint = "right"
        elif "off" in pos_list:
            joint = "off"
        else:
            joint = "forward"

        for st in states:
            if st["available"]:
                st["deviated"] = joint != "forward"
                st["direction"] = joint if joint != "forward" else "center"

        return states

    @staticmethod
    def _direction_from_projection(
        raw_val: float,
        f_ref: float,
        left_ref: float,
        right_ref: float,
        sigma: float,
        current: str,
    ) -> str | None:
        """Направление взгляда по одному датчику через нормированные проекции.

        ``s = (raw − forward) / (ref_стороны − forward)`` — доля пройденного
        пути к опорной точке стороны (≈0 в центре, ≈1 в её точке). В отличие
        от nearest-neighbor это не «притягивается» к ближней опорной точке при
        асимметричных смещениях (например, «вправо» может смещать датчик на
        500 отсчётов, а «влево» — на 2000).

        Сторона различима, только если её смещение от «вперёд» не меньше
        ``max(2.5·σ, 40)`` отсчётов. Если не различима ни одна — возвращается
        ``None`` (вызывающий использует признак «виден зрачок»).

        Гистерезис: уход из центра при s ≥ 0.5, возврат — при s ≤ 0.25.
        """
        min_sep = max(GAZE_PROJECTION_MIN_SEP_SIGMA * float(sigma),
                      GAZE_PROJECTION_MIN_SEP_RAW)
        scores: dict[str, float] = {}
        if np.isfinite(left_ref) and abs(float(left_ref) - float(f_ref)) >= min_sep:
            scores["left"] = (float(raw_val) - float(f_ref)) / (
                float(left_ref) - float(f_ref)
            )
        if np.isfinite(right_ref) and abs(float(right_ref) - float(f_ref)) >= min_sep:
            scores["right"] = (float(raw_val) - float(f_ref)) / (
                float(right_ref) - float(f_ref)
            )
        if not scores:
            return None

        best_dir = max(scores, key=scores.get)
        if scores[best_dir] >= GAZE_PROJECTION_ON:
            return best_dir
        # Гистерезис: пока отклонение в текущую сторону не спало ниже OFF,
        # состояние не меняется (не «дёргаемся» у границы).
        if current in scores and scores[current] >= GAZE_PROJECTION_OFF:
            return current
        return "forward"

    def _fallback_eye_target(self, idx: int, raw_val: float, s: dict) -> str:
        """Оценка положения глаза, когда по калибровке сторону не различить.

        Считаем отклонение от опорной точки «вперёд» в σ датчика: ушли
        дальше GAZE_FALLBACK_SIGMA·σ — взгляд смещён (направление при этом
        неизвестно, поэтому «off»), иначе «вперёд».

        Признак «виден зрачок» от прошивки используется только когда
        калибровки нет вовсе: этот признак зависит от порога, который сама
        прошивка получила из калибровки, и от неизвестной нам конвенции
        «белок = низкий raw» — опереться на него нельзя.
        """
        cal = self.ir_calibration if isinstance(self.ir_calibration, dict) else {}
        f_ref = np.nan
        ref = cal.get("forward")
        if isinstance(ref, dict):
            value = ref.get(f"s{idx}", np.nan)
            if isinstance(value, (int, float)) and np.isfinite(value):
                f_ref = float(value)

        if np.isfinite(raw_val) and np.isfinite(f_ref):
            sigma = max(self._sensor_scale(idx) or MIN_SENSOR_SCALE, MIN_SENSOR_SCALE)
            if abs(raw_val - f_ref) > GAZE_FALLBACK_SIGMA * sigma:
                return "off"
            return "forward"

        # Калибровки нет — остаётся признак «зрачок/белок» от прошивки.
        return "forward" if bool(s.get("reflect", True)) else "off"

    def _sensor_scale(self, idx: int) -> float | None:
        """Нормировка датчика (σ из калибровки) или None, если её нет.

        Старые калибровки без нормировки работают в сырых единицах,
        как раньше.
        """
        cal = self.ir_calibration if isinstance(self.ir_calibration, dict) else {}
        scale = cal.get("scale")
        if isinstance(scale, dict):
            value = scale.get(f"s{idx}", np.nan)
            if isinstance(value, (int, float)) and np.isfinite(value) and value > 0:
                return float(value)
        return None

    def _position_from_eye_states(self, states: list[dict]) -> str:
        """Единая классификация позиции взгляда по состояниям глаз.

        Глаза сопряжены, поэтому позиция выводится из того же сопряжённого
        результата, что и индикатор: если хоть один доступный глаз смещён —
        берём его направление (влево/вправо), иначе «вперёд». Направление
        «off» означает «смещено, но направление не определено».
        """
        for st in states:
            if st.get("available") and st.get("deviated"):
                direction = st.get("direction")
                return direction if direction in ("left", "right") else "off"
        return "forward"

    def _update_calibration_progress_status(self):
        """Живой счётчик отсчётов текущей фазы калибровки в статус-строке."""
        if not (0 <= self._calibration_phase < 3):
            return
        direction = self._calibration_instructions[self._calibration_phase][0]
        phase = self._calibration_samples[self._calibration_phase]
        n0, n1 = len(phase["sensor0"]), len(phase["sensor1"])
        self.status_bar.showMessage(
            f"Калибровка [{direction}] {self._calibration_phase + 1}/3: сбор данных…  "
            f"отсчётов: право {n0} · лево {n1}"
        )

    def _set_current_ir(self, sensor_id: int, ir_data: dict):
        sensor_id = int(sensor_id) % 2
        is_reflect = bool(ir_data.get("reflect", False))
        now = time.time()

        if self._calibration_in_progress and self._calibration_phase_collecting:
            if 0 <= self._calibration_phase < 3:
                raw_val = ir_data.get("raw", np.nan)
                # Первые CALIBRATION_SETTLE_SECONDS фазы не собираем: взгляд
                # ещё перемещается к цели, иначе медиана «уезжает» к прошлому
                # положению (главная причина неточной калибровки).
                settled = (
                    now - self._calibration_phase_started_at
                ) >= CALIBRATION_SETTLE_SECONDS
                if settled and isinstance(raw_val, (int, float)) and np.isfinite(raw_val):
                    self._calibration_samples[self._calibration_phase][
                        f"sensor{sensor_id}"
                    ].append({"raw": float(raw_val), "reflect": is_reflect})

        with self._ir_lock:
            s = self._ir_sensors[sensor_id]
            prev_update = float(s.get("last_update") or 0.0)
            s.update(ir_data)
            s["available"] = True
            s["last_update"] = now

            # Скользящая медиана для классификации: гасит шум АЦП и моргания.
            # В экспорт LSL и панель идёт сырой raw — он не меняется.
            raw_val = s.get("raw", np.nan)
            history = self._ir_raw_history[sensor_id]
            if isinstance(raw_val, (int, float)) and np.isfinite(raw_val):
                if prev_update and (now - prev_update) > IR_STALE_SECONDS:
                    history.clear()  # датчик «пропадал» — начинаем окно заново
                history.append(float(raw_val))
                s["raw_f"] = float(np.median(history))
            else:
                s["raw_f"] = s.get("raw_f", np.nan)

            if self._recording_active:
                prev = self._last_ir_reflect_state[sensor_id]
                if prev is None:
                    self._last_ir_reflect_state[sensor_id] = is_reflect
                elif is_reflect != prev:
                    self._last_ir_reflect_state[sensor_id] = is_reflect
                    self._ir_events.append(
                        {
                            "sensor": sensor_id,
                            "app_timestamp": now,
                            "lsl_timestamp": float(local_clock()) if local_clock else np.nan,
                            "reflect": is_reflect,
                            "raw": ir_data.get("raw", np.nan),
                            "volts": ir_data.get("volts", np.nan),
                        }
                    )
                    if len(self._ir_events) > 50000:
                        del self._ir_events[:10000]

        if self._calibration_in_progress:
            if self._calibration_phase_collecting:
                self._update_calibration_progress_status()
            self._update_gaze_indicator_text()
            return

        states = self._gaze_eye_states()
        new_pos = self._position_from_eye_states(states)
        old_pos = self._current_gaze_pos
        old_off = self._gaze_off
        new_off = (new_pos != "forward")

        if new_pos != old_pos or new_off != old_off:
            self._current_gaze_pos = new_pos
            self._gaze_off = new_off
            self._set_gaze_indicator_style(new_pos)

        # ── Управление экспериментом по взгляду (уровневая логика) ───────
        # Решение принимается по текущему уровню, а не только по фронту
        # перехода: если пакет с переходом потерялся или состояние успело
        # обновиться, пауза не «залипает». Пауза ставится, пока эксперимент
        # идёт и взгляд смещён; снимается, как только взгляд вернулся в центр
        # (и продолжается с того же места последовательности).
        if self._ignore_gaze():
            # «Игнорировать направление взгляда»: отклонение не ставит на
            # паузу, а уже стоящая пауза по взгляду снимается.
            if self._paused_by_gaze:
                self._resume_experiment()
        elif self._paused_by_gaze:
            if not new_off:
                self._resume_experiment()
        elif self._running and new_off:
            # Небольшая пауза-«антидребезг» сразу после возобновления:
            # старые отсчёты в медиане не должны мгновенно ставить паузу.
            if time.monotonic() >= self._gaze_pause_block_until:
                self._pause_experiment(reason=f"взгляд {gaze_position_label(new_pos)}")

        # Ожидание старта после калибровки: взгляд вернулся в центр — запускаем.
        if (
            self._waiting_gaze_center
            and not self._gaze_off
            and not self._running
            and not self._calibration_in_progress
        ):
            self._waiting_gaze_center = False
            self.stop_btn.setEnabled(False)
            self._log("Взгляд в центре — запуск эксперимента.", COLORS["green"])
            QTimer.singleShot(250, self._start_experiment)

        self._update_gaze_indicator_with_states(states)

    def _get_current_ir(self) -> dict:
        """Aggregate snapshot for LSL recording (first available sensor) + position."""
        with self._ir_lock:
            info = {
                "raw": np.nan, "volts": np.nan, "reflect": True,
                "threshold": np.nan, "min": np.nan, "max": np.nan,
                "available": False,
            }
            for s in self._ir_sensors:
                if s["available"]:
                    info = {
                        "raw": s.get("raw", np.nan),
                        "volts": s.get("volts", np.nan),
                        "reflect": s.get("reflect", True),
                        "threshold": s.get("threshold", np.nan),
                        "min": s.get("min", np.nan),
                        "max": s.get("max", np.nan),
                        "available": True,
                    }
                    break
            info["gaze_position"] = self._current_gaze_pos
            # Оба датчика по отдельности: по экспорту можно офлайн проверить
            # и при необходимости пересчитать классификацию взгляда.
            for idx, s in enumerate(self._ir_sensors):
                info[f"raw{idx}"] = s.get("raw", np.nan)
                info[f"volts{idx}"] = s.get("volts", np.nan)
                info[f"reflect{idx}"] = bool(s.get("reflect", True))
                info[f"available{idx}"] = bool(s.get("available", False))
                info[f"eye{idx}"] = self._sensor_eye_pos[idx]
            return info

    def _append_ir_table_row(self, sensor_id: int, raw, volts, refl, mn, mx):
        try:
            ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            while self.ir_table.rowCount() >= self._ir_table_max_rows:
                self.ir_table.removeRow(0)

            row = self.ir_table.rowCount()
            self.ir_table.insertRow(row)
            items = [
                self._make_sequence_table_item(str(sensor_id)),
                self._make_sequence_table_item(ts),
                self._make_sequence_table_item(str(raw)),
                self._make_sequence_table_item(f"{volts:.3f}"),
                self._make_sequence_table_item("ЕСТЬ (белок)" if refl else "ЗРАЧОК (чёрный)"),
                self._make_sequence_table_item(f"{mn} / {mx}"),
            ]
            items[4].setForeground(
                QColor(COLORS["green"]) if refl else QColor(COLORS["red"])
            )
            for col, item in enumerate(items):
                self.ir_table.setItem(row, col, item)
            self.ir_table.setRowHeight(row, 28)
            self.ir_table.scrollToBottom()
        except Exception:
            pass

    # ── Calibration ──────────────────────────────────────────────────────────
    def _update_calibration_status_label(self):
        if not hasattr(self, "calibration_status_label"):
            return
        cal = self.ir_calibration
        if not cal or "forward" not in cal:
            mode_note = " (инвертированные показания)" if INVERT_SENSOR_RAW else ""
            self.calibration_status_label.setText(
                "Калибровка: не выполнена. По умолчанию смещение определяется "
                "по видимости зрачка датчиком (нет отражения — смещён)."
                f"{mode_note}"
            )
            return
        parts = []
        for pos_name, label in (("left", "влево"), ("forward", "вперёд"), ("right", "вправо")):
            if pos_name in cal:
                p = cal[pos_name]
                parts.append(f"{label} = ({p['s0']:.0f}; {p['s1']:.0f})")
        scale = cal.get("scale")
        if isinstance(scale, dict):
            parts.append(f"σ: право={scale.get('s0', 0):.0f}, лево={scale.get('s1', 0):.0f}")
        self.calibration_status_label.setText("Калибровка: " + "   |   ".join(parts))

    def _load_calibration(self):
        try:
            raw = self.settings.value("ir/calibration", "", type=str)
            if not raw:
                self.ir_calibration = {}
                return
            data = json.loads(raw)
            if not isinstance(data, dict):
                self.ir_calibration = {}
                return
            clean: dict = {}
            for name in ("left", "forward", "right"):
                if name not in data or not isinstance(data[name], dict):
                    continue
                try:
                    clean[name] = {
                        "s0": float(data[name]["s0"]),
                        "s1": float(data[name]["s1"]),
                    }
                except (KeyError, ValueError, TypeError):
                    continue
            # Нормировка датчиков (σ) — необязательная часть; старые
            # сохранённые калибровки её не содержат и работают в сырых единицах.
            scale_data = data.get("scale")
            if isinstance(scale_data, dict):
                scale: dict = {}
                for key in ("s0", "s1"):
                    try:
                        value = float(scale_data[key])
                        if np.isfinite(value) and value > 0:
                            scale[key] = max(value, MIN_SENSOR_SCALE)
                    except (KeyError, ValueError, TypeError):
                        continue
                if scale:
                    clean["scale"] = scale
            if "forward" not in clean:
                self.ir_calibration = {}
            else:
                self.ir_calibration = clean
        except Exception:
            self.ir_calibration = {}

    def _save_calibration(self):
        try:
            self.settings.setValue(
                "ir/calibration", json.dumps(self.ir_calibration, ensure_ascii=False)
            )
        except Exception:
            pass

    def _apply_calibration_to_firmware(self):
        """Send firmware thresholds derived from calibration points.

        Firmware still uses its own interpretation (raw < th = ref), but this
        is only for firmware-internal display. Python does NOT use firmware
        `ref` — Python recomputes reflect from inverted raw and threshold
        using the correct physics (low raw = white = OK).
        """
        if not self.serial.is_open:
            return
        cal = self.ir_calibration
        if not cal or "forward" not in cal:
            return

        f = cal["forward"]
        l = cal.get("left", f)
        r = cal.get("right", f)

        for idx in (0, 1):
            key = f"s{idx}"
            fwd_val = f.get(key, np.nan)
            left_val = l.get(key, fwd_val)
            right_val = r.get(key, fwd_val)
            if not (np.isfinite(fwd_val) and np.isfinite(left_val) and np.isfinite(right_val)):
                continue
            off_val = left_val if abs(left_val - fwd_val) >= abs(right_val - fwd_val) else right_val
            # Threshold in INVERTED scale (Python internal): midpoint between
            # forward (low raw in inverted scale) and lateral (high raw).
            threshold_inv = int(round((fwd_val + off_val) / 2.0))
            # Convert back to firmware's raw scale (before inversion) before sending:
            threshold_fw = int(round(invert_adc(threshold_inv)))
            hysteresis = max(80, int(round(abs(off_val - fwd_val) * 0.2)))
            self._send(f"SET IRTH{idx} {threshold_fw}")
            self._send(f"SET IRHY{idx} {hysteresis}")
        self._log("Пороги прошивки обновлены (только для отображения).", COLORS["green"])

    def _start_calibration(self, after=None) -> bool:
        """Запускает калибровку взгляда с первой фазы.

        ``after`` — необязательная функция, которая будет вызвана после
        успешного завершения калибровки (так старт эксперимента продолжается
        автоматически). Возвращает False, если калибровку начать нельзя.
        """
        if self._calibration_in_progress:
            return False
        if not self.serial.is_open:
            self._log(
                "Калибровка невозможна: нет подключения к Arduino по Serial.",
                COLORS["red"],
            )
            self.status_bar.showMessage("Нет подключения к Arduino для калибровки")
            return False
        if self._running or self._paused_by_gaze:
            self._stop(reason="Калибровка")

        self._calibration_after = after
        self._calibration_in_progress = True
        self._calibration_phase = 0
        self._calibration_phase_collecting = False
        self._calibration_retries = 0
        self._calibration_samples = [
            {"sensor0": [], "sensor1": []},
            {"sensor0": [], "sensor1": []},
            {"sensor0": [], "sensor1": []},
        ]
        self._log("═══ Калибровка взгляда: всегда перед стартом (3 фазы) ═══", COLORS["accent"])
        self._log(
            "Будут записаны точки: ВЛЕВО → ВПЕРЁД → ВПРАВО. "
            "Каждая фаза — 3 секунды.",
            COLORS["accent"],
        )
        self.status_bar.showMessage("Калибровка взгляда: ВЛЕВО → ВПЕРЁД → ВПРАВО")
        QTimer.singleShot(100, self._show_calibration_phase_dialog)
        return True

    def _calibration_abort(self, reason: str) -> None:
        """Прерывает калибровку: эксперимент не стартует, кнопки возвращаются."""
        self._calibration_in_progress = False
        self._calibration_phase_collecting = False
        self._calibration_after = None
        self._waiting_gaze_center = False
        self.start_btn.setEnabled(self.serial.is_open)
        self.stop_btn.setEnabled(False)
        self._log(f"Калибровка не завершена ({reason}). Эксперимент не запущен.", COLORS["yellow"])
        self.status_bar.showMessage(f"Калибровка не завершена: {reason}")

    def _show_calibration_phase_dialog(self):
        if not self._calibration_in_progress:
            return
        if self._calibration_phase >= 3:
            self._finish_calibration()
            return

        direction, instruction = self._calibration_instructions[self._calibration_phase]
        phase_no = self._calibration_phase + 1

        msg = QMessageBox(self)
        msg.setIcon(QMessageBox.Information)
        msg.setWindowTitle(f"Калибровка {phase_no}/3 — {direction}")
        msg.setText(f"{instruction}.")
        msg.setInformativeText(
            "Нажмите «ОК», чтобы начать запись (3 секунды).\n"
            "Держите взгляд неподвижно и не моргайте."
        )
        msg.setStandardButtons(QMessageBox.Ok | QMessageBox.Cancel)
        msg.setDefaultButton(QMessageBox.Ok)
        msg.setInformativeText(msg.informativeText() + "\n\n«Отмена» — эксперимент не будет запущен.")
        result = msg.exec()

        if result != QMessageBox.Ok:
            self._calibration_abort("отменена пользователем")
            return

        self._calibration_samples[self._calibration_phase] = {
            "sensor0": [],
            "sensor1": [],
        }
        self._calibration_phase_collecting = True
        self._calibration_phase_started_at = time.time()
        self._log(f"Калибровка [{direction}]: сбор данных 3 с...", COLORS["accent"])
        self.status_bar.showMessage(f"Калибровка [{direction}]: сбор данных 3 с…")
        QTimer.singleShot(3000, self._calibration_phase_done)

    def _phase_stds(self, phase_idx: int) -> tuple:
        """Устойчивая σ (MAD) отсчётов фазы по каждому датчику.

        MAD не раздувается от одиночных выбросов (моргание, сбой АЦП), в
        отличие от обычного std — иначе шумный датчик получал бы завышенную
        нормировку и «размытые» пороги.
        """
        out = []
        for key in ("sensor0", "sensor1"):
            raws = [
                r["raw"]
                for r in self._calibration_samples[phase_idx][key]
                if np.isfinite(r["raw"])
            ]
            out.append(_robust_sigma(raws))
        return tuple(out)

    def _calibration_phase_done(self):
        if not self._calibration_in_progress:
            return
        self._calibration_phase_collecting = False

        n0 = len(self._calibration_samples[self._calibration_phase]["sensor0"])
        n1 = len(self._calibration_samples[self._calibration_phase]["sensor1"])
        direction = self._calibration_instructions[self._calibration_phase][0]
        self._log(
            f"Калибровка [{direction}]: собрано S0={n0}, S1={n1}",
            COLORS["accent"],
        )

        # Качество фазы: слишком мало отсчётов или сильное дрожание взгляда
        # — фаза повторяется (ограниченное число раз), а не проваливает
        # всю калибровку.
        bad_reason = None
        if n0 < CALIBRATION_MIN_SAMPLES or n1 < CALIBRATION_MIN_SAMPLES:
            bad_reason = f"мало отсчётов (нужно ≥{CALIBRATION_MIN_SAMPLES})"
        else:
            std0, std1 = self._phase_stds(self._calibration_phase)
            for sensor_name, std in (("право", std0), ("лево", std1)):
                if std is not None and std > CALIBRATION_MAX_STD:
                    bad_reason = f"дрожание ({sensor_name}: σ={std:.0f})"
                    break

        if bad_reason and self._calibration_retries < CALIBRATION_MAX_RETRIES:
            self._calibration_retries += 1
            self._log(
                f"Калибровка [{direction}]: {bad_reason} — повтор фазы "
                f"({self._calibration_retries}/{CALIBRATION_MAX_RETRIES}). "
                "Держите взгляд неподвижнее.",
                COLORS["yellow"],
            )
            self.status_bar.showMessage(
                f"Калибровка [{direction}]: {bad_reason} — повтор фазы…"
            )
            QTimer.singleShot(300, self._show_calibration_phase_dialog)
            return
        if bad_reason:
            self._log(
                f"Калибровка [{direction}]: {bad_reason} — повторов больше нет, "
                "фаза принята как есть.",
                COLORS["yellow"],
            )

        self._calibration_retries = 0
        self._calibration_phase += 1
        QTimer.singleShot(200, self._show_calibration_phase_dialog)

    def _finish_calibration(self):
        self._calibration_in_progress = False
        self._calibration_phase_collecting = False

        left = self._calibration_samples[0]
        fwd = self._calibration_samples[1]
        right = self._calibration_samples[2]

        MIN_SAMPLES = CALIBRATION_MIN_SAMPLES

        def median_for(phase_data, sensor_key):
            raws = [r["raw"] for r in phase_data[sensor_key] if np.isfinite(r["raw"])]
            # Медиана с отбрасыванием выбросов: моргания и неполные переводы
            # взгляда не сдвигают опорную точку.
            med_val, _sigma, n = _robust_median(raws)
            return med_val if n >= MIN_SAMPLES else None

        med: dict = {}
        for phase_name, phase_data in (
            ("left", left),
            ("forward", fwd),
            ("right", right),
        ):
            s0 = median_for(phase_data, "sensor0")
            s1 = median_for(phase_data, "sensor1")
            if s0 is None or s1 is None:
                counts0 = len(phase_data["sensor0"])
                counts1 = len(phase_data["sensor1"])
                self._log(
                    f"Калибровка: фаза «{phase_name}» — мало данных "
                    f"(S0={counts0}, S1={counts1}, нужно ≥{MIN_SAMPLES})",
                    COLORS["yellow"],
                )
                QMessageBox.warning(
                    self, "Калибровка",
                    f"Недостаточно данных для фазы «{phase_name}».\n"
                    f"Отсчётов: sensor0={counts0}, sensor1={counts1}. "
                    f"Нужно ≥{MIN_SAMPLES}.\n\n"
                    "Проверьте, что оба датчика шлют данные "
                    "(вкладка «Сервис» → История показаний)."
                )
                self._update_calibration_status_label()
                self._calibration_abort(f"фаза «{phase_name}»: мало данных")
                return
            med[phase_name] = {"s0": s0, "s1": s1}
            self._log(
                f"Калибровка [{phase_name}]: s0={s0:.0f}, s1={s1:.0f}",
                COLORS["accent"],
            )

        f = med["forward"]
        l = med["left"]
        r = med["right"]

        d_lf = float(np.hypot(l["s0"] - f["s0"], l["s1"] - f["s1"]))
        d_rf = float(np.hypot(r["s0"] - f["s0"], r["s1"] - f["s1"]))
        d_lr = float(np.hypot(l["s0"] - r["s0"], l["s1"] - r["s1"]))

        self._log(
            f"Калибровка: расстояния "
            f"left↔forward={d_lf:.0f}, right↔forward={d_rf:.0f}, "
            f"left↔right={d_lr:.0f}",
            COLORS["accent"],
        )

        if d_lf < MIN_CALIBRATION_SEPARATION and d_rf < MIN_CALIBRATION_SEPARATION:
            QMessageBox.warning(
                self, "Калибровка",
                f"Точки «влево» и «вправо» слишком близки к «вперёд»:\n"
                f"  left↔forward  = {d_lf:.0f}\n"
                f"  right↔forward = {d_rf:.0f}\n"
                f"  left↔right    = {d_lr:.0f}\n\n"
                f"Минимальное расстояние — {int(MIN_CALIBRATION_SEPARATION)}.\n"
                "Датчики слабо различают положения взгляда.\n\n"
                "Рекомендации:\n"
                "• Уменьшите резистор ИК-светодиода (220 Ом вместо 680 Ом).\n"
                "• Приблизьте датчики к глазу (0,5–1 см).\n"
                "• Проверьте, что при взгляде в стороны один из датчиков "
                "видит зрачок (raw заметно меняется)."
            )
            self._update_calibration_status_label()
            self._calibration_abort("точки «влево»/«вправо» слишком близки к «вперёд»")
            return

        # Нормировка датчиков: σ каждого датчика (медиана по трём фазам).
        # Классификация будет мерить отклонения в долях σ — датчики с разной
        # чувствительностью сравниваются одинаково.
        scale: dict = {}
        for idx, key in ((0, "sensor0"), (1, "sensor1")):
            stds = [self._phase_stds(i)[0 if idx == 0 else 1] for i in range(3)]
            finite = [s for s in stds if s is not None and np.isfinite(s)]
            if finite:
                sigma = float(np.median(finite))
                if np.isfinite(sigma) and sigma > 0:
                    scale[f"s{idx}"] = max(sigma, MIN_SENSOR_SCALE)

        # Различимость сторон по каждому датчику: |ref_стороны − forward| в σ.
        # Асимметрия — норма (у «вправо» датчик может смещаться на 500
        # отсчётов, у «влево» — на 2000), но слишком слабую сторону датчик
        # физически не видит; об этом предупреждаем сразу.
        weak_sides = []
        for idx in (0, 1):
            sigma = float(scale.get(f"s{idx}", MIN_SENSOR_SCALE))
            min_sep = max(
                GAZE_PROJECTION_MIN_SEP_SIGMA * sigma, GAZE_PROJECTION_MIN_SEP_RAW
            )
            parts = []
            for side_name, side_ref in (("влево", l), ("вправо", r)):
                delta = abs(float(side_ref[f"s{idx}"]) - float(f[f"s{idx}"]))
                parts.append(
                    f"{side_name} Δ={delta:.0f} ({delta / max(sigma, 1e-6):.2f}σ)"
                )
                if delta < min_sep:
                    weak_sides.append(f"датчик {idx} — {side_name}")
            self._log(
                f"   датчик {idx}: " + ", ".join(parts),
                COLORS["text_dim"],
            )
        if weak_sides:
            self._log(
                "Внимание: слабо различимы стороны — "
                + "; ".join(weak_sides)
                + ". Направление в эту сторону будет определяться хуже "
                "(используется второй датчик).",
                COLORS["yellow"],
            )

        # ── Проверки геометрии калибровки ────────────────────────────────────
        # Ошибка здесь не ломает эксперимент мгновенно, но искажает
        # классификацию взгляда, а значит — моменты пауз и всю разметку
        # эксперимента. Поэтому предупреждаем явно.
        geometry_problems: list[str] = []

        # 1. Опорная точка «вперёд» должна лежать между «влево» и «вправо»,
        #    причём примерно посередине: взгляд «вперёд» — это середина дуги
        #    между крайними положениями. Смещение говорит о том, что в фазе
        #    «вперёд» пользователь смотрел не в центр.
        mid_s0 = (float(l["s0"]) + float(r["s0"])) / 2.0
        mid_s1 = (float(l["s1"]) + float(r["s1"])) / 2.0
        lr_span = float(
            np.hypot(float(l["s0"]) - float(r["s0"]),
                     float(l["s1"]) - float(r["s1"]))
        )
        if lr_span > 0.0:
            offset = float(
                np.hypot(float(f["s0"]) - mid_s0, float(f["s1"]) - mid_s1)
            )
            if offset > CALIBRATION_FORWARD_TOLERANCE * lr_span:
                geometry_problems.append(
                    f"«вперёд» смещён от середины «влево»↔«вправо» "
                    f"({offset:.0f} из {lr_span:.0f})"
                )
        for idx in (0, 1):
            key = f"s{idx}"
            lo = min(float(l[key]), float(r[key]))
            hi = max(float(l[key]), float(r[key]))
            span = hi - lo
            if span <= 0.0:
                continue
            f_val = float(f[key])
            tol = 0.15 * span
            if not (lo - tol <= f_val <= hi + tol):
                geometry_problems.append(
                    f"датчик {idx}: «вперёд»={f_val:.0f} вне диапазона "
                    f"[{lo:.0f}…{hi:.0f}]"
                )

        # 2. Датчики должны реагировать навстречу друг другу: зрачок уходит
        #    к одному глазу и открывает другой. Оба датчика в одну сторону —
        #    признак перекрытого или «сбитого» датчика.
        sig0 = max(float(scale.get("s0", MIN_SENSOR_SCALE)), MIN_SENSOR_SCALE)
        sig1 = max(float(scale.get("s1", MIN_SENSOR_SCALE)), MIN_SENSOR_SCALE)
        for side_name, side in (("влево", l), ("вправо", r)):
            d0 = float(side["s0"]) - float(f["s0"])
            d1 = float(side["s1"]) - float(f["s1"])
            if abs(d0) > 2.0 * sig0 and abs(d1) > 2.0 * sig1 and (d0 > 0) == (d1 > 0):
                geometry_problems.append(
                    f"«{side_name}»: датчики сместились синхронно "
                    f"(Δ0={d0:+.0f}, Δ1={d1:+.0f})"
                )

        # 3. Фазы «влево» и «вправо» должны различаться, иначе направление
        #    взгляда определить не по чему.
        if d_lr < MIN_CALIBRATION_SEPARATION:
            geometry_problems.append(
                f"«влево» и «вправо» почти неразличимы (d={d_lr:.0f})"
            )

        if geometry_problems:
            self._log(
                "Калибровка: проверка геометрии — " + "; ".join(geometry_problems),
                COLORS["yellow"],
            )
            self._log(
                "   Если направление взгляда отслеживается неверно — "
                "переделайте калибровку.",
                COLORS["yellow"],
            )

        self.ir_calibration = {"left": l, "forward": f, "right": r}
        if scale:
            self.ir_calibration["scale"] = scale
        self._save_calibration()
        self._apply_calibration_to_firmware()
        # После калибровки накопленное состояние сбрасываем: последняя фаза
        # была «ВПРАВО», а в окне медианы и в счётчиках дебаунса ещё лежат
        # отсчёты старого положения. Реальную позицию определит свежая
        # телеметрия — иначе приложение считало бы взгляд «вперёд», хотя
        # пользователь смотрит в сторону: эксперимент стартовал бы с
        # отведённым взглядом и тут же вставал бы на паузу.
        with self._ir_lock:
            for history in self._ir_raw_history:
                history.clear()
        self._sensor_eye_pos = ["forward", "forward"]
        self._sensor_eye_pending = [None, None]
        self._current_gaze_pos = "forward"
        self._gaze_off = False
        self._update_calibration_status_label()
        self._update_gaze_indicator_text()
        self._set_gaze_indicator_style("forward")

        self._log("Калибровка взгляда завершена и сохранена:", COLORS["green"])
        self._log(
            f"   влево:  sensor0={l['s0']:.0f}, sensor1={l['s1']:.0f}",
            COLORS["text_dim"],
        )
        self._log(
            f"   вперёд: sensor0={f['s0']:.0f}, sensor1={f['s1']:.0f}",
            COLORS["text_dim"],
        )
        self._log(
            f"   вправо: sensor0={r['s0']:.0f}, sensor1={r['s1']:.0f}",
            COLORS["text_dim"],
        )
        if scale:
            self._log(
                f"   нормировка (σ): право={scale.get('s0', 0):.0f}, "
                f"лево={scale.get('s1', 0):.0f} — пороги в долях σ",
                COLORS["text_dim"],
            )
        self._log(
            f"   расстояния: left↔forward={d_lf:.0f}, right↔forward={d_rf:.0f}",
            COLORS["text_dim"],
        )

        after = self._calibration_after
        self._calibration_after = None
        if after is not None:
            self.status_bar.showMessage("Калибровка завершена — запуск эксперимента")
            # Пауза, чтобы пришли свежие отсчёты датчиков и классификация
            # взгляда обновилась по новой калибровке: дебаунс — 3 свежих
            # отсчёта (≈300 мс при 10 Гц), плюс время на сам отсчёт. За
            # эти 700 мс приложение уже понимает, куда смотрит человек, и
            # стартует только если взгляд действительно в центре.
            QTimer.singleShot(700, after)
        else:
            self.status_bar.showMessage("Калибровка завершена")

    # ── Pause / resume ───────────────────────────────────────────────────────
    def _pause_experiment(self, reason: str = "пауза"):
        if self._paused_by_gaze:
            return
        if not self._running:
            # Эксперимент уже остановлен (или завершён) — пауза по взгляду
            # не должна его трогать. Иначе возобновление по возврату взгляда
            # перезапустило бы уже законченный прогон, и приложение снова
            # просило бы сохранить результаты вместо спокойного завершения.
            return

        self._paused_by_gaze = True
        self._resuming_after_gaze_pause = False
        self._pause_started_at = time.monotonic()
        self._log(f"⏸ ПАУЗА: {reason}. Отправка STOP.", COLORS["yellow"])
        self.status_bar.showMessage(f"⏸ Пауза: {reason}")

        if self._running and self.sequence_steps:
            last_idx = len(self.sequence_steps) - 1
            if self._sequence_stage_start_time is not None:
                elapsed = time.monotonic() - self._sequence_stage_start_time
                self._pause_remaining_seconds = max(
                    0.0, self._sequence_stage_duration - elapsed
                )
                self._pause_step_index = min(
                    self._sequence_stage_number - 1, last_idx
                )
                self._log(
                    f"   этап {self._sequence_stage_number} приостановлен, "
                    f"осталось {self._pause_remaining_seconds:.2f} с",
                    COLORS["yellow"],
                )
            else:
                # Нет данных о начале этапа (гонка на границе этапов):
                # не теряем этап — продолжаем текущий/последний с его
                # длительностью. Иначе на возобновлении можно было получить
                # «индекс за последним этапом» и досрочное завершение.
                idx = min(max(self._sequence_index - 1, 0), last_idx)
                self._pause_step_index = idx
                self._pause_remaining_seconds = float(
                    self._sequence_stage_duration
                    or self.sequence_steps[idx].seconds
                )
                self._log(
                    f"   этап {idx + 1} будет повторён с начала "
                    f"({self._pause_remaining_seconds:.2f} с)",
                    COLORS["yellow"],
                )

        self._sequence_stage_deadline = None
        self._sequence_timer.stop()
        self._sequence_progress_timer.stop()
        self._auto_apply_timer.stop()
        if self.serial.is_open:
            self._send("STOP")
        self._running = False

    def _resume_experiment(self):
        if not self._paused_by_gaze:
            return
        if self._experiment_finished:
            # Эксперимент уже завершён — «возобновление» означало бы его
            # повторный запуск и ещё одно окно сохранения результатов.
            self._paused_by_gaze = False
            self._log(
                "Эксперимент уже завершён — повторный запуск не выполняется.",
                COLORS["yellow"],
            )
            return
        self._paused_by_gaze = False
        self._running = True
        self._resuming_after_gaze_pause = True
        # Пауза не должна «съедать» общее время эксперимента: сдвигаем старт
        # общего таймера на длительность паузы (иначе после долгой паузы
        # показывается «Эксперимент: осталось 00:00», как будто всё кончилось).
        if self._pause_started_at is not None and self._sequence_total_start_time is not None:
            self._sequence_total_start_time += time.monotonic() - self._pause_started_at
        self._pause_started_at = None
        # В окне медианы ещё лежат отсчёты «смещённого» взгляда — очищаем,
        # чтобы первые пакеты после возобновления не поставили паузу снова.
        with self._ir_lock:
            for history in self._ir_raw_history:
                history.clear()
        self._sensor_eye_pending = [None, None]
        self._gaze_pause_block_until = time.monotonic() + 0.5
        self._log("▶ Возобновление эксперимента.", COLORS["green"])
        self.status_bar.showMessage("▶ Возобновление")

        if self.sequence_steps:
            last_idx = len(self.sequence_steps) - 1
            if self._pause_step_index >= 0:
                # Продолжаем именно приостановленный этап с его остатком;
                # индекс всегда в допустимых пределах — иначе «завершение
                # эксперимента» случилось бы раньше последнего этапа.
                self._sequence_index = min(self._pause_step_index, last_idx)
                self._sequence_override_seconds = max(
                    self._pause_remaining_seconds, 0.01
                )
                self._log(
                    f"   продолжаем этап {self._sequence_index + 1}/"
                    f"{len(self.sequence_steps)} с остатком "
                    f"{self._sequence_override_seconds:.2f} с",
                    COLORS["accent"],
                )
            else:
                # Страховка: возобновление НИКОГДА не должно завершать
                # эксперимент. Если индекс ушёл за последний этап или этап
                # неизвестен — повторяем текущий/последний этап целиком.
                idx = min(max(self._sequence_index, 0), last_idx)
                self._sequence_index = idx
                self._sequence_override_seconds = float(self.sequence_steps[idx].seconds)
                self._log(
                    f"   продолжаем этап {idx + 1}/{len(self.sequence_steps)} "
                    "с начала (этап паузы не был записан).",
                    COLORS["yellow"],
                )
            self._run_next_sequence_step()
        else:
            if self.serial.is_open:
                self._send("START")
                self._send("DBG ON")

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
        with self._stim_lock:
            self._stim_events = []
        with self._ir_lock:
            self._ir_events = []
            self._last_ir_reflect_state = [None, None]

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
            ir_getter=self._get_current_ir,
            error_signal=self._lsl_worker_error,
        )
        self._recording_active = True
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
            "ir_raw": [r.get("ir_raw", np.nan) for r in records],
            "ir_volts": [r.get("ir_volts", np.nan) for r in records],
            "ir_reflect": [
                float(bool(r.get("ir_reflect"))) if r.get("ir_available") else np.nan
                for r in records
            ],
            "ir_threshold": [r.get("ir_threshold", np.nan) for r in records],
            "ir_min": [r.get("ir_min", np.nan) for r in records],
            "ir_max": [r.get("ir_max", np.nan) for r in records],
            "ir_available": [1 if r.get("ir_available") else 0 for r in records],
            "ir_gaze_position": [r.get("ir_gaze_position", "forward") for r in records],
            # Оба датчика по отдельности — чтобы классификацию взгляда можно
            # было проверить и пересчитать офлайн по сырым отсчётам.
            "ir_raw0": [r.get("ir_raw0", np.nan) for r in records],
            "ir_raw1": [r.get("ir_raw1", np.nan) for r in records],
            "ir_eye0": [r.get("ir_eye0", "forward") for r in records],
            "ir_eye1": [r.get("ir_eye1", "forward") for r in records],
        }
        for idx, name in enumerate(safe_names):
            data[f"raw_{name}"] = raw[:, idx]
            data[f"filtered_{name}"] = filtered[:, idx]
        data_df = pd.DataFrame(data)

        if records:
            t_lo = records[0]["app_timestamp"]
            t_hi = records[-1]["app_timestamp"]
            with self._ir_lock:
                ir_events_for_export = [
                    e
                    for e in self._ir_events
                    if t_lo <= e.get("app_timestamp", 0.0) <= t_hi
                ]
        else:
            ir_events_for_export = []

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
            "ir_events_json": self._json_value(ir_events_for_export),
            "ir_was_used": bool(any(r.get("ir_available") for r in records)),
            "ir_calibration_points_json": self._json_value(self.ir_calibration),
            "ir_invert_sensor_raw": bool(INVERT_SENSOR_RAW),
            "ir_ignore_gaze": bool(self._ignore_gaze()),
            "ir_skip_calibration": bool(self._skip_calibration()),
            "ir_classification_note": (
                "ir_gaze_position is classified in Python per sensor (nearest-neighbor "
                "against ir_calibration_points_json, distances normalized by per-sensor "
                "sigma from 'scale' when present; sliding-median smoothed input, "
                "state-change debounce, conjugate-gaze propagation to both eyes). "
                "'forward' means gaze OK; 'left'/'right'/'off' trigger pause "
                "unless ir_ignore_gaze is set (then gaze never pauses the "
                "experiment and does not block start; ir_skip_calibration "
                "means start skipped gaze calibration). "
                "In default mode (no calibration) deviation uses the per-sensor "
                "reflect flag. "
                "Values in ir_* columns are already inverted if ir_invert_sensor_raw=True: "
                "LOW raw = white/sclera, HIGH raw = black/pupil."
            ),
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
        # 8000 точек достаточно для пиков/минимумов и клиппинга, но в разы
        # дешевле для цикла событий, чем 50000 — анимации остаются плавными.
        t = np.linspace(0, 0.1, 8000)
        raw0, sig0 = self._build_channel_signal(p, p["dac0_f1"], p["dac0_f2"], t)
        raw1, sig1 = self._build_channel_signal(p, p["dac1_f1"], p["dac1_f2"], t)

        values = (
            (self.clip_label, self._check_clipping(raw0, raw1)),
            (self.stat_dac0_peak, f"{np.max(sig0):.3f} В"),
            (self.stat_dac0_trough, f"{np.min(sig0):.3f} В"),
            (self.stat_dac0_range, f"{np.max(sig0) - np.min(sig0):.3f} В"),
            (self.stat_dac1_peak, f"{np.max(sig1):.3f} В"),
            (self.stat_dac1_trough, f"{np.min(sig1):.3f} В"),
            (self.stat_dac1_range, f"{np.max(sig1) - np.min(sig1):.3f} В"),
            (self.stat_nyquist, f"{p['sr']/2:.0f} Гц"),
        )
        # Обновляем только изменившиеся надписи — иначе каждые 250 мс
        # запускается переразметка панели и страдает плавность анимаций.
        for label, text in values:
            if label.text() != text:
                label.setText(text)

    # ── Serial telemetry drain ────────────────────────────────────────────────
    def _drain_serial(self):
        for raw in self.serial.drain():
            self._process_incoming(raw)

    def _process_incoming(self, raw: str):
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
            sensor_id = int(d.get("sensor", 0))

            # ── Raw values from firmware (original scale) ──
            raw_orig = d.get("raw", 0)
            th_orig = d.get("th", 1500)
            min_orig = d.get("min", 0)
            max_orig = d.get("max", 0)

            try:
                raw_f = float(raw_orig) if raw_orig is not None else np.nan
                th_f = float(th_orig) if th_orig is not None else 1500.0
                min_f = float(min_orig) if min_orig is not None else 0.0
                max_f = float(max_orig) if max_orig is not None else 0.0
            except (TypeError, ValueError):
                raw_f, th_f, min_f, max_f = np.nan, 1500.0, 0.0, 0.0

            # ── Invert to the "physical intuition" scale: ──
            #   low raw = white/sclera (gaze OK)
            #   high raw = black/pupil (gaze off)
            if INVERT_SENSOR_RAW:
                raw_val = invert_adc(raw_f)
                th_val = invert_adc(th_f)
                min_val = invert_adc(max_f)   # min of inverted = inverted of max
                max_val = invert_adc(min_f)   # max of inverted = inverted of min
            else:
                raw_val = raw_f
                th_val = th_f
                min_val = min_f
                max_val = max_f

            # Standard logic (now correct in the inverted scale):
            if np.isfinite(raw_val) and np.isfinite(th_val):
                refl = raw_val < th_val
            else:
                refl = False

            volts = float(d.get("v", 0.0))
            state = "ЕСТЬ отражение (белок)" if refl else "ЗРАЧОК (чёрный)"

            raw_display = int(round(raw_val)) if np.isfinite(raw_val) else "?"
            th_display = int(round(th_val)) if np.isfinite(th_val) else "?"
            min_display = int(round(min_val)) if np.isfinite(min_val) else "?"
            max_display = int(round(max_val)) if np.isfinite(max_val) else "?"

            # Панель «Сервис»: обновляем колонку ТОЛЬКО этого датчика.
            labels = self.ir_metrics.get(sensor_id, {})
            if labels:
                labels["raw"].setText(str(raw_display))
                labels["volts"].setText(f"{volts:.3f}")
                labels["span"].setText(f"{min_display} / {max_display}")
                labels["th"].setText(str(th_display))
                labels["state"].setText(state)
                labels["state"].setStyleSheet(
                    f"color:{COLORS['green'] if refl else COLORS['red']};"
                    f"font-family:'JetBrains Mono';font-size:11px;font-weight:600;"
                )

            try:
                self._set_current_ir(
                    sensor_id,
                    {
                        "raw": raw_val,
                        "volts": volts,
                        "reflect": refl,
                        "threshold": th_val,
                        "min": min_val,
                        "max": max_val,
                    },
                )
            except (TypeError, ValueError):
                pass

            self._append_ir_table_row(
                sensor_id, raw_display, volts, refl, min_display, max_display
            )

            self._log(
                f"[IR{sensor_id}] raw={raw_display}  U={volts:.3f}В  "
                f"мин/макс={min_display}/{max_display}  "
                f"{state}  (порог {th_display})"
                + ("  [inv]" if INVERT_SENSOR_RAW else ""),
                COLORS["accent2"] if refl else COLORS["red"],
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
        self._log(text, COLORS["green"])

    @Slot(str, str)
    def _append_console(self, text: str, color: str):
        cursor = self.console.textCursor()
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
        self.console.setTextCursor(cursor)
        self.console.ensureCursorVisible()

    def _clear_console(self):
        self.console.clear()
        self._total_clip_hi = [0, 0]
        self._total_clip_lo = [0, 0]
        if hasattr(self, "ir_table"):
            self.ir_table.setRowCount(0)
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
                "Частоты задаются загруженным файлом последовательности.",
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
        """Старт эксперимента: сначала ВСЕГДА калибровка взгляда с нуля."""
        if self._calibration_in_progress:
            self._log("Калибровка уже выполняется.", COLORS["yellow"])
            return
        if self._waiting_gaze_center:
            # Повторное нажатие «Старт» — не ждём взгляда бесконечно:
            # отменяем ожидание и запускаемся сразу (один раз).
            self._waiting_gaze_center = False
            self._start_now_once = True
            self._log(
                "Ожидание возврата взгляда в центр отменено — "
                "старт по кнопке «Старт».",
                COLORS["yellow"],
            )
        if not self.serial.is_open:
            self._log("Нельзя начать: нет подключения к Arduino.", COLORS["red"])
            self.status_bar.showMessage("Подключитесь к Arduino перед стартом")
            return

        # Эксперимент стоит на паузе из-за отведения взгляда: «Старт» должен
        # ПРОДОЛЖИТЬ его, а не прерывать с окном сохранения результатов.
        if self._paused_by_gaze and not self._experiment_finished:
            if self._gaze_off and not self._ignore_gaze():
                self._log(
                    "Эксперимент на паузе: дождитесь возврата взгляда в центр "
                    "— продолжение произойдёт автоматически.",
                    COLORS["yellow"],
                )
                self.status_bar.showMessage("Верните взгляд в центр — продолжение")
            else:
                self._log("Продолжаем эксперимент с паузы.", COLORS["green"])
                self._resume_experiment()
            return

        # Новый эксперимент поверх ещё идущего прогона: сначала
        # корректно завершаем предыдущий (STOP, экспорт записи, сброс
        # состояния) — иначе его пауза/запись «протекают» в новый.
        if self._running:
            self._stop(reason="Запуск нового эксперимента")

        self.start_btn.setEnabled(False)
        # «Пропустить калибровку» (флажок в «Сервисе»): стартуем сразу.
        # Классификация взгляда работает по сохранённой калибровке, а если её
        # нет — по порогам по умолчанию (порог АЦП прошивки, «виден зрачок»).
        if self._skip_calibration():
            self._log(
                "Калибровка пропущена: используются сохранённые пороги "
                "или значения по умолчанию.",
                COLORS["yellow"],
            )
            self._start_experiment()
            return
        # После успешной калибровки эксперимент стартует автоматически.
        if not self._start_calibration(after=self._start_experiment):
            self.start_btn.setEnabled(self.serial.is_open)

    def _reset_gaze_pause_state(self):
        """Новый эксперимент не наследует паузу по взгляду от прошлого.

        Если эксперимент остался на паузе (пользователь нажал «Старт», а не
        «Стоп»), флаг ``_paused_by_gaze`` переживает калибровку и запуск —
        и тогда уровневая логика в ``_set_current_ir`` навсегда остаётся в
        ветке «пауза уже стоит»: взгляд виден (индикатор показывает «влево»),
        но эксперимент не останавливается.
        """
        self._paused_by_gaze = False
        self._resuming_after_gaze_pause = False
        self._pause_step_index = -1
        self._pause_remaining_seconds = 0.0
        self._sequence_override_seconds = None
        self._pause_started_at = None
        self._sequence_stage_deadline = None
        self._experiment_finished = False

    def _start_experiment(self):
        """Продолжение старта — вызывается сразу после калибровки взгляда."""
        # Одноразовый обход проверки взгляда (повторное нажатие «Старт»).
        force_gaze = self._start_now_once
        self._start_now_once = False
        # Чистый старт: никакой паузы/остатков от предыдущего эксперимента.
        self._reset_gaze_pause_state()
        if self._gaze_off and not self._ignore_gaze() and not force_gaze:
            # Последняя фаза калибровки — «ВПРАВО»: взгляд ещё не успел
            # вернуться в центр. Не считаем это ошибкой — ждём возврата
            # взгляда и стартуем автоматически.
            self._waiting_gaze_center = True
            prefix = (
                "Калибровка пропущена."
                if self._skip_calibration()
                else "Калибровка завершена."
            )
            self._log(
                f"{prefix} Верните взгляд в центр — "
                "эксперимент стартует автоматически.",
                COLORS["accent"],
            )
            self.status_bar.showMessage("Верните взгляд в центр — эксперимент начнётся автоматически")
            self.stop_btn.setEnabled(True)  # Стоп отменяет ожидание старта
            return

        if self.sequence_steps:
            self._start_sequence(force_gaze=force_gaze)
            return

        if not self._validate_lsl_recording_ready():
            self.start_btn.setEnabled(self.serial.is_open)
            return

        self._auto_apply_timer.stop()
        if not self._apply_params():
            self.start_btn.setEnabled(self.serial.is_open)
            return
        time.sleep(0.05)
        self._log("→ START", COLORS["green"])
        if self._start_generation():
            p = self._get_params()
            if not self._begin_lsl_recording(
                step_index=0,
                dac0_f1=p["dac0_f1"],
                dac0_f2=p["dac0_f2"],
                dac1_f1=p["dac1_f1"],
                dac1_f2=p["dac1_f2"],
            ):
                self._send("STOP")
                self.start_btn.setEnabled(self.serial.is_open)
                return
            self._running = True
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.status_bar.showMessage("▶ Выполняется")
            self._log("▶ Выполняется", COLORS["green"])
            self._send("DBG ON")
        else:
            self.start_btn.setEnabled(self.serial.is_open)

    def _start_generation(self) -> bool:
        """Отправляет START и убеждается, что генерация действительно идёт.

        Ответ прошивки может потеряться или затеряться в потоке телеметрии.
        Это не повод прерывать эксперимент: если подтверждения нет, мы
        спрашиваем у прошивки STATUS и проверяем признак «run».
        """
        for attempt in range(3):
            resp = self._send("START")
            if resp and resp.startswith("OK"):
                return True
            if attempt < 2:
                time.sleep(0.05)
        running = self._firmware_running()
        if running is True:
            self._log(
                "Ответ на START не получен, но генерация запущена "
                "(проверено по STATUS).",
                COLORS["yellow"],
            )
            return True
        self._log(
            "Не удалось подтвердить запуск генерации (нет ответа на START).",
            COLORS["red"],
        )
        return False

    def _firmware_running(self) -> bool | None:
        """Спрашивает у прошивки, идёт ли генерация. None — связи нет."""
        data = self.serial.status()
        if not data:
            return None
        run = data.get("run")
        if run is None:
            return None
        return bool(run)

    def _start_sequence(self, force_gaze: bool = False):
        if not self.serial.is_open or not self.sequence_steps:
            self._log(
                "Старт последовательности невозможен: нет подключения к Arduino "
                "или не загружен файл последовательности.",
                COLORS["red"],
            )
            self.start_btn.setEnabled(self.serial.is_open)
            return
        if self._gaze_off and not self._ignore_gaze() and not force_gaze:
            self._log("Нельзя начать: взгляд смещён. Скорректируйте взгляд.", COLORS["red"])
            self.status_bar.showMessage("Скорректируйте взгляд перед стартом")
            self.start_btn.setEnabled(self.serial.is_open)
            return
        if not self._validate_lsl_recording_ready():
            self.start_btn.setEnabled(self.serial.is_open)
            return

        self._auto_apply_timer.stop()
        self._sequence_index = 0
        self._pause_step_index = -1
        self._pause_remaining_seconds = 0.0
        self._sequence_override_seconds = None
        self._resuming_after_gaze_pause = False
        self._pause_started_at = None
        self._sequence_stage_deadline = None
        self._experiment_finished = False
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

        # Защита от досрочного перехода: если таймер сработал раньше конца
        # этапа (устаревшее событие после паузы/возобновления), просто
        # перевзводим его на остаток, а не начинаем следующий этап.
        if self._sequence_stage_deadline is not None:
            remaining = self._sequence_stage_deadline - time.monotonic()
            if remaining > 0.05:
                self._sequence_timer.start(max(1, int(remaining * 1000)))
                return
            self._sequence_stage_deadline = None

        if self._sequence_index >= len(self.sequence_steps):
            self._log(
                f"Все этапы выполнены ({len(self.sequence_steps)}/"
                f"{len(self.sequence_steps)}) — завершение эксперимента.",
                COLORS["green"],
            )
            self._finish_sequence()
            return

        step = self.sequence_steps[self._sequence_index]
        step_no = self._sequence_index + 1
        total = len(self.sequence_steps)

        effective_seconds = step.seconds
        if self._sequence_override_seconds is not None:
            effective_seconds = self._sequence_override_seconds
            self._sequence_override_seconds = None

        self._log(
            f"[SEQ {step_no}/{total}] "
            f"DAC0=({step.dac0_f1:.4f}, {step.dac0_f2:.4f}) Гц  "
            f"DAC1=({step.dac1_f1:.4f}, {step.dac1_f2:.4f}) Гц  "
            f"длительность={effective_seconds:.3f} с",
            COLORS["accent"],
        )

        needs_start = self._sequence_index == 0 or self._resuming_after_gaze_pause
        if needs_start:
            ok = self._apply_sequence_step_params(
                step,
                include_static=True,
                header="Применение параметров последовательности…",
            )
            if not ok:
                self._abort_sequence("serial-порт не подключен")
                return
            time.sleep(0.05)
            self._log("→ START", COLORS["green"])
            # Ответ на START мог потеряться в потоке телеметрии — пробуем
            # ещё раз, прежде чем прерывать эксперимент. Повторный START
            # безопасен: прошивка просто перезапустит генерацию. Если ответ
            # не пришёл совсем, проверяем у прошивки признак «run» — сам
            # по себе потерянный ответ не повод ронять эксперимент.
            if not self._start_generation():
                self._abort_sequence("команда START не подтверждена")
                return
            if not self._recording_active:
                if not self._begin_lsl_recording(
                    step_index=step_no,
                    dac0_f1=step.dac0_f1,
                    dac0_f2=step.dac0_f2,
                    dac1_f1=step.dac1_f1,
                    dac1_f2=step.dac1_f2,
                ):
                    self._abort_sequence("LSL-запись не началась")
                    return
            else:
                self._set_current_stim(
                    step_no,
                    step.dac0_f1,
                    step.dac0_f2,
                    step.dac1_f1,
                    step.dac1_f2,
                )
            self._send("DBG ON")
            self._resuming_after_gaze_pause = False
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

        self._pause_step_index = -1
        self._begin_sequence_stage_progress(step_no, total, effective_seconds)
        self._sequence_index += 1
        self._sequence_stage_deadline = time.monotonic() + float(effective_seconds)
        self._sequence_timer.start(max(1, int(effective_seconds * 1000)))

    def _finish_sequence(self):
        # Эксперимент завершён: помечаем ДО остановки, потому что остановка
        # открывает модальное окно сохранения, в котором крутится цикл
        # событий — логика взгляда не должна успеть «возобновить» прогон.
        self._experiment_finished = True
        self._log("Последовательность завершена.", COLORS["green"])
        self._stop()
        self._finish_sequence_progress()
        self.status_bar.showMessage("Последовательность завершена")

    def _abort_sequence(self, reason: str):
        # Эксперимент прерван: снимаем «идёт» до модального окна экспорта,
        # чтобы логика взгляда не перезапустила прерванный прогон.
        self._running = False
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
        self._pause_step_index = -1
        self._pause_remaining_seconds = 0.0
        self._sequence_override_seconds = None
        self._paused_by_gaze = False
        self._resuming_after_gaze_pause = False
        self._pause_started_at = None
        self._sequence_stage_deadline = None
        self.start_btn.setEnabled(self.serial.is_open)
        self.stop_btn.setEnabled(False)
        self.sequence_load_btn.setEnabled(True)
        self.sequence_clear_btn.setEnabled(bool(self.sequence_steps))
        self.status_bar.showMessage(f"Последовательность прервана: {reason}")

    def _stop(self, checked=False, export: bool = True, reason: str = "Остановлено"):
        # Прогон считается остановленным СРАЗУ, до модального окна
        # сохранения: окно живёт в отдельном цикле событий, и пока флаг
        # «идёт» стоял, логика взгляда успевала поставить паузу и затем
        # возобновить уже завершённый эксперимент — приложение снова и
        # снова просило сохранить результаты вместо завершения.
        self._running = False
        try:
            self._sequence_timer.stop()
            self._auto_apply_timer.stop()
            self._pause_step_index = -1
            self._pause_remaining_seconds = 0.0
            self._sequence_override_seconds = None
            self._paused_by_gaze = False
            self._resuming_after_gaze_pause = False
            if self._calibration_in_progress:
                # Стоп прерывает и калибровку: иначе флаг остаётся включённым
                # и все следующие «Старт» молча упираются в «Калибровка уже
                # выполняется» до перезапуска программы.
                self._calibration_in_progress = False
                self._calibration_phase_collecting = False
                self._calibration_after = None
                self._log("Калибровка прервана остановкой.", COLORS["yellow"])
            if self._waiting_gaze_center:
                self._waiting_gaze_center = False
                self._log("Автостарт после калибровки отменён.", COLORS["yellow"])
            if self.sequence_steps:
                self._stop_sequence_progress()
            self._log("→ STOP", COLORS["red"])
            if self.serial.is_open:
                self._send("STOP")
            if self._recording_active or self.lsl_worker is not None:
                self._stop_lsl_recording(export=export)
        except Exception as e:
            # Остановка проходит всегда: сбой экспорта/воркера/сериала не должен
            # блокировать запуск следующего эксперимента.
            self._log(f"Ошибка при остановке: {e}", COLORS["red"])
            self.status_bar.showMessage(f"Ошибка при остановке: {e}")
            self.lsl_worker = None
            self._recording_active = False
        finally:
            # Гарантированный сброс состояния — даже если что-то выше упало.
            self._running = False
            self._paused_by_gaze = False
            self._resuming_after_gaze_pause = False
            self._calibration_in_progress = False
            self._calibration_phase_collecting = False
            self._calibration_after = None
            self._waiting_gaze_center = False
            self._start_now_once = False
            self._sequence_index = 0
            self._pause_started_at = None
            self._sequence_stage_deadline = None
            self.start_btn.setEnabled(self.serial.is_open)
            self.stop_btn.setEnabled(False)
            self.sequence_load_btn.setEnabled(True)
            self.sequence_clear_btn.setEnabled(bool(self.sequence_steps))
            self.status_bar.showMessage(f"■ {reason}")
            self._log(f"■ {reason}", COLORS["red"])
            try:
                if self.serial.is_open:
                    self._send("DBG OFF")
            except Exception:
                pass

    # ── Connected slot ────────────────────────────────────────────────────────
    @Slot(bool)
    def _on_connected(self, connected: bool):
        if connected:
            self._set_serial_checkbox(True)
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)
            # ИК-датчик (TCRT5000) включён по умолчанию при подключении.
            self._send("IR ON")
            # DBG-телеметрия включена по умолчанию при подключении.
            self._send("DBG ON")
            self._auto_apply_timer.start(200)
            QTimer.singleShot(400, self._apply_calibration_to_firmware)
        else:
            if self._running or self._recording_active or self._waiting_gaze_center:
                self._waiting_gaze_center = False
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