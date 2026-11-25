"""Константы приложения, цветовая палитра и QSS-стиль."""

from PySide6.QtWidgets import QMainWindow, QWidget, QLabel, QDoubleSpinBox, QSpinBox, QPushButton, QComboBox, QGroupBox, QStatusBar, QFrame, QCheckBox, QTextEdit, QLineEdit, QTabWidget, QProgressBar, QTableWidget, QHeaderView, QAbstractItemView


ADC_MAX_VALUE = 4095

INVERT_SENSOR_RAW = True

GAZE_PROJECTION_ON = 0.5
GAZE_PROJECTION_OFF = 0.25
GAZE_PROJECTION_MIN_SEP_RAW = 40.0

GAZE_PROJECTION_MIN_SEP_SIGMA = 2.5
MIN_CALIBRATION_SEPARATION = 80.0

IR_MEDIAN_WINDOW = 5

IR_DEBOUNCE_COUNT = 3

MIN_SENSOR_SCALE = 30.0

CALIBRATION_MIN_SAMPLES = 8
CALIBRATION_MAX_STD = 250.0
CALIBRATION_MAX_RETRIES = 2

CALIBRATION_SETTLE_SECONDS = 1.0

CALIBRATION_FORWARD_TOLERANCE = 0.3

GAZE_FALLBACK_SIGMA = 3.0

IR_STALE_SECONDS = 3.0

LSL_RESOLVE_WAIT = 2.5
LSL_RESOLVE_ATTEMPTS = 3
LSL_RESOLVE_WAIT_BG = 1.5
LSL_AUTO_REFRESH_MS = 5000

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
