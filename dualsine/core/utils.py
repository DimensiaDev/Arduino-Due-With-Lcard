"""Чистые вспомогательные функции (без привязки к UI)."""

import re
import numpy as np
from PySide6.QtWidgets import QDoubleSpinBox
from dualsine.config import ADC_MAX_VALUE, INVERT_SENSOR_RAW


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
    if not np.isfinite(value):
        return value
    if INVERT_SENSOR_RAW:
        return float(ADC_MAX_VALUE) - float(value)
    return float(value)


def _robust_sigma(values) -> float | None:
    vals = np.asarray([float(v) for v in values if np.isfinite(v)], dtype=float)
    if vals.size < 2:
        return None
    med = float(np.median(vals))
    mad = float(np.median(np.abs(vals - med)))
    return float(1.4826 * mad)


def _robust_median(values, trim_sigma: float = 3.0, min_abs: float = 30.0):
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
