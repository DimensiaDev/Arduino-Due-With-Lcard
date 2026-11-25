"""Определение направления взгляда по IR-датчикам и индикация."""

import time
import numpy as np
from dualsine.config import COLORS, GAZE_FALLBACK_SIGMA, GAZE_PROJECTION_MIN_SEP_RAW, GAZE_PROJECTION_MIN_SEP_SIGMA, GAZE_PROJECTION_OFF, GAZE_PROJECTION_ON, IR_DEBOUNCE_COUNT, IR_STALE_SECONDS, MIN_SENSOR_SCALE
from dualsine.core.utils import gaze_position_label


class GazeMixin:
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

            tail = " (эксперимент продолжается)"
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
                    target = self._fallback_eye_target(idx, float(raw_val), s)
            else:
                target = self._fallback_eye_target(idx, float(raw_val), s)

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

        pos_list = [
            self._sensor_eye_pos[idx]
            for idx, st in enumerate(states)
            if st["available"]
        ]
        has_left = "left" in pos_list
        has_right = "right" in pos_list
        if has_left and has_right:
            joint = "off"
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

        if current in scores and scores[current] >= GAZE_PROJECTION_OFF:
            return current
        return "forward"


    def _fallback_eye_target(self, idx: int, raw_val: float, s: dict) -> str:
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

        return "forward" if bool(s.get("reflect", True)) else "off"


    def _sensor_scale(self, idx: int) -> float | None:
        cal = self.ir_calibration if isinstance(self.ir_calibration, dict) else {}
        scale = cal.get("scale")
        if isinstance(scale, dict):
            value = scale.get(f"s{idx}", np.nan)
            if isinstance(value, (int, float)) and np.isfinite(value) and value > 0:
                return float(value)
        return None


    def _position_from_eye_states(self, states: list[dict]) -> str:
        for st in states:
            if st.get("available") and st.get("deviated"):
                direction = st.get("direction")
                return direction if direction in ("left", "right") else "off"
        return "forward"
