"""Сбор IR-данных, калибровка датчиков и её сохранение."""

import json, time
from datetime import datetime
import numpy as np
try:
    import pylsl
    from pylsl import StreamInlet, local_clock, resolve_streams

    LSL_IMPORT_ERROR = None
except Exception as e:
    pylsl = None
    StreamInlet = None
    local_clock = None
    resolve_streams = None
    LSL_IMPORT_ERROR = e
from PySide6.QtCore import QTimer
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QMessageBox
from dualsine.config import CALIBRATION_FORWARD_TOLERANCE, CALIBRATION_MAX_RETRIES, CALIBRATION_MAX_STD, CALIBRATION_MIN_SAMPLES, CALIBRATION_SETTLE_SECONDS, COLORS, GAZE_PROJECTION_MIN_SEP_RAW, GAZE_PROJECTION_MIN_SEP_SIGMA, INVERT_SENSOR_RAW, IR_STALE_SECONDS, MIN_CALIBRATION_SEPARATION, MIN_SENSOR_SCALE
from dualsine.core.utils import _robust_median, _robust_sigma, invert_adc


class CalibrationMixin:
    def _update_calibration_progress_status(self):
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

            raw_val = s.get("raw", np.nan)
            history = self._ir_raw_history[sensor_id]
            if isinstance(raw_val, (int, float)) and np.isfinite(raw_val):
                if prev_update and (now - prev_update) > IR_STALE_SECONDS:
                    history.clear()
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

        self._update_gaze_indicator_with_states(states)


    def _get_current_ir(self) -> dict:
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

            threshold_inv = int(round((fwd_val + off_val) / 2.0))

            threshold_fw = int(round(invert_adc(threshold_inv)))
            hysteresis = max(80, int(round(abs(off_val - fwd_val) * 0.2)))
            self._send(f"SET IRTH{idx} {threshold_fw}")
            self._send(f"SET IRHY{idx} {hysteresis}")
        self._log("Пороги прошивки обновлены (только для отображения).", COLORS["green"])


    def _start_calibration(self, after=None) -> bool:
        if self._calibration_in_progress:
            return False
        if not self.serial.is_open:
            self._log(
                "Калибровка невозможна: нет подключения к Arduino по Serial.",
                COLORS["red"],
            )
            self.status_bar.showMessage("Нет подключения к Arduino для калибровки")
            return False
        if self._running:
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
        self._calibration_in_progress = False
        self._calibration_phase_collecting = False
        self._calibration_after = None
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

        scale: dict = {}
        for idx, key in ((0, "sensor0"), (1, "sensor1")):
            stds = [self._phase_stds(i)[0 if idx == 0 else 1] for i in range(3)]
            finite = [s for s in stds if s is not None and np.isfinite(s)]
            if finite:
                sigma = float(np.median(finite))
                if np.isfinite(sigma) and sigma > 0:
                    scale[f"s{idx}"] = max(sigma, MIN_SENSOR_SCALE)

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

        geometry_problems: list[str] = []

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

            QTimer.singleShot(700, after)
        else:
            self.status_bar.showMessage("Калибровка завершена")


    def _snapshot_calibration(self) -> dict:
        cal = self.ir_calibration if isinstance(self.ir_calibration, dict) else {}
        snapshot: dict = {}
        for name, point in cal.items():
            if not isinstance(point, dict):
                continue
            values = {
                key: float(value)
                for key, value in point.items()
                if isinstance(value, (int, float)) and np.isfinite(value)
            }
            if values:
                snapshot[name] = values
        return snapshot


    def _calibration_export_rows(self) -> list[tuple[str, str, str, float]]:
        cal = self._record_calibration
        if not isinstance(cal, dict) or not cal:
            cal = self._snapshot_calibration()
        rows: list[tuple[str, str, str, float]] = []
        for phase in ("left", "forward", "right"):
            point = cal.get(phase)
            if not isinstance(point, dict):
                continue
            for side, idx in (("RIGHT", 0), ("LEFT", 1)):
                value = point.get(f"s{idx}")
                if isinstance(value, (int, float)) and np.isfinite(value):
                    rows.append((phase, side, str(idx), float(value)))
        scale = cal.get("scale")
        if isinstance(scale, dict):
            for side, idx in (("RIGHT", 0), ("LEFT", 1)):
                value = scale.get(f"s{idx}")
                if isinstance(value, (int, float)) and np.isfinite(value):
                    rows.append(("sigma", side, str(idx), float(value)))
        return rows
