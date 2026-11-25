"""LSL-запись, экспорт данных и статистика сигнала."""

import json, time
from datetime import datetime
from pathlib import Path
import numpy as np
try:
    import pandas as pd

    PANDAS_IMPORT_ERROR = None
except Exception as e:
    pd = None
    PANDAS_IMPORT_ERROR = e
try:
    from scipy.signal import butter, filtfilt, iirnotch, lfilter, sosfilt, sosfiltfilt

    SCIPY_IMPORT_ERROR = None
except Exception as e:
    butter = filtfilt = iirnotch = lfilter = sosfilt = sosfiltfilt = None
    SCIPY_IMPORT_ERROR = e
from PySide6.QtCore import Slot
from PySide6.QtWidgets import QFileDialog
from dualsine.config import COLORS, INVERT_SENSOR_RAW
from dualsine.core.lsl_recording_worker import LSLRecordingWorker
from dualsine.core.utils import safe_column_name, unique_column_names


class RecordingMixin:
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

        self._record_calibration = self._snapshot_calibration()
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

            "ir_raw0": [r.get("ir_raw0", np.nan) for r in records],
            "ir_raw1": [r.get("ir_raw1", np.nan) for r in records],
            "ir_eye0": [r.get("ir_eye0", "forward") for r in records],
            "ir_eye1": [r.get("ir_eye1", "forward") for r in records],

            "LEFT": [r.get("ir_raw1", np.nan) for r in records],
            "RIGHT": [r.get("ir_raw0", np.nan) for r in records],
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
            "ir_calibration_points_json": self._json_value(
                self._record_calibration or self._snapshot_calibration()
            ),
            "ir_invert_sensor_raw": bool(INVERT_SENSOR_RAW),
            "ir_gaze_pauses_experiment": False,
            "ir_skip_calibration": bool(self._skip_calibration()),
            "ir_photodiode_columns": (
                "LEFT — показания левого фотодиода TCRT5000 (датчик 1, A3); "
                "RIGHT — правого (датчик 0, A2). Значения — raw АЦП в шкале "
                "приложения: после инверсии меньше = белок, больше = зрачок. "
                "Калибровка этих датчиков — в блоке # CALIBRATION в начале файла."
            ),
            "ir_classification_note": (
                "ir_gaze_position is classified in Python per sensor (nearest-neighbor "
                "against ir_calibration_points_json, distances normalized by per-sensor "
                "sigma from 'scale' when present; sliding-median smoothed input, "
                "state-change debounce, conjugate-gaze propagation to both eyes). "
                "'forward' means gaze OK; 'left'/'right'/'off' are informational "
                "only — gaze deviation NEVER pauses the experiment and does not "
                "block start. ir_skip_calibration means start skipped gaze "
                "calibration. "
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

            f.write("\n# CALIBRATION\n")
            cal_rows = self._calibration_export_rows()
            if cal_rows:
                f.write("phase\tside\tsensor\traw\n")
                for phase, side, sensor, value in cal_rows:
                    f.write(f"{phase}\t{side}\t{sensor}\t{value:.1f}\n")
            else:
                f.write("none\n")
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

        for label, text in values:
            if label.text() != text:
                label.setText(text)
