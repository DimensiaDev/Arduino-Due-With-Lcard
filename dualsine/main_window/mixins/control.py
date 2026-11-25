"""Параметры, команды прошивке, старт/остановка эксперимента."""

import time
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
from dualsine.config import COLORS
from dualsine.core.sequence import SequenceStep


class ControlMixin:
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


    def _get_filter_config(self) -> dict:
        return {
            "bandpass_enabled": True,
            "bandpass_low_hz": 0.5,
            "bandpass_high_hz": 40.0,
            "bandpass_order": 4,
            "notch_enabled": True,
            "notch_hz": 50.0,
            "notch_q": 30.0,
        }


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
        if self._calibration_in_progress:
            self._log("Калибровка уже выполняется.", COLORS["yellow"])
            return
        if not self.serial.is_open:
            self._log("Нельзя начать: нет подключения к Arduino.", COLORS["red"])
            self.status_bar.showMessage("Подключитесь к Arduino перед стартом")
            return

        if self._running:
            self._stop(reason="Запуск нового эксперимента")

        self.start_btn.setEnabled(False)

        if self._skip_calibration():
            self._log(
                "Калибровка пропущена: используются сохранённые пороги "
                "или значения по умолчанию.",
                COLORS["yellow"],
            )
            self._start_experiment()
            return

        if not self._start_calibration(after=self._start_experiment):
            self.start_btn.setEnabled(self.serial.is_open)


    def _start_experiment(self):
        self._sequence_stage_deadline = None

        if self.sequence_steps:
            self._start_sequence()
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
        data = self.serial.status()
        if not data:
            return None
        run = data.get("run")
        if run is None:
            return None
        return bool(run)


    def _stop(self, checked=False, export: bool = True, reason: str = "Остановлено"):
        self._running = False
        try:
            self._sequence_timer.stop()
            self._auto_apply_timer.stop()
            if self._calibration_in_progress:
                self._calibration_in_progress = False
                self._calibration_phase_collecting = False
                self._calibration_after = None
                self._log("Калибровка прервана остановкой.", COLORS["yellow"])
            if self.sequence_steps:
                self._stop_sequence_progress()
            self._log("→ STOP", COLORS["red"])
            if self.serial.is_open:
                self._send("STOP")
            if self._recording_active or self.lsl_worker is not None:
                self._stop_lsl_recording(export=export)
        except Exception as e:
            self._log(f"Ошибка при остановке: {e}", COLORS["red"])
            self.status_bar.showMessage(f"Ошибка при остановке: {e}")
            self.lsl_worker = None
            self._recording_active = False
        finally:
            self._running = False
            self._calibration_in_progress = False
            self._calibration_phase_collecting = False
            self._calibration_after = None
            self._sequence_index = 0
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
