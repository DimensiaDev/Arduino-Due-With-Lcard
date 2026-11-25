"""Загрузка файла последовательности и прогон шагов."""

import re, time
from pathlib import Path
from PySide6.QtCore import Qt
from PySide6.QtGui import QColor
from PySide6.QtWidgets import QFileDialog, QTableWidgetItem
from dualsine.config import COLORS
from dualsine.core.sequence import SequenceStep


class SequenceMixin:
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


    def _start_sequence(self):
        if not self.serial.is_open or not self.sequence_steps:
            self._log(
                "Старт последовательности невозможен: нет подключения к Arduino "
                "или не загружен файл последовательности.",
                COLORS["red"],
            )
            self.start_btn.setEnabled(self.serial.is_open)
            return
        if not self._validate_lsl_recording_ready():
            self.start_btn.setEnabled(self.serial.is_open)
            return

        self._auto_apply_timer.stop()
        self._sequence_index = 0
        self._sequence_stage_deadline = None
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

        self._log(
            f"[SEQ {step_no}/{total}] "
            f"DAC0=({step.dac0_f1:.4f}, {step.dac0_f2:.4f}) Гц  "
            f"DAC1=({step.dac1_f1:.4f}, {step.dac1_f2:.4f}) Гц  "
            f"длительность={effective_seconds:.3f} с",
            COLORS["accent"],
        )

        needs_start = self._sequence_index == 0
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

        self._begin_sequence_stage_progress(step_no, total, effective_seconds)
        self._sequence_index += 1
        self._sequence_stage_deadline = time.monotonic() + float(effective_seconds)
        self._sequence_timer.start(max(1, int(effective_seconds * 1000)))


    def _finish_sequence(self):
        self._log("Последовательность завершена.", COLORS["green"])
        self._stop()
        self._finish_sequence_progress()
        self.status_bar.showMessage("Последовательность завершена")


    def _abort_sequence(self, reason: str):
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
        self._sequence_stage_deadline = None
        self.start_btn.setEnabled(self.serial.is_open)
        self.stop_btn.setEnabled(False)
        self.sequence_load_btn.setEnabled(True)
        self.sequence_clear_btn.setEnabled(bool(self.sequence_steps))
        self.status_bar.showMessage(f"Последовательность прервана: {reason}")


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
