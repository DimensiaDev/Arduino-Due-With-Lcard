"""Фоновый поток записи сэмплов из LSL-потока."""

import threading, time
import numpy as np


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
