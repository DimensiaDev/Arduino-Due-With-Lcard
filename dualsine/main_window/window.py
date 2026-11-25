"""Главное окно приложения — собирается из миксинов-«частей»."""

import threading
from collections import deque
import numpy as np
from PySide6.QtCore import QSettings, QTimer, Signal
from PySide6.QtWidgets import QMainWindow
from dualsine.config import IR_MEDIAN_WINDOW, LSL_AUTO_REFRESH_MS
from dualsine.core.lsl_recording_worker import LSLRecordingWorker
from dualsine.core.sequence import SequenceStep
from dualsine.core.serial_worker import SerialWorker
from dualsine.main_window.mixins.ui_build import UiBuildMixin
from dualsine.main_window.mixins.connections import ConnectionMixin
from dualsine.main_window.mixins.sequence import SequenceMixin
from dualsine.main_window.mixins.gaze import GazeMixin
from dualsine.main_window.mixins.calibration import CalibrationMixin
from dualsine.main_window.mixins.recording import RecordingMixin
from dualsine.main_window.mixins.control import ControlMixin


class MainWindow(
        UiBuildMixin,
        ConnectionMixin,
        SequenceMixin,
        GazeMixin,
        CalibrationMixin,
        RecordingMixin,
        ControlMixin,
        QMainWindow,
):
    _console_append = Signal(str, str)
    _lsl_worker_error = Signal(str)

    _lsl_streams_found = Signal(object, bool)


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
        self._lsl_streams_found.connect(self._on_lsl_streams_found)
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

        self._sequence_stage_deadline: float | None = None

        self._gaze_off = False
        self._current_gaze_pos = "forward"

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

        self._ir_raw_history = [
            deque(maxlen=IR_MEDIAN_WINDOW),
            deque(maxlen=IR_MEDIAN_WINDOW),
        ]
        self._sensor_eye_pending: list[tuple[str, int] | None] = [None, None]
        self._gaze_last_seen = [0.0, 0.0]
        self._ir_table_max_rows = 100

        self.ir_calibration: dict = {}
        self._calibration_in_progress = False
        self._calibration_phase = 0
        self._calibration_phase_collecting = False

        self._calibration_phase_started_at = 0.0

        self._calibration_after = None

        self._calibration_retries = 0
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

        self._lsl_resolving = False
        self._lsl_auto_connect = True
        self._lsl_hint_logged = False
        self.lsl_worker: LSLRecordingWorker | None = None
        self.lsl_samples: list[dict] = []
        self.lsl_sample_lock = threading.Lock()

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

        self._record_calibration: dict = {}

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

        self._lsl_auto_timer = QTimer(self)
        self._lsl_auto_timer.setInterval(LSL_AUTO_REFRESH_MS)
        self._lsl_auto_timer.timeout.connect(self._auto_refresh_lsl_streams)
        self._lsl_auto_timer.start()

        self._total_clip_hi = [0, 0]
        self._total_clip_lo = [0, 0]
        QTimer.singleShot(0, self._attempt_initial_connections)


    def _on_lsl_combo_changed(self, idx: int):
        self._update_lsl_stream_preview()
        if self.lsl_inlet is not None:
            self._connect_selected_lsl_stream()


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
