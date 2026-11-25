"""Построение всех карточек/вкладок главного окна."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QSpinBox, QPushButton, QComboBox, QStatusBar, QFrame, QCheckBox, QTextEdit, QLineEdit, QTabWidget, QProgressBar, QScrollArea, QSizePolicy, QTableWidget, QHeaderView, QAbstractItemView
from dualsine.config import COLORS, INVERT_SENSOR_RAW
from dualsine.core.utils import make_spinbox
from dualsine.ui.gaze_eyes_widget import GazeEyesWidget


class UiBuildMixin:
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
        self.lsl_stream_combo.addItem("(ищем LSL-потоки…)")

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
        gaze_hint = QLabel(
            "Отведение взгляда не прерывает эксперимент: направление взгляда "
            "показывается в индикаторе и пишется в файл эксперимента "
            "(колонки LEFT/RIGHT)."
        )
        gaze_hint.setObjectName("hint_label")
        gaze_hint.setWordWrap(True)
        gaze_layout.addWidget(gaze_hint)

        self.skip_calibration_check = QCheckBox("Пропустить калибровку")
        self.skip_calibration_check.setToolTip(
            "«Старт» сразу запускает эксперимент без калибровки взгляда.\n"
            "Классификация использует сохранённую калибровку, а если её нет — "
            "пороги по умолчанию (порог АЦП прошивки и признак «виден зрачок»)."
        )
        gaze_layout.addWidget(self.skip_calibration_check)

        self.skip_calibration_check.setChecked(
            bool(self.settings.value("service/skip_calibration", False, type=bool))
        )
        self.skip_calibration_check.toggled.connect(self._on_skip_calibration_toggled)
        v.addWidget(gaze_box)

        console_box, console_layout = self._make_subcard("Журнал", COLORS["accent"])
        console_layout.addWidget(self.console, 1)
        v.addWidget(console_box, 1)
        return w


    def _skip_calibration(self) -> bool:
        return (
            hasattr(self, "skip_calibration_check")
            and self.skip_calibration_check.isChecked()
        )


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
