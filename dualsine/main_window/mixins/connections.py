"""Подключение COM-порта и LSL-потока, консольный лог."""

import json, socket, threading, time
import numpy as np
import serial.tools.list_ports
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
from PySide6.QtCore import QTimer, Slot
from PySide6.QtGui import QColor, QTextCursor
from PySide6.QtWidgets import QApplication, QCheckBox
from dualsine.config import COLORS, INVERT_SENSOR_RAW, LSL_RESOLVE_ATTEMPTS, LSL_RESOLVE_WAIT, LSL_RESOLVE_WAIT_BG
from dualsine.core.utils import invert_adc


class ConnectionMixin:
    def _set_status_indicator(self, widget: QCheckBox, state: str, tooltip: str):
        color = {
            "disconnected": COLORS["red"],
            "connecting": COLORS["yellow"],
            "connected": COLORS["green"],
        }[state]
        widget.blockSignals(True)
        widget.setChecked(state == "connected")
        widget.setToolTip(tooltip)
        widget.blockSignals(False)
        widget.setStyleSheet(
            f"QCheckBox#{widget.objectName()} {{ spacing: 0px; }}"
            f"QCheckBox#{widget.objectName()}::indicator {{"
            f" width: 14px; height: 14px;"
            f" border: 1px solid {color};"
            f" border-radius: 3px;"
            f" background: {color};"
            f" }}"
        )


    def _set_serial_indicator(self, state: str):
        tooltips = {
            "disconnected": "Устройство не подключено",
            "connecting": "Подключение к устройству...",
            "connected": "Устройство подключено",
        }
        self._set_status_indicator(self.serial_connect_check, state, tooltips[state])


    def _set_lsl_indicator(self, state: str):
        tooltips = {
            "disconnected": "LSL не подключен",
            "connecting": "Подключение к LSL...",
            "connected": "LSL подключен",
        }
        self._set_status_indicator(self.lsl_connect_check, state, tooltips[state])


    def _valid_serial_port_selected(self) -> bool:
        port = self.port_combo.currentText()
        return bool(port) and "no ports" not in port and "порты не найдены" not in port


    def _valid_lsl_stream_selected(self) -> bool:
        idx = self.lsl_stream_combo.currentIndex()
        return 0 <= idx < len(self.lsl_streams)


    def _save_serial_selection(self, port: str):
        self.settings.setValue("serial/port", port)


    def _restore_serial_selection(self):
        saved_port = str(self.settings.value("serial/port", "", type=str))
        if not saved_port:
            return
        idx = self.port_combo.findText(saved_port)
        if idx >= 0:
            self.port_combo.setCurrentIndex(idx)


    def _lsl_stream_key_from_meta(self, meta: dict) -> str:
        return meta.get("source_id") or meta.get("uid") or "|".join(
            [
                str(meta.get("name", "")),
                str(meta.get("type", "")),
                str(meta.get("channel_count", 0)),
                str(meta.get("nominal_srate_hz", 0.0)),
            ]
        )


    def _save_lsl_selection(self, idx: int):
        if idx < 0 or idx >= len(self.lsl_streams):
            return
        meta = self._read_lsl_stream_meta(self.lsl_streams[idx])
        self.settings.setValue("lsl/stream_key", self._lsl_stream_key_from_meta(meta))

        self.settings.setValue("lsl/stream_pref", self._lsl_stream_pref_key(meta))


    def _restore_lsl_selection(self) -> int:
        saved_key = str(self.settings.value("lsl/stream_key", "", type=str))
        if saved_key:
            for idx, info in enumerate(self.lsl_streams):
                meta = self._read_lsl_stream_meta(info)
                if self._lsl_stream_key_from_meta(meta) == saved_key:
                    self.lsl_stream_combo.setCurrentIndex(idx)
                    return idx

        idx = self._find_lsl_by_pref(
            str(self.settings.value("lsl/stream_pref", "", type=str))
        )
        if idx >= 0:
            self.lsl_stream_combo.setCurrentIndex(idx)
        return idx


    def _attempt_initial_connections(self):
        if not self.serial.is_open and self._valid_serial_port_selected():
            self._connect_selected_serial_port()
        if self.lsl_inlet is None and self._valid_lsl_stream_selected():
            self._connect_selected_lsl_stream()


    def _refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)
        if not ports:
            self.port_combo.addItem("(порты не найдены)")
        self._restore_serial_selection()


    def _connect_selected_serial_port(self, idx: int | None = None):
        port = self.port_combo.currentText()
        if not port or "no ports" in port or "порты не найдены" in port:
            self._log("Serial-порт не выбран.", COLORS["yellow"])
            self._set_serial_checkbox(False)
            return

        if self.serial.is_open:
            if self._running or self._recording_active:
                self._stop(reason="Serial переключен")
            else:
                self.serial.close()

        self._save_serial_selection(port)
        self._set_serial_indicator("connecting")
        self.status_bar.showMessage(f"Подключение к {port} …")
        self._log(f"Подключение к {port} @ 115200 …", COLORS["accent"])
        QApplication.processEvents()
        ok = self.serial.open(port)
        if ok:
            self.status_bar.showMessage(f"Подключено  ·  {port}  @  115200 бод")
            self._log(f"Подключено к {port}.", COLORS["green"])
        else:
            self._set_serial_checkbox(False)


    def _refresh_lsl_streams(self, user_initiated: bool = True):
        if self._lsl_resolving:
            return

        if resolve_streams is None:
            msg = f"pylsl недоступен: {LSL_IMPORT_ERROR}"
            self.lsl_stream_combo.clear()
            self.lsl_stream_combo.addItem("(pylsl недоступен)")
            self.lsl_status_label.setText(msg)
            self._log(msg, COLORS["red"])
            self._set_lsl_checkbox(False, enabled=False)
            return

        self._lsl_resolving = True
        if user_initiated:
            self.lsl_status_label.setText("Поиск LSL-потоков...")
            QApplication.processEvents()

        def worker():
            attempts = LSL_RESOLVE_ATTEMPTS if user_initiated else 1
            wait = LSL_RESOLVE_WAIT if user_initiated else LSL_RESOLVE_WAIT_BG
            found = []
            for _attempt in range(attempts):
                try:
                    found = resolve_streams(wait_time=wait) or []
                except Exception:
                    found = []
                if found:
                    break
            self._lsl_streams_found.emit(found, user_initiated)

        threading.Thread(target=worker, daemon=True).start()


    @Slot(object, bool)
    def _on_lsl_streams_found(self, streams, user_initiated: bool):
        self._lsl_resolving = False

        prev_pref = self._current_lsl_pref_key()

        self.lsl_streams = list(streams or [])

        if not self.lsl_streams:
            self.lsl_stream_combo.clear()
            self.lsl_stream_combo.addItem("(LSL-потоки не найдены)")
            self.lsl_status_label.setText("LSL-потоки не найдены")
            self._set_lsl_checkbox(False, enabled=False)
            if self.lsl_inlet is not None:
                self._log(
                    "LSL-поток исчез из сети — устройство перестало публиковать. "
                    "Запись будет пустой, пока поток не появится снова.",
                    COLORS["red"],
                )
            self._log_lsl_not_found(user_initiated)
            return

        self.lsl_stream_combo.blockSignals(True)
        self.lsl_stream_combo.clear()
        for info in self.lsl_streams:
            meta = self._read_lsl_stream_meta(info)
            self.lsl_stream_combo.addItem(
                f"{meta['name']} | {meta['type']} | "
                f"{meta['channel_count']} кан. | {meta['nominal_srate_hz']} Гц"
            )
        self.lsl_stream_combo.blockSignals(False)

        idx = -1
        if prev_pref:
            idx = self._find_lsl_by_pref(prev_pref)
        if idx < 0:
            idx = self._find_lsl_by_pref(
                str(self.settings.value("lsl/stream_pref", "", type=str))
            )
        if idx < 0:
            idx = self._restore_lsl_selection()
        if idx >= 0 and idx != self.lsl_stream_combo.currentIndex():
            self.lsl_stream_combo.blockSignals(True)
            self.lsl_stream_combo.setCurrentIndex(idx)
            self.lsl_stream_combo.blockSignals(False)

        self.lsl_status_label.setText(f"Найдено LSL-потоков: {len(self.lsl_streams)}")
        connected = self.lsl_inlet is not None
        self._set_lsl_checkbox(connected, enabled=True)
        self._update_lsl_stream_preview()

        if user_initiated:
            for info in self.lsl_streams:
                meta = self._read_lsl_stream_meta(info)
                self._log(
                    f"LSL-поток: {meta['name']} | тип {meta['type']} | "
                    f"{meta['channel_count']} кан. | {meta['nominal_srate_hz']} Гц",
                    COLORS["text_dim"],
                )

        if self.lsl_inlet is not None:
            self._reconnect_if_stream_recreated()

        if (
            self.lsl_inlet is None
            and self._lsl_auto_connect
            and self._valid_lsl_stream_selected()
        ):
            self._connect_selected_lsl_stream()


    def _auto_refresh_lsl_streams(self):
        if self._lsl_resolving:
            return
        self._refresh_lsl_streams(user_initiated=False)


    def _current_lsl_pref_key(self) -> str:
        idx = self.lsl_stream_combo.currentIndex()
        if 0 <= idx < len(self.lsl_streams):
            meta = self._read_lsl_stream_meta(self.lsl_streams[idx])
            return self._lsl_stream_pref_key(meta)
        return ""


    def _lsl_stream_pref_key(self, meta: dict) -> str:
        name = str(meta.get("name", ""))
        prefix = name
        for i, ch in enumerate(name):
            if ch.isdigit():
                prefix = name[:i]
                break
        return f"{meta.get('type', '')}|{prefix.strip()}"


    def _find_lsl_by_pref(self, pref_key: str) -> int:
        if not pref_key:
            return -1
        for idx, info in enumerate(self.lsl_streams):
            meta = self._read_lsl_stream_meta(info)
            if self._lsl_stream_pref_key(meta) == pref_key:
                return idx
        return -1


    def _reconnect_if_stream_recreated(self):
        cur_uid = str(self.lsl_stream_meta.get("uid", ""))
        uids = {
            str(self._read_lsl_stream_meta(i).get("uid", ""))
            for i in self.lsl_streams
        }
        if not cur_uid or cur_uid in uids:
            return
        pref = self._lsl_stream_pref_key(self.lsl_stream_meta)
        idx = self._find_lsl_by_pref(pref)
        if idx < 0:
            self._log(
                "LSL-поток исчез из списка — устройство, похоже, остановило "
                "публикацию.",
                COLORS["yellow"],
            )
            return
        new_meta = self._read_lsl_stream_meta(self.lsl_streams[idx])
        self._log(
            f"Устройство пересоздало поток ({new_meta['name']}) — переподключаемся.",
            COLORS["yellow"],
        )
        self.lsl_stream_combo.setCurrentIndex(idx)
        self._connect_selected_lsl_stream(idx)


    def _log_lsl_not_found(self, user_initiated: bool):
        if self._lsl_hint_logged and not user_initiated:
            return
        self._lsl_hint_logged = True
        self._log(
            "LSL-потоки не найдены. Проверьте: 1) устройство реально публикует "
            "поток (запущен ли checkup на Neuron Spectrum); 2) кабель подключён "
            "и IP устройства в одной подсети с этим компьютером; 3) в брандмауэре "
            "Windows разрешён Python для частной сети (UDP 11578 и 16572-16579).",
            COLORS["yellow"],
        )
        try:
            host = socket.gethostname()
            addrs = sorted({
                info[4][0]
                for info in socket.getaddrinfo(host, None, socket.AF_INET)
            })
        except Exception:
            addrs = []
        if addrs:
            self._log(
                "   IP этого компьютера: " + ", ".join(addrs) +
                " — он должен быть в одной подсети с устройством.",
                COLORS["text_dim"],
            )


    def _update_lsl_stream_preview(self):
        idx = self.lsl_stream_combo.currentIndex()
        if idx < 0 or idx >= len(self.lsl_streams):
            return
        meta = self._read_lsl_stream_meta(self.lsl_streams[idx])
        names = self._read_lsl_channel_names(self.lsl_streams[idx], meta["channel_count"])
        self.lsl_status_label.setToolTip(
            "\n".join(
                [
                    f"имя: {meta['name']}",
                    f"тип: {meta['type']}",
                    f"source_id: {meta['source_id']}",
                    f"uid: {meta['uid']}",
                    f"каналов: {meta['channel_count']}",
                    f"частота_дискретизации_Гц: {meta['nominal_srate_hz']}",
                    f"формат_канала: {meta['channel_format']}",
                    f"имена_каналов: {', '.join(names)}",
                ]
            )
        )


    def _connect_lsl_stream(self):
        if StreamInlet is None:
            self._log(f"Не удалось подключить LSL: {LSL_IMPORT_ERROR}", COLORS["red"])
            self._set_lsl_checkbox(False, enabled=False)
            return
        idx = self.lsl_stream_combo.currentIndex()
        if idx < 0 or idx >= len(self.lsl_streams):
            self._log("Сначала выберите LSL-поток.", COLORS["yellow"])
            self._set_lsl_checkbox(False, enabled=bool(self.lsl_streams))
            return

        self._save_lsl_selection(idx)
        self._set_lsl_indicator("connecting")
        self.lsl_status_label.setText("Подключение к LSL-потоку...")
        QApplication.processEvents()
        try:
            info = self.lsl_streams[idx]
            self.lsl_inlet = StreamInlet(info, max_buflen=60)
            self.lsl_stream_meta = self._read_lsl_stream_meta(info)
            self.lsl_channel_names = self._read_lsl_channel_names(
                info, self.lsl_stream_meta["channel_count"]
            )
        except Exception as e:
            self.lsl_inlet = None
            self.lsl_stream_meta = {}
            self.lsl_channel_names = []
            self._log(f"Ошибка подключения LSL: {e}", COLORS["red"])
            self.lsl_status_label.setText(f"Ошибка подключения LSL: {e}")
            self._set_lsl_checkbox(False, enabled=True)
            return

        self._set_lsl_checkbox(True, enabled=True)
        self.lsl_refresh_btn.setEnabled(False)
        self.lsl_status_label.setText(
            f"Подключено: {self.lsl_stream_meta['name']} | "
            f"{self.lsl_stream_meta['channel_count']} кан. | "
            f"{self.lsl_stream_meta['nominal_srate_hz']} Гц"
        )
        self._log(f"LSL-поток подключен: {self.lsl_stream_meta['name']}", COLORS["green"])


    def _connect_selected_lsl_stream(self, idx: int | None = None):
        if self.lsl_inlet is not None:
            self._disconnect_lsl_stream()
        self._connect_lsl_stream()


    def _disconnect_lsl_stream(self):
        if self._recording_active or self._running:
            self._log("LSL отключен во время эксперимента.", COLORS["red"])
            self._stop(reason="LSL отключен")

        if self.lsl_inlet is not None:
            try:
                self.lsl_inlet.close_stream()
            except Exception:
                pass
        self.lsl_inlet = None
        self.lsl_stream_meta = {}
        self.lsl_channel_names = []
        self._set_lsl_checkbox(False, enabled=bool(self.lsl_streams))
        self.lsl_refresh_btn.setEnabled(True)
        self.lsl_status_label.setText("Отключено")
        self._log("LSL-поток отключен.", COLORS["text_dim"])


    def _toggle_lsl_connection(self, checked: bool):
        if checked:
            self._lsl_auto_connect = True
            self._connect_lsl_stream()
        else:
            self._lsl_auto_connect = False
            self._disconnect_lsl_stream()


    def _set_lsl_checkbox(self, checked: bool, enabled: bool = True):
        self.lsl_connect_check.setEnabled(enabled)
        self._set_lsl_indicator("connected" if checked else "disconnected")


    def _read_lsl_stream_meta(self, info) -> dict:
        def read(method_name: str, default):
            try:
                return getattr(info, method_name)()
            except Exception:
                return default

        protocol_version = ""
        library_version = ""
        if pylsl is not None:
            for attr, target in (
                ("protocol_version", "protocol_version"),
                ("library_version", "library_version"),
            ):
                try:
                    val = getattr(pylsl, attr)
                    value = val() if callable(val) else val
                except Exception:
                    value = ""
                if target == "protocol_version":
                    protocol_version = value
                else:
                    library_version = value

        return {
            "name": read("name", ""),
            "type": read("type", ""),
            "source_id": read("source_id", ""),
            "uid": read("uid", ""),
            "channel_count": int(read("channel_count", 0) or 0),
            "nominal_srate_hz": float(read("nominal_srate", 0.0) or 0.0),
            "channel_format": read("channel_format", ""),
            "lsl_protocol_version": protocol_version,
            "lsl_library_version": library_version,
        }


    def _read_lsl_channel_names(self, info, channel_count: int) -> list[str]:
        names = []
        try:
            ch = info.desc().child("channels").child("channel")
            while ch is not None and not ch.empty():
                label = ch.child_value("label") or ch.child_value("name")
                if label:
                    names.append(label)
                ch = ch.next_sibling()
        except Exception:
            names = []

        while len(names) < channel_count:
            names.append(f"ch_{len(names) + 1}")
        return names[:channel_count]


    def _drain_serial(self):
        for raw in self.serial.drain():
            self._process_incoming(raw)


    def _process_incoming(self, raw: str):
        try:
            d = json.loads(raw)
        except json.JSONDecodeError:
            self._log(raw, COLORS["text_dim"])
            return

        if d.get("dbg"):
            self._total_clip_hi[0] += int(d.get("clipHi0", 0))
            self._total_clip_hi[1] += int(d.get("clipHi1", 0))
            self._total_clip_lo[0] += int(d.get("clipLo0", 0))
            self._total_clip_lo[1] += int(d.get("clipLo1", 0))
            self.dstat_isr.setText(f"{d.get('isrUs', '?')} мкс")
            self.dstat_cliphi0.setText(str(self._total_clip_hi[0]))
            self.dstat_cliplo0.setText(str(self._total_clip_lo[0]))
            self.dstat_cliphi1.setText(str(self._total_clip_hi[1]))
            self.dstat_cliplo1.setText(str(self._total_clip_lo[1]))
            self.dstat_samps.setText(str(d.get("samps", "?")))
            self.dstat_env0.setText(f"{float(d.get('env0', 0)):.3f}")
            self.dstat_env1.setText(f"{float(d.get('env1', 0)):.3f}")

            for clip_hi, clip_lo, labels in (
                (
                    int(d.get("clipHi0", 0)),
                    int(d.get("clipLo0", 0)),
                    (self.dstat_cliphi0, self.dstat_cliplo0),
                ),
                (
                    int(d.get("clipHi1", 0)),
                    int(d.get("clipLo1", 0)),
                    (self.dstat_cliphi1, self.dstat_cliplo1),
                ),
            ):
                clip_color = COLORS["red"] if (clip_hi or clip_lo) else COLORS["text_dim"]
                for lbl in labels:
                    lbl.setStyleSheet(
                        f"color:{clip_color};font-family:'JetBrains Mono';font-size:11px;"
                    )

            self._log(
                f"[DBG] DAC0 V={d['v0']:.3f}В  огиб={d['env0']:.3f}  "
                f"пик={d['peak0']:.3f}  мин={d['trough0']:.3f}  "
                f"клип↑={d['clipHi0']}  клип↓={d['clipLo0']}  "
                f"DAC1 V={d['v1']:.3f}В  огиб={d['env1']:.3f}  "
                f"пик={d['peak1']:.3f}  мин={d['trough1']:.3f}  "
                f"клип↑={d['clipHi1']}  клип↓={d['clipLo1']}  "
                f"ISR={d['isrUs']}мкс  n={d['samps']}",
                COLORS["text_dim"],
            )

        elif d.get("ir"):
            sensor_id = int(d.get("sensor", 0))

            raw_orig = d.get("raw", 0)
            th_orig = d.get("th", 1500)
            min_orig = d.get("min", 0)
            max_orig = d.get("max", 0)

            try:
                raw_f = float(raw_orig) if raw_orig is not None else np.nan
                th_f = float(th_orig) if th_orig is not None else 1500.0
                min_f = float(min_orig) if min_orig is not None else 0.0
                max_f = float(max_orig) if max_orig is not None else 0.0
            except (TypeError, ValueError):
                raw_f, th_f, min_f, max_f = np.nan, 1500.0, 0.0, 0.0

            if INVERT_SENSOR_RAW:
                raw_val = invert_adc(raw_f)
                th_val = invert_adc(th_f)
                min_val = invert_adc(max_f)
                max_val = invert_adc(min_f)
            else:
                raw_val = raw_f
                th_val = th_f
                min_val = min_f
                max_val = max_f

            if np.isfinite(raw_val) and np.isfinite(th_val):
                refl = raw_val < th_val
            else:
                refl = False

            volts = float(d.get("v", 0.0))
            state = "ЕСТЬ отражение (белок)" if refl else "ЗРАЧОК (чёрный)"

            raw_display = int(round(raw_val)) if np.isfinite(raw_val) else "?"
            th_display = int(round(th_val)) if np.isfinite(th_val) else "?"
            min_display = int(round(min_val)) if np.isfinite(min_val) else "?"
            max_display = int(round(max_val)) if np.isfinite(max_val) else "?"

            labels = self.ir_metrics.get(sensor_id, {})
            if labels:
                labels["raw"].setText(str(raw_display))
                labels["volts"].setText(f"{volts:.3f}")
                labels["span"].setText(f"{min_display} / {max_display}")
                labels["th"].setText(str(th_display))
                labels["state"].setText(state)
                labels["state"].setStyleSheet(
                    f"color:{COLORS['green'] if refl else COLORS['red']};"
                    f"font-family:'JetBrains Mono';font-size:11px;font-weight:600;"
                )

            try:
                self._set_current_ir(
                    sensor_id,
                    {
                        "raw": raw_val,
                        "volts": volts,
                        "reflect": refl,
                        "threshold": th_val,
                        "min": min_val,
                        "max": max_val,
                    },
                )
            except (TypeError, ValueError):
                pass

            self._append_ir_table_row(
                sensor_id, raw_display, volts, refl, min_display, max_display
            )

            self._log(
                f"[IR{sensor_id}] raw={raw_display}  U={volts:.3f}В  "
                f"мин/макс={min_display}/{max_display}  "
                f"{state}  (порог {th_display})"
                + ("  [inv]" if INVERT_SENSOR_RAW else ""),
                COLORS["accent2"] if refl else COLORS["red"],
            )

        elif d.get("status"):
            self._log(f"[STATUS] {json.dumps(d)}", COLORS["accent"])
        else:
            self._log(f"[JSON] {raw}", COLORS["text_dim"])


    def _log(self, text: str, color: str = COLORS["text"]):
        ts = time.strftime("%H:%M:%S")
        self._console_append.emit(f"[{ts}] {text}", color)


    def _log_rx(self, text: str):
        self._log(text, COLORS["green"])


    @Slot(str, str)
    def _append_console(self, text: str, color: str):
        cursor = self.console.textCursor()
        doc = self.console.document()
        if doc.blockCount() >= 70:
            trim_cursor = QTextCursor(doc.begin())
            trim_cursor.select(QTextCursor.BlockUnderCursor)
            trim_cursor.movePosition(QTextCursor.EndOfBlock, QTextCursor.KeepAnchor)
            trim_cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.KeepAnchor)
            trim_cursor.removeSelectedText()

        cursor.movePosition(QTextCursor.End)
        fmt = cursor.charFormat()
        fmt.setForeground(QColor(color))
        cursor.setCharFormat(fmt)
        cursor.insertText(text + "\n")
        self.console.setTextCursor(cursor)
        self.console.ensureCursorVisible()


    def _clear_console(self):
        self.console.clear()
        self._total_clip_hi = [0, 0]
        self._total_clip_lo = [0, 0]
        if hasattr(self, "ir_table"):
            self.ir_table.setRowCount(0)
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
            getattr(self, attr).setText("—")
        for attr in (
            "dstat_cliphi0",
            "dstat_cliplo0",
            "dstat_cliphi1",
            "dstat_cliplo1",
            "ir_state",
        ):
            getattr(self, attr).setStyleSheet(
                f"color:{COLORS['text_dim']};font-family:'JetBrains Mono';font-size:11px;"
            )


    def _toggle_connect(self, checked: bool):
        if not checked:
            if not self.serial.is_open:
                self._set_serial_checkbox(False)
                return
            self._stop()
            self.serial.close()
            self._log("Отключено.", COLORS["text_dim"])
        else:
            port = self.port_combo.currentText()
            if port and "no ports" not in port and "порты не найдены" not in port:
                self.status_bar.showMessage(f"Подключение к {port} …")
                self._log(f"Подключение к {port} @ 115200 …", COLORS["accent"])
                ok = self.serial.open(port)
                if ok:
                    self.status_bar.showMessage(f"Подключено  ·  {port}  @  115200 бод")
                    self._log(f"Подключено к {port}.", COLORS["green"])
                else:
                    self._set_serial_checkbox(False)
            else:
                self._log("Serial-порт не выбран.", COLORS["yellow"])
                self._set_serial_checkbox(False)


    @Slot(bool)
    def _on_connected(self, connected: bool):
        if connected:
            self._set_serial_checkbox(True)
            self.start_btn.setEnabled(True)
            self.stop_btn.setEnabled(False)

            self._send("IR ON")

            self._send("DBG ON")
            self._auto_apply_timer.start(200)
            QTimer.singleShot(400, self._apply_calibration_to_firmware)
        else:
            if self._running or self._recording_active:
                self._stop(reason="Serial отключен")
            self._sequence_timer.stop()
            self._set_serial_checkbox(False)
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(False)
            self.sequence_load_btn.setEnabled(True)
            self.sequence_clear_btn.setEnabled(bool(self.sequence_steps))
            self._running = False


    @Slot(str)
    def _on_serial_error(self, msg: str):
        self.status_bar.showMessage(f"Ошибка Serial: {msg}")
        self._log(f"Ошибка Serial: {msg}", COLORS["red"])


    def _set_serial_checkbox(self, checked: bool):
        self._set_serial_indicator("connected" if checked else "disconnected")
