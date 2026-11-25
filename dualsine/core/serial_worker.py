"""Qt-воркер для работы с COM-портом (Arduino Due)."""

import json, threading, time
import serial
from PySide6.QtCore import QObject, Signal


class SerialWorker(QObject):
    message_received = Signal(str)
    error_occurred = Signal(str)
    connected = Signal(bool)

    def __init__(self):
        super().__init__()
        self._port: serial.Serial | None = None
        self._lock = threading.Lock()

    def open(self, port: str, baud: int = 115200) -> bool:
        try:
            with self._lock:
                if self._port and self._port.is_open:
                    self._port.close()
                self._port = serial.Serial(port, baud, timeout=0.1)
                time.sleep(2)
                self._port.reset_input_buffer()
            self.connected.emit(True)
            return True
        except Exception as e:
            self.error_occurred.emit(str(e))
            self.connected.emit(False)
            return False

    def close(self):
        with self._lock:
            if self._port and self._port.is_open:
                self._port.close()
        self.connected.emit(False)

    def send(self, cmd: str, timeout: float = 0.3) -> str | None:
        with self._lock:
            if not self._port or not self._port.is_open:
                return None
            try:
                self._port.reset_input_buffer()
                self._port.write((cmd.strip() + "\n").encode())
                deadline = time.time() + timeout
                while time.time() < deadline:
                    line = self._port.readline().decode(errors="replace").strip()
                    if not line:
                        continue
                    self.message_received.emit(f"← {line}")
                    if line.startswith("OK") or line.startswith("ERR"):
                        return line

                    continue
                return None
            except Exception as e:
                self.error_occurred.emit(str(e))
                return None

    def status(self, timeout: float = 0.6) -> dict | None:
        with self._lock:
            if not self._port or not self._port.is_open:
                return None
            try:
                self._port.reset_input_buffer()
                self._port.write(b"STATUS\n")
                deadline = time.time() + timeout
                buf = ""
                while time.time() < deadline:
                    chunk = self._port.readline().decode(errors="replace")
                    if not chunk:
                        continue
                    self.message_received.emit(f"← {chunk.strip()}")
                    if '"status"' not in chunk:
                        continue
                    buf += chunk
                    try:
                        data = json.loads(buf.strip())
                    except Exception:
                        continue
                    if isinstance(data, dict):
                        return data
                return None
            except Exception as e:
                self.error_occurred.emit(str(e))
                return None

    def drain(self) -> list[str]:
        lines = []
        with self._lock:
            if not self._port or not self._port.is_open:
                return lines
            try:
                while self._port.in_waiting:
                    line = self._port.readline().decode(errors="replace").strip()
                    if line:
                        lines.append(line)
            except Exception:
                pass
        return lines

    @property
    def is_open(self) -> bool:
        with self._lock:
            return self._port is not None and self._port.is_open
