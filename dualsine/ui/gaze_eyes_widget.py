"""Кастомный виджет с «глазами»: визуализация направления взгляда."""

import math, time
from PySide6.QtCore import QPointF, QRectF, QSize, QTimer, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QLinearGradient, QPainter, QPainterPath, QPen, QRadialGradient
from PySide6.QtWidgets import QWidget, QSizePolicy
from dualsine.config import COLORS


class GazeEyesWidget(QWidget):
    _COLOR_CENTER = "#34d399"
    _COLOR_ALERT = "#f87171"
    _COLOR_IDLE = "#c3ccd6"
    _PUPIL_RATIO = 0.32
    _IRIS_RATIO = 0.645
    _SHIFT_RATIO = 0.30

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(198)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setToolTip(
            "Индикация взгляда: зелёный — взгляд в центре, "
            "красный — смещён (влево/вправо), серо-белый — датчик "
            "не подключён. Датчики фиксируют только горизонтальное движение."
        )

        self._target = [self._blank_state(), self._blank_state()]
        self._current = [self._blank_state(), self._blank_state()]
        self._phase = 0.0
        self._last_tick = time.monotonic()

        self._timer = QTimer(self)
        self._timer.setInterval(16)
        self._timer.timeout.connect(self._tick)
        self._timer.start()

    @staticmethod
    def _blank_state() -> dict:
        return {
            "enabled": 0.0,
            "mix": 0.0,
            "dx": 0.0,
            "dy": 0.0,
            "label": "нет сигнала",
        }

    def sizeHint(self):
        return QSize(420, 198)

    def set_eyes(self, eyes: list) -> None:
        for i in range(2):
            state = eyes[i] if i < len(eyes) and isinstance(eyes[i], dict) else {}
            target = self._target[i]
            available = bool(state.get("available"))
            deviated = bool(state.get("deviated")) and available
            target["enabled"] = 1.0 if available else 0.0
            target["mix"] = 1.0 if deviated else 0.0

            direction = state.get("direction", "center")
            if not available:
                target["dx"] = target["dy"] = 0.0
                target["label"] = "нет сигнала"
            elif not deviated:
                target["dx"] = target["dy"] = 0.0
                target["label"] = "центр"
            elif direction == "left":
                target["dx"], target["dy"] = 1.0, 0.0
                target["label"] = "влево"
            elif direction == "right":
                target["dx"], target["dy"] = -1.0, 0.0
                target["label"] = "вправо"
            else:
                target["dx"] = target["dy"] = 0.0
                target["label"] = "смещён"

    def _tick(self) -> None:
        now = time.monotonic()
        dt = min(0.25, max(0.001, now - self._last_tick))
        self._last_tick = now
        alpha = 1.0 - math.exp(-dt / 0.075)

        changed = False
        for i in range(2):
            target, current = self._target[i], self._current[i]
            for key in ("enabled", "mix", "dx", "dy"):
                delta = target[key] - current[key]
                if abs(delta) > 0.002:
                    current[key] += delta * alpha
                    changed = True
                elif delta:
                    current[key] = target[key]
                    changed = True
            if target["label"] != current["label"]:
                current["label"] = target["label"]
                changed = True

        alerting = self._target[0]["mix"] > 0.01 or self._target[1]["mix"] > 0.01
        if alerting:
            self._phase = (self._phase + dt * 5.2) % (2.0 * math.pi)
            changed = True

        if changed:
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing, True)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing, True)

        rect = QRectF(0.5, 0.5, self.width() - 1.0, self.height() - 1.0)
        self._paint_panel(painter, rect)

        header_h = 24.0
        labels_h = 36.0
        content = rect.adjusted(14.0, 10.0, -14.0, -10.0)
        self._paint_header(painter, content, header_h)

        eyes_top = content.top() + header_h
        eyes_h = max(20.0, content.height() - header_h - labels_h)
        gap = max(26.0, content.width() * 0.07)
        radius = min(
            52.0,
            eyes_h / 2.0 - 2.0,
            (content.width() - gap) / 4.0 - 2.0,
        )
        radius = max(16.0, radius)
        center_y = eyes_top + eyes_h / 2.0
        center_x = content.center().x()

        eye0_x = center_x + gap / 2.0 + radius
        eye1_x = center_x - gap / 2.0 - radius

        for idx, eye_x in enumerate((eye0_x, eye1_x)):
            self._paint_eye(painter, eye_x, center_y, radius, self._current[idx])
            self._paint_eye_caption(
                painter, eye_x, center_y + radius, idx, self._current[idx]
            )

    def _paint_panel(self, painter: QPainter, rect: QRectF) -> None:
        path = QPainterPath()
        path.addRoundedRect(rect, 18.0, 18.0)

        body = QLinearGradient(rect.topLeft(), rect.bottomRight())
        body.setColorAt(0.0, QColor("#18212e"))
        body.setColorAt(0.55, QColor("#121a24"))
        body.setColorAt(1.0, QColor("#0d141c"))
        painter.fillPath(path, QBrush(body))

        inner = QPainterPath()
        inner.addRoundedRect(rect.adjusted(1.0, 1.0, -1.0, -1.0), 17.0, 17.0)
        sheen = QLinearGradient(rect.topLeft(), QPointF(rect.left(), rect.top() + 30.0))
        sheen.setColorAt(0.0, QColor(255, 255, 255, 16))
        sheen.setColorAt(1.0, QColor(255, 255, 255, 0))
        painter.fillPath(inner, QBrush(sheen))

        painter.setPen(QPen(QColor(COLORS["border"]), 1.0))
        painter.drawPath(path)

    def _paint_header(self, painter: QPainter, content: QRectF, header_h: float) -> None:
        caption_font = QFont(self.font())
        caption_font.setPointSizeF(8.2)
        caption_font.setBold(True)
        caption_font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.6)
        painter.setFont(caption_font)
        painter.setPen(QColor(COLORS["text_dim"]))
        painter.drawText(
            QRectF(content.left() + 2.0, content.top(), content.width() * 0.6, header_h),
            int(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter),
            "НАПРАВЛЕНИЕ ВЗГЛЯДА",
        )

        chip_text, chip_color = self._overall_status()
        chip_font = QFont(self.font())
        chip_font.setPointSizeF(8.4)
        chip_font.setBold(True)
        chip_font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 0.6)
        painter.setFont(chip_font)
        metrics = painter.fontMetrics()
        text_w = float(metrics.horizontalAdvance(chip_text))
        chip_h = 20.0
        chip_w = text_w + 22.0
        chip = QRectF(content.right() - chip_w, content.top() + 2.0, chip_w, chip_h)

        chip_path = QPainterPath()
        chip_path.addRoundedRect(chip, chip_h / 2.0, chip_h / 2.0)
        fill = QColor(chip_color)
        fill.setAlpha(34)
        edge = QColor(chip_color)
        edge.setAlpha(150)
        painter.fillPath(chip_path, QBrush(fill))
        painter.setPen(QPen(edge, 1.0))
        painter.drawPath(chip_path)
        painter.setPen(QColor(chip_color))
        painter.drawText(chip, int(Qt.AlignmentFlag.AlignCenter), chip_text)

    def _overall_status(self) -> tuple:
        available = [c["enabled"] > 0.5 for c in self._target]
        deviated = [
            available[i] and self._target[i]["mix"] > 0.5 for i in range(2)
        ]
        if not any(available):
            return "НЕТ СИГНАЛА", self._COLOR_IDLE
        if any(deviated):
            if all(deviated):
                return "ВЗГЛЯД СМЕЩЁН", self._COLOR_ALERT

            side = "ПРАВО"
            for i in range(2):
                if deviated[i]:
                    side = "ПРАВО" if i == 0 else "ЛЕВО"
            return f"СМЕЩЁН · {side}", self._COLOR_ALERT
        return "ВЗГЛЯД В ЦЕНТРЕ", self._COLOR_CENTER

    def _paint_eye(
        self, painter: QPainter, cx: float, cy: float, radius: float, current: dict
    ) -> None:
        enabled = current["enabled"]
        mix = current["mix"] * enabled
        color = self._lerp_color(
            self._lerp_color(QColor(self._COLOR_IDLE), QColor(self._COLOR_CENTER), enabled),
            QColor(self._COLOR_ALERT),
            mix,
        )

        pupil_dx = current["dx"] * radius * self._SHIFT_RATIO * enabled
        pupil_dy = current["dy"] * radius * self._SHIFT_RATIO * enabled
        pulse = 0.5 + 0.5 * math.sin(self._phase)

        shadow = QRadialGradient(QPointF(cx, cy + radius * 0.86), radius * 1.05)
        shadow.setColorAt(0.0, QColor(0, 0, 0, 110))
        shadow.setColorAt(1.0, QColor(0, 0, 0, 0))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(shadow))
        painter.drawEllipse(
            QPointF(cx, cy + radius * 0.86), radius * 1.05, radius * 0.36
        )

        glow_alpha = int(round(22 + 34 * enabled))
        if mix > 0.01:
            glow_alpha = int(round(46 + (72 + 46 * pulse) * mix))
        glow = QRadialGradient(QPointF(cx, cy), radius * 2.0)
        glow_core = QColor(color)
        glow_core.setAlpha(glow_alpha)
        glow_mid = QColor(color)
        glow_mid.setAlpha(max(0, int(glow_alpha * 0.45)))
        glow.setColorAt(0.0, glow_core)
        glow.setColorAt(0.40, glow_mid)
        glow.setColorAt(1.0, QColor(color.red(), color.green(), color.blue(), 0))
        painter.setBrush(QBrush(glow))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(cx, cy), radius * 2.0, radius * 2.0)

        rim = QColor(color)
        rim.setAlpha(215)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.setPen(QPen(rim, 2.2))
        painter.drawEllipse(QPointF(cx, cy), radius - 1.0, radius - 1.0)
        halo = QColor(color)
        halo.setAlpha(60)
        painter.setPen(QPen(halo, 1.0))
        painter.drawEllipse(QPointF(cx, cy), radius + 2.6, radius + 2.6)

        sclera = QRadialGradient(
            QPointF(cx - radius * 0.30, cy - radius * 0.34), radius * 1.9
        )
        sclera.setColorAt(0.0, QColor("#fbfdff"))
        sclera.setColorAt(0.55, QColor("#e6edf4"))
        sclera.setColorAt(1.0, QColor("#c4ceda") if enabled > 0.5 else QColor("#c9cfd6"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(sclera))
        painter.drawEllipse(QPointF(cx, cy), radius - 2.2, radius - 2.2)

        painter.save()
        clip = QPainterPath()
        clip.addEllipse(QPointF(cx, cy), radius - 2.2, radius - 2.2)
        painter.setClipPath(clip)
        lid = QLinearGradient(QPointF(cx, cy - radius), QPointF(cx, cy + radius * 0.15))
        lid.setColorAt(0.0, QColor(9, 12, 18, 120))
        lid.setColorAt(1.0, QColor(9, 12, 18, 0))
        painter.setBrush(QBrush(lid))
        painter.drawRect(QRectF(cx - radius, cy - radius, radius * 2.0, radius * 1.2))
        painter.restore()

        iris_cx, iris_cy = cx + pupil_dx, cy + pupil_dy
        iris_r = radius * self._IRIS_RATIO
        iris = QRadialGradient(
            QPointF(iris_cx - iris_r * 0.25, iris_cy - iris_r * 0.30), iris_r * 1.6
        )
        iris.setColorAt(0.0, color.lighter(126) if enabled > 0.5 else color.lighter(110))
        iris.setColorAt(0.68, color)
        iris.setColorAt(1.0, color.darker(158))
        painter.setBrush(QBrush(iris))
        painter.drawEllipse(QPointF(iris_cx, iris_cy), iris_r, iris_r)

        iris_ring = color.darker(165)
        iris_ring.setAlpha(190)
        painter.setPen(QPen(iris_ring, 1.1))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QPointF(iris_cx, iris_cy), iris_r, iris_r)

        pupil_r = radius * self._PUPIL_RATIO
        pupil = QRadialGradient(
            QPointF(iris_cx - pupil_r * 0.2, iris_cy - pupil_r * 0.25), pupil_r * 1.8
        )
        pupil.setColorAt(0.0, QColor("#131c26"))
        pupil.setColorAt(1.0, QColor("#05080c"))
        painter.setBrush(QBrush(pupil))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(QPointF(iris_cx, iris_cy), pupil_r, pupil_r)

        painter.setBrush(QBrush(QColor(255, 255, 255, 200)))
        painter.drawEllipse(
            QPointF(iris_cx - iris_r * 0.32, iris_cy - iris_r * 0.36), radius * 0.115, radius * 0.10
        )
        painter.setBrush(QBrush(QColor(255, 255, 255, 80)))
        painter.drawEllipse(
            QPointF(iris_cx + iris_r * 0.26, iris_cy + iris_r * 0.30), radius * 0.05, radius * 0.045
        )

    def _paint_eye_caption(
        self, painter: QPainter, cx: float, bottom: float, idx: int, current: dict
    ) -> None:
        name_font = QFont(self.font())
        name_font.setPointSizeF(7.6)
        name_font.setBold(True)
        name_font.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 1.2)
        painter.setFont(name_font)
        painter.setPen(QColor(COLORS["text_dim"]))
        painter.drawText(
            QRectF(cx - 70.0, bottom + 7.0, 140.0, 14.0),
            int(Qt.AlignmentFlag.AlignCenter),

            "ПРАВО" if idx == 0 else "ЛЕВО",
        )

        state_font = QFont(self.font())
        state_font.setPointSizeF(8.8)
        state_font.setBold(True)
        painter.setFont(state_font)
        enabled = current["enabled"]
        mix = current["mix"] * enabled
        color = self._lerp_color(
            self._lerp_color(QColor(self._COLOR_IDLE), QColor(self._COLOR_CENTER), enabled),
            QColor(self._COLOR_ALERT),
            mix,
        )
        painter.setPen(color)
        painter.drawText(
            QRectF(cx - 70.0, bottom + 21.0, 140.0, 15.0),
            int(Qt.AlignmentFlag.AlignCenter),
            current["label"],
        )

    @staticmethod
    def _lerp_color(c1: QColor, c2: QColor, t: float) -> QColor:
        t = max(0.0, min(1.0, t))
        return QColor(
            int(round(c1.red() + (c2.red() - c1.red()) * t)),
            int(round(c1.green() + (c2.green() - c1.green()) * t)),
            int(round(c1.blue() + (c2.blue() - c1.blue()) * t)),
        )
