"""Запуск QApplication и показ главного окна."""

import sys
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QColor, QPalette
from PySide6.QtWidgets import QApplication
from dualsine.config import COLORS, STYLE
from dualsine.main_window import MainWindow


def _show_main_window(win: MainWindow, app: QApplication):
    if not sys.platform.startswith("linux"):
        win.showMaximized()
        return

    def apply_linux_maximized():
        screen = win.screen() or app.primaryScreen()
        if screen is not None:
            win.setGeometry(screen.availableGeometry())
        win.setWindowState(win.windowState() | Qt.WindowState.WindowMaximized)
        win.showMaximized()

    screen = app.primaryScreen()
    if screen is not None:
        win.setGeometry(screen.availableGeometry())

    win.show()
    apply_linux_maximized()
    QTimer.singleShot(0, apply_linux_maximized)
    QTimer.singleShot(150, apply_linux_maximized)
    QTimer.singleShot(500, apply_linux_maximized)


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(COLORS["bg"]))
    palette.setColor(QPalette.WindowText, QColor(COLORS["text"]))
    palette.setColor(QPalette.Base, QColor(COLORS["surface"]))
    palette.setColor(QPalette.AlternateBase, QColor(COLORS["surface2"]))
    palette.setColor(QPalette.Text, QColor(COLORS["text"]))
    palette.setColor(QPalette.Button, QColor(COLORS["surface2"]))
    palette.setColor(QPalette.ButtonText, QColor(COLORS["text"]))
    palette.setColor(QPalette.Highlight, QColor(COLORS["accent"]))
    palette.setColor(QPalette.HighlightedText, QColor(COLORS["bg"]))
    app.setPalette(palette)
    app.setStyleSheet(STYLE)

    win = MainWindow()
    _show_main_window(win, app)
    sys.exit(app.exec())
