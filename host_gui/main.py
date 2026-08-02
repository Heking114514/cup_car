import sys

from PySide6.QtGui import QFont, QFontDatabase
from PySide6.QtWidgets import QApplication

from car_serial_gui.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("Cup Car Console")
    preferred_fonts = ("Microsoft YaHei UI", "Microsoft YaHei", "SimSun")
    available_fonts = set(QFontDatabase.families())
    family = next((name for name in preferred_fonts if name in available_fonts), "")
    if family:
        app.setFont(QFont(family, 9))
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
