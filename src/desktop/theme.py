"""Consistent typography, surfaces and interaction states for the desktop workspace."""

from PySide6.QtGui import QColor, QFont, QPalette
from PySide6.QtWidgets import QApplication


def apply_theme(application: QApplication) -> None:
    application.setStyle("Fusion")
    application.setFont(QFont("DejaVu Sans", 12))
    palette = application.palette()
    for role, color in (
        (QPalette.ColorRole.Window, "#111b29"),
        (QPalette.ColorRole.Base, "#0d1725"),
        (QPalette.ColorRole.Text, "#dbe7f3"),
        (QPalette.ColorRole.WindowText, "#dbe7f3"),
        (QPalette.ColorRole.Button, "#23364c"),
        (QPalette.ColorRole.ButtonText, "#dbe7f3"),
        (QPalette.ColorRole.Highlight, "#1c726a"),
    ):
        palette.setColor(role, QColor(color))
    application.setPalette(palette)
    application.setStyleSheet("""
        QWidget { background: #111b29; color: #dbe7f3; }
        QLabel#operationFeedback { border: 1px solid #42617c; border-radius: 6px; padding: 10px; background: #1b3046; color: #dbeaff; }
        QLabel#operationFeedback[state="success"] { background: #123c38; border-color: #287e6c; color: #a5f0d9; }
        QLabel#operationFeedback[state="error"] { background: #472b35; border-color: #bd6577; color: #ffe0e6; }
        QPushButton:focus, QToolButton:focus, QCheckBox:focus { border: 2px solid #81dfcc; }
        QCheckBox::indicator { width: 18px; height: 18px; }
        QAbstractItemView { outline: none; }
        QMainWindow { background: #0a111c; }
        QLabel { background: transparent; }
        QLabel#targetSummary { color: #8be0c9; font-size: 13px; }
        QGroupBox { border: 1px solid #3c576e; border-radius: 7px; margin-top: 16px; padding: 14px 8px 8px; font-weight: 600; }
        QGroupBox::title { subcontrol-origin: margin; left: 10px; color: #a7ead8; padding: 0 5px; }
        QLabel#brand { color: #f5f9ff; font-size: 17px; font-weight: 700; }
        QLabel#sectionTitle { color: #e9f2fa; font-size: 16px; font-weight: 700; }
        QLabel#sectionSubtitle { color: #91a8be; font-size: 13px; }
        QLabel#metricCaption { color: #8da7bd; font-size: 13px; }
        QLabel#metricValue { color: #f1f7fc; font-size: 25px; font-weight: 600; }
        QLabel#statusBadge { background: #243549; color: #b8ccdd; border-radius: 5px; padding: 5px 9px; }
        QLabel#statusBadge[state="success"] { background: #123c38; color: #78dfc5; }
        QLabel#statusBadge[state="warning"] { background: #493b20; color: #f6cc80; }
        QFrame#workspaceSection { background: #111b29; border: 1px solid #283a4d; border-radius: 8px; }
        QFrame#appHeader { background: #0b1522; border-bottom: 1px solid #293e53; }
        QFrame#metricCard { background: #19283a; border: 1px solid #2b4054; border-radius: 6px; }
        QFrame#qgcQuickActions { background: #142437; border: 1px solid #36516a; border-radius: 7px; }
        QPushButton[role="landing"] { background: #633842; border-color: #925160; color: #ffe4e9; }
        QPushButton[role="landing"]:disabled, QPushButton[role="primary"]:disabled { background: #1a2737; color: #667b90; border-color: #27394d; }
        QToolBar { background: #101a28; border: none; border-bottom: 1px solid #293c50; spacing: 5px; padding: 8px; }
        QToolButton { background: #203247; border: 1px solid #31485e; border-radius: 5px; padding: 7px 10px; }
        QToolButton:hover { background: #2b425a; border-color: #5c819f; }
        QToolButton[role="camera"] { font-size: 13px; padding: 3px 7px; border-radius: 4px; }
        QToolButton:disabled { background: #192433; color: #60758b; border-color: #253448; }
        QToolButton[role="start"] { background: #146f65; border-color: #278f7f; color: #eafff9; }
        QToolButton[role="stop"] { background: #633842; border-color: #925160; color: #ffe4e9; }
        QPushButton { background: #23364c; border: 1px solid #36516a; border-radius: 5px; padding: 7px 10px; }
        QPushButton:hover { background: #2b465f; border-color: #6188a7; }
        QPushButton:pressed { background: #172b40; }
        QPushButton:disabled { color: #667b90; background: #1a2737; border-color: #27394d; }
        QPushButton[role="primary"] { background: #137b70; border-color: #299b8a; color: #f0fffb; }
        QPushButton[role="module"] { font-size: 14px; font-weight: 600; padding: 9px 5px; border: 1px solid transparent; background: #172537; color: #9fb5c9; }
        QPushButton[role="module"]:checked { background: #194a4b; border-color: #359788; color: #aaf4df; }
        QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { background: #0d1725; border: 1px solid #354c63; border-radius: 5px; padding: 5px; selection-background-color: #1c726a; }
        QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus { border-color: #57bca6; }
        QComboBox QAbstractItemView { background: #152538; selection-background-color: #285861; }
        QTabWidget::pane { border: 1px solid #2c4258; border-radius: 5px; }
        QTabBar::tab { background: #172638; color: #9eb5c9; padding: 7px 9px; border-bottom: 2px solid transparent; }
        QTabBar::tab:selected { background: #223d50; color: #b5f2df; border-bottom-color: #4cc3a4; }
        QTableWidget { background: #101d2c; alternate-background-color: #182b3c; gridline-color: #283e52; selection-background-color: #245663; border: 1px solid #2b4358; border-radius: 5px; }
        QHeaderView::section { background: #203449; color: #adc8dd; padding: 6px; border: none; border-bottom: 1px solid #36536a; font-weight: 600; }
        QPlainTextEdit { background: #0a1420; color: #bcd2e4; border: 1px solid #2f475e; border-radius: 5px; padding: 5px; font-family: "DejaVu Sans Mono"; font-size: 13px; }
        QScrollArea { border: none; }
        QScrollBar:vertical { background: #111d2b; width: 9px; margin: 0; }
        QScrollBar::handle:vertical { background: #3b546c; border-radius: 4px; min-height: 25px; }
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
        QSplitter::handle { background: #0b1320; }
        QSplitter::handle:hover { background: #316f71; }
        QCheckBox { spacing: 7px; }
        QStatusBar { background: #0a1420; color: #9cb4ca; border-top: 1px solid #283d50; padding: 3px; }
        QToolTip { background: #20394d; color: #e5f3ff; border: 1px solid #55829c; padding: 5px; }
    """)
