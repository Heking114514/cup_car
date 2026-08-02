from __future__ import annotations

import html
import time
from collections import deque

from PySide6.QtCharts import QChart, QChartView, QLineSeries, QValueAxis
from PySide6.QtCore import QPointF, Qt, QTimer
from PySide6.QtGui import QColor, QPainter
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSpinBox,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from serial.tools import list_ports

from .protocol import EncoderFrame, VelocityCommand, parse_encoder_frame
from .serial_worker import SerialWorker


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self._worker: SerialWorker | None = None
        self._serial_connected = False
        self._started_at = time.monotonic()
        self._last_frame: EncoderFrame | None = None
        self._left_points: deque[tuple[float, int]] = deque(maxlen=500)
        self._right_points: deque[tuple[float, int]] = deque(maxlen=500)

        self.setWindowTitle("视觉导航小车串口控制台")
        self.resize(1180, 760)
        self.setMinimumSize(980, 640)
        self._build_ui()
        self._apply_style()
        self._refresh_ports()

        self._send_timer = QTimer(self)
        self._send_timer.timeout.connect(self._send_velocity)
        self._send_timer.start(50)

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        page = QVBoxLayout(root)
        page.setContentsMargins(18, 16, 18, 16)
        page.setSpacing(12)

        page.addLayout(self._build_connection_bar())

        body = QHBoxLayout()
        body.setSpacing(12)
        body.addLayout(self._build_control_column(), 0)
        body.addWidget(self._build_monitor_panel(), 1)
        page.addLayout(body, 1)

    def _build_connection_bar(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self.status_dot = QLabel()
        self.status_dot.setFixedSize(10, 10)
        self.status_text = QLabel("未连接")
        self.status_text.setObjectName("statusText")

        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(220)
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "230400", "460800", "921600"])
        self.refresh_button = QPushButton("刷新")
        self.refresh_button.clicked.connect(self._refresh_ports)
        self.connect_button = QPushButton("连接")
        self.connect_button.setObjectName("primaryButton")
        self.connect_button.clicked.connect(self._toggle_connection)

        row.addWidget(self.status_dot)
        row.addWidget(self.status_text)
        row.addSpacing(16)
        row.addWidget(QLabel("串口"))
        row.addWidget(self.port_combo)
        row.addWidget(QLabel("波特率"))
        row.addWidget(self.baud_combo)
        row.addWidget(self.refresh_button)
        row.addStretch()
        row.addWidget(self.connect_button)
        self._set_connection_state(False)
        return row

    def _build_control_column(self) -> QVBoxLayout:
        column = QVBoxLayout()
        column.setSpacing(12)

        command_group = QGroupBox("导航速度指令")
        command_group.setMinimumWidth(300)
        form = QFormLayout(command_group)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        self.vx_spin = self._make_float_spin(-2.0, 2.0, 0.05, " m/s")
        self.az_spin = self._make_float_spin(-10.0, 10.0, 0.10, " rad/s")
        self.rate_spin = QSpinBox()
        self.rate_spin.setRange(1, 50)
        self.rate_spin.setValue(20)
        self.rate_spin.setSuffix(" Hz")
        self.rate_spin.valueChanged.connect(self._update_send_period)
        self.auto_send = QCheckBox("连续发送")
        self.auto_send.setChecked(True)

        form.addRow("线速度 vx", self.vx_spin)
        form.addRow("角速度 az", self.az_spin)
        form.addRow("发送频率", self.rate_spin)
        form.addRow("", self.auto_send)

        button_row = QHBoxLayout()
        send_button = QPushButton("发送一次")
        send_button.clicked.connect(self._send_velocity)
        stop_button = QPushButton("急停 / 清零")
        stop_button.setObjectName("stopButton")
        stop_button.clicked.connect(self._send_stop)
        button_row.addWidget(send_button)
        button_row.addWidget(stop_button)
        form.addRow(button_row)
        column.addWidget(command_group)

        encoder_group = QGroupBox("编码器")
        grid = QGridLayout(encoder_group)
        self.left_value = self._metric_label("0")
        self.right_value = self._metric_label("0")
        self.left_delta = QLabel("Δ 0")
        self.right_delta = QLabel("Δ 0")
        grid.addWidget(QLabel("左轮累计"), 0, 0)
        grid.addWidget(QLabel("右轮累计"), 0, 1)
        grid.addWidget(self.left_value, 1, 0)
        grid.addWidget(self.right_value, 1, 1)
        grid.addWidget(self.left_delta, 2, 0)
        grid.addWidget(self.right_delta, 2, 1)
        column.addWidget(encoder_group)

        protocol_group = QGroupBox("链路状态")
        protocol_form = QFormLayout(protocol_group)
        self.rx_count = QLabel("0")
        self.tx_count = QLabel("0")
        self.last_rx = QLabel("--")
        protocol_form.addRow("接收帧", self.rx_count)
        protocol_form.addRow("发送帧", self.tx_count)
        protocol_form.addRow("最近接收", self.last_rx)
        column.addWidget(protocol_group)
        column.addStretch()
        return column

    def _build_monitor_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        self.left_series = QLineSeries()
        self.left_series.setName("左轮")
        self.left_series.setColor(QColor("#16697a"))
        self.right_series = QLineSeries()
        self.right_series.setName("右轮")
        self.right_series.setColor(QColor("#d1495b"))

        self.chart = QChart()
        self.chart.addSeries(self.left_series)
        self.chart.addSeries(self.right_series)
        self.chart.setTitle("编码器累计计数")
        self.chart.legend().setAlignment(Qt.AlignTop)
        self.chart.setBackgroundRoundness(4)

        self.axis_x = QValueAxis()
        self.axis_x.setTitleText("时间 / s")
        self.axis_x.setRange(0, 20)
        self.axis_y = QValueAxis()
        self.axis_y.setTitleText("计数")
        self.axis_y.setRange(-100, 100)
        self.chart.addAxis(self.axis_x, Qt.AlignBottom)
        self.chart.addAxis(self.axis_y, Qt.AlignLeft)
        for series in (self.left_series, self.right_series):
            series.attachAxis(self.axis_x)
            series.attachAxis(self.axis_y)

        chart_view = QChartView(self.chart)
        chart_view.setRenderHint(QPainter.Antialiasing)
        chart_view.setMinimumHeight(370)
        layout.addWidget(chart_view, 3)

        log_group = QGroupBox("收发日志")
        log_layout = QVBoxLayout(log_group)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(500)
        clear_button = QPushButton("清空日志")
        clear_button.clicked.connect(self.log_view.clear)
        log_layout.addWidget(self.log_view)
        log_layout.addWidget(clear_button, alignment=Qt.AlignRight)
        layout.addWidget(log_group, 2)
        return panel

    @staticmethod
    def _make_float_spin(minimum: float, maximum: float, step: float, suffix: str) -> QDoubleSpinBox:
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(3)
        spin.setSingleStep(step)
        spin.setSuffix(suffix)
        spin.setKeyboardTracking(False)
        return spin

    @staticmethod
    def _metric_label(text: str) -> QLabel:
        label = QLabel(text)
        label.setObjectName("metricValue")
        label.setAlignment(Qt.AlignCenter)
        return label

    def _refresh_ports(self) -> None:
        current = self.port_combo.currentData()
        self.port_combo.clear()
        ports = sorted(list_ports.comports(), key=lambda item: item.device)
        for port in ports:
            label = f"{port.device}  {port.description}"
            self.port_combo.addItem(label, port.device)
        if not ports:
            self.port_combo.addItem("未发现串口", None)
        elif current:
            index = self.port_combo.findData(current)
            if index >= 0:
                self.port_combo.setCurrentIndex(index)

    def _toggle_connection(self) -> None:
        if self._worker is not None:
            self._disconnect_serial()
            return

        port = self.port_combo.currentData()
        if not port:
            self._append_log("SYS", "没有可用串口", "#a23b3b")
            return

        self.connect_button.setEnabled(False)
        self.status_text.setText("连接中...")
        worker = SerialWorker(port, int(self.baud_combo.currentText()), self)
        worker.connected.connect(self._on_connected)
        worker.disconnected.connect(self._on_disconnected)
        worker.line_received.connect(self._on_line_received)
        worker.error_occurred.connect(self._on_serial_error)
        worker.finished.connect(worker.deleteLater)
        self._worker = worker
        worker.start()

    def _disconnect_serial(self) -> None:
        worker = self._worker
        if worker is None:
            return
        self.connect_button.setEnabled(False)
        self.status_text.setText("正在断开...")
        worker.stop()
        if not worker.wait(1000):
            self._append_log("ERR", "串口线程未能及时停止", "#a23b3b")

    def _on_connected(self) -> None:
        if self.sender() is not self._worker:
            return
        self._serial_connected = True
        self._set_connection_state(True)
        self._append_log("SYS", "串口已连接", "#16697a")

    def _on_disconnected(self) -> None:
        if self.sender() is not self._worker:
            return
        self._serial_connected = False
        self._worker = None
        self._set_connection_state(False)

    def _on_serial_error(self, message: str) -> None:
        self._append_log("ERR", message, "#a23b3b")

    def _set_connection_state(self, connected: bool) -> None:
        color = "#2a9d6f" if connected else "#a0a7ad"
        self.status_dot.setStyleSheet(f"background: {color}; border-radius: 5px;")
        self.status_text.setText("已连接" if connected else "未连接")
        self.connect_button.setText("断开" if connected else "连接")
        self.connect_button.setEnabled(True)
        self.port_combo.setEnabled(not connected)
        self.baud_combo.setEnabled(not connected)
        self.refresh_button.setEnabled(not connected)

    def _send_velocity(self) -> None:
        if self.sender() is self._send_timer and not self.auto_send.isChecked():
            return
        command = VelocityCommand(self.vx_spin.value(), self.az_spin.value()).encode()
        self._send_line(command)

    def _send_stop(self) -> None:
        self.vx_spin.setValue(0.0)
        self.az_spin.setValue(0.0)
        self._send_line(VelocityCommand(0.0, 0.0).encode())

    def _send_line(self, line: str) -> None:
        if self._worker is None or not self._serial_connected:
            return
        self._worker.send_line(line)
        self.tx_count.setText(str(int(self.tx_count.text()) + 1))
        self._append_log("TX", line, "#16697a")

    def _on_line_received(self, line: str) -> None:
        self.rx_count.setText(str(int(self.rx_count.text()) + 1))
        self.last_rx.setText(time.strftime("%H:%M:%S"))
        frame = parse_encoder_frame(line)
        if frame is not None:
            self._update_encoder(frame)
            self._append_log("RX", line, "#495057")
        else:
            self._append_log("RX", line, "#8a5a00")

    def _update_encoder(self, frame: EncoderFrame) -> None:
        left_delta = frame.left - self._last_frame.left if self._last_frame else 0
        right_delta = frame.right - self._last_frame.right if self._last_frame else 0
        self._last_frame = frame

        self.left_value.setText(str(frame.left))
        self.right_value.setText(str(frame.right))
        self.left_delta.setText(f"Δ {left_delta:+d}")
        self.right_delta.setText(f"Δ {right_delta:+d}")

        elapsed = time.monotonic() - self._started_at
        self._left_points.append((elapsed, frame.left))
        self._right_points.append((elapsed, frame.right))
        self.left_series.replace([QPointF(t, value) for t, value in self._left_points])
        self.right_series.replace([QPointF(t, value) for t, value in self._right_points])

        start = max(0.0, elapsed - 20.0)
        self.axis_x.setRange(start, max(20.0, elapsed))
        visible_values = [value for t, value in self._left_points if t >= start]
        visible_values.extend(value for t, value in self._right_points if t >= start)
        if visible_values:
            low = min(visible_values)
            high = max(visible_values)
            padding = max(20, int((high - low) * 0.1))
            self.axis_y.setRange(low - padding, high + padding)

    def _update_send_period(self, frequency: int) -> None:
        self._send_timer.setInterval(max(20, round(1000 / frequency)))

    def _append_log(self, direction: str, text: str, color: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        safe_text = html.escape(text)
        self.log_view.append(
            f'<span style="color:#8a9298">{timestamp}</span> '
            f'<b style="color:{color}">{direction}</b> {safe_text}'
        )

    def closeEvent(self, event) -> None:
        self._disconnect_serial()
        event.accept()

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #f5f7f8; color: #20272b; font-size: 13px; }
            QGroupBox { background: #ffffff; border: 1px solid #d8dde0; border-radius: 6px;
                        margin-top: 12px; padding: 14px 10px 10px 10px; font-weight: 600; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
            QPushButton { background: #ffffff; border: 1px solid #c8cfd3; border-radius: 5px;
                          min-height: 30px; padding: 0 12px; }
            QPushButton:hover { border-color: #16697a; }
            QPushButton:disabled { color: #98a0a5; background: #edf0f1; }
            QPushButton#primaryButton { background: #16697a; color: white; border-color: #16697a; }
            QPushButton#stopButton { background: #b33a3a; color: white; border-color: #b33a3a; }
            QComboBox, QSpinBox, QDoubleSpinBox { background: white; border: 1px solid #c8cfd3;
                                                  border-radius: 5px; min-height: 30px; padding: 0 7px; }
            QTextEdit { background: #fbfcfc; border: 1px solid #d8dde0; border-radius: 4px;
                        font-family: Consolas; font-size: 12px; }
            QLabel#metricValue { background: #eef3f4; border-radius: 4px; padding: 10px;
                                 font-size: 22px; font-weight: 700; color: #1f5965; }
            QLabel#statusText { font-weight: 600; }
            """
        )
