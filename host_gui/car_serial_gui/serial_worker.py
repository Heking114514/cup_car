from __future__ import annotations

import queue
import threading

import serial
from PySide6.QtCore import QThread, Signal


class SerialWorker(QThread):
    connected = Signal()
    disconnected = Signal()
    line_received = Signal(str)
    error_occurred = Signal(str)

    def __init__(self, port: str, baudrate: int, parent=None) -> None:
        super().__init__(parent)
        self._port = port
        self._baudrate = baudrate
        self._stop_event = threading.Event()
        self._outgoing: queue.Queue[str] = queue.Queue()

    def send_line(self, line: str) -> None:
        if not self._stop_event.is_set():
            self._outgoing.put(line.rstrip("\r\n"))

    def stop(self) -> None:
        self._stop_event.set()

    def run(self) -> None:
        device: serial.Serial | None = None
        buffer = bytearray()

        try:
            device = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.03,
                write_timeout=0.2,
            )
            device.reset_input_buffer()
            self.connected.emit()

            while not self._stop_event.is_set():
                self._flush_outgoing(device)
                chunk = device.read(max(1, device.in_waiting))
                if chunk:
                    buffer.extend(chunk)
                    self._emit_complete_lines(buffer)
        except (serial.SerialException, OSError) as exc:
            if not self._stop_event.is_set():
                self.error_occurred.emit(str(exc))
        finally:
            if device is not None and device.is_open:
                device.close()
            self.disconnected.emit()

    def _flush_outgoing(self, device: serial.Serial) -> None:
        while True:
            try:
                line = self._outgoing.get_nowait()
            except queue.Empty:
                return
            device.write((line + "\r\n").encode("ascii"))

    def _emit_complete_lines(self, buffer: bytearray) -> None:
        while True:
            newline = buffer.find(b"\n")
            if newline < 0:
                if len(buffer) > 4096:
                    buffer.clear()
                return

            raw_line = bytes(buffer[:newline]).rstrip(b"\r")
            del buffer[: newline + 1]
            self.line_received.emit(raw_line.decode("ascii", errors="replace"))
