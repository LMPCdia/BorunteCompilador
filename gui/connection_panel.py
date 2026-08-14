"""Panel para elegir entre simulador y robot real, y armar el cliente Modbus."""

from __future__ import annotations

from PySide6.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QRadioButton,
    QVBoxLayout,
)

from comms.robot_client import BorunteRobotClient
from comms.robot_simulator import SimulatedBorunteRobot


class ConnectionPanel(QGroupBox):
    def __init__(self) -> None:
        super().__init__("Conexión")

        self.mode_sim = QRadioButton("Simulador (sin hardware)")
        self.mode_real = QRadioButton("Robot real")
        self.mode_sim.setChecked(True)

        self.ip_edit = QLineEdit("192.168.1.10")
        self.ip_edit.setEnabled(False)
        self.mode_real.toggled.connect(self.ip_edit.setEnabled)

        self.status_label = QLabel("● Sin conectar")
        self.status_label.setStyleSheet("color: gray;")

        self.connect_btn = QPushButton("Conectar")
        self.connect_btn.clicked.connect(self._on_connect_clicked)

        row = QHBoxLayout()
        row.addWidget(self.mode_sim)
        row.addWidget(self.mode_real)
        row.addWidget(QLabel("IP:"))
        row.addWidget(self.ip_edit)
        row.addWidget(self.connect_btn)
        row.addStretch()
        row.addWidget(self.status_label)

        layout = QVBoxLayout()
        layout.addLayout(row)
        self.setLayout(layout)

        self._robot: BorunteRobotClient | None = None

    def _on_connect_clicked(self) -> None:
        try:
            self._robot = self._build_client()
            if self.mode_sim.isChecked():
                self.status_label.setText("● Conectado (simulador)")
                self.status_label.setStyleSheet("color: green;")
            else:
                self.status_label.setText(f"● Conectado ({self.ip_edit.text()})")
                self.status_label.setStyleSheet("color: green;")
        except Exception as e:  # noqa: BLE001
            self._robot = None
            self.status_label.setText(f"● Error: {e}")
            self.status_label.setStyleSheet("color: red;")

    def _build_client(self) -> BorunteRobotClient:
        if self.mode_sim.isChecked():
            sim = SimulatedBorunteRobot(move_duration_s=0.3)
            client = BorunteRobotClient(host="simulador", client=sim)
        else:
            client = BorunteRobotClient(host=self.ip_edit.text())
        if not client.connect():
            raise RuntimeError("No se pudo conectar")
        return client

    @property
    def robot(self) -> BorunteRobotClient | None:
        return self._robot

    def is_connected(self) -> bool:
        return self._robot is not None
