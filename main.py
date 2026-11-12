"""
main.py - Task 3: PySide6 window (telemetry, controls, keyboard).

The window never changes the simulator directly: it calls ROVController
methods and only READS sim.state for display.

Run:  python main.py
"""

import sys
from collections import deque

from PySide6.QtCore import Qt, QTimer, QElapsedTimer, QEvent, QUrl, QCoreApplication
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QDoubleSpinBox, QButtonGroup,
    QGridLayout, QVBoxLayout, QHBoxLayout, QGroupBox,
)

from PySide6.QtWebEngineWidgets import QWebEngineView  # import before QApplication
import pyqtgraph as pg

from geo import local_to_latlon, LAT0, LON0
from controller import ROVController, MANUAL, DEPTH_HOLD, HEADING_HOLD, MODES
from rov_sim import DT

PHYSICS_INTERVAL_MS = 20   # physics timer (50 Hz)
DISPLAY_INTERVAL_MS = 50   # label refresh (20 Hz)
MAX_CATCHUP = 0.25         # s - cap so a long stall can't cause a flood of steps
PLOT_WINDOW_S = 60.0       # depth plot shows the last 60 s
MAP_INTERVAL_MS = 200      # map marker update (5 Hz)

# Small Leaflet page, shown inside the app via QWebEngineView (an embedded widget)
MAP_HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>html, body, #map { height: 100%; margin: 0; }</style>
</head><body><div id="map"></div>
<script>
var map = L.map('map').setView([__LAT__, __LON__], 18);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 19,
  attribution: '&copy; OpenStreetMap contributors'
}).addTo(map);
var marker = L.marker([__LAT__, __LON__]).addTo(map);
function moveMarker(lat, lon) {
  if (typeof marker === 'undefined') { return; }  // Leaflet not loaded (offline)
  marker.setLatLng([lat, lon]);
  if (!map.getBounds().pad(-0.3).contains([lat, lon])) { map.panTo([lat, lon]); }
}
</script></body></html>"""

KEYS = {
    Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D,
    Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right,
}


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROV Telemetry Ground Station")
        self.resize(1000, 600)
        self.setFocusPolicy(Qt.StrongFocus)

        self.controller = ROVController()
        self.held = set()          # keys currently held down
        self._had_keys = False     # were keys held on the previous tick?

        # Depth plot buffers (about 60 s at 20 Hz, with a little spare)
        self.t_buf = deque(maxlen=int(PLOT_WINDOW_S * 1000 / DISPLAY_INTERVAL_MS) + 40)
        self.d_buf = deque(maxlen=self.t_buf.maxlen)
        self.map_ready = False
        self._last_latlon = None

        self._build_ui()

        # Physics timer with wall-clock accumulator (keeps sim time == real time)
        self.clock = QElapsedTimer()
        self.clock.start()
        self.accumulator = 0.0
        self.physics_timer = QTimer(self)
        self.physics_timer.timeout.connect(self.on_physics_tick)
        self.physics_timer.start(PHYSICS_INTERVAL_MS)

        # Display timer (slower than physics)
        self.display_timer = QTimer(self)
        self.display_timer.timeout.connect(self.refresh_display)
        self.display_timer.start(DISPLAY_INTERVAL_MS)

        # Map timer (slower still)
        self.map_timer = QTimer(self)
        self.map_timer.timeout.connect(self.update_map)
        self.map_timer.start(MAP_INTERVAL_MS)

        self.refresh_display()
        self.setFocus()

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        root = QHBoxLayout(self)

        # ---------- left column: telemetry + controls ----------
        left = QVBoxLayout()
        root.addLayout(left, 0)

        telemetry = QGroupBox("Telemetry")
        grid = QGridLayout(telemetry)
        big = QFont()
        big.setPointSize(14)
        big.setBold(True)

        self.armed_label = QLabel()
        self.armed_label.setFont(big)
        self.mode_label = QLabel()
        self.mode_label.setFont(big)
        self.roll_label = QLabel()
        self.pitch_label = QLabel()
        self.yaw_label = QLabel()
        self.depth_label = QLabel()
        self.battery_label = QLabel()
        for lbl in (self.roll_label, self.pitch_label, self.yaw_label,
                    self.depth_label, self.battery_label):
            lbl.setFont(big)

        grid.addWidget(QLabel("State:"), 0, 0)
        grid.addWidget(self.armed_label, 0, 1)
        grid.addWidget(QLabel("Mode:"), 1, 0)
        grid.addWidget(self.mode_label, 1, 1)
        grid.addWidget(QLabel("Roll:"), 2, 0)
        grid.addWidget(self.roll_label, 2, 1)
        grid.addWidget(QLabel("Pitch:"), 3, 0)
        grid.addWidget(self.pitch_label, 3, 1)
        grid.addWidget(QLabel("Yaw:"), 4, 0)
        grid.addWidget(self.yaw_label, 4, 1)
        grid.addWidget(QLabel("Depth:"), 5, 0)
        grid.addWidget(self.depth_label, 5, 1)
        grid.addWidget(QLabel("Battery:"), 6, 0)
        grid.addWidget(self.battery_label, 6, 1)
        left.addWidget(telemetry)

        controls = QGroupBox("Controls")
        cv = QVBoxLayout(controls)

        arm_row = QHBoxLayout()
        self.arm_btn = QPushButton("Arm")
        self.disarm_btn = QPushButton("Disarm")
        arm_row.addWidget(self.arm_btn)
        arm_row.addWidget(self.disarm_btn)
        cv.addLayout(arm_row)

        mode_row = QHBoxLayout()
        self.mode_group = QButtonGroup(self)
        self.mode_group.setExclusive(True)
        self.mode_buttons = {}
        for mode in MODES:
            btn = QPushButton(mode)
            btn.setCheckable(True)
            self.mode_group.addButton(btn)
            self.mode_buttons[mode] = btn
            mode_row.addWidget(btn)
        self.mode_buttons[MANUAL].setChecked(True)
        for btn in self.mode_buttons.values():
            btn.setStyleSheet("QPushButton:checked { background-color: #2a82da; color: white; }")
        cv.addLayout(mode_row)

        depth_row = QHBoxLayout()
        depth_row.addWidget(QLabel("Target depth (m):"))
        self.depth_spin = QDoubleSpinBox()
        self.depth_spin.setRange(0.0, 100.0)
        self.depth_spin.setDecimals(1)
        self.depth_spin.setSingleStep(0.5)
        self.depth_set_btn = QPushButton("Set")
        depth_row.addWidget(self.depth_spin)
        depth_row.addWidget(self.depth_set_btn)
        cv.addLayout(depth_row)

        heading_row = QHBoxLayout()
        heading_row.addWidget(QLabel("Target heading (°):"))
        self.heading_spin = QDoubleSpinBox()
        self.heading_spin.setRange(0.0, 359.9)
        self.heading_spin.setDecimals(1)
        self.heading_spin.setSingleStep(5.0)
        self.heading_spin.setWrapping(True)
        self.heading_set_btn = QPushButton("Set")
        heading_row.addWidget(self.heading_spin)
        heading_row.addWidget(self.heading_set_btn)
        cv.addLayout(heading_row)

        help_label = QLabel(
            "Manual keys: W/S forward/back, A/D left/right,\n"
            "Up/Down ascend/descend, Left/Right turn.\n"
            "In a hold mode: pick the mode first, then set the target."
        )
        cv.addWidget(help_label)
        left.addWidget(controls)
        left.addStretch(1)

        # Buttons must not steal the keyboard focus from the window
        for w in (self.arm_btn, self.disarm_btn, self.depth_set_btn,
                  self.heading_set_btn, *self.mode_buttons.values()):
            w.setFocusPolicy(Qt.NoFocus)

        # ---------- right column: placeholders for Task 4 ----------
        right = QVBoxLayout()
        root.addLayout(right, 1)

        # Depth vs time plot
        self.plot = pg.PlotWidget()
        self.plot.setTitle("Depth vs Time")
        self.plot.setLabel("left", "Depth", units="m")
        self.plot.setLabel("bottom", "Time", units="s")
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.invertY(True)  # depth is positive down -> deeper goes lower
        self.plot.setMouseEnabled(x=False, y=False)
        self.plot.hideButtons()
        self.depth_curve = self.plot.plot(pen=pg.mkPen("c", width=2))
        right.addWidget(self.plot, 1)

        # OpenStreetMap view (embedded browser widget showing a Leaflet page)
        self.map_view = QWebEngineView()
        self.map_view.setFocusPolicy(Qt.NoFocus)
        self.map_view.page().profile().setHttpUserAgent("ROVGroundStation/1.0 (internship assessment)")
        self.map_view.loadFinished.connect(self.on_map_loaded)
        html = MAP_HTML.replace("__LAT__", repr(LAT0)).replace("__LON__", repr(LON0))
        self.map_view.setHtml(html, QUrl("https://rov-ground-station.local/"))
        right.addWidget(self.map_view, 1)

        # ---------- signals ----------
        self.arm_btn.clicked.connect(self.controller.arm)
        self.disarm_btn.clicked.connect(self.controller.disarm)
        for mode, btn in self.mode_buttons.items():
            btn.clicked.connect(lambda _=False, m=mode: self.on_mode_clicked(m))
        self.depth_set_btn.clicked.connect(self.on_set_depth)
        self.heading_set_btn.clicked.connect(self.on_set_heading)

    # ------------------------------------------------------------ handlers
    def on_mode_clicked(self, mode):
        self.controller.set_mode(mode)
        self.sync_spinboxes()
        self.setFocus()

    def on_set_depth(self):
        self.controller.set_target_depth(self.depth_spin.value())
        self.depth_spin.clearFocus()
        self.setFocus()

    def on_set_heading(self):
        self.controller.set_target_heading(self.heading_spin.value())
        self.heading_spin.clearFocus()
        self.setFocus()

    def sync_spinboxes(self):
        """Show the controller's real targets (they change on mode switch)."""
        for spin, value in ((self.depth_spin, self.controller.target_depth),
                            (self.heading_spin, min(self.controller.target_heading, 359.9))):
            spin.blockSignals(True)
            spin.setValue(value)
            spin.blockSignals(False)

    # ------------------------------------------------------------ keyboard
    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() in KEYS:
            self.held.add(event.key())
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() in KEYS:
            self.held.discard(event.key())
        else:
            super().keyReleaseEvent(event)

    def focusOutEvent(self, event):
        # A key release may go to another widget -> never leave keys "stuck"
        self.held.clear()
        super().focusOutEvent(event)

    def changeEvent(self, event):
        if event.type() == QEvent.ActivationChange and not self.isActiveWindow():
            self.held.clear()  # e.g. alt-tab while a key is held
        super().changeEvent(event)

    def mousePressEvent(self, event):
        self.setFocus()  # click on empty area gives keys back to the window
        super().mousePressEvent(event)

    def axes_from_keys(self):
        h = self.held
        surge = (Qt.Key_W in h) - (Qt.Key_S in h)
        sway = (Qt.Key_D in h) - (Qt.Key_A in h)
        heave = (Qt.Key_Down in h) - (Qt.Key_Up in h)   # heave +1 = DOWN
        yaw = (Qt.Key_Right in h) - (Qt.Key_Left in h)
        return surge, sway, heave, yaw

    # ---------------------------------------------------------------- loop
    def on_physics_tick(self):
        self.accumulator += self.clock.restart() / 1000.0
        self.accumulator = min(self.accumulator, MAX_CATCHUP)
        while self.accumulator >= DT:
            self.apply_keys()
            self.controller.update()
            self.accumulator -= DT

    def apply_keys(self):
        if self.held:
            # keep re-sending so the 1 s watchdog stays fed while a key is held
            self.controller.set_manual_input(*self.axes_from_keys())
            self._had_keys = True
        elif self._had_keys:
            # last key released -> drop the command immediately
            self.controller.set_manual_input(0, 0, 0, 0)
            self._had_keys = False

    def refresh_display(self):
        c = self.controller
        s = c.sim.state
        if c.armed:
            self.armed_label.setText("ARMED")
            self.armed_label.setStyleSheet("color: green;")
        else:
            self.armed_label.setText("DISARMED")
            self.armed_label.setStyleSheet("color: red;")
        self.mode_label.setText(c.mode)
        self.roll_label.setText(f"{round(s.roll, 1) + 0.0:6.1f} °")
        self.pitch_label.setText(f"{round(s.pitch, 1) + 0.0:6.1f} °")
        self.yaw_label.setText(f"{s.yaw:6.1f} °")
        self.depth_label.setText(f"{s.depth:6.2f} m")
        self.battery_label.setText(f"{s.battery_v:6.2f} V")

        # Depth plot: append a sample and slide the 60 s window
        now = c.sim.time
        self.t_buf.append(now)
        self.d_buf.append(s.depth)
        self.depth_curve.setData(list(self.t_buf), list(self.d_buf))
        self.plot.setXRange(now - PLOT_WINDOW_S, now, padding=0)
        self.plot.setYRange(0.0, max(5.0, max(self.d_buf) * 1.1), padding=0)

    def on_map_loaded(self, ok):
        self.map_ready = ok

    def update_map(self):
        if not self.map_ready:
            return
        s = self.controller.sim.state
        lat, lon = local_to_latlon(s.x, s.y)
        key = (round(lat, 7), round(lon, 7))
        if key == self._last_latlon:  # nothing moved -> skip the JS call
            return
        self._last_latlon = key
        self.map_view.page().runJavaScript(f"moveMarker({lat!r}, {lon!r});")


def main():
    QCoreApplication.setAttribute(Qt.AA_ShareOpenGLContexts)  # needed by WebEngine
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
