import sys
import os
import subprocess
import signal
import time
import yaml
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLineEdit, QLabel, QComboBox, QGroupBox, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer, QSize
from PyQt5.QtGui import QColor, QPainter, QBrush

CAMERA_PROCESS = None
TELEOP_PROCESS = None
COLLECT_PROCESS = None
DEPLOY_PROCESS = None

class StatusIndicator(QWidget):
    def __init__(self, color=Qt.red, size=15):
        super().__init__()
        self._color = color
        self._size = size
        self.setFixedSize(QSize(size, size))

    def set_color(self, color):
        self._color = color
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setBrush(QBrush(self._color))
        painter.drawEllipse(0, 0, self._size, self._size)

class ControlPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenTeach Control Panel")
        self.setMinimumWidth(600)

        self._read_network_config()

        global CAMERA_PROCESS, TELEOP_PROCESS, COLLECT_PROCESS, DEPLOY_PROCESS
        self.camera_process = CAMERA_PROCESS
        self.teleop_process = TELEOP_PROCESS
        self.collect_process = COLLECT_PROCESS
        self.deploy_process = DEPLOY_PROCESS

        self.init_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_process_dataset_status)
        self.timer.start(500)
    
    def _read_network_config(self, yaml_file='configs/network.yaml'):
        try:
            with open(yaml_file, 'r') as file:
                config = yaml.safe_load(file)
                self.initial_host_address = config.get('host_address', 'localhost')
        except Exception as e:
            print(f"Error reading network.yaml: {e}")

    def init_ui(self):
        main_layout = QVBoxLayout()

        main_layout.addWidget(self._create_camera_status_bar())

        main_layout.addWidget(self._create_teleop_group())
        main_layout.addWidget(self._create_collect_group())
        main_layout.addWidget(self._create_deploy_group())

        self.status_label = QLabel("Ready.")
        self.status_label.setStyleSheet("font-weight: bold;")
        main_layout.addWidget(self.status_label)

        self.setLayout(main_layout)

    def _create_camera_status_bar(self):
        group = QGroupBox("Camera Control")
        layout = QHBoxLayout()

        # Host Address
        layout.addWidget(QLabel(f"Host Address: "))
        self.camera_host_combo = QComboBox()
        self.camera_host_combo.addItems([self.initial_host_address, "147.46.76.120", "192.168.1.90"])
        self.camera_host_combo.setCurrentText(self.initial_host_address)
        layout.addWidget(self.camera_host_combo)

        # Status Indicator
        self.camera_status_indicator = StatusIndicator(Qt.red)
        layout.addWidget(self.camera_status_indicator)
        self.camera_status_label = QLabel("Camera Stream: OFF")
        layout.addWidget(self.camera_status_label)

        # Buttons
        self.camera_start_btn = QPushButton("Start Camera")
        self.camera_start_btn.clicked.connect(self.start_camera)
        self.camera_stop_btn = QPushButton("Stop Camera")
        self.camera_stop_btn.clicked.connect(self.stop_camera)
        self.camera_stop_btn.setEnabled(False)
        
        layout.addWidget(self.camera_start_btn)
        layout.addWidget(self.camera_stop_btn)

        group.setLayout(layout)
        return group

    def _create_teleop_group(self):
        group = QGroupBox("Teleoperation Control")
        layout = QHBoxLayout()

        # Robot Selection
        layout.addWidget(QLabel("Robot: "))
        self.teleop_robot_combo = QComboBox()
        self.teleop_robot_combo.addItems(["UR10e"])
        self.teleop_robot_combo.setCurrentText("UR10e")
        layout.addWidget(self.teleop_robot_combo)

        # Buttons
        self.teleop_start_btn = QPushButton("Start Teleop")
        self.teleop_start_btn.clicked.connect(self.start_teleop)

        self.teleop_stop_btn = QPushButton("Stop Teleop")
        self.teleop_stop_btn.clicked.connect(self.stop_teleop)
        self.teleop_stop_btn.setEnabled(False)

        layout.addWidget(self.teleop_start_btn)
        layout.addWidget(self.teleop_stop_btn)

        group.setLayout(layout)
        return group

    def _create_collect_group(self):
        group = QGroupBox("Data Collection Control")
        v_layout = QVBoxLayout()

        # Path & Task
        h1_layout = QHBoxLayout()
        h1_layout.addWidget(QLabel("Save Path: "))
        self.base_path_input = QLineEdit("extracted_data/") # Default path
        h1_layout.addWidget(self.base_path_input)

        h1_layout.addWidget(QLabel("Task: "))
        self.task_input = QLineEdit("debug_task") # Default task
        h1_layout.addWidget(self.task_input)
        v_layout.addLayout(h1_layout)

        # Demo Number & Buttons
        h2_layout = QHBoxLayout()

        self.use_last_num_checkbox = QCheckBox("Use Last Num")
        self.use_last_num_checkbox.setChecked(True)
        self.use_last_num_checkbox.stateChanged.connect(self.toggle_demo_num_control)
        h2_layout.addWidget(self.use_last_num_checkbox)

        h2_layout.addWidget(QLabel("Demo Num: "))
        latest_demo_num = self.check_latest_data_num(os.path.join(self.base_path_input.text(), self.task_input.text().strip()))
        self.demo_num_input = QLineEdit(str(latest_demo_num)) # Latest demo number
        self.demo_num_input.setEnabled(not self.use_last_num_checkbox.isChecked())
        h2_layout.addWidget(self.demo_num_input)

        self.collect_start_btn = QPushButton("Start Collect")
        self.collect_start_btn.clicked.connect(self.start_collect)

        self.collect_stop_btn = QPushButton("Stop Collect")
        self.collect_stop_btn.clicked.connect(self.stop_collect)
        self.collect_stop_btn.setEnabled(False)

        h2_layout.addWidget(self.collect_start_btn)
        h2_layout.addWidget(self.collect_stop_btn)
        v_layout.addLayout(h2_layout)

        group.setLayout(v_layout)
        return group
    
    def _create_deploy_group(self):
        group = QGroupBox("Deploy Server Control")
        layout = QHBoxLayout()
        
        # Robot Selection
        layout.addWidget(QLabel("Robot:"))
        self.deploy_robot_combo = QComboBox()
        self.deploy_robot_combo.addItems(["UR10e"])
        self.deploy_robot_combo.setCurrentText("UR10e") 
        layout.addWidget(self.deploy_robot_combo)
        
        # Status Indicator
        self.deploy_status_indicator = StatusIndicator(Qt.red)
        layout.addWidget(self.deploy_status_indicator)
        
        # Buttons
        self.deploy_start_btn = QPushButton("Start Deploy")
        self.deploy_start_btn.clicked.connect(self.start_deploy)
        
        self.deploy_stop_btn = QPushButton("Stop Deploy")
        self.deploy_stop_btn.clicked.connect(self.stop_deploy)
        self.deploy_stop_btn.setEnabled(False)
        
        layout.addWidget(self.deploy_start_btn)
        layout.addWidget(self.deploy_stop_btn)
        
        group.setLayout(layout)
        return group

    def _run_process(self, command, process_name):
        try:
            process = subprocess.Popen(command, shell=True, preexec_fn=os.setsid)
            self.update_status(f"{process_name} started. PID: {process.pid}")
            return process
        except Exception as e:
            self.update_status(f"Failed to start {process_name}: {e}")
            return None
    
    def _stop_process(self, process_handle, process_name):
        if process_handle and process_handle.poll() is None:
            try:
                os.killpg(process_handle.pid, signal.SIGINT)
                process_handle.wait(timeout=5)
                self.update_status(f"{process_name} stopped.")
            except Exception as e:
                process_handle.kill()
                self.update_status(f"{process_name} forcibly stopped: {e}")
            return None
        return process_handle
    
    def toggle_demo_num_control(self, state):
        is_checked = (state == Qt.Checked)
        self.demo_num_input.setEnabled(not is_checked)

        if not is_checked:
            latest_demo_num = self.check_latest_data_num(os.path.join(self.base_path_input.text(), self.task_input.text().strip()))
            self.demo_num_input.setText(str(latest_demo_num))

    def map_robot_name(self, display_name):
        robot_name_mapping = {
            "UR10e": "ur"
        }
        return robot_name_mapping.get(display_name, "ur")
    
    def start_camera(self):
        global CAMERA_PROCESS
        if CAMERA_PROCESS is None:
            selected_host = self.camera_host_combo.currentText()

            while CAMERA_PROCESS is None:
                command = f"python robot_camera.py host_address={selected_host}"
                CAMERA_PROCESS = self._run_process(command, "Camera Stream")

        if CAMERA_PROCESS:
            self.camera_start_btn.setEnabled(False)
            self.camera_stop_btn.setEnabled(True)
            self.camera_status_indicator.set_color(Qt.green)
            self.camera_status_label.setText(f"Camera Stream: ON @ {selected_host}")
        
    def stop_camera(self):
        global CAMERA_PROCESS
        CAMERA_PROCESS = self._stop_process(CAMERA_PROCESS, "Camera Stream")
        if CAMERA_PROCESS is None:
            self.camera_start_btn.setEnabled(True)
            self.camera_stop_btn.setEnabled(False)
            self.camera_status_indicator.set_color(Qt.red)
            self.camera_status_label.setText("Camera Stream: OFF")

    def start_teleop(self):
        global TELEOP_PROCESS
        if TELEOP_PROCESS is None:
            robot_name = self.map_robot_name(self.teleop_robot_combo.currentText())
            host_network = self.camera_host_combo.currentText()
            command = f"python teleop.py robot={robot_name} host_address={host_network}"

            TELEOP_PROCESS = self._run_process(command, "Teleop")

            if TELEOP_PROCESS:
                self.teleop_start_btn.setEnabled(False)
                self.teleop_stop_btn.setEnabled(True)
    
    def stop_teleop(self):
        global TELEOP_PROCESS
        TELEOP_PROCESS = self._stop_process(TELEOP_PROCESS, "Teleop")
        if TELEOP_PROCESS is None:
            self.teleop_start_btn.setEnabled(True)
            self.teleop_stop_btn.setEnabled(False)

    def check_latest_data_num(self, path):
        if not os.path.exists(path):
            return 0
        existing_demos = [
            d for d in os.listdir(path)
            if os.path.isdir(os.path.join(path, d)) and d.startswith("demonstration_")
        ]
        if not existing_demos:
            return 0
        demo_nums = [
            int(d.split("_")[1]) for d in existing_demos
            if d.split("_")[1].isdigit()
        ]
        return max(demo_nums) + 1 if demo_nums else 0

    def start_collect(self):
        global COLLECT_PROCESS
        if COLLECT_PROCESS is None:
            base_path = self.base_path_input.text()
            robot_name = self.map_robot_name(self.teleop_robot_combo.currentText())
            host_network = self.camera_host_combo.currentText()
            task_name = self.task_input.text().strip()
            demo_num = self.demo_num_input.text()

            storage_path = os.path.join(base_path, task_name)
            command = (
                f"python data_collect.py robot={robot_name} host_address={host_network} "
                f"storage_path={storage_path} demo_num={demo_num}"
            )

            COLLECT_PROCESS = self._run_process(command, "Data Collection")

            if COLLECT_PROCESS:
                self.collect_start_btn.setEnabled(False)
                self.collect_stop_btn.setEnabled(True)
    
    def stop_collect(self):
        global COLLECT_PROCESS
        COLLECT_PROCESS = self._stop_process(COLLECT_PROCESS, "Data Collection")
        if COLLECT_PROCESS is None:
            self.collect_start_btn.setEnabled(True)
            self.collect_stop_btn.setEnabled(False)
    
    def start_deploy(self):
        global DEPLOY_PROCESS
        if DEPLOY_PROCESS is None:
            robot_name = self.map_robot_name(self.deploy_robot_combo.currentText())
            host_network = self.camera_host_combo.currentText()
            command = (
                f"python deploy_server.py robot={robot_name} host_address={host_network} "
                f"robot.controllers.0.control=true"
            )
            
            DEPLOY_PROCESS = self._run_process(command, "Deploy Server")
            
            if DEPLOY_PROCESS:
                self.deploy_start_btn.setEnabled(False)
                self.deploy_stop_btn.setEnabled(True)
                self.deploy_status_indicator.set_color(Qt.green)

    def stop_deploy(self):
        global DEPLOY_PROCESS
        DEPLOY_PROCESS = self._stop_process(DEPLOY_PROCESS, "Deploy Server")
        if DEPLOY_PROCESS is None:
            self.deploy_start_btn.setEnabled(True)
            self.deploy_stop_btn.setEnabled(False)
            self.deploy_status_indicator.set_color(Qt.red)

    def update_status(self, message):
        self.status_label.setText(f"Status: {message}")
        print(f"[GUI] {message}")
    
    def check_process_dataset_status(self):
        global CAMERA_PROCESS, TELEOP_PROCESS, COLLECT_PROCESS, DEPLOY_PROCESS

        if CAMERA_PROCESS and CAMERA_PROCESS.poll() is not None:
            exit_code = CAMERA_PROCESS.poll()
            self.update_status(f"Camera process has exited with code {exit_code}.")
            CAMERA_PROCESS = None
            self.camera_start_btn.setEnabled(True)
            self.camera_stop_btn.setEnabled(False)
            self.camera_status_indicator.set_color(Qt.red)
            self.camera_status_label.setText("Camera Stream: OFF")

        if TELEOP_PROCESS and TELEOP_PROCESS.poll() is not None:
            exit_code = TELEOP_PROCESS.poll()
            self.update_status(f"Teleop process has exited with code {exit_code}.")
            TELEOP_PROCESS = None
            self.teleop_start_btn.setEnabled(True)
            self.teleop_stop_btn.setEnabled(False)

        if COLLECT_PROCESS and COLLECT_PROCESS.poll() is not None:
            exit_code = COLLECT_PROCESS.poll()
            self.update_status(f"Data Collection process has exited with code {exit_code}.")
            COLLECT_PROCESS = None
            self.collect_start_btn.setEnabled(True)
            self.collect_stop_btn.setEnabled(False)
        
        if self.use_last_num_checkbox.isChecked():
            latest_demo_num = self.check_latest_data_num(os.path.join(self.base_path_input.text(), self.task_input.text().strip()))
            self.demo_num_input.setText(str(latest_demo_num))
    
        if DEPLOY_PROCESS and DEPLOY_PROCESS.poll() is not None:
            exit_code = DEPLOY_PROCESS.poll()
            self.update_status(f"Deploy process has exited with code {exit_code}.")
            DEPLOY_PROCESS = None
            self.deploy_start_btn.setEnabled(True)
            self.deploy_stop_btn.setEnabled(False)
            self.deploy_status_indicator.set_color(Qt.red)
    
    def closeEvent(self, event):
        print("GUI closing. Stopping active processes...")
        self.stop_camera()
        self.stop_teleop()
        self.stop_collect()
        self.stop_deploy()
        self.timer.stop()
        super().closeEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    panel = ControlPanel()
    panel.show()
    sys.exit(app.exec_())