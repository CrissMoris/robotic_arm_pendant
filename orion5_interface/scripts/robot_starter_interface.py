#!/usr/bin/python3

import math
import time
import copy
import threading
from ErrorMessages import ZeroErrMessages
from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtGui import QPixmap, QFont, QIcon
from PyQt5.QtWidgets import QPushButton, QComboBox, QLabel
from RosThreads import *
import rospy, rospkg
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
from controller_manager_msgs.srv import SwitchController, ListControllers
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

"""
 @author: Mehmet Kahraman
 @date: 04.04.2024
 @about: Robot Starter GUI — Orion 5 kolu
         Görev sistemi: NamedPose + senaryo tabanlı genel Pick/Place
"""


# ══════════════════════════════════════════════════════════════════════════════
#  Pozisyon tanımları
#  Tüm açılar derece cinsinden.  send_joint_target() içinde radyana çevrilir.
# ══════════════════════════════════════════════════════════════════════════════

class NamedPose:
    """İsimlendirilmiş eklem açısı seti."""
    def __init__(self, name: str, joints_deg: list, duration: float = 3.0):
        self.name = name
        self.joints_deg = joints_deg   # [J1..J6] derece
        self.duration = duration       # saniye


# ── Temel pozisyonlar ─────────────────────────────────────────────────────────
HOME    = NamedPose("Home",         [0,   0,   0, 0,  0,  0], duration=3.0)
UP_SAFE = NamedPose("Up (güvenli)", [0, -90,   0, 0,  0,  0], duration=3.0)

# ── Barista senaryosu ─────────────────────────────────────────────────────────
#    Robot kahve makinesinden fincanı alır, servis tepsisine bırakır.
SCENARIO_BARISTA = {
    "label": "Barista (kahve servisi)",
    "pick":  NamedPose("Barista Pick  — kahve makinesi", [  0, -45,  90, 0, 45, 90], duration=3.0),
    "place": NamedPose("Barista Place — servis tepsisi", [ 90, -30,  70, 0, 20,  0], duration=3.0),
}

# ── Konveyör bandı senaryosu ──────────────────────────────────────────────────
#    Robot giriş bandından ürünü alır, 90° dönerek yan banda bırakır.
SCENARIO_CONVEYOR = {
    "label": "Konveyör (bant aktarımı)",
    "pick":  NamedPose("Conveyor Pick  — giriş bandı", [  0, -50,  95, 0,  5, 90], duration=3.0),
    "place": NamedPose("Conveyor Place — çıkış bandı", [ 90, -50,  95, 0,  5,  0], duration=3.0),
}

# ── Genel varsayılan (geriye dönük uyumluluk) ─────────────────────────────────
SCENARIO_DEFAULT = {
    "label": "Varsayılan Pick/Place",
    "pick":  NamedPose("Pick (genel)",  [  0, -45, 90, 0,  0, 90], duration=3.0),
    "place": NamedPose("Place (genel)", [ 90, -45, 90, 0, 45,  0], duration=3.0),
}

# Açılır menüde gösterilecek senaryolar (sıra önemlidir)
ALL_SCENARIOS = [SCENARIO_DEFAULT, SCENARIO_BARISTA, SCENARIO_CONVEYOR]


# ══════════════════════════════════════════════════════════════════════════════
#  Ana arayüz sınıfı
# ══════════════════════════════════════════════════════════════════════════════

class RobotInterface(QtWidgets.QMainWindow):

    def __init__(self, ros_pkg):
        super(RobotInterface, self).__init__()

        rospy.init_node("Robot_starter_interface_node")
        pkg_name = ros_pkg
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path(pkg_name)
        uic.loadUi(self.pkg_path + "/assets/interface.ui", self)

        self.selected_robot = None
        self.robot_enable_state = False
        self.robot_fault_state = False
        self.is_fault_received = False
        self.allow_home_first = True

        # Görev durumu — execute_task() birden fazla çağrıyı engeller
        self._task_running = False

        self.arm_controller_name = ""
        self.joint_traj_control_name = ""

        self.error_message_object = ZeroErrMessages()

        self.home_speed = 15
        self.home_pos = [0, 0, 0, 0, 0, 0]
        self.curr_pos = [0, 0, 0, 0, 0, 0]
        self.joint_names = ["joint_1", "joint_2", "joint_3",
                             "joint_4", "joint_5", "joint_6"]

        self.joint_pos_control_name  = "orion_arm_joint_group_pos_controller"
        self.joint_traj_control_name = "orion_arm_pos_joint_traj_controller"
        self.joint_traj_control_topic = "/" + self.joint_traj_control_name + "/command"
        self.joint_effort_control_name = "orion_arm_effort_controller"

        # Aktif senaryo (açılır menüden değiştirilebilir)
        self.active_scenario = SCENARIO_DEFAULT

        self.init_ros_services()
        self.init_elements()

        rospy.loginfo("Interface is ready.")
        self.show()

    # ─────────────────────────────────────────────────────────────────────────
    #  ROS
    # ─────────────────────────────────────────────────────────────────────────
    def init_ros_services(self):
        self.enable_state_topic = "/soem_ros_control/ethercat/enable_state"
        rospy.Subscriber(self.enable_state_topic, Bool, self.enable_sub_callback)

        self.fault_state_topic = "/soem_ros_control/ethercat/fault_state"
        rospy.Subscriber(self.fault_state_topic, Bool, self.fault_sub_callback)

        self.joint_state_topic = "/joint_states"
        rospy.Subscriber(self.joint_state_topic, JointState, self.joint_state_callback)

        self.traj_publisher = rospy.Publisher(
            self.joint_traj_control_topic, JointTrajectory, queue_size=10)

        self.switch_srv_topic = "/controller_manager/switch_controller"
        self.switch_service = rospy.ServiceProxy(self.switch_srv_topic, SwitchController)

        self.list_ctrls_srv_topic = "/controller_manager/list_controllers"
        self.list_ctrls_srv_client = rospy.ServiceProxy(self.list_ctrls_srv_topic, ListControllers)

        self.fault_srv_topic = "/soem_ros_control/ethercat/clear_fault"
        self.fault_clear_srv_client = rospy.ServiceProxy(self.fault_srv_topic, Trigger)

        self.enable_srv_topic = "/soem_ros_control/ethercat/enable_robot"
        self.enable_robot_srv_client = rospy.ServiceProxy(self.enable_srv_topic, Trigger)

        self.disable_srv_topic = "/soem_ros_control/ethercat/disable_robot"
        self.disable_robot_srv_client = rospy.ServiceProxy(self.disable_srv_topic, Trigger)

        self.warn_topic = "/soem_ros_control/ethercat/get_warning_code"
        self.get_warn_src_client = rospy.ServiceProxy(self.warn_topic, Trigger)

        self.err_topic = "/soem_ros_control/ethercat/get_error_code"
        self.get_err_src_client = rospy.ServiceProxy(self.err_topic, Trigger)

        rospy.on_shutdown(self.on_shutdown)

    # ─────────────────────────────────────────────────────────────────────────
    #  UI
    # ─────────────────────────────────────────────────────────────────────────
    def init_elements(self):
        self.setWindowIcon(QIcon(self.pkg_path + "/assets/mery6_app_icon.png"))
        self.setMaximumSize(774, 540)
        self.setMinimumSize(774, 540)

        font_size = 14
        self.textBrowser.setStyleSheet("font-size: {}px;".format(str(font_size)))

        font = QFont('Arial', 12)
        font.setItalic(True)
        self.orion_label.setFont(font)

        self.state_label.setAlignment(QtCore.Qt.AlignCenter)
        self.set_state_label_off()

        self.pixmap = QPixmap(self.pkg_path + "/assets/mery6_horizontal_logo.png")
        self.label.setPixmap(self.pixmap)
        self.label.setScaledContents(True)

        # ── Mevcut butonlar ───────────────────────────────────────────────────
        self.select_button.clicked.connect(self.select_robot)
        self.clear_fault_button.clicked.connect(self.clear_fault)
        self.enable_robot_button.clicked.connect(self.enable_robot)
        self.disable_robot_button.clicked.connect(self.disable_robot)

        self.home_button.pressed.connect(self.start_go_home)
        self.home_button.released.connect(self.stop_go_home)
        self.home_button.setEnabled(False)

        self.clear_log_button.clicked.connect(self.clear_log)
        self.space_log_button.clicked.connect(self.space_log)

        # ── Senaryo seçici (açılır menü + etiket) ────────────────────────────
        self.scenario_label = QLabel("Görev Senaryosu:")
        self.scenario_label.setFont(QFont('Arial', 10))

        self.scenario_combo = QComboBox()
        for sc in ALL_SCENARIOS:
            self.scenario_combo.addItem(sc["label"])
        self.scenario_combo.currentIndexChanged.connect(self._on_scenario_changed)

        scenario_layout = QtWidgets.QHBoxLayout()
        scenario_layout.addWidget(self.scenario_label)
        scenario_layout.addWidget(self.scenario_combo)

        # ── Simülasyon görev butonları ────────────────────────────────────────
        self.btn_home    = QPushButton("🏠 Home")
        self.btn_up      = QPushButton("⬆ Up (Güvenli)")
        self.btn_linear  = QPushButton("➡ Lineer (+J1)")
        self.btn_pick    = QPushButton("🤏 Pick (Al)")
        self.btn_place   = QPushButton("📦 Place (Bırak)")
        self.btn_task    = QPushButton("▶ Görevi Çalıştır")

        # Görev butonunu vurgula
        self.btn_task.setStyleSheet(
            "QPushButton { background-color: #2a7a2a; color: white; font-weight: bold; }"
            "QPushButton:disabled { background-color: #888; color: #ccc; }"
        )

        self.btn_home.clicked.connect(self.go_home_sim)
        self.btn_up.clicked.connect(self.go_up_sim)
        self.btn_linear.clicked.connect(self.go_linear_sim)
        self.btn_pick.clicked.connect(lambda: self.go_pick_sim())
        self.btn_place.clicked.connect(lambda: self.go_place_sim())
        self.btn_task.clicked.connect(self.execute_task)

        # Butonları bir container içine yerleştir
        btn_layout = QtWidgets.QVBoxLayout()
        btn_layout.setSpacing(4)
        btn_layout.addLayout(scenario_layout)
        btn_layout.addWidget(self.btn_home)
        btn_layout.addWidget(self.btn_up)
        btn_layout.addWidget(self.btn_linear)
        btn_layout.addWidget(self.btn_pick)
        btn_layout.addWidget(self.btn_place)
        btn_layout.addWidget(self.btn_task)

        btn_container = QtWidgets.QWidget(self)
        btn_container.setLayout(btn_layout)
        btn_container.setGeometry(QtCore.QRect(10, 330, 210, 200))
        btn_container.show()

        # ── ROS thread'leri ───────────────────────────────────────────────────
        self.connect_thread = ConnectionWorkerThread(self.enable_state_topic)
        self.connect_thread.thread_signal.connect(self.thread_result_callback)
        self.connect_thread.connection_signal.connect(self.set_connect_state_cb)
        self.connect_thread.start()

        self.receive_error_thread = ReceiveErrorWorkerThread(
            self.get_err_src_client, self.error_message_object)
        self.receive_error_thread.thread_signal.connect(self.thread_result_callback)
        self.receive_error_thread.error_signal.connect(self.error_msg_cb)

        self.clear_fault_thread = ClearFaultWorkerThread(self.fault_clear_srv_client)
        self.clear_fault_thread.thread_signal.connect(self.thread_result_callback)
        self.clear_fault_thread.error_signal.connect(self.error_msg_cb)
        self.clear_fault_thread.home_state_signal.connect(self.home_state_cb)

        self.enable_robot_thread = EnableRobotWorkerThread(
            self.enable_robot_srv_client, self.switch_service, self.list_ctrls_srv_client)
        self.enable_robot_thread.thread_signal.connect(self.thread_result_callback)
        self.enable_robot_thread.home_state_signal.connect(self.home_state_cb)
        self.enable_robot_thread.joint_traj_control_name = self.joint_traj_control_name

        self.disable_robot_thread = DisableRobotWorkerThread(
            self.disable_robot_srv_client, self.switch_service, self.list_ctrls_srv_client)
        self.disable_robot_thread.thread_signal.connect(self.thread_result_callback)
        self.disable_robot_thread.home_state_signal.connect(self.home_state_cb)

        self.textBrowser.append("Interface is ready. Please select a controller.")

    # ─────────────────────────────────────────────────────────────────────────
    #  Senaryo seçimi
    # ─────────────────────────────────────────────────────────────────────────
    def _on_scenario_changed(self, index):
        self.active_scenario = ALL_SCENARIOS[index]
        self._log("Senaryo seçildi: {}".format(self.active_scenario["label"]))

    # ─────────────────────────────────────────────────────────────────────────
    #  Hareket primitifi
    # ─────────────────────────────────────────────────────────────────────────
    def send_joint_target(self, degrees, duration):
        """
        Tek gerçek hareket komutu.  Her pozisyon fonksiyonu bunu çağırır.
        degrees  : [J1..J6] derece listesi
        duration : saniye cinsinden süre
        """
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = [math.radians(d) for d in degrees]
        point.velocities = [0.0] * 6
        point.time_from_start = rospy.Duration(duration)
        traj_msg.points.append(point)
        self.traj_publisher.publish(traj_msg)
        self._log("  → Hedef gönderildi: {}".format(
            [round(d, 1) for d in degrees]))

    def _move_to(self, pose: NamedPose):
        """NamedPose nesnesine git ve log yaz."""
        self._log(">>> {}: {}".format(pose.name, pose.joints_deg))
        self.send_joint_target(pose.joints_deg, duration=pose.duration)

    # ─────────────────────────────────────────────────────────────────────────
    #  Temel pozisyon butonları
    # ─────────────────────────────────────────────────────────────────────────
    def go_home_sim(self):
        self._move_to(HOME)

    def go_up_sim(self):
        """Tüm hareketlerden önce güvenli ara pozisyon."""
        self._move_to(UP_SAFE)

    def go_linear_sim(self, axis: int = 0, delta_deg: float = 30.0):
        """
        Mevcut pozisyondan belirtilen eksende delta_deg kadar döndür.
        axis      : 0..5 eklem indeksi (varsayılan 0 → J1)
        delta_deg : eklenecek derece miktarı (negatif olabilir)
        """
        current = list(self.curr_pos[:6]) if self.curr_pos else [0.0] * 6
        target_deg = [math.degrees(a) for a in current]
        target_deg[axis] += delta_deg
        self._log(">>> Lineer hareket — J{} {:+.1f}°".format(axis + 1, delta_deg))
        self.send_joint_target(target_deg, duration=2.0)

    # ─────────────────────────────────────────────────────────────────────────
    #  Genel Pick / Place
    # ─────────────────────────────────────────────────────────────────────────
    def go_pick_sim(self, pose: NamedPose = None):
        """
        Genel 'al' hareketi.
        pose=None ise aktif senaryonun pick pozisyonu kullanılır.
        Farklı bir pozisyon vermek için:
            self.go_pick_sim(NamedPose("Özel al", [10, -40, 85, 0, 10, 90]))
        """
        target = pose or self.active_scenario["pick"]
        self._move_to(target)

    def go_place_sim(self, pose: NamedPose = None):
        """
        Genel 'bırak' hareketi.
        pose=None ise aktif senaryonun place pozisyonu kullanılır.
        """
        target = pose or self.active_scenario["place"]
        self._move_to(target)

    # ─────────────────────────────────────────────────────────────────────────
    #  Tam görev sekansı
    # ─────────────────────────────────────────────────────────────────────────
    def execute_task(self,
                     scenario: dict = None,
                     return_home: bool = True,
                     safe_up: bool = True):
        """
        Seçili (veya verilen) senaryonun tam döngüsünü çalıştır:
            [Up →] Pick → Place [→ Home]

        Parametreler
        ────────────
        scenario    : {"label":..., "pick": NamedPose, "place": NamedPose}
                      None ise arayüzdeki açılır menüden seçili senaryo kullanılır.
        return_home : görev sonunda Home'a dön (varsayılan True)
        safe_up     : her taşıma öncesi UP_SAFE'e uğra (varsayılan True)

        Doğrudan çağrı örnekleri:
            self.execute_task()                              # aktif senaryo
            self.execute_task(scenario=SCENARIO_BARISTA)    # barista
            self.execute_task(scenario=SCENARIO_CONVEYOR, return_home=False)
            self.execute_task(scenario={                     # anlık özel
                "label": "Test",
                "pick":  NamedPose("Özel al",    [10, -40, 85, 0, 10, 90]),
                "place": NamedPose("Özel bırak",  [80, -40, 85, 0, 30,  0]),
            })
        """
        if self._task_running:
            self._log("⚠ Görev zaten çalışıyor, lütfen bekleyin.")
            return

        sc = scenario or self.active_scenario
        pick_pose  = sc["pick"]
        place_pose = sc["place"]

        def _run():
            self._task_running = True
            self._set_task_buttons_enabled(False)
            try:
                self._log("═" * 44)
                self._log("▶ Görev: {}".format(sc.get("label", "—")))
                self._log("  Pick  → {}".format(pick_pose.name))
                self._log("  Place → {}".format(place_pose.name))

                # 1. Güvenli yukarı pozisyon
                if safe_up:
                    self._move_to(UP_SAFE)
                    time.sleep(UP_SAFE.duration + 0.3)

                # 2. Pick
                self._log("↓ Alma hareketi...")
                self._move_to(pick_pose)
                time.sleep(pick_pose.duration + 0.3)
                self._log("  ✓ Nesne alındı  [gripper_close() buraya]")

                # 3. Güvenli yukarı (taşıma esnasında çarpışmayı önler)
                if safe_up:
                    self._move_to(UP_SAFE)
                    time.sleep(UP_SAFE.duration + 0.3)

                # 4. Place
                self._log("↓ Bırakma hareketi...")
                self._move_to(place_pose)
                time.sleep(place_pose.duration + 0.3)
                self._log("  ✓ Nesne bırakıldı  [gripper_open() buraya]")

                # 5. Home'a dön
                if return_home:
                    self._move_to(HOME)
                    time.sleep(HOME.duration + 0.3)

                self._log("■ Görev tamamlandı")
                self._log("═" * 44)

            finally:
                self._task_running = False
                self._set_task_buttons_enabled(True)

        threading.Thread(target=_run, daemon=True).start()

    # ─────────────────────────────────────────────────────────────────────────
    #  Yardımcı
    # ─────────────────────────────────────────────────────────────────────────
    def _log(self, msg: str):
        """Thread-safe log yazma (textBrowser ana thread'de güncellenmelidir)."""
        try:
            # PyQt5'te başka thread'den widget güncellemek için invokeMethod
            QtCore.QMetaObject.invokeMethod(
                self.textBrowser, "append",
                QtCore.Qt.QueuedConnection,
                QtCore.Q_ARG(str, msg)
            )
        except Exception:
            print(msg)

    def _set_task_buttons_enabled(self, enabled: bool):
        """Görev çalışırken butonları devre dışı bırak / aktif et."""
        for btn in [self.btn_pick, self.btn_place, self.btn_task,
                    self.btn_home, self.btn_up, self.btn_linear]:
            try:
                QtCore.QMetaObject.invokeMethod(
                    btn, "setEnabled",
                    QtCore.Qt.QueuedConnection,
                    QtCore.Q_ARG(bool, enabled)
                )
            except Exception:
                pass

    # ─────────────────────────────────────────────────────────────────────────
    #  Callbacks ve mevcut fonksiyonlar (değiştirilmedi)
    # ─────────────────────────────────────────────────────────────────────────
    def home_state_cb(self, msg):
        self.home_button.setEnabled(msg)

    def error_msg_cb(self, msg):
        self.error_label.setText(msg)

    def set_state_label(self, state):
        if (state == True) and (self.state_label.text() != "Enabled"):
            self.state_label.setStyleSheet("background-color: green")
            self.state_label.setText("Enabled")
        elif (state == False) and (self.state_label.text() != "Disabled"):
            self.state_label.setStyleSheet("background-color: red")
            self.state_label.setText("Disabled")

    def set_connect_state_cb(self, msg):
        if msg == True:
            self.textBrowser.append("Robot is connected now !!")
        else:
            self.textBrowser.append("Robot connection lost !!")
            self.set_state_label_off()

    def set_state_label_off(self):
        self.state_label.setStyleSheet("background-color: lightgray")
        self.state_label.setText("No Data")

    def joint_state_callback(self, msg):
        self.curr_pos = msg.position

    def thread_result_callback(self, msg):
        self.textBrowser.append(msg)

    def enable_sub_callback(self, msg):
        self.robot_enable_state = msg.data
        self.set_state_label(self.robot_enable_state)

    def fault_sub_callback(self, msg):
        self.robot_fault_state = msg.data
        if self.robot_fault_state == False:
            self.is_fault_received = False
        if (self.robot_fault_state == True) and (self.is_fault_received == False):
            self.is_fault_received = True
            self.receive_error_code()

    def receive_error_code(self):
        if not self.receive_error_thread.isRunning():
            self.receive_error_thread.start()
        else:
            self.textBrowser.append("receive_error_thread is already running !!")

    def clear_log(self):
        self.textBrowser.clear()

    def space_log(self):
        self.textBrowser.append("\n")

    def select_robot(self):
        self.selected_robot = "orion5_arm"
        if self.comboBox.currentText() == "orion5_traj_control":
            self.arm_controller_name = self.joint_traj_control_name
            self.textBrowser.append("orion5_traj_control is initialized.")
        elif self.comboBox.currentText() == "orion5_pos_control":
            self.arm_controller_name = self.joint_pos_control_name
            self.textBrowser.append("orion5_pos_control is initialized.")
        elif self.comboBox.currentText() == "orion5_effort_control":
            self.arm_controller_name = self.joint_effort_control_name
            self.textBrowser.append("orion5_effort_control is initialized.")
        else:
            assert False, "Wrong selection !!"

        if self.allow_home_first == True:
            self.allow_home_first = False
            if (self.robot_enable_state == True) and \
               (self.arm_controller_name == self.joint_traj_control_name):
                self.home_button.setEnabled(True)

    def clear_fault(self):
        if self.selected_robot is None:
            QtWidgets.QMessageBox.information(
                self, "Info", "Please select a robot controller first.")
        else:
            if self.robot_enable_state == True:
                self.disable_robot()
            if not self.clear_fault_thread.isRunning():
                self.clear_fault_thread.start()
            else:
                self.textBrowser.append("clear_fault_thread is already running !!")

    def enable_robot(self):
        if self.selected_robot is None:
            QtWidgets.QMessageBox.information(
                self, "Info", "Please select a robot controller first.")
        else:
            if not self.enable_robot_thread.isRunning():
                self.enable_robot_thread.arm_controller_name = self.arm_controller_name
                self.enable_robot_thread.start()
            else:
                self.textBrowser.append("enable_robot_thread is already running !!")

    def disable_robot(self):
        if self.selected_robot is None:
            QtWidgets.QMessageBox.information(
                self, "Info", "Please select a robot controller first.")
        else:
            if not self.disable_robot_thread.isRunning():
                self.disable_robot_thread.start()
            else:
                self.textBrowser.append("disable_robot_thread is already running !!")

    def start_go_home(self):
        self.textBrowser.append("Going home ...")
        duration = self.calculate_traj_duration()
        self.go_joint_target(self.home_pos, duration)

    def go_joint_target(self, joint_angles, duration):
        """Ham radyan listesiyle doğrudan hedef gönder (mevcut home sistemi için)."""
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.velocities = [0, 0, 0, 0, 0, 0]
        point.time_from_start = rospy.Duration(duration)
        traj_msg.points.append(point)
        self.traj_publisher.publish(traj_msg)

    def calculate_traj_duration(self):
        rad_speed = math.radians(self.home_speed)
        diff = [abs(self.home_pos[i] - self.curr_pos[i]) for i in range(6)]
        max_diff = max(diff)
        duration = max_diff / rad_speed
        return duration

    def stop_go_home(self):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names
        self.traj_publisher.publish(traj_msg)
        self.textBrowser.append("Going home stopped.")

    def on_shutdown(self):
        rospy.logwarn("Shutdown received.")
