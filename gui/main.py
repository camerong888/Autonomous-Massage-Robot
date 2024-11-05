# main.py

import sys
import rospy
from PyQt6 import QtWidgets
from ui_design import Ui_MainWindow
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import String

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up the user interface from ui_design.py
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Initialize ROS node
        rospy.init_node('gui_node', anonymous=True)

        # Publishers
        self.pose_pub = rospy.Publisher('/desired_pose', Pose, queue_size=10)
        self.twist_pub = rospy.Publisher('/desired_twist', Twist, queue_size=10)
        self.control_pub = rospy.Publisher('/control_commands', String, queue_size=10)


        # Connect buttons to functions
        self.ui.pushButton_start.clicked.connect(self.startMassage)
        self.ui.pushButton_stop.clicked.connect(self.stopMassage)
        self.ui.slider_intensity.valueChanged.connect(self.updateIntensityLabel)

    def startMassage(self):
        # Gather selected muscle groups
        muscle_groups = []
        if self.ui.checkBox_neck.isChecked():
            muscle_groups.append('Neck')
        if self.ui.checkBox_shoulders.isChecked():
            muscle_groups.append('Shoulders')
        if self.ui.checkBox_back.isChecked():
            muscle_groups.append('Back')
        if self.ui.checkBox_legs.isChecked():
            muscle_groups.append('Legs')
        if self.ui.checkBox_arms.isChecked():
            muscle_groups.append('Arms')

        # Get selected intensity
        intensity = self.ui.slider_intensity.value()

        self.control_pub.publish("start")

        # Placeholder for starting massage logic
        print("Starting massage with settings:")
        print(f"Muscle Groups: {muscle_groups}")
        print(f"Intensity: {intensity}")

        # Update status bar
        self.statusBar().showMessage("Massage started.")

    def stopMassage(self):
        # Publish control command to stop massage
        self.control_pub.publish("stop")

        rospy.loginfo("Massage stopped.")

        # Update status bar
        self.statusBar().showMessage("Massage stopped.")

    def closeEvent(self, event):
        # Handle GUI Closure Gracefully
        self.stopMassage()
        event.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.setWindowTitle("Autonomous Massage Robot")
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
