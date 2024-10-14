# main.py

import sys
from PyQt6 import QtWidgets
from ui_design import Ui_MainWindow

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        # Set up the user interface from ui_design.py
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Connect buttons to functions
        self.ui.pushButton_start.clicked.connect(self.startMassage)
        self.ui.pushButton_stop.clicked.connect(self.stopMassage)

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

        # Get selected intensity
        intensity = self.ui.slider_intensity.value()

        # Placeholder for starting massage logic
        print("Starting massage with settings:")
        print(f"Muscle Groups: {muscle_groups}")
        print(f"Intensity: {intensity}")

        # Update status bar
        self.statusBar().showMessage("Massage started.")

    def stopMassage(self):
        # Placeholder for stopping massage logic
        print("Stopping massage.")

        # Update status bar
        self.statusBar().showMessage("Massage stopped.")

def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.setWindowTitle("Autonomous Massage Robot")
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
