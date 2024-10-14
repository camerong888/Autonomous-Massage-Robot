# ui_design.py

from PyQt6 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(400, 300)

        # Central widget
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        MainWindow.setCentralWidget(self.centralwidget)

        # Vertical layout for the central widget
        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)

        # Title Label
        self.label_title = QtWidgets.QLabel(self.centralwidget)
        self.label_title.setText("Autonomous Massage Robot Control Panel")
        self.label_title.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        self.label_title.setFont(font)
        self.verticalLayout.addWidget(self.label_title)

        # GroupBox for Muscle Group Selection
        self.groupBox_muscle = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_muscle.setTitle("Select Sore Muscle Groups")
        self.verticalLayout_muscle = QtWidgets.QVBoxLayout(self.groupBox_muscle)

        self.checkBox_neck = QtWidgets.QCheckBox("Neck")
        self.checkBox_shoulders = QtWidgets.QCheckBox("Shoulders")
        self.checkBox_back = QtWidgets.QCheckBox("Back")
        self.checkBox_legs = QtWidgets.QCheckBox("Legs")

        self.verticalLayout_muscle.addWidget(self.checkBox_neck)
        self.verticalLayout_muscle.addWidget(self.checkBox_shoulders)
        self.verticalLayout_muscle.addWidget(self.checkBox_back)
        self.verticalLayout_muscle.addWidget(self.checkBox_legs)

        self.verticalLayout.addWidget(self.groupBox_muscle)

        # GroupBox for Massage Intensity
        self.groupBox_intensity = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_intensity.setTitle("Select Massage Intensity")
        self.verticalLayout_intensity = QtWidgets.QVBoxLayout(self.groupBox_intensity)

        self.slider_intensity = QtWidgets.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.slider_intensity.setMinimum(1)
        self.slider_intensity.setMaximum(10)
        self.slider_intensity.setValue(5)
        self.slider_intensity.setTickPosition(QtWidgets.QSlider.TickPosition.TicksBelow)
        self.slider_intensity.setTickInterval(1)

        self.label_intensity = QtWidgets.QLabel("Intensity: 5")
        self.label_intensity.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_intensity.addWidget(self.slider_intensity)
        self.verticalLayout_intensity.addWidget(self.label_intensity)

        self.verticalLayout.addWidget(self.groupBox_intensity)

        # Horizontal layout for Start and Stop buttons
        self.horizontalLayout_buttons = QtWidgets.QHBoxLayout()

        self.pushButton_start = QtWidgets.QPushButton("Start Massage")
        self.pushButton_stop = QtWidgets.QPushButton("Stop Massage")

        self.horizontalLayout_buttons.addWidget(self.pushButton_start)
        self.horizontalLayout_buttons.addWidget(self.pushButton_stop)

        self.verticalLayout.addLayout(self.horizontalLayout_buttons)

        # Status Bar
        MainWindow.setStatusBar(QtWidgets.QStatusBar(MainWindow))

        # Connect signals and slots
        self.slider_intensity.valueChanged.connect(self.updateIntensityLabel)

    def updateIntensityLabel(self, value):
        self.label_intensity.setText(f"Intensity: {value}")
