# Autonomous Massage Robot Using Kinova Gen3 Arm

## Overview

This project aims to develop an innovative system that combines robotics, machine vision, and user-friendly interfaces to deliver personalized massage therapy. By integrating a Kinova Gen3 robotic arm with a massage gun attachment and an Intel RealSense Depth Camera D455, the system can autonomously plan and execute massage paths tailored to the user's body position and preferences.

## Features

* Real-Time Body Position Estimation
  * Uses Intel RealSense D455 depth camera for accurate detection of user's body posture and joint positions.
* Personalized Massage Path Planning
  * Generates custom massage paths based on user-specified sore muscle groups and real-time posture data.
* Impedance Control for Safety
  * Implements an impedance controller to ensure safe force application during massage.
* User-Friendly PyQt6 GUI
  * Provides an intuitive interface for users to input preferences and control the massage session.
* Safety Mechanisms
  * Includes emergency stop features and collision avoidance to protect the user.

## Repository Contents

* gui/
  * PyQt6 GUI code for user interaction.
* control/
  * Control code for the Kinova Gen3 robotic arm and impedance controller implementation.
* vision/
  * Machine vision code for depth camera processing and body position estimation.
* impedance_control/
  * Modules related to the impedance controller.
* docs/
  * Documentation, images, and additional resources.
* requirements.txt
  * List of Python dependencies.
* README.md
  * Project overview and instructions.

## Installation

### Prerequisites

* Operating System
  * Ubuntu 20.04 LTS or compatible Linux distribution.
* Hardware
  * Kinova Gen3 robotic arm.
  * Intel RealSense Depth Camera D455.
* Python
  * Version 3.8 or higher.
* Software Dependencies
  * Kinova SDK (Gen3) installed and configured.
  * Intel RealSense SDK 2.0 installed.
  * Python libraries as per requirements.txt.

### Steps

#### Clone the Repository

```
git clone https://github.com/camerong888/autonomous-massage-robot.git
cd autonomous-massage-robot
```

#### Install Python Dependencies

```
pip install -r requirements.txt
```

#### Set Up Kinova SDK

Follow the Kinova SDK Installation Guide to install and configure the SDK.
Ensure the Kinova Gen3 arm is properly connected and recognized by the system.

#### Set Up Intel RealSense SDK

Install the Intel RealSense SDK 2.0 by following the installation instructions.
Test the camera using provided tools to ensure it's working correctly.

#### Verify Hardware Connections

Connect the Kinova Gen3 arm and Intel RealSense D455 camera to your computer.
Ensure all devices are powered on and communicating with the system.

## Usage

### Activate Virtual Enviornment

For macOS/Linux:

```
source venv/bin/activate
```

For Windows Command Prompt:

```
venv\Scripts\activate
```

For Windows PowerShell:

```
venv\Scripts\Activate.ps1
```

### Install Dependencies

```
pip install -r requirements.txt
```

### Launch the GUI

```
python gui/main.py
```

The GUI will automatically detect connected devices.
Verify that the Kinova arm and RealSense camera show as connected.
Configure Massage Settings

Use the GUI to select sore muscle groups and set massage intensity preferences.
Options may include muscle group selection, massage duration, and force settings.
Start the Massage Session

Click the "Start Massage" button.
The system will perform body position estimation and plan the massage path.
Monitor the Session

View real-time feedback on the GUI, including current massage position and status.
Use the "Pause" or "Stop" buttons to control the session as needed.
Safety Features

In case of emergency, use the "Emergency Stop" button to immediately halt all operations.
The system includes collision detection and will stop if an unexpected obstacle is detected.

### Deactiviating Virtual Enviornment

```
deactivate
```

## Project Structure

```
autonomous-massage-robot/
├── gui/
│   ├── main.py
│   ├── ui_design.py
│   └── resources/
├── control/
│   ├── kinova_control.py
│   ├── impedance_controller.py
│   └── utils.py
├── vision/
│   ├── depth_camera.py
│   ├── pose_estimation.py
│   └── camera_calibration.py
├── impedance_control/
│   ├── impedance_algorithm.py
│   └── force_feedback.py
├── docs/
│   ├── images/
│   └── user_manual.pdf
├── config/
│   └── settings.yaml
├── requirements.txt
└── README.md
```
