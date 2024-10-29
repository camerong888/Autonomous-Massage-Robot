# Autonomous Massage Robot Using Kinova Gen3 Arm

## Overview

This project aims to develop an innovative system that combines robotics, machine vision, and user-friendly interfaces to deliver personalized massage therapy. By integrating a Kinova Gen3 robotic arm with a massage gun attachment and an Intel RealSense Depth Camera D455, the system can autonomously plan and execute massage paths tailored to the user's body position and preferences.

## Repository Contents

* gui/

  * PyQt6 GUI code for user interaction.
* control/

  * Control code for the Kinova Gen3 robotic arm and impedance controller implementation.
* vision/

  * Machine vision code for depth camera processing and body position estimation.
* requirements.txt

  * List of Python dependencies.
* README.md

  * Project overview and instructions.

## Installation

### Prerequisites

* Operating System

  * Ubuntu 20.04 LTS or compatible Linux distribution.
* Python

  * Version 3.8 or higher.

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

## Usage

### Install Dependencies

```
pip install -r requirements.txt
```

### Launch the GUI

```
python gui/main.py
```

## Project Structure

```
autonomous-massage-robot/
├── gui/
│   ├── main.py
│   └── ui_design.py
├── control/
│   └── 
├── vision/
│   └── 
├── requirements.txt
└── README.md
```
