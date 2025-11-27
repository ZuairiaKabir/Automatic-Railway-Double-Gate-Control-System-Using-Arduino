# Automatic-Railway-Double-Gate-Control-System-Using-Arduino
An Arduino-based automated railway level-crossing gate control system designed to reduce accidents, minimize human error, and improve efficiency at railway crossings. This project uses ultrasonic sensors and servo motors to detect an approaching train and automatically open/close two railway gates for enhanced safety.

📌 Project Overview

Manual level-crossing gates are prone to human error, leading to severe accidents each year. This project introduces a double-gate automated railway control system using low-cost, reliable components such as Arduino Uno and ultrasonic sensors.

The system automatically:
Detects trains approaching from either side
Closes both gates in the correct sequence
Reopens gates once the train has completely passed
Alerts nearby traffic using buzzers

🎯 Objectives

Increase the reliability of level-crossing systems
Reduce accidents at railway intersections
Minimize human error through automation

🚊 How It Works

Ultrasonic sensors (two on each side) detect the train’s arrival and departure.
Firstly Arduino triggers buzzer 1 and closes Gate 1 using a servo motor.
Then Arduino triggers buzzer 2 and closes Gate 2 after the remaining vehicles clear the zone.
After the train crosses all sensors, both gates reopen automatically.
Servo motors are controlled using PWM signals from the Arduino.

Workflow

Train detected → Buzzer ON → Gate 1 closes → Vehicles move to buffer zone → Gate 2 closes → Train passes → Both gates open.

🛠️ Hardware Requirements

Arduino Uno (2 units)
HC-SR04 Ultrasonic Sensors (4 units)
Servo Motors (for gate control)
Active Buzzers
Jumper Wires (M-M, M-F)
9V Batteries & Connectors
USB Cable
PVC Foam Board (for structural setup)

💻 Software Requirements

Arduino IDE
Custom Arduino Sketch controlling:
Ultrasonic sensing
Servo motor actuation
Buzzer alerts

📈 Expected Outcome

A low-cost, efficient, and reliable automatic double-gate railway control system that:
Reduces closure duration
Minimizes collisions
Eliminates manual dependency
Enhances roadside and railway safety


🤝 Contributors
Zuairia Kabir
Md Spondon Sarwar
Israt Tabassum Kochy
Shamsee Subah
Department of Computer Science & Engineering
Bangladesh University of Professionals (BUP)
