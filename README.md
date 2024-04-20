# Inveter Control And Power Management System 

## Overview

This Inveter Control and Power Management System is designed to monitor and control various electrical parameters and devices. It includes functionalities for measuring AC output voltages, currents, and power across three phases (U, V, W), monitoring battery voltage, managing MOSFET temperatures, and controlling an inverter, fans, and contactors based on specific conditions. The system utilizes an Arduino or compatible microcontroller along with several sensors and a Nextion touch display for interactive data visualization.

## Features

- **AC Measurement:** Calculates and displays AC voltage, current, and power for three phases.
- **Battery Monitoring:** Checks and displays battery voltage and includes safety features for low battery conditions.
- **Temperature Monitoring:** Measures and displays temperatures of MOSFETs and includes safety features for overheating.
- **Inverter Control:** Controls the inverter based on battery voltage and temperature conditions.
- **Interactive Display:** Utilizes a Nextion display to show real-time data and system status.
- **Safety Features:** Automatic shutdown for low battery and overheating conditions.

## Hardware Requirements

- Arduino or compatible microcontroller
- Nextion Display
- Energy Monitor Sensors for current measurement (EmonLib)
- Voltage sensors for AC phases and battery
- Temperature sensors for MOSFETs
- Buzzer, fan, and contactor for alerts and controls
- Resistor for voltage divider circuit (specific to battery and MOSFET sensors)

## Software Dependencies

- **[EasyNextionLibrary](https://github.com/Seithan/EasyNextionLibrary)**: For interfacing with the Nextion display.
- **[EmonLib](https://github.com/openenergymonitor/EmonLib)**: For current measurement and calculation.
- Arduino IDE for programming and uploading the code to the microcontroller.

## Installation

1. **Set up the Arduino environment:**
   - Download and install the Arduino IDE from [the Arduino website](https://www.arduino.cc/en/software).
   - Install the EasyNextionLibrary and EmonLib through the Library Manager in the Arduino IDE.

2. **Hardware setup:**
   - Connect the voltage and current sensors to the respective analog pins on the Arduino.
   - Connect the Nextion display to the Serial1 port of the Arduino.
   - Attach the fan, buzzer, and contactor to their designated digital pins.
   - Ensure all components are properly powered.

3. **Programming:**
   - Open the provided code in the Arduino IDE.
   - Select the correct board and port under the Tools menu.
   - Upload the code to the Arduino.

4. **Testing:**
   - Power the system and verify that all readings are displayed correctly on the Nextion display.
   - Test the responsiveness of the touch interface and the functionality of the control and safety features.

## Configuration

Adjust the constants in the code to match the specifications of your sensors and setup:

- **currentCalibFactor:** Calibration factor for the current sensors.
- **ResistorVal:** The value of the resistor used in the voltage divider for the MOSFET temperature sensors.
- **sensor calibration constants (`c1`, `c2`, `c3`):** These should be set according to the temperature sensor's datasheet.

## Usage

- **Monitoring Interface:** View real-time data on the Nextion display, including voltage, current, power, and temperature.
- **Control Interface:** Use the touch screen to manually turn on/off the inverter and control other settings.
- **Alerts:** The system will automatically alert (using the buzzer) and/or shut down (using contactors) based on pre-defined thresholds for battery voltage and temperature.

## Safety Information

Ensure that all electrical connections are secure and handled only when the power is disconnected. Follow proper electrical safety protocols when installing and operating the system.

## Support

For issues and support, please check the issues section of the GitHub repository or submit a new issue for help.

---

By deploying this system, you can efficiently manage and monitor a multi-phase power setup, ensuring operational safety and efficiency.
