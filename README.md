# Patient Fall Detector

## Description

This Patient Fall Detector is a specialized device designed to assist elderly citzens, by alerting caregivers or emergency services in the event of a fall. Unlike traditional fall detectors, this device aims to be cost-effective while providing reliable fall detection capabilities adaptable to various home environments.

## Features

- **Immediate Alerts:** Triggers an alarm within 45 seconds upon detecting a fall event.
- **Adaptability:** Can be attached to walking aids, furniture, or clothing, accommodating different living environments.
- **Cost-Effective:** Designed to be accessible to individuals from diverse financial backgrounds.

## Installation

To set up the Patient Fall Detector, follow these steps:

1. **Hardware Assembly:**
   - Connect the STM32 microcontroller, ADXL345 accelerometer, and MakerHawk speaker according to the circuit diagram provided.
   - Ensure all connections are secure and aligned correctly.

2. **Power On:**
   - Plug the microcontroller into a power source using the provided AC adapter.
   
3. **Clone Repository:**
   - Clone this repository to your local machine using Git.

4. **Open STM CubeIDE:**
   - Launch STM CubeIDE and open the project.

5. **Build and Run:**
   - Use the build and run buttons in STM CubeIDE to compile and upload the code to the STM32 microcontroller.
   - The device will start measuring acceleration values once the code downloads to the board and it detects a change in movement.

An image of the circuit diagram used for assembly is provided below.

<p align="center">
<img src="/Design_Files/design_iteration_2.png" height="60%" width="60%">
</p>

## Usage

1. **Deployment:**
   - Position the device upright on a flat surface or attach it securely to a designated area (e.g., furniture, walking aid).

2. **Alert Mechanism:**
   - Upon detecting a fall (acceleration values exceeding thresholds), the speaker will emit an alarm sound up to 80 dB.

3. **Maintenance:**
   - No regular maintenance is required beyond ensuring the device is powered and operational.
   - Store the device in a secure area free from environmental conditions that may interfere with its functionality.

## Files Involved

All changes were made in `Core/Src/main.c`.

## Roadmap

Future updates may include enhancements to:
- Improve speed and accuracy.
- Integrate with mobile application for remote monitoring.

## Authors and Acknowledgments

- Veera Jhuti, Chandise Anderson