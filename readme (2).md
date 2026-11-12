# ROV Telemetry Ground Station
**Coratia Technologies - Software Internship Technical Assessment**

This project is a native Python desktop application that simulates a small underwater vehicle and provides an operator dashboard to monitor and control it. It consists of two standalone components communicating over local UDP sockets via the standard MAVLink protocol.

## Architecture
1. **`pixhawk_sim.py`**: A headless backend physics simulator acting as the ROV's onboard flight controller. It simulates hydrodynamic drag, a 200ms thruster-response delay, PID controllers for depth/heading hold, and enforces safety interlocks (thrust zeroes upon disarm or 1 second of lost manual input).
2. **`gcs_dashboard.py`**: The Ground Control Station UI built with PyQt5. It features a live telemetry dashboard, NavBall/Compass orientation widgets, interactive Map (OpenStreetMap), Depth-vs-Time plot, and Xbox Controller / Keyboard polling.

## Prerequisites & Setup
Ensure you have Python 3.8+ installed. You will need to install the following dependencies:

```bash
pip install PyQt5 PyQtWebEngine pyqtgraph pymavlink pygame numpy
```

## How to Run
Because this project utilizes a genuine MAVLink UDP loopback, you must run both scripts simultaneously.

1. **Start the Simulator (Backend)**
   Open a terminal and run:
   ```bash
   python pixhawk_sim.py
   ```
2. **Start the GCS Dashboard (UI)**
   Open a second terminal and run:
   ```bash
   python gcs_dashboard.py
   ```

### Controls
* **Xbox Controller**: Left Stick (Lateral/Forward), Right Stick (Yaw/Vertical), Button 'A' (Arm), Button 'B' (Disarm).
* **Keyboard Fallback**: `W/S` (Forward/Back), `A/D` (Lateral), `Up/Down` (Surface/Dive), `Left/Right` (Yaw).

## AI Tools Disclosure
*Per the technical assessment requirements, here is the disclosure of AI tool usage for this project:*

* **Tool Used:** Google Gemini
* **How it was used:** 
  * Assisted in structuring the local MAVLink UDP loopback architecture (bypassing the need for a heavy Linux ArduSub SITL environment on Windows).
  * Assisted in formulating the physics math for the ROV simulator (first-order low-pass filter for thruster delay and standard linear/quadratic hydrodynamic drag equations).
  * Helped translate my existing ROS UI layout into a PyMavlink-compatible format.