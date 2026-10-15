# Controlling the Robot

This guide covers building the low-level Cyclone DDS bindings and setting up control interfaces (Joystick Commander or the M2 API) to command the robot.

## 1. Prerequisites

```{important}
The network profile configuration from [Receiving Topics](receiving_topics.md) must be completed and active. You should be able to reach `192.168.123.3` over Ethernet.
```

## 2. Setting Up the Python Environment

Create an isolated virtual environment and activate it:

```bash
python3 -m venv ~/m2_control_env
source ~/m2_control_env/bin/activate
```

## 3. Building Cyclone DDS from Source

Clone and compile the Eclipse Cyclone DDS C library locally:

```bash
git clone https://github.com/eclipse-cyclonedds/cyclonedds
cd cyclonedds && mkdir build install && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=../install
cmake --build . --config RelWithDebInfo --target install
cd ../..
export CYCLONEDDS_HOME="$(pwd)/cyclonedds/install"
```

```{tip}
To avoid exporting `CYCLONEDDS_HOME` on every terminal session, add it to your `~/.bashrc`:
```bash
echo "export CYCLONEDDS_HOME=\"$(pwd)/cyclonedds/install\"" >> ~/.bashrc
```

## 4. Installing Python Bindings

With your virtual environment activated and `CYCLONEDDS_HOME` set, install `cyclonedds-python`:

```bash
pip install git+https://github.com/eclipse-cyclonedds/cyclonedds-python
```

## 5. Verifying Topic Discovery

Verify that the robot's topics are being discovered by your host environment:

```bash
cyclonedds ls
```

*(Or via ROS 2 if sourced: `ros2 topic list`)*

Ensure the relevant telemetry and command channels are listed in the output.

---

## 6. Control Interfaces

```{warning}
This part is still in an experimental and live stage, further information or corrections will be added as and when verified to be true.
```

Once topic communication is confirmed, you can control the robot using either the Joystick Commander or the M2 API.

### Option A: Joystick Commander

Use a connected gamepad / joystick to send velocity and posture commands to the robot:

```bash
# Launch the joystick commander interface
# [Insert joystick node / command execution here]
```

```{warning}
Always ensure the robot is in a safe space or suspended on a rig before engaging joystick motion. Test emergency stop / disarm triggers first.
```

### Option B: M2 API

Use the Python M2 API to programmatically command gaits, waypoints, and posture:

```python
# Example: Basic M2 API Initialization
# from m2_api import M2Robot

# robot = M2Robot()
# robot.connect()
# robot.stand_up()
```

```{note}
<!-- USER INPUT PLACEHOLDER -->
**M2 API & Joystick Details**:
Insert your custom joystick configuration bindings or specific M2 API package installation and usage script here.
```
