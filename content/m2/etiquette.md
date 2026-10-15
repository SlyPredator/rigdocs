# Robot Etiquette

Guidelines and standard operating procedures for safely handling, operating, and working around the robot in the lab.

## 1. Physical Safety and Workspace Awareness

- **Clear the Area**: Ensure the operating floor is completely free of loose cables, tools, packaging materials, or liquids before enabling motor power.

- **Pinch Hazards**: The actuator linkages and joint assemblies deliver high torque. Never place fingers, hands, or tools near joints while motors are energized.

```{warning}
Never attempt to lift the robot when in trot mode, switch to stance mode first
```

## 2. Handling and Transport

- **Carrying the Robot**: Always lift the robot in stance mode, not in trot mode. If needed, switch to stance mode using the joystick and then lift the robot.

- **Resting Position**: When not actively under test, ensure the robot is powered down and placed in a stable prone (crouched) position.

## 3. Battery and Power Discipline

- **Power-Up Sequence**:
  1. Inspect the battery pack and wiring harness for any signs of physical damage or swelling.
  2. Secure the battery firmly into the chassis mount.
  3. Connect the primary power connector securely.
  4. Turn on the main system power switch.
  5. Wait for the robot to stand up automatically, indicating it is ready for joystick commands.
- **Joystick Calibration**:
  Calibrate the gamepad by moving:
  1. Both joysticks around fully twice.
  2. All 4 back buttons once.
  
- **Power-Down Sequence**:
  1. Transition the robot to a resting prone position and disarm motors.
  2. Turn off the main system power switch.
  3. Disconnect the battery connector.
  4. Fold the robot to its crouched state.
- **Charging Protocol**: Always charge batteries in a safe area away from flammables. Never leave charging battery packs unattended, and do not deplete cells below their critical voltage thresholds.

## 4. Lab and Network Etiquette

- **Announce Intended Actions**: Verbally notify lab members before enabling motors, standing up, or initiating gait sequences.
- **Network Coordination**: Only one host workstation should transmit control commands or interact with the robot's control nodes at a time to avoid conflicting command streams.
- **Post-Operation Cleanliness**: Return the robot to its case, stow tether cables, and store discharged batteries in the designated charging bay.

