// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Interface for physical override/toggle switches on a dedicated operator console (HID port 5).
 *
 * <p>Button layout assumed:
 *
 * <ul>
 *   <li>Buttons 1–3: driver-side switches (index 0–2, left to right)
 *   <li>Buttons 4–5: multi-directional switch (4 = left, 5 = right)
 *   <li>Buttons 8–12: operator-side switches (index 0–4, left to right)
 * </ul>
 *
 * <p>Suggested mappings:
 *
 * <ul>
 *   <li>Driver switch 0 → robot-relative drive
 *   <li>Driver switch 1 → superstructure coast (only takes effect when disabled)
 *   <li>Operator switch 0 → disable auto flywheel spin-up
 *   <li>Multi-direction switch → lost/won auto override for game-state logic
 * </ul>
 */
public class OverrideSwitches {
  private final GenericHID joystick;

  public OverrideSwitches(int port) {
    joystick = new GenericHID(port);
  }

  /** Returns whether the override console is connected. */
  public boolean isConnected() {
    return joystick.isConnected();
  }

  /**
   * Returns the raw state of a driver-side switch.
   *
   * @param index 0–2 from left to right
   */
  public boolean getDriverSwitch(int index) {
    if (index < 0 || index > 2) {
      throw new RuntimeException("Invalid driver override index " + index + ". Must be 0–2.");
    }
    return joystick.getRawButton(index + 1);
  }

  /**
   * Returns the raw state of an operator-side switch.
   *
   * @param index 0–4 from left to right
   */
  public boolean getOperatorSwitch(int index) {
    if (index < 0 || index > 4) {
      throw new RuntimeException("Invalid operator override index " + index + ". Must be 0–4.");
    }
    return joystick.getRawButton(index + 8);
  }

  /** Returns the state of the multi-directional switch. */
  public MultiDirectionSwitchState getMultiDirectionSwitch() {
    if (joystick.getRawButton(4)) return MultiDirectionSwitchState.LEFT;
    if (joystick.getRawButton(5)) return MultiDirectionSwitchState.RIGHT;
    return MultiDirectionSwitchState.NEUTRAL;
  }

  /**
   * Returns a {@link Trigger} for a driver-side switch.
   *
   * @param index 0–2 from left to right
   */
  public Trigger driverSwitch(int index) {
    return new Trigger(() -> getDriverSwitch(index));
  }

  /**
   * Returns a {@link Trigger} for an operator-side switch.
   *
   * @param index 0–4 from left to right
   */
  public Trigger operatorSwitch(int index) {
    return new Trigger(() -> getOperatorSwitch(index));
  }

  /** Returns a {@link Trigger} that is active when the multi-directional switch is pushed left. */
  public Trigger multiDirectionSwitchLeft() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.LEFT);
  }

  /** Returns a {@link Trigger} that is active when the multi-directional switch is pushed right. */
  public Trigger multiDirectionSwitchRight() {
    return new Trigger(() -> getMultiDirectionSwitch() == MultiDirectionSwitchState.RIGHT);
  }

  /** The position state of the multi-directional switch. */
  public enum MultiDirectionSwitchState {
    LEFT,
    NEUTRAL,
    RIGHT
  }
}
