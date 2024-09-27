// Copyright 2016-2024 FRC 5829, FRC 6328
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.EnvironmentalConstants;
import org.littletonrobotics.junction.AutoLogOutput;

// ! TODO Seperate target positions but they're controlled by one so that they can be controlled
// ! seperately from smartdashboard

/** This subsystem is for the chain Elevator. */
public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  /** Creates a new Elevator. */
  public Elevator(ElevatorIO io) {
    this.io = io;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (EnvironmentalConstants.currentMode) {
      case REAL:
        io.configurePID(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
      case REPLAY:
        io.configurePID(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        break;
      case SIM:
        io.configurePID(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  /** Run open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Sets the targeted PID position.
   *
   * @param position meters.
   */
  public void runTargetPosition(double position) {
    io.setTargetPosition(position);
  }

  /**
   * Sets the targeted PID position.
   *
   * @param positionLeft meters.
   * @param positionRight meters.
   */
  public void runTargetPosition(double positionLeft, double positionRight) {
    io.setTargetPosition(positionLeft, positionRight);
  }

  /** Stops the Elevator. */
  public void stop() {
    io.stop();
  }

  /** Returns the current velocity in ms-1. */
  @AutoLogOutput(key = "Elevator/LeftVelocity")
  public double getLeftVelocity() {
    return inputs.leftVelocity;
  }

  /** Returns the current right velocity in ms-1. */
  @AutoLogOutput(key = "Elevator/RightVelocity")
  public double getRightVelocity() {
    return inputs.rightVelocity;
  }

  @AutoLogOutput(key = "Elevator/LeftPositionRad")
  public double getLeftPosition() {
    return inputs.leftPosition;
  }

  @AutoLogOutput(key = "Elevator/RightPositionRad")
  public double getRightPosition() {
    return inputs.rightPosition;
  }

  @AutoLogOutput(key = "Elevator/TargetPosition")
  public double getTargetPosition() {
    return inputs.targetPosition;
  }

  /** Returns the current velocity in meters per second. */
  public double getCharacterizationVelocity() {
    return 0.5 * (inputs.leftVelocity + inputs.rightVelocity);
  }
}
