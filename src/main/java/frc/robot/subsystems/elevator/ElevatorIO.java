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

import frc.robot.Constants.ElevatorConstants;
import org.littletonrobotics.junction.AutoLog;

/** Interface for operative components of (chain) Elevator subsystem. */
public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public double leftPosition = ElevatorConstants.initialPosition;
    public double leftVelocity = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};

    public double rightPosition = ElevatorConstants.initialPosition;
    public double rightVelocity = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};

    public double targetPosition = ElevatorConstants.initialPosition;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop for the specified position. */
  public default void setTargetPosition(double position) {}

  /** Run closed loop for the specified position. */
  public default void setTargetPosition(double positionLeft, double positionRight) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
