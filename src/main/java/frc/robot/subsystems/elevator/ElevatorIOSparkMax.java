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

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.ElevatorConstants;

/**
 * This implementation of ElevatorIO is for the chain Elevator, in the case of using two NEO
 * SparkMax motors.
 *
 * <p>Note: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class ElevatorIOSparkMax implements ElevatorIO {

  private final CANSparkMax leftMotor =
      new CANSparkMax(ElevatorConstants.kLeftElevatorMotorId, MotorType.kBrushless);
  private final CANSparkMax rightMotor =
      new CANSparkMax(ElevatorConstants.kRightElevatorMotorId, MotorType.kBrushless);

  private final RelativeEncoder leftRelativeEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightRelativeEncoder = rightMotor.getEncoder();

  private final SparkPIDController leftPID = leftMotor.getPIDController();
  private final SparkPIDController rightPID = rightMotor.getPIDController();

  private final PIDController leftMathPID;
  // private final PIDController rightMathPID;

  private double targetPosition = ElevatorConstants.initialPosition;

  // private double targetPositionLeft = ElevatorConstants.initialPosition;
  // private double targetPositionRight = ElevatorConstants.initialPosition;

  public ElevatorIOSparkMax() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setCANTimeout(250);
    rightMotor.setCANTimeout(250);

    leftMotor.setInverted(true);

    leftMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
    rightMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);

    // leftMotor.burnFlash();
    // rightMotor.burnFlash();

    leftMathPID =
        new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    // rightMathPID =
    //     new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftPosition =
        ElevatorConstants.gearCircumfrence
            * (leftRelativeEncoder.getPosition() / ElevatorConstants.GEAR_RATIO);
    inputs.leftVelocity =
        ElevatorConstants.gearCircumfrence
            * (leftRelativeEncoder.getVelocity() / ElevatorConstants.GEAR_RATIO);
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.leftCurrentAmps = new double[] {leftMotor.getOutputCurrent()};

    inputs.rightPosition =
        ElevatorConstants.gearCircumfrence
            * (rightRelativeEncoder.getPosition() / ElevatorConstants.GEAR_RATIO);
    inputs.rightVelocity =
        ElevatorConstants.gearCircumfrence
            * (rightRelativeEncoder.getVelocity() / ElevatorConstants.GEAR_RATIO);
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.rightCurrentAmps = new double[] {rightMotor.getOutputCurrent()};

    inputs.targetPosition = targetPosition;
  }

  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
  }

  @Override
  public void setTargetPosition(double position) {
    // targetPosition =
    //     MathUtil.clamp(targetPosition + position, ElevatorConstants.minPosition,
    // ElevatorConstants.maxPosition);

    // leftPID.setReference(
    //     (position / ElevatorConstants.gearCircumfrence) * ElevatorConstants.GEAR_RATIO,
    //     ControlType.kPosition,
    //     0,
    //     0,
    //     ArbFFUnits.kVoltage);

    // rightPID.setReference(
    //     (position / ElevatorConstants.gearCircumfrence) * ElevatorConstants.GEAR_RATIO,
    //     ControlType.kPosition,
    //     0,
    //     0,
    //     ArbFFUnits.kVoltage);
    targetPosition =
        MathUtil.clamp(
            targetPosition + position,
            ElevatorConstants.minPosition,
            ElevatorConstants.maxPosition);

    leftPID.setReference(targetPosition, ControlType.kPosition, 0, 0);
    rightPID.setReference(targetPosition, ControlType.kPosition, 0, 0);

    double leftPosition = leftRelativeEncoder.getPosition();
    double rightPosition = rightRelativeEncoder.getPosition();

    leftMotor.set(
        leftMathPID.calculate(
            leftPosition,
            (targetPosition / ElevatorConstants.gearCircumfrence) * ElevatorConstants.GEAR_RATIO));
    // + Constants.FlywheelConstants.kv * targetPosition
    // + Constants.FlywheelConstants.ks);
    rightMotor.set(
        leftMathPID.calculate(
            rightPosition,
            (targetPosition / ElevatorConstants.gearCircumfrence) * ElevatorConstants.GEAR_RATIO));
    // + Constants.FlywheelConstants.kv * velocityRevPerSec
    // + Constants.FlywheelConstants.ks);
  }

  // @Override
  // public void setTargetPosition(double positionLeft, double positionRight) {
  //   targetPositionLeft =
  //       MathUtil.clamp(positionLeft, ElevatorConstants.minPosition,
  // ElevatorConstants.maxPosition);

  //   targetPositionRight =
  //       MathUtil.clamp(positionRight, ElevatorConstants.minPosition,
  // ElevatorConstants.maxPosition);

  //   leftPID.setReference(
  //       (positionLeft / ElevatorConstants.gearCircumfrence) * GEAR_RATIO,
  //       ControlType.kPosition,
  //       0,
  //       0,
  //       ArbFFUnits.kVoltage);

  //   rightPID.setReference(
  //       (positionRight / ElevatorConstants.gearCircumfrence) * GEAR_RATIO,
  //       ControlType.kPosition,
  //       0,
  //       0,
  //       ArbFFUnits.kVoltage);
  // }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    leftPID.setP(kP, 0);
    leftPID.setI(kI, 0);
    leftPID.setD(kD, 0);
    leftPID.setFF(0, 0);

    rightPID.setP(kP, 0);
    rightPID.setI(kI, 0);
    rightPID.setD(kD, 0);
    rightPID.setFF(0, 0);
  }
}
