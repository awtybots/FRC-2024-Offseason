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

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class IntakeIOSparkMax implements IntakeIO {
  private static final double GEAR_RATIO = 52.0 / 34.0; // May be reciprocal

  private final CANSparkMax intakeMotor =
      new CANSparkMax(IntakeConstants.kIntakeSparkMaxCanId, MotorType.kBrushless);
  private final CANSparkMax followerIntakeMotor =
      new CANSparkMax(IntakeConstants.kFollowerIntakeSparkMaxCanId, MotorType.kBrushless);

  private final RelativeEncoder intakeEncoder = intakeMotor.getEncoder();
  // private final RelativeEncoder followerIntakeEncoder = followerIntakeMotor.getEncoder();

  private final SparkPIDController intakePID = intakeMotor.getPIDController();

  public IntakeIOSparkMax() {
    intakeMotor.restoreFactoryDefaults();
    followerIntakeMotor.restoreFactoryDefaults();

    intakeMotor.setCANTimeout(250);
    followerIntakeMotor.setCANTimeout(250);

    intakeMotor.setInverted(false);

    intakeMotor.setSmartCurrentLimit(30);
    followerIntakeMotor.setSmartCurrentLimit(30);

    followerIntakeMotor.follow(intakeMotor, false);

    intakeMotor.burnFlash();
    followerIntakeMotor.burnFlash();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(intakeEncoder.getPosition() / GEAR_RATIO);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(intakeEncoder.getVelocity() / GEAR_RATIO);
    inputs.appliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.currentAmps = new double[] {intakeMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    intakeMotor.setVoltage(volts);
  }

  @Override
  public void setPercentSpeed(double percentRun) {
    intakeMotor.set(percentRun);
  }

  public void runFull(double pct) {
    intakeMotor.set(1);
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }

  @Override
  public boolean getIsStalled() {
    double currentThreshold = 0.2;
    double velocityThreshold = 0.05;

    double current = intakeMotor.getOutputCurrent();
    double velocity = intakeEncoder.getVelocity();

    return Math.abs(current) > currentThreshold && Math.abs(velocity) < velocityThreshold;
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    intakePID.setP(kP, 0);
    intakePID.setI(kI, 0);
    intakePID.setD(kD, 0);
    intakePID.setFF(0, 0);
  }
}
