package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.intake.Intake;

public class ShootCloseCommand extends Command {

  private Arm arm;

  double ARMANGLE = 0.599;



  public ShootCloseCommand(Arm arm) {
    this.arm = arm;
  }

  // Called once at the beginning
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.runTargetAngle(ARMANGLE);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return arm.hasReachedDestination();

  }
}
