package frc.robot.commands.intake.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;

public class SetIntakePivotCommand extends Command {
  private final PivotIntakeSubsystem pivotIntake;
  private final double targetAngle;

  public SetIntakePivotCommand(PivotIntakeSubsystem pivotIntake, double targetAngle) {
    this.pivotIntake = pivotIntake;
    this.targetAngle = targetAngle;
    addRequirements(pivotIntake);
  }

  @Override
  public void initialize() {
    pivotIntake.setAngle(targetAngle);
  }

  @Override
  public void execute() {
    pivotIntake.setAngle(targetAngle);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
