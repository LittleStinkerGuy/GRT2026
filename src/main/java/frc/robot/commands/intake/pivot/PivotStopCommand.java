package frc.robot.commands.intake.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;

public class PivotStopCommand extends Command {
  private final PivotIntakeSubsystem pivotIntake;

  public PivotStopCommand(PivotIntakeSubsystem pivotIntake) {
    this.pivotIntake = pivotIntake;
    addRequirements(pivotIntake);
  }

  @Override
  public void initialize() {
    pivotIntake.stop();
  }

  @Override
  public void execute() {
    pivotIntake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
