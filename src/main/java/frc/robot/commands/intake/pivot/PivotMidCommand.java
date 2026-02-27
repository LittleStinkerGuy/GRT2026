package frc.robot.commands.intake.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;

public class PivotMidCommand extends Command {
  private final PivotIntakeSubsystem pivotIntake;

  public PivotMidCommand(PivotIntakeSubsystem pivotIntake) {
    this.pivotIntake = pivotIntake;
    addRequirements(pivotIntake);
  }

  @Override
  public void initialize() {
    pivotIntake.setPosition(IntakeConstants.PIVOT_MID_POS);
  }

  @Override
  public void execute() {
    pivotIntake.setPosition(IntakeConstants.PIVOT_MID_POS);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
