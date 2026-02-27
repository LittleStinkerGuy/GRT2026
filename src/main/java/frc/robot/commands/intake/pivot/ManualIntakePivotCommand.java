package frc.robot.commands.intake.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;
import java.util.function.DoubleSupplier;

public class ManualIntakePivotCommand extends Command {
  private final PivotIntakeSubsystem pivotIntake;
  private final DoubleSupplier speedSupplier;


  public ManualIntakePivotCommand(PivotIntakeSubsystem pivotIntake, DoubleSupplier speedSupplier) {
    this.pivotIntake = pivotIntake;
    this.speedSupplier = speedSupplier;
    addRequirements(pivotIntake);
  }
  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Get the current speed value from R2 and L2
    double speedValue = speedSupplier.getAsDouble();
    pivotIntake.setManualSpeed(speedValue * IntakeConstants.MANUAL_PIVOT_SPEED);

  }

  @Override
  public void end(boolean interrupted) {
    pivotIntake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}