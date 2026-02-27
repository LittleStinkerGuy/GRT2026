package frc.robot.commands.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;

public class StopRollerCommand extends Command {
  private final RollerIntakeSubsystem rollerIntake;

  public StopRollerCommand(RollerIntakeSubsystem rollerIntake) {
    this.rollerIntake = rollerIntake;
    addRequirements(rollerIntake);
  }

  @Override
  public void initialize() {
    rollerIntake.stop();
  }

  @Override
  public void execute() {
    rollerIntake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}
