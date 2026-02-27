package frc.robot.commands.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;

public class RollerOutCommand extends Command {
  private final RollerIntakeSubsystem rollerIntake;

  public RollerOutCommand(RollerIntakeSubsystem rollerIntake) {
    this.rollerIntake = rollerIntake;
    addRequirements(rollerIntake);
  }

  @Override
  public void initialize() {
    rollerIntake.runOut();
  }

  @Override
  public void execute() {
    rollerIntake.runOut();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    rollerIntake.stop();
  }
}
