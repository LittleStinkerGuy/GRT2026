package frc.robot.commands.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.RollerIntakeSubsystem;

public class RollerInCommand extends Command {
  private final RollerIntakeSubsystem rollerIntake;

  public RollerInCommand(RollerIntakeSubsystem rollerIntake) {
    this.rollerIntake = rollerIntake;
    addRequirements(rollerIntake);
  }

  @Override
  public void initialize() {
    rollerIntake.runIn();
  }

  @Override
  public void execute() {
    rollerIntake.runIn();
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
