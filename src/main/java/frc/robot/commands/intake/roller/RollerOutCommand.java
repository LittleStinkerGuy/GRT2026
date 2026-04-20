package frc.robot.commands.intake.roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.intake.RollerIntakeSubsystem;

public class RollerOutCommand extends Command {
    private final RollerIntakeSubsystem rollerIntake;
    private double aggnumber;

    public RollerOutCommand(RollerIntakeSubsystem rollerIntake) {
        this.rollerIntake = rollerIntake;
        this.aggnumber = -1;
        addRequirements(rollerIntake);
    }

    public RollerOutCommand(RollerIntakeSubsystem rollerIntake, double aggnumber) {
        this.rollerIntake = rollerIntake;
        this.aggnumber = aggnumber;
        addRequirements(rollerIntake);
    }

    @Override
    public void initialize() {
        rollerIntake.setDutyCycle(aggnumber);
    }

    @Override
    public void execute() {
        rollerIntake.setDutyCycle(aggnumber);
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
