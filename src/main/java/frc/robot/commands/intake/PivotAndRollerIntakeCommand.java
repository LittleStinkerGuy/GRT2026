package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;
import frc.robot.subsystems.intake.RollerIntakeSubsystem;

public class PivotAndRollerIntakeCommand extends Command {
    private final PivotIntakeSubsystem pivotIntake;
    private final RollerIntakeSubsystem rollerIntake;

    public PivotAndRollerIntakeCommand(
        PivotIntakeSubsystem pivotIntake,
        RollerIntakeSubsystem rollerIntake) {
        this.pivotIntake = pivotIntake;
        this.rollerIntake = rollerIntake;
        addRequirements(pivotIntake, rollerIntake);
    }

    @Override
    public void initialize() {
        pivotIntake.setPosition(IntakeConstants.PIVOT_OUT_POS);
        rollerIntake.runIn();
    }

    @Override
    public void execute() {
        pivotIntake.setPosition(IntakeConstants.PIVOT_OUT_POS);
        rollerIntake.runIn();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivotIntake.setPosition(IntakeConstants.PIVOT_IN_POS);
        rollerIntake.stop();
    }
}
