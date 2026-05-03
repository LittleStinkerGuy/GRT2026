package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.roller.RollerSubsystem;

public class PivotAndRollerIntakeCommand extends ParallelDeadlineGroup {
    public PivotAndRollerIntakeCommand(PivotSubsystem pivot, RollerSubsystem roller) {
        super(
            Commands.startEnd(
                () -> pivot.setPosition(IntakeConstants.PIVOT_OUT_POS),
                () -> pivot.setPosition(IntakeConstants.PIVOT_IN_POS),
                pivot),
            roller.runRollerIn());
    }
}
