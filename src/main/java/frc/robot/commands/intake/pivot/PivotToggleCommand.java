package frc.robot.commands.intake.pivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SmashAndShootConstants;
import frc.robot.subsystems.Intake.PivotIntakeSubsystem;

/**
 * Runs the intake pivot pattern:
 * - starts out
 * - waits INITIAL_DELAY_SECONDS
 * - then toggles every TOGGLE_INTERVAL_SECONDS
 */
public class PivotToggleCommand extends Command {

    private final PivotIntakeSubsystem pivotIntake;
    private final Timer pivotTimer = new Timer();

    private boolean pivotIsIn = false;
    private boolean initialDelayDone = false;

    public PivotToggleCommand(PivotIntakeSubsystem pivotIntake) {
        this.pivotIntake = pivotIntake;
        addRequirements(pivotIntake);
    }

    @Override
    public void initialize() {
        pivotIsIn = false;
        initialDelayDone = false;

        pivotIntake.setPosition(IntakeConstants.PIVOT_OUT_POS);
        pivotTimer.restart();
    }

    @Override
    public void execute() {
        if (!initialDelayDone) {
            if (pivotTimer.hasElapsed(SmashAndShootConstants.INITIAL_DELAY_SECONDS)) {
                initialDelayDone = true;
                pivotIsIn = true;
                pivotTimer.restart();
            }
        } else if (pivotTimer.hasElapsed(SmashAndShootConstants.TOGGLE_INTERVAL_SECONDS)) {
            pivotIsIn = !pivotIsIn;
            pivotTimer.restart();
        }

        pivotIntake.setPosition(
            pivotIsIn
                ? IntakeConstants.PIVOT_MID_UPPER
                : IntakeConstants.PIVOT_MID_LOWER);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        pivotIntake.setPosition(IntakeConstants.PIVOT_OUT_POS);
        pivotTimer.stop();
    }
}
