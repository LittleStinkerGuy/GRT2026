package frc.robot.commands.intake.pivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;

public class PivotDownTimedCommand extends Command {
    private final PivotIntakeSubsystem pivotIntake;
    private final Timer timer = new Timer();

    public PivotDownTimedCommand(PivotIntakeSubsystem pivotIntake) {
        this.pivotIntake = pivotIntake;
        addRequirements(pivotIntake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        pivotIntake.setManualSpeed(IntakeConstants.PIVOT_DOWN_DUTY_CYCLE);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(IntakeConstants.PIVOT_DOWN_DURATION_SECONDS);
    }

    @Override
    public void end(boolean interrupted) {
        pivotIntake.stop();
        timer.stop();
    }
}
