package frc.robot.commands.intake.pivot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;

public class PivotUpTimedCommand extends Command {
    private final PivotIntakeSubsystem pivotIntake;
    private final Timer timer = new Timer();

    public PivotUpTimedCommand(PivotIntakeSubsystem pivotIntake) {
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
        pivotIntake.setManualSpeed(IntakeConstants.PIVOT_UP_DUTY_CYCLE);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(IntakeConstants.PIVOT_UP_DURATION_SECONDS);
    }

    @Override
    public void end(boolean interrupted) {
        pivotIntake.stop();
        timer.stop();
    }
}
