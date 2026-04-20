package frc.robot.commands.intake.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.PivotIntakeSubsystem;

public class SetIntakePivotCommand extends Command {
    private final PivotIntakeSubsystem pivotIntake;
    private final double targetPosition;

    /**
     * Set pivot to a specific position in rotations
     * 
     * @param pivotIntake the pivot subsystem
     * @param targetPosition target position in mechanism rotations
     */
    public SetIntakePivotCommand(PivotIntakeSubsystem pivotIntake, double targetPosition) {
        this.pivotIntake = pivotIntake;
        this.targetPosition = targetPosition;
        addRequirements(pivotIntake);
    }

    @Override
    public void initialize() {
        pivotIntake.setPosition(targetPosition);
    }

    @Override
    public void execute() {
        pivotIntake.setPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
