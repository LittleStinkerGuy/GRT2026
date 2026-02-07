package frc.robot.commands.climb.WinchCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class RunClawIntoLimitCommand extends Command {
    private final ClimbSubsystem climbSubsystem;
    private final double dutyCycle;

    public RunClawIntoLimitCommand(ClimbSubsystem climbSubsystem, double dutyCycle) {
        this.climbSubsystem = climbSubsystem;
        this.dutyCycle = dutyCycle;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setWinchDutyCycle(dutyCycle);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.setWinchDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isWinchHardstopPressed();
    }
}
