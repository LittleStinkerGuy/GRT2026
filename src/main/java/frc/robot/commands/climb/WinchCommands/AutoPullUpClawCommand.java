package frc.robot.commands.climb.WinchCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class AutoPullUpClawCommand extends Command {
    private final ClimbSubsystem climbSubsystem;

    public AutoPullUpClawCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setWinchTorqueCurrent(-ClimbConstants.WINCH_TORQUE_CURRENT);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isWinchAtDistance(ClimbConstants.WINCH_HOME_DISTANCE);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopWinch();
    }
}
