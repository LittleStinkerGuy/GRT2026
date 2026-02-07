package frc.robot.commands.climb.WinchCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class AutoPullDownClawCommand extends Command {
    private final ClimbSubsystem climbSubsystem;

    public AutoPullDownClawCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.setWinchPositionSetpoint(ClimbConstants.WINCH_DEPLOYED_POS);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.isWinchAtSetPosition();
    }
}