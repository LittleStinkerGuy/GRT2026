package frc.robot.commands.climb.WinchCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class AutoPullUpClawCommand extends SequentialCommandGroup {
    private Command setWinchPositionSetpoint;
    private Command waitForHardstop;
    private Command runIntoHardstop;

    public AutoPullUpClawCommand(ClimbSubsystem climbSubsystem) {
        setWinchPositionSetpoint = Commands.runOnce(
                () -> climbSubsystem.setWinchPositionSetpoint(
                        ClimbConstants.WINCH_HOME_POS),
                climbSubsystem);
        waitForHardstop = Commands.waitUntil(climbSubsystem::isWinchHardstopPressed)
                .withTimeout(ClimbConstants.WINCH_POS_TIMEOUT);
        runIntoHardstop = new RunClawIntoLimitCommand(climbSubsystem,
                -ClimbConstants.WINCH_MAX_SAFETY_DUTY_CYCLE);

        addCommands(
                setWinchPositionSetpoint,
                waitForHardstop,
                Commands.either(
                        Commands.none(),
                        runIntoHardstop,
                        climbSubsystem::isWinchHardstopPressed));
    }
}
