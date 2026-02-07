package frc.robot.commands.climb.ClimbCommands;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.climb.ArmCommands.AutoRetractArmCommand;
import frc.robot.commands.climb.WinchCommands.AutoPullUpClawCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class AutoClimbDownCommand extends SequentialCommandGroup {
    private Command pullUpClaw;
    private Command retractArm;
    private Command retractArmIfWinchSucceeded;

    private AtomicBoolean winchInterrupted = new AtomicBoolean(false);

    public AutoClimbDownCommand(ClimbSubsystem climbSubsystem) {
        pullUpClaw = new AutoPullUpClawCommand(climbSubsystem).withTimeout(ClimbConstants.WINCH_POS_TIMEOUT.times(2))
                .finallyDo(interrupted -> winchInterrupted.set(interrupted));
        retractArm = new AutoRetractArmCommand(climbSubsystem).withTimeout(ClimbConstants.ARM_POS_TIMEOUT);
        retractArmIfWinchSucceeded = Commands.either(
                Commands.none(),
                retractArm,
                () -> winchInterrupted.get());

        addCommands(
                Commands.runOnce(() -> System.out.println("climbDown"), climbSubsystem),
                pullUpClaw,
                retractArmIfWinchSucceeded);
    }
}
