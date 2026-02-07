package frc.robot.commands.climb.ClimbCommands;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimbConstants;
import frc.robot.commands.climb.ArmCommands.AutoDeployArmCommand;
import frc.robot.commands.climb.WinchCommands.AutoPullDownClawCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class AutoClimbUpCommand extends SequentialCommandGroup {
    private Command deployArm;
    private Command pullDownClaw;
    private Command pullDownClawIfArmSucceeded;

    private AtomicBoolean armInterrupted = new AtomicBoolean(false);

    public AutoClimbUpCommand(ClimbSubsystem climbSubsystem) {

        deployArm = new AutoDeployArmCommand(climbSubsystem).withTimeout(ClimbConstants.ARM_POS_TIMEOUT)
                .finallyDo(interrupted -> armInterrupted.set(interrupted));

        pullDownClaw = new AutoPullDownClawCommand(climbSubsystem).withTimeout(ClimbConstants.WINCH_POS_TIMEOUT);
        pullDownClawIfArmSucceeded = Commands.either(
                Commands.none(),
                pullDownClaw,
                () -> armInterrupted.get());

        addCommands(
                Commands.runOnce(() -> System.out.println("climbUp"), climbSubsystem),
                deployArm,
                pullDownClawIfArmSucceeded);
    }
}
