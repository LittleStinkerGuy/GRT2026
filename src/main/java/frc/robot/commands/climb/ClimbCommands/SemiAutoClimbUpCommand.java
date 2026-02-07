package frc.robot.commands.climb.ClimbCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ArmCommands.ManualDeployArmCommand;
import frc.robot.commands.climb.WinchCommands.ManualPullDownClawCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class SemiAutoClimbUpCommand extends SequentialCommandGroup {
    private ManualDeployArmCommand deployArm;
    private ManualPullDownClawCommand pullDownClaw;

    public SemiAutoClimbUpCommand(ClimbSubsystem climbSubsystem, BooleanSupplier stepSupplier) {
        deployArm = new ManualDeployArmCommand(climbSubsystem, stepSupplier);
        pullDownClaw = new ManualPullDownClawCommand(climbSubsystem, stepSupplier);

        addCommands(
                waitForButtonRelease(stepSupplier),
                deployArm.raceWith(Commands.waitUntil(climbSubsystem::isArmReverseLimitActive)),
                waitForNextStep(stepSupplier),
                pullDownClaw.raceWith(Commands.waitUntil(climbSubsystem::isWinchReverseLimitActive)));
    }

    private static Command waitForButtonRelease(BooleanSupplier supplier) {
        return Commands.waitUntil(() -> !supplier.getAsBoolean());
    }

    private static Command waitForNextStep(BooleanSupplier supplier) {
        return waitForButtonRelease(supplier)
                .andThen(Commands.waitUntil(supplier))
                .andThen(waitForButtonRelease(supplier));
    }
}
