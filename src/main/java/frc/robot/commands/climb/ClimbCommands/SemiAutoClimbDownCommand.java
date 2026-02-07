package frc.robot.commands.climb.ClimbCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ArmCommands.ManualRetractArmCommand;
import frc.robot.commands.climb.WinchCommands.ManualPullUpClawCommand;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class SemiAutoClimbDownCommand extends SequentialCommandGroup {
    private ManualRetractArmCommand retractArm;
    private ManualPullUpClawCommand pullUpClaw;

    public SemiAutoClimbDownCommand(ClimbSubsystem climbSubsystem, BooleanSupplier stepSupplier) {
        retractArm = new ManualRetractArmCommand(climbSubsystem, stepSupplier);
        pullUpClaw = new ManualPullUpClawCommand(climbSubsystem, stepSupplier);

        addCommands(
                waitForButtonRelease(stepSupplier),
                pullUpClaw.raceWith(Commands.waitUntil(climbSubsystem::isWinchReverseLimitActive)),
                waitForNextStep(stepSupplier),
                retractArm.raceWith(Commands.waitUntil(climbSubsystem::isArmForwardLimitActive)));
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
