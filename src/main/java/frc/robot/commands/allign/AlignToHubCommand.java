package frc.robot.commands.allign;

import java.util.Set;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.Constants.AlignToHubConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AlignToHubCommand {

    /**
     * Creates a command that calculates the angle from the robot's current position
     * to the hub, then rotates to face it.
     */
    public static Command create(SwerveSubsystem swerve, BooleanSupplier cancelCondition) {
        return new DeferredCommand(() -> {
            Pose2d robotPose = swerve.getRobotPosition();
            double deltaX = AlignToHubConstants.HUB_POSITION.getX() - robotPose.getX();
            double deltaY = AlignToHubConstants.HUB_POSITION.getY() - robotPose.getY();
            double targetDegrees = Math.toDegrees(Math.atan2(deltaY, deltaX));

            // Add 90° because shooter is on the right side of the robot, not the front
            targetDegrees += 90;

            return new RotateToAngleCommand(swerve, targetDegrees, cancelCondition);
        }, Set.of(swerve));
    }
}
