package frc.robot.commands.auton;

import static frc.robot.Constants.SwerveConstants.ROTATION_KD;
import static frc.robot.Constants.SwerveConstants.ROTATION_KI;
import static frc.robot.Constants.SwerveConstants.ROTATION_KP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Rotates the robot 90 degrees counter-clockwise from its starting heading,
 * closing the loop on the pose estimator's rotation.
 */
public class Turn90Auton extends Command {
    private static final double TOLERANCE_RAD = Math.toRadians(2.0);
    private static final double MAX_POWER = 0.5;

    private final SwerveSubsystem swerveSubsystem;
    private final PIDController rotationPID;

    public Turn90Auton(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.rotationPID = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
        this.rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationPID.setTolerance(TOLERANCE_RAD);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Rotation2d currentHeading = swerveSubsystem.getRobotPosition().getRotation();
        Rotation2d targetHeading = currentHeading.plus(Rotation2d.fromDegrees(90));
        rotationPID.reset();
        rotationPID.setSetpoint(targetHeading.getRadians());
    }

    @Override
    public void execute() {
        double currentRad = swerveSubsystem.getRobotPosition().getRotation().getRadians();
        double angularPower = MathUtil.clamp(
            rotationPID.calculate(currentRad),
            -MAX_POWER,
            MAX_POWER);
        swerveSubsystem.setRobotRelativeDrivePowers(0.0, 0.0, angularPower);
    }

    @Override
    public boolean isFinished() {
        return rotationPID.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setRobotRelativeDrivePowers(0.0, 0.0, 0.0);
    }
}
