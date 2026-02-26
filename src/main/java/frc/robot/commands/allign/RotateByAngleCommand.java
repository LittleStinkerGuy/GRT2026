package frc.robot.commands.allign;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotateToAngleConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RotateByAngleCommand extends Command {
    private final SwerveSubsystem swerve;
    private final PIDController pid;
    private final double targetDegrees;
    private final BooleanSupplier cancelCondition;

    public RotateByAngleCommand(SwerveSubsystem swerve, double targetDegrees, BooleanSupplier cancelCondition) {
        this.swerve = swerve;
        this.targetDegrees = targetDegrees;
        
        this.cancelCondition = cancelCondition;
        this.pid = new PIDController(
            RotateToAngleConstants.kP,
            RotateToAngleConstants.kI,
            RotateToAngleConstants.kD
        );
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(RotateToAngleConstants.TOLERANCE_DEGREES);
        addRequirements(swerve);

        // Publish initial PID values to NetworkTables so you can edit them in Shuffleboard/SmartDashboard
        SmartDashboard.putNumber("RotateToAngle/kP", RotateToAngleConstants.kP);
        SmartDashboard.putNumber("RotateToAngle/kI", RotateToAngleConstants.kI);
        SmartDashboard.putNumber("RotateToAngle/kD", RotateToAngleConstants.kD);
        SmartDashboard.putNumber("RotateToAngle/Tolerance", RotateToAngleConstants.TOLERANCE_DEGREES);
    }

    @Override
    public void initialize() {
        // Read PID values from NetworkTables each time the command starts
        pid.setP(SmartDashboard.getNumber("RotateToAngle/kP", RotateToAngleConstants.kP));
        pid.setI(SmartDashboard.getNumber("RotateToAngle/kI", RotateToAngleConstants.kI));
        pid.setD(SmartDashboard.getNumber("RotateToAngle/kD", RotateToAngleConstants.kD));
        pid.setTolerance(SmartDashboard.getNumber("RotateToAngle/Tolerance", RotateToAngleConstants.TOLERANCE_DEGREES));
        pid.reset();
    }

    @Override
    public void execute() {
        // Use field-absolute rotation from pose, not driver heading
        double currentAngle = normalizeAngle(swerve.getRobotPosition().getRotation().getDegrees());
        double normalizedTarget = normalizeAngle(targetDegrees);
        double rotationPower = pid.calculate(currentAngle, normalizedTarget);
        // Don't negate rotation power - Pigeon negation in pose estimator already accounts for direction
        swerve.setDrivePowers(0, 0, rotationPower);

        SmartDashboard.putNumber("RotateToAngle/Goal", normalizedTarget);
        SmartDashboard.putNumber("RotateToAngle/Actual", currentAngle);
        // Calculate error with wrap-around
        double error = normalizedTarget - currentAngle;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        SmartDashboard.putNumber("RotateToAngle/Error", error);
    }

    /**
     * Normalizes an angle to the range [-180, 180]
     */
    private double normalizeAngle(double degrees) {
        double angle = degrees % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint() || cancelCondition.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setDrivePowers(0, 0, 0);
    }
    

}
