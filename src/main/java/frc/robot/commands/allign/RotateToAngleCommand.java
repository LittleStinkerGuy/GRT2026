package frc.robot.commands.allign;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RotateToAngleConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RotateToAngleCommand extends Command {
    private final SwerveSubsystem swerve;
    private final PIDController pid;
    private final double targetDegrees;
    private final BooleanSupplier cancelCondition;

    public RotateToAngleCommand(SwerveSubsystem swerve, double targetDegrees, BooleanSupplier cancelCondition) {
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
        double currentAngle = swerve.getDriverHeading().getDegrees();
        double rotationPower = pid.calculate(currentAngle, targetDegrees);
        swerve.setDrivePowers(0, 0, -rotationPower);

        SmartDashboard.putNumber("RotateToAngle/Goal", targetDegrees);
        SmartDashboard.putNumber("RotateToAngle/Actual", currentAngle);
        SmartDashboard.putNumber("RotateToAngle/Error", pid.getPositionError());
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
